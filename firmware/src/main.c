#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>

/* Groupe SMP custom pour piloter la CV via mcumgr (même canal que le DFU). */
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
#include <zcbor_common.h>
#include <zcbor_encode.h>
#include <zcbor_decode.h>
#include <mgmt/mcumgr/util/zcbor_bulk.h>

#include <string.h>
#include <stdlib.h>

#include "nfc_handler.h"
#include "ble_manager.h"
#include "gatt_service.h"
#include "power_manager.h"
#include "app_events.h"
#include "ad5940_port.h"
#include "methods.h"
#include "CV.h"
#include "CA.h"
#include "DPV.h"
#include "ad5940_lp.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ══════════════════════════════════════════════════════════════════════════
 * MODE OPÉRATOIRE — mesure réelle À LA DEMANDE (capteur AD5940), SANS sleep.
 *
 * Une voltampérométrie cyclique (CV) est lancée à chaque déclenchement :
 *   - CMD_REQUEST_DATA reçu via BLE (l'app l'envoie auto à la connexion), OU
 *   - appui sur le bouton sw0 (pour tester en serial seul, sans téléphone).
 *
 * Pendant la CV, chaque point (index, tension mV, courant µA) est :
 *   - imprimé en CSV sur le serial (USB CDC), et
 *   - streamé à l'app via BLE (data point [0x10][ts BE][float BE]) si connectée.
 * En fin de cycle : pic + résistance estimée (serial), et trame de fin 0xFF (app).
 *
 * Pas de veille profonde : le scheduler fait juste WFI quand inactif.
 * ══════════════════════════════════════════════════════════════════════════ */

/* ── Sémaphores (définis ici, extern dans app_events.h) ─────────────────── */
K_SEM_DEFINE(data_request_sem, 0, 1);
K_SEM_DEFINE(paired_sem,       0, 1);

/* ── USB device (NCS 3.x / USBD_NEXT) ───────────────────────────────────── */
USBD_DEVICE_DEFINE(my_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
                   0x2FE3, 0x0001);

USBD_DESC_LANG_DEFINE(my_lang);
USBD_DESC_MANUFACTURER_DEFINE(my_manufacturer, "L.A - PBT");
USBD_DESC_PRODUCT_DEFINE(my_product, "NFC_AUTH");
USBD_DESC_SERIAL_NUMBER_DEFINE(my_sn);
USBD_DESC_CONFIG_DEFINE(my_fs_cfg_desc, "FS Configuration");
USBD_CONFIGURATION_DEFINE(my_config, USB_SCD_SELF_POWERED, 250, &my_fs_cfg_desc);

/* ── Callback NFC tap ────────────────────────────────────────────────────── */
static void on_nfc_tap(void)
{
    LOG_INF("NFC tap → démarrage advertising BLE");
    ble_manager_start_adv();
}

/* ── Callback post-authentification BLE ─────────────────────────────────── */
static void on_ble_secured(struct bt_conn *conn)
{
    ARG_UNUSED(conn);
    /* Pas de mode économie d'énergie pour l'instant : on garde l'intervalle de
     * connexion rapide pour un streaming réactif des points de la CV. */
    LOG_INF("BLE sécurisé — prêt à recevoir CMD_REQUEST_DATA");
}

/* ── Connexion cible pour le streaming BLE des points CV (NULL = serial seul) ── */
static struct bt_conn *g_stream_conn;

/* Vrai pendant qu'une CV est en cours : sert à ignorer l'entrée série pendant la
 * mesure (évite d'entrelacer des réponses de commande avec les lignes CSV, et
 * tout re-déclenchement intempestif). */
static volatile bool s_cv_running;

/* ── Buffer RAM des points de la CV ───────────────────────────────────────
 * Le port USB CDC est réservé au SMP (DFU mcumgr + groupe CV custom) : on n'y
 * envoie AUCUN texte brut (ça corromprait le SMP). Les points sont stockés ici
 * pendant la mesure, puis récupérés par le PC via SMP (groupe CV, cmd DATA).
 * Les logs partent sur RTT (J-Link), pas sur l'USB. */
#define CV_MAX_POINTS 4096
/* Axe X : tension (mV) pour CV/DPV, temps (ms) pour CA → int32 (le temps dépasse
 * 32767 ms). Axe Y : courant (nA), ou ΔI (nA) pour la DPV. */
static int32_t           s_cv_x[CV_MAX_POINTS];      /* tension mV ou temps ms */
static int32_t           s_cv_cur[CV_MAX_POINTS];    /* courant / ΔI (nA)      */
static volatile uint32_t s_cv_npts;                  /* nb de points collectés */

/* Méthode d'analyse en cours + gamme RTIA demandée (ohms) pour CA/DPV. */
static analysis_method_t s_method = METHOD_CV;
static uint32_t          s_meas_rtia_ohm = 4000;

/* ── Callback par point CV : stocke dans le buffer + stream BLE ──────────── */
static void cv_point(uint32_t index, float volt_mv, float current_ua)
{
    if (s_cv_npts < CV_MAX_POINTS) {
        s_cv_x[s_cv_npts]   = (int32_t)volt_mv;   /* mV (CV/DPV) ou ms (CA) */
        s_cv_cur[s_cv_npts] = (int32_t)(current_ua * 1000.0f);
        s_cv_npts++;
    }
    /* Stream BLE best-effort (chemin app/NFC) : [0x10][index 8B BE][float 4B BE] */
    if (g_stream_conn) {
        uint8_t pkt[13];
        pkt[0] = 0x10;
        sys_put_be64((uint64_t)index, &pkt[1]);
        union { float f; uint32_t u; } conv = { .f = current_ua };
        sys_put_be32(conv.u, &pkt[9]);
        (void)gatt_service_notify(g_stream_conn, pkt, sizeof(pkt));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * MESURE — PLACEHOLDER
 * Remplacer le corps de cette fonction par la vraie acquisition capteur.
 * Retourne la valeur à envoyer via BLE (int32_t, unité libre).
 * L'HFXO est activé avant l'appel et désactivé après.
 * ══════════════════════════════════════════════════════════════════════════ */
static int32_t app_measure(void)
{
    /* 1. Alimenter le capteur (P1.00 HIGH + 50 ms de stabilisation) */
    ad5940_power_on();

    /* 2. Initialisation AD5940 : reset, horloge, FIFO, séquenceur, LFOSC */
    float lfosc_hz = ad5940_platform_init();

    /* 2b. Test de vie SPI : on annule si l'AD5940 ne répond pas (ADIID),
     *     pour ne pas rester bloqué dans la boucle d'attente de la CV. */
    {
        uint32_t adiid = ad5940_read_adiid();
        LOG_INF("AD5940 ADIID=0x%04X (attendu 0x4144)", (unsigned)adiid);
        if (lfosc_hz < 0.0f || adiid != 0x4144) {
            LOG_ERR("Mesure annulée : AD5940 muet (ADIID=0x%04X) — cablage SPI ?",
                    (unsigned)adiid);
            ad5940_power_off();
            return -1;
        }
    }

    /* 2c. Autorange : si activé, sonde rapidement pour choisir le RTIA avant
     *     le vrai cycle. Sans effet si l'autorange est désactivé. */
    CV_Autorange(lfosc_hz);

    /* 3. Configurer et lancer un cycle de voltampérométrie cyclique */
    if (CV_Init(lfosc_hz) != AD5940ERR_OK) {
        LOG_ERR("CV_Init échoué");
        ad5940_power_off();
        return -1;
    }
    LOG_INF("CV RTIA nominal=%u ohm calibre=%u ohm autorange=%d",
            (unsigned)CV_GetActiveRtiaOhm(), (unsigned)CV_GetCalibratedRtiaOhm(),
            CV_GetAutorange());

    if (CV_Start() != AD5940ERR_OK) {
        LOG_ERR("CV_Start échoué");
        ad5940_power_off();
        return -1;
    }

    /* 4. Attendre la fin du cycle (~ CV_DURATION_MS = 24 s).
     *    k_sleep() laisse le scheduler maintenir la connexion BLE.
     *    Timeout de sécurité : si la CV ne se termine jamais (pas de données
     *    FIFO), on abandonne au lieu de boucler indéfiniment. */
    int64_t cv_start_ms = k_uptime_get();
    while (!CV_IsFinished()) {
        k_sleep(K_MSEC(20));
        CV_Service();
        if (k_uptime_get() - cv_start_ms > (CV_GetActiveDurationMs() + 6000)) {
            LOG_ERR("CV timeout — la CV ne se termine pas (FIFO vide ?)");
            ad5940_power_off();
            return -1;
        }
    }

    /* 5. Lire le courant de pic (en nA) */
    int32_t peak_na = CV_GetPeakCurrentNa();
    LOG_INF("CV terminée — pic = %d nA", peak_na);

    /* 6. Éteindre le capteur */
    ad5940_power_off();

    return peak_na;
}

/* ── Enveloppe commune CA/DPV : alim + init + contrôle vie SPI ─────────────
 * Renvoie le pic (nA) via la fonction de run passée, ou -1 si AD5940 muet. */
static int32_t ca_measure(void)
{
    ad5940_power_on();
    float lfosc_hz = ad5940_platform_init();
    uint32_t adiid = ad5940_read_adiid();
    LOG_INF("AD5940 ADIID=0x%04X (attendu 0x4144)", (unsigned)adiid);
    if (lfosc_hz < 0.0f || adiid != 0x4144) {
        LOG_ERR("CA annulée : AD5940 muet (ADIID=0x%04X)", (unsigned)adiid);
        ad5940_power_off();
        return -1;
    }
    int rc = CA_Run(lfosc_hz, s_meas_rtia_ohm ? s_meas_rtia_ohm : 4000);
    ad5940_power_off();
    return (rc == 0) ? CA_GetPeakCurrentNa() : -1;
}

static int32_t dpv_measure(void)
{
    ad5940_power_on();
    float lfosc_hz = ad5940_platform_init();
    uint32_t adiid = ad5940_read_adiid();
    LOG_INF("AD5940 ADIID=0x%04X (attendu 0x4144)", (unsigned)adiid);
    if (lfosc_hz < 0.0f || adiid != 0x4144) {
        LOG_ERR("DPV annulée : AD5940 muet (ADIID=0x%04X)", (unsigned)adiid);
        ad5940_power_off();
        return -1;
    }
    int rc = DPV_Run(lfosc_hz, s_meas_rtia_ohm ? s_meas_rtia_ohm : 4000);
    ad5940_power_off();
    return (rc == 0) ? DPV_GetPeakCurrentNa() : -1;
}

/* Bilan du dernier cycle, exposé via SMP (STATUS) et loggé sur RTT. */
static volatile int32_t s_cv_peak_na;
static volatile int32_t s_cv_r_ohm;

/* ── Lance la mesure de la méthode courante : remplit le buffer + stream BLE ── */
static void run_measurement(void)
{
    struct bt_conn *conn = ble_manager_get_conn();  /* NULL = pas d'app BLE */
    g_stream_conn = conn;
    s_cv_npts     = 0;                               /* nouveau cycle : buffer vide */
    s_cv_running  = true;

    LOG_INF("Declenchement mesure (method=%d)%s", (int)s_method,
            conn ? " (stream BLE actif)" : "");

    power_mgr_hfxo_enable();
    int32_t peak_na;
    switch (s_method) {
    case METHOD_CA:  peak_na = ca_measure();  break;
    case METHOD_DPV: peak_na = dpv_measure(); break;
    case METHOD_CV:
    default:         peak_na = app_measure(); break;
    }
    power_mgr_hfxo_disable();

    s_cv_peak_na = peak_na;
    /* R estimée : pertinente uniquement pour la CV (test résistance). */
    s_cv_r_ohm   = (s_method == METHOD_CV) ? (int32_t)CV_GetResistanceOhm() : 0;
    LOG_INF("Mesure terminee — %u points, pic=%d nA, R~=%d ohm",
            (unsigned)s_cv_npts, peak_na, s_cv_r_ohm);

    if (conn) {
        uint8_t end = 0xFF;                          /* fin de burst → save app */
        (void)gatt_service_notify(conn, &end, sizeof(end));
        bt_conn_unref(conn);
    }
    g_stream_conn = NULL;
    s_cv_running  = false;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Groupe SMP custom « CV » (ID 64) — pilotage depuis le PC via mcumgr, sur le
 * MÊME port USB que le DFU (multiplexé par mcumgr, aucun conflit). Le PC parle
 * SMP (lib smpclient côté Python). Commandes :
 *   START  (cmd 0, write) : sélectionne la méthode + params entiers optionnels, lance.
 *      commun : method (0=CV,1=CA,2=DPV), vzero_mv, rtia (CV: 0=autorange)
 *      CV  : start_mv peak_mv steps dur_ms delay_us
 *      CA  : ca_pot_mv ca_dur_ms ca_interval_ms
 *      DPV : start_mv end_mv dpv_estep_mv dpv_epulse_mv dpv_tpulse_ms dpv_tperiod_ms
 *   STATUS (cmd 1, read)  : { run, n, peak_na, r_ohm, rtia, method }
 *   DATA   (cmd 2, read)  : entrée { off } → { off, n, v:[..], i:[..] } par lots
 *      v[] = tension mV (CV/DPV) ou temps ms (CA) ; i[] = courant nA (ΔI en DPV)
 * ══════════════════════════════════════════════════════════════════════════ */
#define CV_MGMT_GROUP_ID    MGMT_GROUP_ID_PERUSER   /* 64 */
#define CV_MGMT_CMD_START   0
#define CV_MGMT_CMD_STATUS  1
#define CV_MGMT_CMD_DATA    2
#define CV_DATA_CHUNK       64                      /* points max par réponse DATA */

static int cv_mgmt_start(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;
    zcbor_state_t *zse = ctxt->writer->zs;

    /* Paramètres optionnels (entiers) ; INT32_MIN = « non fourni ». */
    int32_t method = INT32_MIN, vzero_mv = INT32_MIN, rtia = INT32_MIN;
    int32_t start_mv = INT32_MIN, peak_mv = INT32_MIN, end_mv = INT32_MIN;
    int32_t steps = INT32_MIN, dur_ms = INT32_MIN, delay_us = INT32_MIN;
    int32_t ca_pot_mv = INT32_MIN, ca_dur_ms = INT32_MIN, ca_interval_ms = INT32_MIN;
    int32_t dpv_estep_mv = INT32_MIN, dpv_epulse_mv = INT32_MIN;
    int32_t dpv_tpulse_ms = INT32_MIN, dpv_tperiod_ms = INT32_MIN;
    size_t decoded;
    struct zcbor_map_decode_key_val dec[] = {
        ZCBOR_MAP_DECODE_KEY_DECODER("method",   zcbor_int32_decode, &method),
        ZCBOR_MAP_DECODE_KEY_DECODER("vzero_mv", zcbor_int32_decode, &vzero_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("rtia",     zcbor_int32_decode, &rtia),
        ZCBOR_MAP_DECODE_KEY_DECODER("start_mv", zcbor_int32_decode, &start_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("peak_mv",  zcbor_int32_decode, &peak_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("end_mv",   zcbor_int32_decode, &end_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("steps",    zcbor_int32_decode, &steps),
        ZCBOR_MAP_DECODE_KEY_DECODER("dur_ms",   zcbor_int32_decode, &dur_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("delay_us", zcbor_int32_decode, &delay_us),
        ZCBOR_MAP_DECODE_KEY_DECODER("ca_pot_mv",      zcbor_int32_decode, &ca_pot_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("ca_dur_ms",      zcbor_int32_decode, &ca_dur_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("ca_interval_ms", zcbor_int32_decode, &ca_interval_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_estep_mv",   zcbor_int32_decode, &dpv_estep_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_epulse_mv",  zcbor_int32_decode, &dpv_epulse_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_tpulse_ms",  zcbor_int32_decode, &dpv_tpulse_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_tperiod_ms", zcbor_int32_decode, &dpv_tperiod_ms),
    };
    (void)zcbor_map_decode_bulk(zsd, dec, ARRAY_SIZE(dec), &decoded);

    if (s_cv_running) {
        return MGMT_ERR_EBUSY;
    }

    s_method = (method != INT32_MIN) ? (analysis_method_t)method : METHOD_CV;

    if (s_method == METHOD_CV) {
        if (start_mv != INT32_MIN || peak_mv != INT32_MIN || vzero_mv != INT32_MIN ||
            steps != INT32_MIN || dur_ms != INT32_MIN || delay_us != INT32_MIN) {
            cv_params_t p;
            CV_GetParams(&p);
            if (start_mv != INT32_MIN) p.start_mv = (float)start_mv;
            if (peak_mv  != INT32_MIN) p.peak_mv  = (float)peak_mv;
            if (vzero_mv != INT32_MIN) p.vzero_mv = (float)vzero_mv;
            if (steps    != INT32_MIN) p.step_number = (uint32_t)steps;
            if (dur_ms   != INT32_MIN) p.duration_ms = (uint32_t)dur_ms;
            if (delay_us != INT32_MIN) p.sample_delay_ms = (float)delay_us / 1000.0f;
            if (CV_SetParams(&p) != 0) {
                return MGMT_ERR_EINVAL;
            }
        }
        if (rtia != INT32_MIN) {
            CV_SetRtia((uint32_t)rtia);          /* 0 = autorange */
        }
    } else if (s_method == METHOD_CA) {
        ca_params_t p;
        CA_GetParams(&p);
        if (ca_pot_mv      != INT32_MIN) p.pot_mv      = (float)ca_pot_mv;
        if (vzero_mv       != INT32_MIN) p.vzero_mv    = (float)vzero_mv;
        if (ca_dur_ms      != INT32_MIN) p.duration_ms = (uint32_t)ca_dur_ms;
        if (ca_interval_ms != INT32_MIN) p.interval_ms = (uint32_t)ca_interval_ms;
        if (CA_SetParams(&p) != 0) {
            return MGMT_ERR_EINVAL;
        }
        s_meas_rtia_ohm = (rtia != INT32_MIN && rtia > 0) ? (uint32_t)rtia : 4000;
    } else if (s_method == METHOD_DPV) {
        dpv_params_t p;
        DPV_GetParams(&p);
        if (start_mv       != INT32_MIN) p.start_mv   = (float)start_mv;
        if (end_mv         != INT32_MIN) p.end_mv     = (float)end_mv;
        if (dpv_estep_mv   != INT32_MIN) p.estep_mv   = (float)dpv_estep_mv;
        if (dpv_epulse_mv  != INT32_MIN) p.epulse_mv  = (float)dpv_epulse_mv;
        if (dpv_tpulse_ms  != INT32_MIN) p.tpulse_ms  = (uint32_t)dpv_tpulse_ms;
        if (dpv_tperiod_ms != INT32_MIN) p.tperiod_ms = (uint32_t)dpv_tperiod_ms;
        if (vzero_mv       != INT32_MIN) p.vzero_mv   = (float)vzero_mv;
        if (DPV_SetParams(&p) != 0) {
            return MGMT_ERR_EINVAL;
        }
        s_meas_rtia_ohm = (rtia != INT32_MIN && rtia > 0) ? (uint32_t)rtia : 4000;
    } else {
        return MGMT_ERR_EINVAL;
    }

    s_cv_npts = 0;                           /* buffer vide → le PC voit n=0 puis la montée */
    k_sem_give(&data_request_sem);           /* déclenche la mesure (boucle principale) */

    bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0);
    return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static int cv_mgmt_status(struct smp_streamer *ctxt)
{
    zcbor_state_t *zse = ctxt->writer->zs;
    bool ok = zcbor_tstr_put_lit(zse, "run")     && zcbor_bool_put(zse, s_cv_running)  &&
              zcbor_tstr_put_lit(zse, "n")       && zcbor_uint32_put(zse, s_cv_npts)   &&
              zcbor_tstr_put_lit(zse, "peak_na") && zcbor_int32_put(zse, s_cv_peak_na) &&
              zcbor_tstr_put_lit(zse, "r_ohm")   && zcbor_int32_put(zse, s_cv_r_ohm)   &&
              zcbor_tstr_put_lit(zse, "rtia")    && zcbor_uint32_put(zse, CV_GetActiveRtiaOhm()) &&
              zcbor_tstr_put_lit(zse, "method")  && zcbor_uint32_put(zse, (uint32_t)s_method);
    return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static int cv_mgmt_data(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;
    zcbor_state_t *zse = ctxt->writer->zs;

    uint32_t off = 0;
    size_t decoded;
    struct zcbor_map_decode_key_val dec[] = {
        ZCBOR_MAP_DECODE_KEY_DECODER("off", zcbor_uint32_decode, &off),
    };
    (void)zcbor_map_decode_bulk(zsd, dec, ARRAY_SIZE(dec), &decoded);

    uint32_t n = s_cv_npts;
    if (off > n) off = n;
    uint32_t end = off + CV_DATA_CHUNK;
    if (end > n) end = n;
    uint32_t cnt = end - off;

    bool ok = zcbor_tstr_put_lit(zse, "off") && zcbor_uint32_put(zse, off) &&
              zcbor_tstr_put_lit(zse, "n")   && zcbor_uint32_put(zse, n)   &&
              zcbor_tstr_put_lit(zse, "v")   && zcbor_list_start_encode(zse, cnt);
    for (uint32_t i = off; i < end && ok; i++) {
        ok = zcbor_int32_put(zse, s_cv_x[i]);
    }
    ok = ok && zcbor_list_end_encode(zse, cnt) &&
         zcbor_tstr_put_lit(zse, "i") && zcbor_list_start_encode(zse, cnt);
    for (uint32_t i = off; i < end && ok; i++) {
        ok = zcbor_int32_put(zse, s_cv_cur[i]);
    }
    ok = ok && zcbor_list_end_encode(zse, cnt);
    return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static const struct mgmt_handler cv_mgmt_handlers[] = {
    [CV_MGMT_CMD_START]  = { NULL,            cv_mgmt_start },
    [CV_MGMT_CMD_STATUS] = { cv_mgmt_status,  NULL },
    [CV_MGMT_CMD_DATA]   = { cv_mgmt_data,    NULL },
};

static struct mgmt_group cv_mgmt_group = {
    .mg_handlers       = cv_mgmt_handlers,
    .mg_handlers_count = ARRAY_SIZE(cv_mgmt_handlers),
    .mg_group_id       = CV_MGMT_GROUP_ID,
};

static void cv_mgmt_register(void)
{
    mgmt_register_group(&cv_mgmt_group);
}
MCUMGR_HANDLER_DEFINE(cv_mgmt, cv_mgmt_register);

/* ── Point d'entrée ──────────────────────────────────────────────────────── */
int main(void)
{
    int err;

    /* Confirme l'image courante auprès de MCUboot : sinon, après un flash en
     * USB DFU, MCUboot la traite comme image de TEST et REVERT à l'ancienne au
     * reset suivant — la carte tournerait éternellement le vieux binaire. */
#if defined(CONFIG_BOOTLOADER_MCUBOOT)
    if (!boot_is_img_confirmed()) {
        int cerr = boot_write_img_confirmed();
        if (cerr) {
            LOG_ERR("Confirmation image MCUboot echouee: %d", cerr);
        } else {
            LOG_INF("Image MCUboot confirmee (permanente, pas de revert)");
        }
    }
#endif

    /* Gestionnaire d'alimentation (GPIO HFXO + ADC VBUS) */
    err = power_mgr_init();
    if (err) {
        LOG_ERR("power_mgr_init: %d", err);
        return err;
    }

    /* GPIO AD5940 : CS, RST, load switch — capteur éteint au démarrage */
    ad5940_hw_init();

    /* Bluetooth */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable: %d", err);
        return err;
    }
    LOG_INF("Bluetooth activé");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_load();
        if (err) {
            LOG_WRN("settings_load: %d", err);
        }
    }

    /* Enregistrer le callback post-pairing avant init */
    ble_manager_set_secured_cb(on_ble_secured);

    err = ble_manager_init();
    if (err) {
        LOG_ERR("ble_manager_init: %d", err);
        return err;
    }

    /* USB */
    usbd_add_descriptor(&my_usbd, &my_lang);
    usbd_add_descriptor(&my_usbd, &my_manufacturer);
    usbd_add_descriptor(&my_usbd, &my_product);
    usbd_add_descriptor(&my_usbd, &my_sn);
    usbd_add_configuration(&my_usbd, USBD_SPEED_FS, &my_config);
    usbd_register_all_classes(&my_usbd, USBD_SPEED_FS, 1, NULL);
    usbd_init(&my_usbd);
    usbd_enable(&my_usbd);

    /* NFC */
    err = nfc_handler_init(on_nfc_tap);
    if (err) {
        LOG_ERR("nfc_handler_init: %d", err);
        return err;
    }
    err = nfc_handler_start();
    if (err) {
        LOG_ERR("nfc_handler_start: %d", err);
        return err;
    }

    /* Lecture VBUS au démarrage (informative) */
    if (power_mgr_vbus_present()) {
        int32_t vbus_mv = power_mgr_vbus_read_mv();
        if (vbus_mv > 0) {
            LOG_INF("VBUS présent: %d mV", vbus_mv);
        }
    } else {
        LOG_INF("VBUS absent (alimentation batterie)");
    }

    /* Streaming des points : même callback pour toutes les méthodes. */
    CV_SetPointCallback(cv_point);
    CA_SetPointCallback(cv_point);
    DPV_SetPointCallback(cv_point);

    LOG_INF("Système prêt — CV à la demande (CMD_REQUEST_DATA BLE, ou commande RUN/SET sur le serial)");

    /* ════════════════════════════════════════════════════════════════════
     * BOUCLE PRINCIPALE — À LA DEMANDE, SANS SLEEP
     *   Attend un déclencheur (CMD_REQUEST_DATA via BLE, ou bouton sw0),
     *   lance une CV, diffuse la courbe (serial CSV + BLE), puis se remet
     *   en attente. Le CPU fait WFI entre deux mesures (pas de veille profonde).
     * ════════════════════════════════════════════════════════════════════ */
    while (1) {
        k_sem_take(&data_request_sem, K_FOREVER);
        run_measurement();
    }

#if 0  /* ancien mode produit autonome (sleep) — réactiver plus tard */
    while (1) {
        {
            k_sleep(K_MSEC(30000U));

            struct bt_conn *conn = ble_manager_get_conn();
            if (!conn) {
                LOG_INF("Pas de connexion BLE — attente du prochain pairing");
                k_sem_take(&paired_sem, K_FOREVER);
                continue;
            }

            power_mgr_hfxo_enable();
            int32_t result = app_measure();
            power_mgr_hfxo_disable();

            LOG_INF("Mesure envoyée: %d", result);

            bt_conn_unref(conn);
        }
    }
#endif  /* ancien mode produit autonome */

    return 0;
}

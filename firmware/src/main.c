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
#include <math.h>
#include <limits.h>

#include "nfc_handler.h"
#include "ble_manager.h"
#include "gatt_service.h"
#include "power_manager.h"
#include "app_events.h"
#include "ad5940_port.h"
#include "methods.h"
#include "CV.h"
#include "RampTest.h"
#include "CA.h"
#include "DPV.h"
#include "ad5940_lp.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Identifiant volontairement stable et lisible dans START/STATUS et dans les
 * rapports PC. Il permet de savoir sans ambiguite quel correctif etait flashe
 * lors d'une mesure terrain. */
#define FIRMWARE_BUILD_ID "20260825-directinit-v8"

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
static volatile bool s_measurement_pending;
/* Une mesure lancée par SMP conserve son buffer jusqu'à RELEASE. Cela empêche
 * une requête BLE d'écraser les points entre le STATUS
 * final et leur téléchargement par le PC. Le bail est borné pour ne pas
 * bloquer indéfiniment le BLE si le logiciel PC disparaît. */
static volatile bool s_smp_run_owner;
static volatile bool s_smp_data_held;
static volatile bool s_smp_start_configuring;
static int64_t s_smp_data_expires_at_ms;
#define SMP_DATA_LEASE_MS 60000
static volatile int32_t s_measure_error;
static struct k_spinlock s_data_lock;
static volatile uint32_t s_cv_npts;                  /* nb de points collectés */
/* STOP est une demande cooperative : le handler SMP ne touche jamais au SPI.
 * Le thread de mesure consulte ce drapeau puis arrete proprement le moteur. */
static volatile bool     s_stop_requested;
static volatile bool     s_measure_cancelled;
static volatile uint32_t s_measure_settle_ms;
/* Méthode d'analyse en cours + gamme RTIA demandée (ohms) pour CA/DPV. */
static analysis_method_t s_method = METHOD_CV;
static uint32_t          s_meas_rtia_ohm = 4000;

/* Codes stables exposés dans STATUS.err. Le détail natif (AD5940Err ou errno
 * SPI) est conservé séparément dans err_detail. */
#define MEAS_ERR_BUFFER_FULL       -2
#define MEAS_ERR_INVALID_SAMPLE    -3
#define MEAS_ERR_PLATFORM_INIT    -10
#define MEAS_ERR_ADIID            -11
#define MEAS_ERR_SPI              -12
#define MEAS_ERR_CV_INIT          -20
#define MEAS_ERR_CV_START         -21
#define MEAS_ERR_CV_SERVICE       -22
#define MEAS_ERR_CV_STALL         -23
#define MEAS_ERR_CV_TIMEOUT       -24
#define MEAS_ERR_CV_INCOMPLETE    -25
#define MEAS_ERR_CV_PRECHARGE     -26
#define MEAS_ERR_CA_RUN           -30
#define MEAS_ERR_DPV_RUN          -40

static volatile int32_t     s_measure_error_detail;
static const char *volatile s_measure_stage = "idle";
static const char *volatile s_measure_error_stage = "none";
static volatile uint32_t    s_measure_run_id;
static volatile uint32_t    s_measure_adiid;
static volatile uint32_t    s_measure_elapsed_ms;
static volatile uint32_t    s_measure_idle_ms;
static int64_t              s_measure_start_at_ms;
static int64_t              s_measure_progress_at_ms;

bool measurement_request_submit(void)
{
    cv_params_t cv_params_snapshot;
    CV_GetParams(&cv_params_snapshot);
    k_spinlock_key_t key = k_spin_lock(&s_data_lock);
    int64_t now = k_uptime_get();
    bool lease_expired = false;
    uint32_t expired_run_id = 0;
    if (s_smp_data_held && !s_cv_running && !s_measurement_pending &&
        s_smp_data_expires_at_ms > 0 && now >= s_smp_data_expires_at_ms) {
        expired_run_id = s_measure_run_id;
        s_smp_data_held = false;
        s_smp_run_owner = false;
        s_smp_data_expires_at_ms = 0;
        lease_expired = true;
    }
    if (s_cv_running || s_measurement_pending || s_smp_data_held) {
        k_spin_unlock(&s_data_lock, key);
        if (lease_expired) {
            LOG_WRN("Bail DATA SMP expire pour run_id=%u",
                    (unsigned)expired_run_id);
        }
        return false;
    }
    /* L'identifiant est réservé dès pending, pour rester immuable pendant
     * toute la vie de la requête BLE comme pour une requête SMP. */
    s_measure_run_id++;
    s_cv_npts = 0;
    s_measure_error = 0;
    s_measure_error_detail = 0;
    s_measure_error_stage = "none";
    s_measure_elapsed_ms = 0u;
    s_measure_idle_ms = 0u;
    s_stop_requested = false;
    s_measure_cancelled = false;
    s_measure_settle_ms = (s_method == METHOD_CV) ?
                          cv_params_snapshot.settle_ms : 0u;
    s_measure_stage = "pending";
    s_measurement_pending = true;
    s_smp_run_owner = false;
    s_smp_data_expires_at_ms = 0;
    k_spin_unlock(&s_data_lock, key);
    if (lease_expired) {
        LOG_WRN("Bail DATA SMP expire pour run_id=%u",
                (unsigned)expired_run_id);
    }
    k_sem_give(&data_request_sem);
    return true;
}

static void measurement_set_stage(const char *stage)
{
    s_measure_stage = stage;
}

static void measurement_update_timing(void)
{
    int64_t now = k_uptime_get();
    if (s_measure_start_at_ms > 0) {
        int64_t elapsed = now - s_measure_start_at_ms;
        s_measure_elapsed_ms = (elapsed > 0) ? (uint32_t)elapsed : 0u;
    }
    if (s_measure_progress_at_ms > 0) {
        int64_t idle = now - s_measure_progress_at_ms;
        s_measure_idle_ms = (idle > 0) ? (uint32_t)idle : 0u;
    }
}

static bool measurement_stop_is_requested(void)
{
    k_spinlock_key_t key = k_spin_lock(&s_data_lock);
    bool requested = s_stop_requested;
    k_spin_unlock(&s_data_lock, key);
    return requested;
}

/* Attente interruptible utilisee pour la pre-polarisation. Des tranches de
 * 20 ms gardent le bouton Stop reactif sans effectuer d'E/S depuis le handler
 * SMP. */
static bool measurement_wait_or_stop(uint32_t delay_ms)
{
    while (delay_ms > 0u) {
        if (measurement_stop_is_requested()) return true;
        uint32_t slice_ms = (delay_ms > 20u) ? 20u : delay_ms;
        k_sleep(K_MSEC(slice_ms));
        delay_ms -= slice_ms;
    }
    return measurement_stop_is_requested();
}

/* Première cause gagnante : un timeout ultérieur ne doit jamais écraser une
 * erreur SPI/ISR plus utile. */
static int measurement_fail(int32_t code, const char *stage, int32_t detail)
{
    AppRAMPDiag_Type diag;

    if (s_measure_error == 0) {
        s_measure_error = code;
        s_measure_error_detail = detail;
        s_measure_error_stage = stage;
    }
    measurement_set_stage(stage);
    measurement_update_timing();
    AppRAMPGetDiag(&diag);
    LOG_ERR("Mesure interrompue: err=%d detail=%d stage=%s n=%u/%u elapsed=%u idle=%u "
            "spi=%u/%d ramp_err=%d intc0=0x%08x intc1=0x%08x fatal=0x%08x "
            "stale=%u retry=%u unconfirmed=%u rejected=0x%08x/0x%08x "
            "seqcon=0x%08x seqcnt=%u seqtimeout=%u "
            "custom=%u refill=%u blocks=%u remain=%u fifo=%u",
            (int)s_measure_error, (int)s_measure_error_detail, stage,
            (unsigned)s_cv_npts, (unsigned)CV_GetActiveTotalPoints(),
            (unsigned)s_measure_elapsed_ms, (unsigned)s_measure_idle_ms,
            (unsigned)ad5940_spi_error_count(), (int)ad5940_spi_last_error(),
            (int)diag.last_error, (unsigned)diag.last_intc0,
            (unsigned)diag.last_intc1, (unsigned)diag.fatal_flags,
            (unsigned)diag.stale_intc_count, (unsigned)diag.intc_retry_count,
            (unsigned)diag.unconfirmed_fatal_count,
            (unsigned)diag.last_rejected_intc0,
            (unsigned)diag.last_rejected_intc1,
            (unsigned)diag.seq_con, (unsigned)diag.seq_count,
            (unsigned)diag.seq_timeout,
            (unsigned)diag.custom_irq_count, (unsigned)diag.refill_count,
            (unsigned)diag.blocks_generated, (unsigned)diag.steps_remaining,
            (unsigned)diag.last_fifo_count);
    return -1;
}

/* Let the SMP handler send its START acknowledgement before doing the
 * potentially long AD5940 initialisation. */
static void measurement_start_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    /* pending reste vrai jusqu'à la transition atomique vers running. */
    k_sem_give(&data_request_sem);
}
K_WORK_DELAYABLE_DEFINE(measurement_start_work, measurement_start_work_handler);

/* ── Buffer RAM des points de la CV ───────────────────────────────────────
 * Le port USB CDC est réservé au SMP (DFU mcumgr + groupe CV custom) : on n'y
 * envoie AUCUN texte brut (ça corromprait le SMP). Les points sont stockés ici
 * pendant la mesure, puis récupérés par le PC via SMP (groupe CV, cmd DATA).
 * Les logs partent sur RTT (J-Link), pas sur l'USB. */
#define CV_MAX_POINTS CV_MAX_TOTAL_POINTS
/* Axe X : tension (mV) pour CV/DPV, temps (ms) pour CA → int32 (le temps dépasse
 * 32767 ms). Axe Y : courant (nA), ou ΔI (nA) pour la DPV. */
static int32_t           s_cv_x[CV_MAX_POINTS];      /* tension mV ou temps ms */
static int32_t           s_cv_cur[CV_MAX_POINTS];    /* courant / ΔI (nA)      */

/* ── Callback par point CV : stocke dans le buffer + stream BLE ──────────── */
static void cv_point(uint32_t index, float volt_mv, float current_ua)
{
    double current_na = (double)current_ua * 1000.0;
    if (!isfinite(volt_mv) || !isfinite(current_ua) ||
        current_na < (double)INT32_MIN || current_na > (double)INT32_MAX) {
        if (s_measure_error == 0) {
            s_measure_error = MEAS_ERR_INVALID_SAMPLE;
            s_measure_error_detail = (int32_t)index;
        }
        return;
    }
    k_spinlock_key_t key = k_spin_lock(&s_data_lock);
    if (s_cv_npts < CV_MAX_POINTS) {
        uint32_t slot = s_cv_npts;
        s_cv_x[slot]   = (int32_t)volt_mv;   /* mV (CV/DPV) ou ms (CA) */
        s_cv_cur[slot] = (int32_t)current_na;
        /* Publier n seulement après les deux valeurs du point. */
        s_cv_npts = slot + 1u;
        k_spin_unlock(&s_data_lock, key);
        s_measure_progress_at_ms = k_uptime_get();
        s_measure_idle_ms = 0u;
    } else {
        k_spin_unlock(&s_data_lock, key);
        if (s_measure_error == 0) {
            /* Should be prevented by CV_SetParams(). Keep a visible failure if a
             * future method/configuration ever violates the shared buffer limit. */
            LOG_ERR("Buffer de points plein (%u)", (unsigned)CV_MAX_POINTS);
            s_measure_error = MEAS_ERR_BUFFER_FULL;
            s_measure_error_detail = (int32_t)CV_MAX_POINTS;
        }
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
 * MESURE CV — acquisition AD5940 et publication des points.
 * OE d'OSC1 est réaffirmé avant le dispatcher et reste actif après la mesure,
 * car USB/BLE peut conserver une demande HFCLK asynchrone.
 * ══════════════════════════════════════════════════════════════════════════ */
static int cv_measure(int32_t *peak_na_out)
{
    AD5940Err rc;

    if (!peak_na_out) {
        return measurement_fail(MEAS_ERR_CV_INIT, "cv/argument", AD5940ERR_NULLP);
    }
    *peak_na_out = 0;

    if (measurement_stop_is_requested()) {
        return MEAS_RUN_STOPPED;
    }

    /* 1. Alimenter le capteur (P1.00 HIGH + stabilisation des rails) */
    measurement_set_stage("sensor/power_on");
    int power_rc = ad5940_power_on();
    if (power_rc != 0) {
        measurement_fail(ad5940_spi_error_count() ? MEAS_ERR_SPI : MEAS_ERR_PLATFORM_INIT,
                         "sensor/power_on", power_rc);
        ad5940_power_off();
        return -1;
    }
    if (measurement_stop_is_requested()) {
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }

    /* 2. Initialisation AD5940 : reset, horloge, FIFO, séquenceur, LFOSC */
    measurement_set_stage("sensor/platform_init");
    float lfosc_hz = ad5940_platform_init();
    /* Conserver la vraie lecture de sonde même si platform_init échoue. Dans
     * les versions précédentes, STATUS affichait seulement la valeur zéro
     * initialisée par run_measurement(), ce qui masquait le diagnostic. */
    s_measure_adiid = ad5940_platform_adiid();

    if (lfosc_hz < 0.0f) {
        int32_t detail = ad5940_platform_last_error();
        measurement_fail(ad5940_spi_error_count() ? MEAS_ERR_SPI : MEAS_ERR_PLATFORM_INIT,
                         "sensor/platform_init", detail);
        ad5940_power_off();
        return -1;
    }
    if (measurement_stop_is_requested()) {
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }

    /* 2b. Test de vie SPI : on annule si l'AD5940 ne répond pas (ADIID),
     *     pour ne pas rester bloqué dans la boucle d'attente de la CV. */
    {
        uint32_t adiid = ad5940_read_adiid();
        s_measure_adiid = adiid;
        LOG_INF("AD5940 ADIID=0x%04X (attendu 0x4144)", (unsigned)adiid);
        if (adiid != 0x4144) {
            LOG_ERR("Mesure annulée : AD5940 muet (ADIID=0x%04X) — cablage SPI ?",
                    (unsigned)adiid);
            measurement_fail(MEAS_ERR_ADIID, "sensor/adiid", (int32_t)adiid);
            ad5940_power_off();
            return -1;
        }
        if (ad5940_spi_error_count() != 0u) {
            measurement_fail(MEAS_ERR_SPI, "sensor/spi_init",
                             ad5940_spi_last_error());
            ad5940_power_off();
            return -1;
        }
    }

    /* 2c. Autorange : si activé, sonde rapidement pour choisir le RTIA avant
     *     le vrai cycle. Sans effet si l'autorange est désactivé. */
    measurement_set_stage("cv/autorange");
    int autorange_rc = CV_Autorange(lfosc_hz, measurement_stop_is_requested);
    if (autorange_rc == MEAS_RUN_STOPPED) {
        *peak_na_out = CV_GetPeakCurrentNa();
        CV_Abort();
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }
    if (autorange_rc != 0) {
        measurement_fail(MEAS_ERR_CV_INIT, "cv/autorange", autorange_rc);
        CV_Abort();
        ad5940_power_off();
        return -1;
    }
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "cv/autorange_spi",
                         ad5940_spi_last_error());
        CV_Abort();
        ad5940_power_off();
        return -1;
    }
    if (measurement_stop_is_requested()) {
        CV_Abort();
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }

    /* 3. Configurer et lancer un cycle de voltampérométrie cyclique */
    measurement_set_stage("cv/init");
    rc = CV_Init(lfosc_hz);
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "cv/init_spi", ad5940_spi_last_error());
        CV_Abort();
        ad5940_power_off();
        return -1;
    }
    if (rc != AD5940ERR_OK) {
        LOG_ERR("CV_Init échoué (rc=%d)", (int)rc);
        measurement_fail(MEAS_ERR_CV_INIT, "cv/init", rc);
        CV_Abort();
        ad5940_power_off();
        return -1;
    }
    LOG_INF("CV RTIA nominal=%u ohm calibre=%u ohm autorange=%d",
            (unsigned)CV_GetActiveRtiaOhm(), (unsigned)CV_GetCalibratedRtiaOhm(),
            CV_GetAutorange());
    if (measurement_stop_is_requested()) {
        CV_Abort();
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }

    /* Pré-polariser au potentiel de départ avant le premier échantillon. Cela
     * remplace le saut brutal depuis la valeur d'initialisation du LPDAC par
     * une phase explicite, sans retirer ni masquer de point de la courbe. */
    measurement_set_stage("cv/settle");
    rc = CV_ApplyStartPotential();
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "cv/settle_spi", ad5940_spi_last_error());
        CV_Abort();
        ad5940_power_off();
        return -1;
    }
    if (rc != AD5940ERR_OK) {
        measurement_fail(MEAS_ERR_CV_PRECHARGE, "cv/settle_potential", rc);
        CV_Abort();
        ad5940_power_off();
        return -1;
    }
    if (measurement_wait_or_stop(CV_GetActiveSettleMs())) {
        *peak_na_out = CV_GetPeakCurrentNa();
        CV_Abort();
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }

    measurement_set_stage("cv/start");
    rc = CV_Start();
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "cv/start_spi", ad5940_spi_last_error());
        CV_Abort();
        ad5940_power_off();
        return -1;
    }
    if (rc != AD5940ERR_OK) {
        LOG_ERR("CV_Start échoué (rc=%d)", (int)rc);
        measurement_fail(MEAS_ERR_CV_START, "cv/start", rc);
        CV_Abort();
        ad5940_power_off();
        return -1;
    }

    /* 4. Attendre la fin du cycle (~ CV_DURATION_MS = 24 s).
     *    k_sleep() laisse le scheduler maintenir la connexion BLE.
     *    Timeout de sécurité : si la CV ne se termine jamais (pas de données
     *    FIFO), on abandonne au lieu de boucler indéfiniment. */
    measurement_set_stage("cv/acquire");
    int64_t cv_start_ms = k_uptime_get();
    s_measure_progress_at_ms = cv_start_ms;
    uint32_t expected = CV_GetActiveTotalPoints();
    uint32_t duration_ms = CV_GetActiveDurationMs();
    uint32_t point_period_ms = (expected > 0u) ? duration_ms / expected : duration_ms;
    uint32_t stall_timeout_ms = point_period_ms * 4u + 1000u;
    uint32_t timeout_margin_ms = duration_ms / 4u;
    if (stall_timeout_ms < 3000u) stall_timeout_ms = 3000u;
    if (timeout_margin_ms < 10000u) timeout_margin_ms = 10000u;
    uint64_t absolute_timeout_ms = (uint64_t)duration_ms + timeout_margin_ms;
    uint32_t last_n = s_cv_npts;

    while (!CV_IsFinished()) {
        if (measurement_stop_is_requested()) {
            *peak_na_out = CV_GetPeakCurrentNa();
            CV_Abort();
            ad5940_power_off();
            return MEAS_RUN_STOPPED;
        }
        k_sleep(K_MSEC(20));
        if (measurement_stop_is_requested()) {
            *peak_na_out = CV_GetPeakCurrentNa();
            CV_Abort();
            ad5940_power_off();
            return MEAS_RUN_STOPPED;
        }
        rc = CV_Service();
        int64_t now = k_uptime_get();
        if (s_cv_npts != last_n) {
            last_n = s_cv_npts;
            s_measure_progress_at_ms = now;
        }
        measurement_update_timing();

        if (ad5940_spi_error_count() != 0u) {
            measurement_fail(MEAS_ERR_SPI, "cv/spi", ad5940_spi_last_error());
            CV_Abort();
            ad5940_power_off();
            return -1;
        }
        if (rc != AD5940ERR_OK) {
            measurement_fail(MEAS_ERR_CV_SERVICE, "cv/service", rc);
            CV_Abort();
            ad5940_power_off();
            return -1;
        }
        if (s_measure_error != 0) {
            measurement_fail(s_measure_error, "cv/point_buffer",
                             s_measure_error_detail);
            CV_Abort();
            ad5940_power_off();
            return -1;
        }
        if ((uint64_t)(now - s_measure_progress_at_ms) > stall_timeout_ms) {
            measurement_fail(MEAS_ERR_CV_STALL, "cv/no_progress",
                             (int32_t)stall_timeout_ms);
            CV_Abort();
            ad5940_power_off();
            return -1;
        }
        if ((uint64_t)(now - cv_start_ms) > absolute_timeout_ms) {
            LOG_ERR("CV timeout global après %u/%u point(s)",
                    (unsigned)s_cv_npts, (unsigned)expected);
            measurement_fail(MEAS_ERR_CV_TIMEOUT, "cv/deadline",
                              (absolute_timeout_ms > INT32_MAX) ? INT32_MAX :
                              (int32_t)absolute_timeout_ms);
            CV_Abort();
            ad5940_power_off();
            return -1;
        }
    }

    measurement_update_timing();
    if (s_cv_npts != expected) {
        measurement_fail(MEAS_ERR_CV_INCOMPLETE, "cv/incomplete",
                         (int32_t)expected);
        CV_Abort();
        ad5940_power_off();
        return -1;
    }

    /* 5. Lire le courant de pic (en nA) */
    *peak_na_out = CV_GetPeakCurrentNa();
    LOG_INF("CV terminée — pic = %d nA", *peak_na_out);

    /* 6. Éteindre le capteur */
    ad5940_power_off();

    return 0;
}

/* ── Enveloppe commune CA/DPV : alim + init + contrôle vie SPI ─────────────
 * Renvoie le pic (nA) via la fonction de run passée, ou -1 si AD5940 muet. */
static int ca_measure(int32_t *peak_na_out)
{
    if (!peak_na_out) return -1;
    *peak_na_out = 0;
    measurement_set_stage("ca/power_on");
    int power_rc = ad5940_power_on();
    if (power_rc != 0) {
        measurement_fail(ad5940_spi_error_count() ? MEAS_ERR_SPI : MEAS_ERR_PLATFORM_INIT,
                         "ca/power_on", power_rc);
        ad5940_power_off();
        return -1;
    }
    measurement_set_stage("ca/platform_init");
    float lfosc_hz = ad5940_platform_init();
    s_measure_adiid = ad5940_platform_adiid();
    if (lfosc_hz < 0.0f) {
        measurement_fail(ad5940_spi_error_count() ? MEAS_ERR_SPI : MEAS_ERR_PLATFORM_INIT,
                         "ca/platform_init", ad5940_platform_last_error());
        ad5940_power_off();
        return -1;
    }
    uint32_t adiid = ad5940_read_adiid();
    s_measure_adiid = adiid;
    LOG_INF("AD5940 ADIID=0x%04X (attendu 0x4144)", (unsigned)adiid);
    if (adiid != 0x4144) {
        LOG_ERR("CA annulée : AD5940 muet (ADIID=0x%04X)", (unsigned)adiid);
        measurement_fail(MEAS_ERR_ADIID, "ca/adiid", (int32_t)adiid);
        ad5940_power_off();
        return -1;
    }
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "ca/spi_init", ad5940_spi_last_error());
        ad5940_power_off();
        return -1;
    }
    measurement_set_stage("ca/acquire");
    int rc = CA_Run(lfosc_hz, s_meas_rtia_ohm ? s_meas_rtia_ohm : 4000,
                    measurement_stop_is_requested);
    measurement_update_timing();
    if (s_measure_error != 0) {
        measurement_fail(s_measure_error, "ca/point_buffer", s_measure_error_detail);
        ad5940_power_off();
        return -1;
    }
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "ca/spi", ad5940_spi_last_error());
        ad5940_power_off();
        return -1;
    }
    if (rc == MEAS_RUN_STOPPED) {
        *peak_na_out = CA_GetPeakCurrentNa();
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }
    if (rc != 0) {
        measurement_fail(MEAS_ERR_CA_RUN, "ca/run", rc);
        ad5940_power_off();
        return -1;
    }
    *peak_na_out = CA_GetPeakCurrentNa();
    ad5940_power_off();
    return 0;
}

static int dpv_measure(int32_t *peak_na_out)
{
    if (!peak_na_out) return -1;
    *peak_na_out = 0;
    measurement_set_stage("dpv/power_on");
    int power_rc = ad5940_power_on();
    if (power_rc != 0) {
        measurement_fail(ad5940_spi_error_count() ? MEAS_ERR_SPI : MEAS_ERR_PLATFORM_INIT,
                         "dpv/power_on", power_rc);
        ad5940_power_off();
        return -1;
    }
    measurement_set_stage("dpv/platform_init");
    float lfosc_hz = ad5940_platform_init();
    s_measure_adiid = ad5940_platform_adiid();
    if (lfosc_hz < 0.0f) {
        measurement_fail(ad5940_spi_error_count() ? MEAS_ERR_SPI : MEAS_ERR_PLATFORM_INIT,
                         "dpv/platform_init", ad5940_platform_last_error());
        ad5940_power_off();
        return -1;
    }
    uint32_t adiid = ad5940_read_adiid();
    s_measure_adiid = adiid;
    LOG_INF("AD5940 ADIID=0x%04X (attendu 0x4144)", (unsigned)adiid);
    if (adiid != 0x4144) {
        LOG_ERR("DPV annulée : AD5940 muet (ADIID=0x%04X)", (unsigned)adiid);
        measurement_fail(MEAS_ERR_ADIID, "dpv/adiid", (int32_t)adiid);
        ad5940_power_off();
        return -1;
    }
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "dpv/spi_init", ad5940_spi_last_error());
        ad5940_power_off();
        return -1;
    }
    measurement_set_stage("dpv/acquire");
    int rc = DPV_Run(lfosc_hz, s_meas_rtia_ohm ? s_meas_rtia_ohm : 4000,
                     measurement_stop_is_requested);
    measurement_update_timing();
    if (s_measure_error != 0) {
        measurement_fail(s_measure_error, "dpv/point_buffer", s_measure_error_detail);
        ad5940_power_off();
        return -1;
    }
    if (ad5940_spi_error_count() != 0u) {
        measurement_fail(MEAS_ERR_SPI, "dpv/spi", ad5940_spi_last_error());
        ad5940_power_off();
        return -1;
    }
    if (rc == MEAS_RUN_STOPPED) {
        *peak_na_out = DPV_GetPeakCurrentNa();
        ad5940_power_off();
        return MEAS_RUN_STOPPED;
    }
    if (rc != 0) {
        measurement_fail(MEAS_ERR_DPV_RUN, "dpv/run", rc);
        ad5940_power_off();
        return -1;
    }
    *peak_na_out = DPV_GetPeakCurrentNa();
    ad5940_power_off();
    return 0;
}

/* Bilan du dernier cycle, exposé via SMP (STATUS) et loggé sur RTT. */
static volatile int32_t s_cv_peak_na;
static volatile int32_t s_cv_r_ohm;

/* ── Lance la mesure de la méthode courante : remplit le buffer + stream BLE ── */
static void run_measurement(void)
{
    struct bt_conn *conn = ble_manager_get_conn();  /* NULL = pas d'app BLE */
    g_stream_conn = conn;
    int64_t run_started_ms = k_uptime_get();
    AppRAMPResetDiag();
    k_spinlock_key_t data_key = k_spin_lock(&s_data_lock);
    s_cv_npts = 0;
    /* SMP et BLE réservent tous deux l'identifiant lors de la transition vers
     * pending : il ne change donc jamais entre l'ACK/la demande et la fin. */
    s_measure_error = 0;
    s_measure_error_detail = 0;
    s_measure_error_stage = "none";
    s_measure_adiid = 0u;
    s_measure_elapsed_ms = 0u;
    s_measure_idle_ms = 0u;
    s_measure_start_at_ms = run_started_ms;
    s_measure_progress_at_ms = run_started_ms;
    s_cv_peak_na = 0;
    s_cv_r_ohm = 0;
    s_measure_cancelled = false;
    s_measure_stage = "starting";
    s_measurement_pending = false;
    s_cv_running = true;
    k_spin_unlock(&s_data_lock, data_key);

    LOG_INF("Declenchement mesure (method=%d)%s", (int)s_method,
            conn ? " (stream BLE actif)" : "");

    power_mgr_hfxo_enable();
    int32_t peak_na = 0;
    int measure_rc;
    if (measurement_stop_is_requested()) {
        measure_rc = MEAS_RUN_STOPPED;
    } else {
        switch (s_method) {
        case METHOD_CA:  measure_rc = ca_measure(&peak_na);  break;
        case METHOD_DPV: measure_rc = dpv_measure(&peak_na); break;
        case METHOD_CV:
        default:         measure_rc = cv_measure(&peak_na); break;
        }
    }

    s_cv_peak_na = peak_na;
    bool cancelled_result = (measure_rc == MEAS_RUN_STOPPED);
    if (cancelled_result) {
        s_measure_error = 0;
        s_measure_error_detail = 0;
        s_measure_error_stage = "none";
        measurement_update_timing();
    } else if (measure_rc == 0) {
        measurement_set_stage("complete");
        measurement_update_timing();
    } else if (s_measure_error == 0) {
        measurement_fail((s_method == METHOD_CA) ? MEAS_ERR_CA_RUN :
                         (s_method == METHOD_DPV) ? MEAS_ERR_DPV_RUN : MEAS_ERR_CV_SERVICE,
                         "measurement/unknown", measure_rc);
    }
    /* R estimée : pertinente uniquement pour la CV (test résistance). */
    s_cv_r_ohm   = (s_method == METHOD_CV) ? (int32_t)CV_GetResistanceOhm() : 0;

    /* Fermer atomiquement le run avant les logs/notifications. Une commande
     * STOP arrivée après la dernière donnée répondra ainsi stop_requested=false
     * au lieu de transformer rétrospectivement un run complet en annulation. */
    data_key = k_spin_lock(&s_data_lock);
    if (cancelled_result) {
        s_measure_cancelled = true;
        s_measure_stage = "cancelled";
    }
    s_cv_running = false;
    if (s_smp_run_owner) {
        s_smp_data_held = true;
        s_smp_data_expires_at_ms = k_uptime_get() + SMP_DATA_LEASE_MS;
    }
    k_spin_unlock(&s_data_lock, data_key);

    LOG_INF("Mesure terminee — run_id=%u points=%u pic=%d nA R~=%d ohm "
            "err=%d stage=%s cancelled=%d stop_requested=%d",
            (unsigned)s_measure_run_id, (unsigned)s_cv_npts, peak_na, s_cv_r_ohm,
            (int)s_measure_error, s_measure_stage, cancelled_result,
            measurement_stop_is_requested());

    if (conn) {
        uint8_t end = 0xFF;                          /* fin de burst → save app */
        (void)gatt_service_notify(conn, &end, sizeof(end));
        bt_conn_unref(conn);
    }
    g_stream_conn = NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Groupe SMP custom « CV » (ID 64) — pilotage depuis le PC via mcumgr, sur le
 * MÊME port USB que le DFU (multiplexé par mcumgr, aucun conflit). Le PC parle
 * SMP (lib smpclient côté Python). Commandes :
 *   START  (cmd 0, write) : sélectionne la méthode + params entiers optionnels, lance.
 *      commun : method (0=CV,1=CA,2=DPV), vzero_mv, rtia (CV: 0=autorange)
 *      CV  : start_mv peak_mv steps cycles dur_ms delay_us settle_ms
 *            (`steps` et `dur_ms` sont définis par cycle)
 *      CA  : ca_pot_mv ca_dur_ms ca_interval_ms
 *      DPV : start_mv end_mv dpv_estep_mv dpv_epulse_mv dpv_tpulse_ms dpv_tperiod_ms
 *   STATUS (cmd 1, read)  : { run_id, run, n, peak_na, r_ohm, rtia, method, diagnostics }
 *   DATA   (cmd 2, read)  : entrée { run_id, off } → points du même run par lots
 *   RELEASE(cmd 3, write) : { run_id, n } acquitte et libère le buffer SMP
 *   STOP   (cmd 4, write) : { run_id } demande un arrêt coopératif sans E/S SPI
 *      v[] = tension mV (CV/DPV) ou temps ms (CA) ; i[] = courant nA (ΔI en DPV)
 * ══════════════════════════════════════════════════════════════════════════ */
#define CV_MGMT_GROUP_ID    MGMT_GROUP_ID_PERUSER   /* 64 */
#define CV_MGMT_CMD_START   0
#define CV_MGMT_CMD_STATUS  1
#define CV_MGMT_CMD_DATA    2
#define CV_MGMT_CMD_RELEASE 3
#define CV_MGMT_CMD_STOP    4
#define CV_DATA_CHUNK       64                      /* points max par réponse DATA */

static void cv_mgmt_cancel_start_reservation(const char *previous_stage)
{
    k_spinlock_key_t key = k_spin_lock(&s_data_lock);
    if (s_measurement_pending && s_smp_start_configuring) {
        s_measurement_pending = false;
        s_smp_start_configuring = false;
        s_measure_stage = previous_stage ? previous_stage : "idle";
    }
    k_spin_unlock(&s_data_lock, key);
}

static int cv_mgmt_start(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;
    zcbor_state_t *zse = ctxt->writer->zs;

    /* Paramètres optionnels (entiers) ; INT32_MIN = « non fourni ». */
    int32_t method = INT32_MIN, vzero_mv = INT32_MIN, rtia = INT32_MIN;
    int32_t start_mv = INT32_MIN, peak_mv = INT32_MIN, end_mv = INT32_MIN;
    int32_t steps = INT32_MIN, cycles = INT32_MIN, dur_ms = INT32_MIN, delay_us = INT32_MIN;
    int32_t settle_ms = INT32_MIN;
    int32_t ca_pot_mv = INT32_MIN, ca_dur_ms = INT32_MIN, ca_interval_ms = INT32_MIN;
    int32_t dpv_estep_mv = INT32_MIN, dpv_epulse_mv = INT32_MIN;
    int32_t dpv_tpulse_ms = INT32_MIN, dpv_tperiod_ms = INT32_MIN;
    uint32_t accepted_cycles = 0;
    uint32_t accepted_steps = 0;
    uint32_t accepted_total_points = 0;
    uint32_t accepted_settle_ms = 0;
    uint32_t accepted_run_id = 0;
    cv_params_t previous_cv_params = {0};
    ca_params_t previous_ca_params = {0};
    dpv_params_t previous_dpv_params = {0};
    uint32_t previous_cv_rtia = 0;
    uint32_t previous_meas_rtia = 0;
    bool previous_data_held = false;
    bool previous_run_owner = false;
    int64_t previous_data_expiry = 0;
    uint32_t previous_run_id = 0;
    uint32_t previous_npts = 0;
    int32_t previous_error = 0;
    int32_t previous_error_detail = 0;
    const char *previous_stage = "idle";
    const char *previous_error_stage = "none";
    uint32_t previous_elapsed_ms = 0;
    uint32_t previous_idle_ms = 0;
    uint32_t previous_settle_ms = 0;
    bool previous_stop_requested = false;
    bool previous_cancelled = false;
    analysis_method_t previous_method = METHOD_CV;
    size_t decoded;
    struct zcbor_map_decode_key_val dec[] = {
        ZCBOR_MAP_DECODE_KEY_DECODER("method",   zcbor_int32_decode, &method),
        ZCBOR_MAP_DECODE_KEY_DECODER("vzero_mv", zcbor_int32_decode, &vzero_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("rtia",     zcbor_int32_decode, &rtia),
        ZCBOR_MAP_DECODE_KEY_DECODER("start_mv", zcbor_int32_decode, &start_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("peak_mv",  zcbor_int32_decode, &peak_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("end_mv",   zcbor_int32_decode, &end_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("steps",    zcbor_int32_decode, &steps),
        ZCBOR_MAP_DECODE_KEY_DECODER("cycles",   zcbor_int32_decode, &cycles),
        ZCBOR_MAP_DECODE_KEY_DECODER("dur_ms",   zcbor_int32_decode, &dur_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("delay_us", zcbor_int32_decode, &delay_us),
        ZCBOR_MAP_DECODE_KEY_DECODER("settle_ms", zcbor_int32_decode, &settle_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("ca_pot_mv",      zcbor_int32_decode, &ca_pot_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("ca_dur_ms",      zcbor_int32_decode, &ca_dur_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("ca_interval_ms", zcbor_int32_decode, &ca_interval_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_estep_mv",   zcbor_int32_decode, &dpv_estep_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_epulse_mv",  zcbor_int32_decode, &dpv_epulse_mv),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_tpulse_ms",  zcbor_int32_decode, &dpv_tpulse_ms),
        ZCBOR_MAP_DECODE_KEY_DECODER("dpv_tperiod_ms", zcbor_int32_decode, &dpv_tperiod_ms),
    };
    if (zcbor_map_decode_bulk(zsd, dec, ARRAY_SIZE(dec), &decoded) != 0) {
        return MGMT_ERR_EINVAL;
    }

    analysis_method_t requested_method = (method != INT32_MIN) ?
                                         (analysis_method_t)method : METHOD_CV;
    if ((requested_method != METHOD_CV && requested_method != METHOD_CA &&
         requested_method != METHOD_DPV) ||
        (rtia != INT32_MIN && rtia < 0) ||
        (cycles != INT32_MIN && cycles <= 0) ||
        (settle_ms != INT32_MIN && (settle_ms < 0 || settle_ms > 60000))) {
        return MGMT_ERR_EINVAL;
    }

    /* Réserver avant toute mutation des paramètres globaux. Sans cette
     * transition atomique, une demande BLE pourrait démarrer entre un
     * SetParams() et la réservation SMP, avec une configuration mélangée. Un
     * START invalide ne doit cependant pas détruire le buffer SMP précédent :
     * le commit run_id/n/bail n'a lieu qu'après validation des SetParams(). */
    k_spinlock_key_t data_key = k_spin_lock(&s_data_lock);
    if (s_cv_running || s_measurement_pending) {
        k_spin_unlock(&s_data_lock, data_key);
        return MGMT_ERR_EBUSY;
    }
    previous_data_held = s_smp_data_held;
    previous_run_owner = s_smp_run_owner;
    previous_data_expiry = s_smp_data_expires_at_ms;
    previous_run_id = s_measure_run_id;
    previous_npts = s_cv_npts;
    previous_error = s_measure_error;
    previous_error_detail = s_measure_error_detail;
    previous_stage = s_measure_stage;
    previous_error_stage = s_measure_error_stage;
    previous_elapsed_ms = s_measure_elapsed_ms;
    previous_idle_ms = s_measure_idle_ms;
    previous_settle_ms = s_measure_settle_ms;
    previous_stop_requested = s_stop_requested;
    previous_cancelled = s_measure_cancelled;
    previous_method = s_method;
    previous_meas_rtia = s_meas_rtia_ohm;
    s_measure_stage = "configuring";
    s_measurement_pending = true;
    s_smp_start_configuring = true;
    k_spin_unlock(&s_data_lock, data_key);

    if (requested_method == METHOD_CV) {
        CV_GetParams(&previous_cv_params);
        previous_cv_rtia = CV_GetRtiaOhm();
        cv_params_t p = previous_cv_params;
        if (start_mv != INT32_MIN || peak_mv != INT32_MIN || vzero_mv != INT32_MIN ||
            steps != INT32_MIN || cycles != INT32_MIN || dur_ms != INT32_MIN ||
            delay_us != INT32_MIN || settle_ms != INT32_MIN) {
            if (start_mv != INT32_MIN) p.start_mv = (float)start_mv;
            if (peak_mv  != INT32_MIN) p.peak_mv  = (float)peak_mv;
            if (vzero_mv != INT32_MIN) p.vzero_mv = (float)vzero_mv;
            if (steps    != INT32_MIN) p.step_number = (uint32_t)steps;
            if (cycles   != INT32_MIN) p.cycle_count = (uint32_t)cycles;
            if (dur_ms   != INT32_MIN) p.duration_ms = (uint32_t)dur_ms;
            if (delay_us != INT32_MIN) p.sample_delay_ms = (float)delay_us / 1000.0f;
            if (settle_ms != INT32_MIN) p.settle_ms = (uint32_t)settle_ms;
            if (CV_SetParams(&p) != 0) {
                cv_mgmt_cancel_start_reservation(previous_stage);
                return MGMT_ERR_EINVAL;
            }
        }
        if (rtia != INT32_MIN) {
            CV_SetRtia((uint32_t)rtia);          /* 0 = autorange */
        }
        /* Tell the GUI that this firmware really accepted the new key. An old
         * firmware skips unknown CBOR keys and would otherwise run one cycle
         * without reporting the mismatch. */
        accepted_cycles = p.cycle_count;
        accepted_steps = p.step_number;
        accepted_total_points = p.step_number * p.cycle_count;
        accepted_settle_ms = p.settle_ms;
    } else if (requested_method == METHOD_CA) {
        CA_GetParams(&previous_ca_params);
        ca_params_t p = previous_ca_params;
        if (ca_pot_mv      != INT32_MIN) p.pot_mv      = (float)ca_pot_mv;
        if (vzero_mv       != INT32_MIN) p.vzero_mv    = (float)vzero_mv;
        if (ca_dur_ms      != INT32_MIN) p.duration_ms = (uint32_t)ca_dur_ms;
        if (ca_interval_ms != INT32_MIN) p.interval_ms = (uint32_t)ca_interval_ms;
        if (CA_SetParams(&p) != 0) {
            cv_mgmt_cancel_start_reservation(previous_stage);
            return MGMT_ERR_EINVAL;
        }
        s_meas_rtia_ohm = (rtia != INT32_MIN && rtia > 0) ? (uint32_t)rtia : 4000;
    } else if (requested_method == METHOD_DPV) {
        DPV_GetParams(&previous_dpv_params);
        dpv_params_t p = previous_dpv_params;
        if (start_mv       != INT32_MIN) p.start_mv   = (float)start_mv;
        if (end_mv         != INT32_MIN) p.end_mv     = (float)end_mv;
        if (dpv_estep_mv   != INT32_MIN) p.estep_mv   = (float)dpv_estep_mv;
        if (dpv_epulse_mv  != INT32_MIN) p.epulse_mv  = (float)dpv_epulse_mv;
        if (dpv_tpulse_ms  != INT32_MIN) p.tpulse_ms  = (uint32_t)dpv_tpulse_ms;
        if (dpv_tperiod_ms != INT32_MIN) p.tperiod_ms = (uint32_t)dpv_tperiod_ms;
        if (vzero_mv       != INT32_MIN) p.vzero_mv   = (float)vzero_mv;
        if (DPV_SetParams(&p) != 0) {
            cv_mgmt_cancel_start_reservation(previous_stage);
            return MGMT_ERR_EINVAL;
        }
        s_meas_rtia_ohm = (rtia != INT32_MIN && rtia > 0) ? (uint32_t)rtia : 4000;
    } else {
        cv_mgmt_cancel_start_reservation(previous_stage);
        return MGMT_ERR_EINVAL;
    }

    data_key = k_spin_lock(&s_data_lock);
    s_method = requested_method;
    s_smp_data_held = true;
    s_smp_data_expires_at_ms = 0;
    s_smp_run_owner = true;
    s_measure_run_id++;
    accepted_run_id = s_measure_run_id;
    s_cv_npts = 0;
    s_measure_error = 0;
    s_measure_error_detail = 0;
    s_measure_error_stage = "none";
    s_measure_elapsed_ms = 0u;
    s_measure_idle_ms = 0u;
    s_measure_settle_ms = (requested_method == METHOD_CV) ? accepted_settle_ms : 0u;
    s_stop_requested = false;
    s_measure_cancelled = false;
    s_measure_stage = "pending";
    s_smp_start_configuring = false;
    k_spin_unlock(&s_data_lock, data_key);
    if (k_work_schedule(&measurement_start_work, K_MSEC(25)) < 0) {
        /* Cas exceptionnel : restaurer aussi bien la configuration que le
         * buffer/status précédent avant de rendre la réservation. */
        if (requested_method == METHOD_CV) {
            (void)CV_SetParams(&previous_cv_params);
            (void)CV_SetRtia(previous_cv_rtia);
        } else if (requested_method == METHOD_CA) {
            (void)CA_SetParams(&previous_ca_params);
            s_meas_rtia_ohm = previous_meas_rtia;
        } else {
            (void)DPV_SetParams(&previous_dpv_params);
            s_meas_rtia_ohm = previous_meas_rtia;
        }
        data_key = k_spin_lock(&s_data_lock);
        s_method = previous_method;
        s_smp_data_held = previous_data_held;
        s_smp_data_expires_at_ms = previous_data_expiry;
        s_smp_run_owner = previous_run_owner;
        s_measure_run_id = previous_run_id;
        s_cv_npts = previous_npts;
        s_measure_error = previous_error;
        s_measure_error_detail = previous_error_detail;
        s_measure_error_stage = previous_error_stage;
        s_measure_elapsed_ms = previous_elapsed_ms;
        s_measure_idle_ms = previous_idle_ms;
        s_measure_settle_ms = previous_settle_ms;
        s_stop_requested = previous_stop_requested;
        s_measure_cancelled = previous_cancelled;
        s_measure_stage = previous_stage;
        s_measurement_pending = false;
        s_smp_start_configuring = false;
        k_spin_unlock(&s_data_lock, data_key);
        return MGMT_ERR_EUNKNOWN;
    }

    bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0) &&
              zcbor_tstr_put_lit(zse, "run_id") &&
              zcbor_uint32_put(zse, accepted_run_id) &&
              zcbor_tstr_put_lit(zse, "fw_build") &&
              zcbor_tstr_put_lit(zse, FIRMWARE_BUILD_ID) &&
              zcbor_tstr_put_lit(zse, "diag_ver") && zcbor_uint32_put(zse, 6u) &&
              zcbor_tstr_put_lit(zse, "release_required") &&
              zcbor_bool_put(zse, true) &&
              zcbor_tstr_put_lit(zse, "settle_ms") &&
              zcbor_uint32_put(zse, (requested_method == METHOD_CV) ?
                                      accepted_settle_ms : 0u);
    if (ok && requested_method == METHOD_CV) {
        ok = zcbor_tstr_put_lit(zse, "cycles") && zcbor_uint32_put(zse, accepted_cycles) &&
             zcbor_tstr_put_lit(zse, "steps") && zcbor_uint32_put(zse, accepted_steps) &&
             zcbor_tstr_put_lit(zse, "total_points") &&
             zcbor_uint32_put(zse, accepted_total_points);
    }
    return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static int cv_mgmt_status(struct smp_streamer *ctxt)
{
    zcbor_state_t *zse = ctxt->writer->zs;
    lp_potential_state_t pstate;
    AppRAMPDiag_Type ramp_diag;

    if (s_cv_running) {
        measurement_update_timing();
    }
    k_spinlock_key_t data_key = k_spin_lock(&s_data_lock);
    bool run_snapshot = s_cv_running || s_measurement_pending;
    uint32_t n_snapshot = s_cv_npts;
    uint32_t run_id_snapshot = s_measure_run_id;
    int32_t peak_snapshot = s_cv_peak_na;
    int32_t resistance_snapshot = s_cv_r_ohm;
    int32_t error_snapshot = s_measure_error;
    int32_t error_detail_snapshot = s_measure_error_detail;
    uint32_t elapsed_snapshot = s_measure_elapsed_ms;
    uint32_t idle_snapshot = s_measure_idle_ms;
    uint32_t adiid_snapshot = s_measure_adiid;
    bool data_held_snapshot = s_smp_data_held;
    bool stop_requested_snapshot = s_stop_requested;
    bool cancelled_snapshot = s_measure_cancelled;
    uint32_t settle_snapshot = s_measure_settle_ms;
    analysis_method_t method_snapshot = s_method;
    const char *stage = s_measure_stage ? s_measure_stage : "unknown";
    const char *error_stage = s_measure_error_stage ? s_measure_error_stage : "none";
    k_spin_unlock(&s_data_lock, data_key);
    AppRAMPGetDiag(&ramp_diag);
    lp_get_potential_state(&pstate);
    uint32_t rtia = (method_snapshot == METHOD_CV) ? CV_GetActiveRtiaOhm()
                                                   : lp_rtia_nominal_ohm();
    uint32_t rtia_cal = (method_snapshot == METHOD_CV) ? CV_GetCalibratedRtiaOhm()
                                                       : lp_rtia_calibrated_ohm();
    uint32_t cv_steps = (method_snapshot == METHOD_CV) ? CV_GetActiveStepNumber() : 0u;
    uint32_t cv_cycles = (method_snapshot == METHOD_CV) ? CV_GetActiveCycleCount() : 0u;
    uint32_t cv_total_points = (method_snapshot == METHOD_CV) ? CV_GetActiveTotalPoints() : 0u;
    bool ok = zcbor_tstr_put_lit(zse, "run")     &&
              zcbor_bool_put(zse, run_snapshot) &&
              zcbor_tstr_put_lit(zse, "fw_build") &&
              zcbor_tstr_put_lit(zse, FIRMWARE_BUILD_ID) &&
              zcbor_tstr_put_lit(zse, "n")       && zcbor_uint32_put(zse, n_snapshot)   &&
              zcbor_tstr_put_lit(zse, "peak_na") && zcbor_int32_put(zse, peak_snapshot) &&
              zcbor_tstr_put_lit(zse, "r_ohm")   && zcbor_int32_put(zse, resistance_snapshot) &&
              zcbor_tstr_put_lit(zse, "rtia")    && zcbor_uint32_put(zse, rtia) &&
               zcbor_tstr_put_lit(zse, "rtia_cal") && zcbor_uint32_put(zse, rtia_cal) &&
               zcbor_tstr_put_lit(zse, "method")  && zcbor_uint32_put(zse, (uint32_t)method_snapshot) &&
               zcbor_tstr_put_lit(zse, "steps")   && zcbor_uint32_put(zse, cv_steps) &&
               zcbor_tstr_put_lit(zse, "cycles")  && zcbor_uint32_put(zse, cv_cycles) &&
               zcbor_tstr_put_lit(zse, "total_points") &&
               zcbor_uint32_put(zse, cv_total_points) &&
               zcbor_tstr_put_lit(zse, "err")     && zcbor_int32_put(zse, error_snapshot) &&
              zcbor_tstr_put_lit(zse, "pot_mv")  && zcbor_int32_put(zse, pstate.e_mv) &&
              zcbor_tstr_put_lit(zse, "vzero_mv") && zcbor_int32_put(zse, pstate.vzero_mv) &&
              zcbor_tstr_put_lit(zse, "dac_vbias") && zcbor_uint32_put(zse, pstate.vbias_code) &&
              zcbor_tstr_put_lit(zse, "dac_vzero") && zcbor_uint32_put(zse, pstate.vzero_code) &&
              zcbor_tstr_put_lit(zse, "dac_reg") && zcbor_uint32_put(zse, pstate.dac_register) &&
              zcbor_tstr_put_lit(zse, "dac_ok") &&
              zcbor_bool_put(zse, pstate.valid && pstate.readback_ok) &&
               /* Diagnostic v6 : toutes les valeurs sont des instantanés RAM,
                * STATUS ne touche jamais le bus SPI pendant une acquisition. */
              zcbor_tstr_put_lit(zse, "diag_ver") && zcbor_uint32_put(zse, 6u) &&
               zcbor_tstr_put_lit(zse, "run_id") && zcbor_uint32_put(zse, run_id_snapshot) &&
               zcbor_tstr_put_lit(zse, "data_held") &&
               zcbor_bool_put(zse, data_held_snapshot) &&
               zcbor_tstr_put_lit(zse, "stop_requested") &&
               zcbor_bool_put(zse, stop_requested_snapshot) &&
               zcbor_tstr_put_lit(zse, "cancelled") &&
               zcbor_bool_put(zse, cancelled_snapshot) &&
               zcbor_tstr_put_lit(zse, "settle_ms") &&
               zcbor_uint32_put(zse, settle_snapshot) &&
               zcbor_tstr_put_lit(zse, "stage") &&
               zcbor_tstr_put_term(zse, stage, 32u) &&
               zcbor_tstr_put_lit(zse, "error_stage") &&
               zcbor_tstr_put_term(zse, error_stage, 32u) &&
               zcbor_tstr_put_lit(zse, "err_detail") &&
               zcbor_int32_put(zse, error_detail_snapshot) &&
               zcbor_tstr_put_lit(zse, "elapsed_ms") &&
               zcbor_uint32_put(zse, elapsed_snapshot) &&
               zcbor_tstr_put_lit(zse, "idle_ms") && zcbor_uint32_put(zse, idle_snapshot) &&
               zcbor_tstr_put_lit(zse, "adiid") && zcbor_uint32_put(zse, adiid_snapshot) &&
               zcbor_tstr_put_lit(zse, "chipid") &&
               zcbor_uint32_put(zse, ad5940_platform_chipid()) &&
               zcbor_tstr_put_lit(zse, "spi_mode") &&
               zcbor_int32_put(zse, ad5940_spi_selected_mode()) &&
               zcbor_tstr_put_lit(zse, "mode_ids01") &&
               zcbor_uint32_put(zse, ad5940_spi_mode_ids01()) &&
               zcbor_tstr_put_lit(zse, "mode_ids23") &&
               zcbor_uint32_put(zse, ad5940_spi_mode_ids23()) &&
               zcbor_tstr_put_lit(zse, "mode_tried") &&
               zcbor_uint32_put(zse, ad5940_spi_mode_tried_mask()) &&
               zcbor_tstr_put_lit(zse, "gpio_err") &&
               zcbor_int32_put(zse, ad5940_gpio_last_error()) &&
               zcbor_tstr_put_lit(zse, "gpio_state") &&
               zcbor_uint32_put(zse, ad5940_platform_gpio_state()) &&
               zcbor_tstr_put_lit(zse, "hfxo_on") &&
               zcbor_bool_put(zse, power_mgr_hfxo_enabled()) &&
               zcbor_tstr_put_lit(zse, "platform_err") &&
               zcbor_int32_put(zse, ad5940_platform_last_error()) &&
              zcbor_tstr_put_lit(zse, "lfosc_meas_hz") &&
              zcbor_uint32_put(zse, ad5940_lfosc_measured_hz()) &&
              zcbor_tstr_put_lit(zse, "lfosc_hz") &&
              zcbor_uint32_put(zse, ad5940_lfosc_used_hz()) &&
               zcbor_tstr_put_lit(zse, "lfosc_err") &&
               zcbor_int32_put(zse, ad5940_lfosc_cal_error()) &&
               zcbor_tstr_put_lit(zse, "lfosc_fallback") &&
               zcbor_bool_put(zse, ad5940_lfosc_fallback_used() != 0u) &&
              zcbor_tstr_put_lit(zse, "spi_errors") &&
              zcbor_uint32_put(zse, ad5940_spi_error_count()) &&
              zcbor_tstr_put_lit(zse, "spi_last") &&
              zcbor_int32_put(zse, ad5940_spi_last_error()) &&
              zcbor_tstr_put_lit(zse, "ramp_error") &&
              zcbor_int32_put(zse, ramp_diag.last_error) &&
              zcbor_tstr_put_lit(zse, "intc0") && zcbor_uint32_put(zse, ramp_diag.last_intc0) &&
              zcbor_tstr_put_lit(zse, "intc1") && zcbor_uint32_put(zse, ramp_diag.last_intc1) &&
              zcbor_tstr_put_lit(zse, "int_fatal") &&
              zcbor_uint32_put(zse, ramp_diag.fatal_flags) &&
              zcbor_tstr_put_lit(zse, "intc0_rejected") &&
              zcbor_uint32_put(zse, ramp_diag.last_rejected_intc0) &&
              zcbor_tstr_put_lit(zse, "intc1_rejected") &&
              zcbor_uint32_put(zse, ramp_diag.last_rejected_intc1) &&
              zcbor_tstr_put_lit(zse, "intc_retries") &&
              zcbor_uint32_put(zse, ramp_diag.intc_retry_count) &&
              zcbor_tstr_put_lit(zse, "intc_stale") &&
              zcbor_uint32_put(zse, ramp_diag.stale_intc_count) &&
              zcbor_tstr_put_lit(zse, "fatal_rechecks") &&
              zcbor_uint32_put(zse, ramp_diag.fatal_recheck_count) &&
              zcbor_tstr_put_lit(zse, "fatal_unconfirmed") &&
              zcbor_uint32_put(zse, ramp_diag.unconfirmed_fatal_count) &&
              zcbor_tstr_put_lit(zse, "seq_timeout") &&
              zcbor_uint32_put(zse, ramp_diag.seq_timeout) &&
              zcbor_tstr_put_lit(zse, "seq_count") &&
              zcbor_uint32_put(zse, ramp_diag.seq_count) &&
              zcbor_tstr_put_lit(zse, "seq_con") &&
              zcbor_uint32_put(zse, ramp_diag.seq_con) &&
              zcbor_tstr_put_lit(zse, "seq0_info") &&
              zcbor_uint32_put(zse, ramp_diag.seq0_info) &&
              zcbor_tstr_put_lit(zse, "seq1_info") &&
              zcbor_uint32_put(zse, ramp_diag.seq1_info) &&
               zcbor_tstr_put_lit(zse, "seq2_info") &&
               zcbor_uint32_put(zse, ramp_diag.seq2_info) &&
               zcbor_tstr_put_lit(zse, "seq3_info") &&
               zcbor_uint32_put(zse, ramp_diag.seq3_info) &&
               zcbor_tstr_put_lit(zse, "init_endseq_missing") &&
               zcbor_uint32_put(zse, ramp_diag.init_endseq_missing) &&
               zcbor_tstr_put_lit(zse, "init_wait_polls") &&
               zcbor_uint32_put(zse, ramp_diag.init_wait_polls) &&
               zcbor_tstr_put_lit(zse, "init_direct_fallback") &&
               zcbor_uint32_put(zse, ramp_diag.init_direct_fallback) &&
               zcbor_tstr_put_lit(zse, "custom_irq") &&
              zcbor_uint32_put(zse, ramp_diag.custom_irq_count) &&
              zcbor_tstr_put_lit(zse, "refills") &&
              zcbor_uint32_put(zse, ramp_diag.refill_count) &&
              zcbor_tstr_put_lit(zse, "blocks") &&
              zcbor_uint32_put(zse, ramp_diag.blocks_generated) &&
              zcbor_tstr_put_lit(zse, "block_steps") &&
              zcbor_uint32_put(zse, ramp_diag.steps_per_block) &&
              zcbor_tstr_put_lit(zse, "steps_remaining") &&
              zcbor_uint32_put(zse, ramp_diag.steps_remaining) &&
              zcbor_tstr_put_lit(zse, "fifo_count") &&
              zcbor_uint32_put(zse, ramp_diag.last_fifo_count) &&
              zcbor_tstr_put_lit(zse, "fifo_max") &&
              zcbor_uint32_put(zse, ramp_diag.max_fifo_count);
    return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static int cv_mgmt_data(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;
    zcbor_state_t *zse = ctxt->writer->zs;

    uint32_t off = 0;
    uint32_t requested_run_id = 0;
    size_t decoded;
    struct zcbor_map_decode_key_val dec[] = {
        ZCBOR_MAP_DECODE_KEY_DECODER("off", zcbor_uint32_decode, &off),
        ZCBOR_MAP_DECODE_KEY_DECODER("run_id", zcbor_uint32_decode, &requested_run_id),
    };
    if (zcbor_map_decode_bulk(zsd, dec, ARRAY_SIZE(dec), &decoded) != 0 ||
        !zcbor_map_decode_bulk_key_found(dec, ARRAY_SIZE(dec), "off") ||
        !zcbor_map_decode_bulk_key_found(dec, ARRAY_SIZE(dec), "run_id")) {
        return MGMT_ERR_EINVAL;
    }

    int32_t x_snapshot[CV_DATA_CHUNK];
    int32_t i_snapshot[CV_DATA_CHUNK];
    k_spinlock_key_t data_key = k_spin_lock(&s_data_lock);
    uint32_t run_id = s_measure_run_id;
    uint32_t n = s_cv_npts;
    if (requested_run_id != run_id || off > n) {
        k_spin_unlock(&s_data_lock, data_key);
        return MGMT_ERR_EINVAL;
    }
    /* Renouveler sous le premier verrou : à l'instant exact de l'expiration,
     * DATA ou BLE gagne ainsi atomiquement, sans fenêtre pendant l'encodage. */
    if (s_smp_data_held && s_smp_run_owner && !s_cv_running &&
        !s_measurement_pending) {
        s_smp_data_expires_at_ms = k_uptime_get() + SMP_DATA_LEASE_MS;
    }
    uint32_t end = off + CV_DATA_CHUNK;
    if (end > n) end = n;
    uint32_t cnt = end - off;
    for (uint32_t i = 0; i < cnt; i++) {
        x_snapshot[i] = s_cv_x[off + i];
        i_snapshot[i] = s_cv_cur[off + i];
    }
    k_spin_unlock(&s_data_lock, data_key);

    bool ok = zcbor_tstr_put_lit(zse, "run_id") && zcbor_uint32_put(zse, run_id) &&
              zcbor_tstr_put_lit(zse, "off") && zcbor_uint32_put(zse, off) &&
              zcbor_tstr_put_lit(zse, "n")   && zcbor_uint32_put(zse, n)   &&
              zcbor_tstr_put_lit(zse, "v")   && zcbor_list_start_encode(zse, cnt);
    for (uint32_t i = 0; i < cnt && ok; i++) {
        ok = zcbor_int32_put(zse, x_snapshot[i]);
    }
    ok = ok && zcbor_list_end_encode(zse, cnt) &&
         zcbor_tstr_put_lit(zse, "i") && zcbor_list_start_encode(zse, cnt);
    for (uint32_t i = 0; i < cnt && ok; i++) {
        ok = zcbor_int32_put(zse, i_snapshot[i]);
    }
    ok = ok && zcbor_list_end_encode(zse, cnt);
    return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

/* Libération explicite : indispensable si une erreur produit zéro point ou si
 * le téléchargement est abandonné avant le dernier lot. L'opération est
 * idempotente pour le run courant. */
static int cv_mgmt_release(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;
    zcbor_state_t *zse = ctxt->writer->zs;
    uint32_t requested_run_id = 0;
    uint32_t acknowledged_n = 0;
    size_t decoded;
    struct zcbor_map_decode_key_val dec[] = {
        ZCBOR_MAP_DECODE_KEY_DECODER("run_id", zcbor_uint32_decode,
                                     &requested_run_id),
        ZCBOR_MAP_DECODE_KEY_DECODER("n", zcbor_uint32_decode,
                                     &acknowledged_n),
    };
    if (zcbor_map_decode_bulk(zsd, dec, ARRAY_SIZE(dec), &decoded) != 0 ||
        !zcbor_map_decode_bulk_key_found(dec, ARRAY_SIZE(dec), "run_id") ||
        !zcbor_map_decode_bulk_key_found(dec, ARRAY_SIZE(dec), "n")) {
        return MGMT_ERR_EINVAL;
    }

    k_spinlock_key_t data_key = k_spin_lock(&s_data_lock);
    if (requested_run_id != s_measure_run_id || acknowledged_n != s_cv_npts) {
        k_spin_unlock(&s_data_lock, data_key);
        return MGMT_ERR_EINVAL;
    }
    if (s_cv_running || s_measurement_pending) {
        k_spin_unlock(&s_data_lock, data_key);
        return MGMT_ERR_EBUSY;
    }
    /* Même règle que DATA : empêcher l'expiration de gagner entre la
     * validation et le second verrou pris après encodage de l'ACK. */
    if (s_smp_data_held && s_smp_run_owner) {
        s_smp_data_expires_at_ms = k_uptime_get() + SMP_DATA_LEASE_MS;
    }
    k_spin_unlock(&s_data_lock, data_key);

    bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0) &&
              zcbor_tstr_put_lit(zse, "run_id") &&
              zcbor_uint32_put(zse, requested_run_id) &&
              zcbor_tstr_put_lit(zse, "n") &&
              zcbor_uint32_put(zse, acknowledged_n) &&
              zcbor_tstr_put_lit(zse, "released") &&
              zcbor_bool_put(zse, true);
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }

    /* Encoder l'ACK avant de libérer : un EMSGSize ne doit jamais perdre la
     * protection du buffer. Revérifier ensuite l'identité sous verrou. */
    data_key = k_spin_lock(&s_data_lock);
    if (requested_run_id != s_measure_run_id || acknowledged_n != s_cv_npts) {
        k_spin_unlock(&s_data_lock, data_key);
        return MGMT_ERR_EINVAL;
    }
    if (s_cv_running || s_measurement_pending) {
        k_spin_unlock(&s_data_lock, data_key);
        return MGMT_ERR_EBUSY;
    }
    s_smp_data_held = false;
    s_smp_run_owner = false;
    s_smp_data_expires_at_ms = 0;
    k_spin_unlock(&s_data_lock, data_key);
    return MGMT_ERR_EOK;
}

/* Le handler STOP ne doit jamais appeler CV_Abort(), lp_teardown() ni aucune
 * primitive AD5940 : il s'execute dans le contexte mcumgr, en concurrence avec
 * le thread de mesure. Il ne fait qu'armer un drapeau cooperatif. */
static int cv_mgmt_stop(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;
    zcbor_state_t *zse = ctxt->writer->zs;
    uint32_t requested_run_id = 0;
    size_t decoded;
    struct zcbor_map_decode_key_val dec[] = {
        ZCBOR_MAP_DECODE_KEY_DECODER("run_id", zcbor_uint32_decode,
                                     &requested_run_id),
    };
    if (zcbor_map_decode_bulk(zsd, dec, ARRAY_SIZE(dec), &decoded) != 0 ||
        !zcbor_map_decode_bulk_key_found(dec, ARRAY_SIZE(dec), "run_id")) {
        return MGMT_ERR_EINVAL;
    }

    k_spinlock_key_t key = k_spin_lock(&s_data_lock);
    if (requested_run_id != s_measure_run_id || !s_smp_run_owner) {
        k_spin_unlock(&s_data_lock, key);
        return MGMT_ERR_EINVAL;
    }
    bool active = s_cv_running || s_measurement_pending;
    if (active) {
        s_stop_requested = true;
    }
    uint32_t n_snapshot = s_cv_npts;
    k_spin_unlock(&s_data_lock, key);

    bool ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, 0) &&
              zcbor_tstr_put_lit(zse, "run_id") &&
              zcbor_uint32_put(zse, requested_run_id) &&
              zcbor_tstr_put_lit(zse, "stop_requested") &&
              zcbor_bool_put(zse, active) &&
              zcbor_tstr_put_lit(zse, "n") &&
              zcbor_uint32_put(zse, n_snapshot);
    return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static const struct mgmt_handler cv_mgmt_handlers[] = {
    [CV_MGMT_CMD_START]  = { NULL,            cv_mgmt_start },
    [CV_MGMT_CMD_STATUS] = { cv_mgmt_status,  NULL },
    [CV_MGMT_CMD_DATA]   = { cv_mgmt_data,    NULL },
    [CV_MGMT_CMD_RELEASE] = { NULL,            cv_mgmt_release },
    [CV_MGMT_CMD_STOP]   = { NULL,             cv_mgmt_stop },
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

    /* Gestionnaire d'alimentation (OE OSC1 + ADC VDDH/VDD) */
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

    /* VBUSDET indique l'USB ; VDDHDIV5 mesure le rail VDDH/VDD, pas VBUS. */
    if (power_mgr_vbus_present()) {
        int32_t vddh_mv = power_mgr_vddh_read_mv();
        if (vddh_mv > 0) {
            LOG_INF("VBUS présent; rail VDDH/VDD: %d mV", vddh_mv);
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
            int32_t result = 0;
            (void)cv_measure(&result);

            LOG_INF("Mesure envoyée: %d", result);

            bt_conn_unref(conn);
        }
    }
#endif  /* ancien mode produit autonome */

    return 0;
}

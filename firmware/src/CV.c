#include "CV.h"
#include "RampTest.h"
#include "ad5940_lp.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <math.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(cv, LOG_LEVEL_INF);

#define CV_APPBUFF_SIZE     1024

/* Le FIFO est lu par polling (pas de broche d'interruption).  Choisir un
 * seuil qui divise exactement le nombre total de points garantit qu'un seuil
 * FIFO est atteint au dernier point, même si l'AD5940 ne lève pas ENDSEQ.
 * 128 conserve une cadence de lecture raisonnable sur les balayages longs. */
#define CV_FIFO_THRESHOLD_MAX              128u

/* Paramètres internes de la courte sonde d'autorange.  60 est conservé car
 * la fonction de choix du seuil ci-dessous lui attribue exactement 60 : la
 * sonde ne dépend donc plus de ENDSEQ pour vider un FIFO de moins de 128 mots. */
#define CV_AUTORANGE_PROBE_STEPS            60u
#define CV_AUTORANGE_PROBE_DURATION_MS    1200u
#define CV_AUTORANGE_PROBE_SAMPLE_DELAY_MS  2.0f
#define CV_AUTORANGE_PROBE_TIMEOUT_MARGIN_MS 2000u
#define CV_AUTORANGE_FALLBACK_RTIA_IDX       1u /* 1 kOhm : choix sûr */

static uint32_t      s_appbuff[CV_APPBUFF_SIZE];
static uint32_t      s_data_index  = 0;
static float         s_peak_ua     = 0.0f;   /* pic absolu du cycle courant (µA) */
static cv_point_cb_t s_point_cb    = NULL;
static uint32_t      s_last_poll_ms;

/* Paramètres réglables (défauts = constantes CV.h). Modifiables via CV_SetParams(). */
static cv_params_t s_params = {
    .start_mv        = CV_START_VOLT_MV,
    .peak_mv         = CV_PEAK_VOLT_MV,
    .vzero_mv        = CV_VZERO_MV,
    .step_number     = CV_STEP_NUMBER,
    .cycle_count     = CV_CYCLE_COUNT,
    .duration_ms     = CV_DURATION_MS,
    .sample_delay_ms = CV_SAMPLE_DELAY_MS,
    .settle_ms       = CV_SETTLE_MS,
};

/* Instantané figé au démarrage d'un cycle (CV_Init) : isole le cycle en cours
 * d'un éventuel CV_SetParams() concurrent. */
static cv_params_t s_run = {
    .start_mv        = CV_START_VOLT_MV,
    .peak_mv         = CV_PEAK_VOLT_MV,
    .vzero_mv        = CV_VZERO_MV,
    .step_number     = CV_STEP_NUMBER,
    .cycle_count     = CV_CYCLE_COUNT,
    .duration_ms     = CV_DURATION_MS,
    .sample_delay_ms = CV_SAMPLE_DELAY_MS,
    .settle_ms       = CV_SETTLE_MS,
};

/* `step_number` and `duration_ms` describe one elementary CV cycle. These
 * helpers describe the complete run sent to the AD5940 sequencer. Values are
 * validated by CV_SetParams() before a user-provided configuration is used. */
static uint32_t cv_total_points(const cv_params_t *p)
{
    return p->step_number * p->cycle_count;
}

static uint32_t cv_total_duration_ms(const cv_params_t *p)
{
    return (uint32_t)((uint64_t)p->duration_ms * (uint64_t)p->cycle_count);
}

/* Plus grand diviseur de total_points qui tient dans le FIFO.  Le dernier
 * échantillon déclenche ainsi DATAFIFOTHRESH et peut être traité par
 * CV_Service() même si ENDSEQ est absent.  Pour 60 points, le seuil vaut 60;
 * pour 1600, il vaut 100. */
static uint32_t cv_fifo_threshold(uint32_t total_points)
{
    uint32_t threshold = total_points;

    if (threshold > CV_FIFO_THRESHOLD_MAX) {
        threshold = CV_FIFO_THRESHOLD_MAX;
    }
    while (threshold > 1u) {
        if ((total_points % threshold) == 0u) {
            return threshold;
        }
        threshold--;
    }
    return 1u;
}

/* Accumulateurs pour l'estimation de résistance (moindres carrés via origine) */
static double        s_sum_vi      = 0.0;    /* Σ (volt_mV · current_µA) */
static double        s_sum_ii      = 0.0;    /* Σ (current_µA²)          */

/* ── Gamme de mesure (RTIA) ───────────────────────────────────────────────────
 * Le RTIA fixe la fenêtre de courant : I_max ≈ V_swing / RTIA. Réglable au
 * runtime (manuel) ou choisi automatiquement (autorange).
 */
struct rtia_entry { uint32_t ohm; uint32_t sel; };
static const struct rtia_entry RTIA_TABLE[] = {
    {   200, LPTIARTIA_200R }, {  1000, LPTIARTIA_1K  }, {  2000, LPTIARTIA_2K  },
    {  3000, LPTIARTIA_3K  }, {  4000, LPTIARTIA_4K  }, {  6000, LPTIARTIA_6K  },
    {  8000, LPTIARTIA_8K  }, { 10000, LPTIARTIA_10K }, { 12000, LPTIARTIA_12K },
    { 16000, LPTIARTIA_16K }, { 20000, LPTIARTIA_20K }, { 24000, LPTIARTIA_24K },
    { 30000, LPTIARTIA_30K }, { 32000, LPTIARTIA_32K }, { 40000, LPTIARTIA_40K },
    { 48000, LPTIARTIA_48K }, { 64000, LPTIARTIA_64K }, { 85000, LPTIARTIA_85K },
    { 96000, LPTIARTIA_96K }, {100000, LPTIARTIA_100K}, {120000, LPTIARTIA_120K},
    {128000, LPTIARTIA_128K}, {160000, LPTIARTIA_160K}, {196000, LPTIARTIA_196K},
    {256000, LPTIARTIA_256K}, {512000, LPTIARTIA_512K},
};
#define RTIA_TABLE_N  (sizeof(RTIA_TABLE) / sizeof(RTIA_TABLE[0]))

/* Tension de sortie utile du LPTIA autour de Vzero (V), pour estimer la gamme. */
#define RTIA_SWING_V        0.9

static uint32_t s_rtia_idx     = 4;       /* index dans RTIA_TABLE (4 = 4 kΩ)   */
static bool     s_autorange    = false;   /* sélection automatique de la gamme  */
static float    s_rtia_cal_ohm = 0.0f;    /* RTIA réellement calibré (dernier run) */

/* Index de table dont l'ohm est le plus proche d'une valeur demandée. */
static uint32_t cv_rtia_idx_nearest(uint32_t ohm)
{
    uint32_t best = 0;
    uint32_t best_diff = UINT32_MAX;
    for (uint32_t i = 0; i < RTIA_TABLE_N; i++) {
        uint32_t d = (RTIA_TABLE[i].ohm > ohm) ? (RTIA_TABLE[i].ohm - ohm)
                                               : (ohm - RTIA_TABLE[i].ohm);
        if (d < best_diff) { best_diff = d; best = i; }
    }
    return best;
}

/* Plus grand index dont l'ohm est <= ideal (gamme sûre, sans saturation). */
static uint32_t cv_rtia_idx_le(double ideal_ohm)
{
    uint32_t idx = 0;
    for (uint32_t i = 0; i < RTIA_TABLE_N; i++) {
        if ((double)RTIA_TABLE[i].ohm <= ideal_ohm) idx = i;
    }
    return idx;
}

/* ── Tension appliquée pour un index d'échantillon donné ─────────────────────
 * Balayage triangulaire : Vstart → Vpeak sur StepNumber/2 pas, puis retour.
 */
static float cv_voltage_for_index(uint32_t idx)
{
    const float v_start = s_run.start_mv;
    const float v_peak  = s_run.peak_mv;
    const uint32_t half = s_run.step_number / 2;
    const float step    = (v_peak - v_start) / (float)half;  /* mV par pas */

    /* `idx` is global over the full run. Reuse the original one-cycle
     * triangle for every requested cycle. */
    idx %= s_run.step_number;

    if (idx <= half) {
        return v_start + step * (float)idx;                  /* montée */
    }
    uint32_t down = idx - half;
    if (down > half) down = half;
    return v_peak - step * (float)down;                      /* descente */
}

/* ── Traitement d'un lot de points ───────────────────────────────────────── */
static void cv_process_results(float *data, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        float cur_ua = data[i];
        float volt   = cv_voltage_for_index(s_data_index);

        float abs_val = fabsf(cur_ua);
        if (abs_val > s_peak_ua) {
            s_peak_ua = abs_val;
        }

        s_sum_vi += (double)volt   * (double)cur_ua;
        s_sum_ii += (double)cur_ua * (double)cur_ua;

        if (s_point_cb) {
            s_point_cb(s_data_index, volt, cur_ua);
        }
        s_data_index++;
    }
}

/* ── Configuration RampTest depuis les paramètres CV.h ───────────────────── */
static void cv_configure_ramp(float lfosc_hz)
{
    AppRAMPCfg_Type *pCfg;
    AppRAMPGetCfg(&pCfg);

    pCfg->SeqStartAddr   = 0x10;
    pCfg->MaxSeqLen      = 1024 - 0x10;

    pCfg->RcalVal        = CV_RCAL_OHM;
    pCfg->ADCRefVolt     = CV_ADC_REF_V;
    pCfg->SysClkFreq     = 16000000.0f;
    pCfg->LFOSCClkFreq   = lfosc_hz;

    pCfg->RampStartVolt  = s_run.start_mv;
    pCfg->RampPeakVolt   = s_run.peak_mv;
    pCfg->VzeroStart     = s_run.vzero_mv;
    pCfg->VzeroPeak      = s_run.vzero_mv;
    /* The Ramp engine needs the total number of samples to generate, while
     * RampDacRegUpdate() needs the elementary-cycle length to reverse the
     * potential at every peak. */
    pCfg->StepsPerCycle  = s_run.step_number;
    pCfg->CycleCount     = s_run.cycle_count;
    pCfg->StepNumber     = cv_total_points(&s_run);
    pCfg->RampDuration   = cv_total_duration_ms(&s_run);
    pCfg->SampleDelay    = s_run.sample_delay_ms;

    pCfg->LPTIARtiaSel   = RTIA_TABLE[s_rtia_idx].sel;   /* gamme réglable */
    pCfg->LPTIARloadSel  = LPTIARLOAD_SHORT;
    pCfg->AdcPgaGain     = CV_ADC_PGA_GAIN;
    /* Le seuil est adapté à la géométrie complète (pas × cycles). Cela évite
     * de laisser les derniers points sous le seuil lorsque ENDSEQ est absent
     * après le dernier bloc DAC dynamique. */
    pCfg->FifoThresh     = cv_fifo_threshold(pCfg->StepNumber);
    pCfg->bRampOneDir    = bFALSE;
    /* Le capteur est remis à zéro/coupé entre deux mesures : les séquences
     * SRAM du run précédent ne sont plus valides. */
    pCfg->RAMPInited     = bFALSE;
    pCfg->bParaChanged   = bTRUE;

    LOG_INF("CV configuree : Vstart=%d mV Vpeak=%d mV %u pas x %u cycles (%u points, %u ms, FIFO=%u)",
            (int)s_run.start_mv, (int)s_run.peak_mv,
            (unsigned)s_run.step_number, (unsigned)s_run.cycle_count,
            (unsigned)pCfg->StepNumber, (unsigned)pCfg->RampDuration,
            (unsigned)pCfg->FifoThresh);
}

/* ════════════════════════════════════════════════════════════════════════════
   API PUBLIQUE
   ════════════════════════════════════════════════════════════════════════════ */

void CV_SetPointCallback(cv_point_cb_t cb)
{
    s_point_cb = cb;
}

void CV_GetParams(cv_params_t *out)
{
    if (out) {
        *out = s_params;
    }
}

int CV_SetParams(const cv_params_t *in)
{
    if (!in) {
        return -1;
    }
    /* Validation : bornes raisonnables + StepNumber pair (balayage triangulaire). */
    if (in->step_number < 2u || in->step_number > CV_MAX_TOTAL_POINTS) return -1;
    if ((in->step_number & 1u) != 0u)                    return -1;
    if (in->cycle_count == 0u)                            return -1;
    /* The acquisition/result buffer belongs to main.c and cannot be allowed
     * to truncate a valid-looking multi-cycle measurement silently. */
    if (in->cycle_count > CV_MAX_TOTAL_POINTS / in->step_number) return -1;
    if (in->duration_ms < 200u || in->duration_ms > 600000u) return -1;
    if (in->sample_delay_ms < 0.1f || in->sample_delay_ms > 1000.0f) return -1;
    if (in->settle_ms > CV_SETTLE_MAX_MS)                  return -1;
    /* One point must leave time after the DAC update for the ADC sample.
     * The ratio is the same for the complete run because duration and points
     * are both multiplied by cycle_count. */
    if ((float)in->duration_ms / (float)in->step_number <= in->sample_delay_ms) return -1;
    if ((uint64_t)in->duration_ms * (uint64_t)in->cycle_count > UINT32_MAX) return -1;
    if (in->vzero_mv < 200.0f ||
        in->vzero_mv > 200.0f + 63.0f * DAC6BITVOLT_1LSB) return -1;
    if (in->vzero_mv + in->start_mv < 200.0f ||
        in->vzero_mv + in->start_mv > 2400.0f) return -1;
    if (in->vzero_mv + in->peak_mv < 200.0f ||
        in->vzero_mv + in->peak_mv > 2400.0f) return -1;

    s_params = *in;
    LOG_INF("CV params: Vstart=%d Vpeak=%d Vzero=%d steps=%u cycles=%u dur/cycle=%u ms settle=%u ms",
            (int)s_params.start_mv, (int)s_params.peak_mv, (int)s_params.vzero_mv,
            (unsigned)s_params.step_number, (unsigned)s_params.cycle_count,
            (unsigned)s_params.duration_ms, (unsigned)s_params.settle_ms);
    return 0;
}

uint32_t CV_GetActiveDurationMs(void)
{
    return cv_total_duration_ms(&s_run);
}

uint32_t CV_GetActiveStepNumber(void)
{
    return s_run.step_number;
}

uint32_t CV_GetActiveCycleCount(void)
{
    return s_run.cycle_count;
}

uint32_t CV_GetActiveTotalPoints(void)
{
    return cv_total_points(&s_run);
}

uint32_t CV_GetActiveSettleMs(void)
{
    return s_run.settle_ms;
}

AD5940Err CV_ApplyStartPotential(void)
{
    /* AppRAMPInit() a deja configure et alimente la boucle LPDAC/LPTIA. Ecrire
     * ici le potentiel initial permet a la cellule de se stabiliser avant le
     * demarrage du WUPT, sans creer de point artificiel dans le FIFO. La
     * primitive commune effectue aussi une lecture de controle du registre. */
    if (AD5940_WakeUp(10) > 10) {
        return AD5940ERR_WAKEUP;
    }
    if (lp_set_potential_mv(s_run.vzero_mv, s_run.start_mv) != 0) {
        return AD5940ERR_APPERROR;
    }
    LOG_INF("Potentiel initial applique : E=%d mV Vzero=%d mV (stabilisation=%u ms)",
            (int)s_run.start_mv, (int)s_run.vzero_mv,
            (unsigned)s_run.settle_ms);
    return AD5940ERR_OK;
}

float CV_GetResistanceOhm(void)
{
    if (s_sum_ii <= 0.0) {
        return 0.0f;
    }
    /* V en volts, I en ampères : R = Σ(V·I)/Σ(I²).
     * Ici volt en mV et courant en µA → facteur 1000 :
     *   R[Ω] = 1000 · Σ(mV·µA) / Σ(µA²) */
    return (float)(1000.0 * s_sum_vi / s_sum_ii);
}

/* ── Gamme de mesure (RTIA) : API ─────────────────────────────────────────── */

int CV_SetRtia(uint32_t ohm)
{
    if (ohm == 0) {                    /* 0 = autorange */
        s_autorange = true;
        LOG_INF("RTIA : autorange activé");
        return 0;
    }
    s_autorange = false;
    s_rtia_idx  = cv_rtia_idx_nearest(ohm);
    LOG_INF("RTIA réglé sur %u ohm", (unsigned)RTIA_TABLE[s_rtia_idx].ohm);
    return (int)RTIA_TABLE[s_rtia_idx].ohm;
}

uint32_t CV_GetRtiaOhm(void)
{
    return s_autorange ? 0u : RTIA_TABLE[s_rtia_idx].ohm;
}

uint32_t CV_GetActiveRtiaOhm(void)
{
    return RTIA_TABLE[s_rtia_idx].ohm;   /* nominal courant (même en autorange) */
}

int CV_GetAutorange(void)
{
    return s_autorange ? 1 : 0;
}

uint32_t CV_GetCalibratedRtiaOhm(void)
{
    return (uint32_t)(s_rtia_cal_ohm + 0.5f);
}

uint32_t CV_RtiaSelNearest(uint32_t ohm, uint32_t *ohm_out)
{
    uint32_t idx = cv_rtia_idx_nearest(ohm);
    if (ohm_out) {
        *ohm_out = RTIA_TABLE[idx].ohm;
    }
    return RTIA_TABLE[idx].sel;
}

/* Arrête explicitement une sonde échouée. Sans cela, un WUPT encore actif
 * peut exécuter l'ancienne séquence pendant l'initialisation de la sonde
 * suivante ou de la vraie CV, ce qui se manifeste par n=0. */
static void cv_autorange_abort_probe(void)
{
    AppRAMPCfg_Type *pCfg = NULL;

    (void)AppRAMPCtrl(APPCTRL_STOPNOW, NULL);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    if (AppRAMPGetCfg(&pCfg) == AD5940ERR_OK && pCfg) {
        pCfg->bTestFinished = bTRUE;
        pCfg->StopRequired  = bFALSE;
        /* Force une reconstruction complète avant le prochain essai. */
        pCfg->RAMPInited    = bFALSE;
        pCfg->bParaChanged  = bTRUE;
    }
}

/* Un balayage de sonde rapide. Échec et pic nul sont volontairement
 * distingués : un vrai courant nul justifie une grande gamme, tandis qu'une
 * sonde incomplète doit conduire à un repli sûr et jamais à 512 kOhm. */
static int cv_autorange_probe(float lfosc_hz, int32_t *peak_na,
                              measurement_stop_cb_t stop_cb)
{
    const uint32_t expected_points = CV_AUTORANGE_PROBE_STEPS;
    const int64_t timeout_ms = (int64_t)CV_AUTORANGE_PROBE_DURATION_MS +
                               CV_AUTORANGE_PROBE_TIMEOUT_MARGIN_MS;

    if (!peak_na) {
        return -1;
    }
    *peak_na = 0;

    if (stop_cb && stop_cb()) {
        return MEAS_RUN_STOPPED;
    }

    if (CV_Init(lfosc_hz) != AD5940ERR_OK || CV_Start() != AD5940ERR_OK) {
        cv_autorange_abort_probe();
        return -1;
    }

    int64_t t0 = k_uptime_get();
    while (!CV_IsFinished()) {
        if (stop_cb && stop_cb()) {
            cv_autorange_abort_probe();
            return MEAS_RUN_STOPPED;
        }
        k_sleep(K_MSEC(20));
        if (stop_cb && stop_cb()) {
            cv_autorange_abort_probe();
            return MEAS_RUN_STOPPED;
        }
        if (CV_Service() != AD5940ERR_OK) {
            cv_autorange_abort_probe();
            return -1;
        }
        if (k_uptime_get() - t0 > timeout_ms) {
            break;
        }
    }

    if (!CV_IsFinished() || s_data_index != expected_points) {
        LOG_WRN("Autorange sonde incomplete : %u/%u points",
                (unsigned)s_data_index, (unsigned)expected_points);
        cv_autorange_abort_probe();
        return -1;
    }

    *peak_na = CV_GetPeakCurrentNa();
    return 0;
}

int CV_Autorange(float lfosc_hz, measurement_stop_cb_t stop_cb)
{
    int32_t pk = 0;
    int probe_rc;

    if (!s_autorange) {
        return 0;
    }

    /* La sonde modifie temporairement params/callback : on sauvegarde. */
    cv_point_cb_t saved_cb     = s_point_cb;
    cv_params_t   saved_params = s_params;
    s_point_cb = NULL;                       /* pas de streaming des points de sonde */

    /* Balayage de sonde : rapide, même plage de tension/Vzero. */
    s_params.step_number     = CV_AUTORANGE_PROBE_STEPS;
    s_params.cycle_count     = 1;
    s_params.duration_ms     = CV_AUTORANGE_PROBE_DURATION_MS;
    s_params.sample_delay_ms = CV_AUTORANGE_PROBE_SAMPLE_DELAY_MS;

    /* Sonde 1 : gain faible (1 kΩ → jusqu'à ~900 µA sans saturer) pour estimer. */
    s_rtia_idx = CV_AUTORANGE_FALLBACK_RTIA_IDX;
    probe_rc = cv_autorange_probe(lfosc_hz, &pk, stop_cb);
    if (probe_rc == MEAS_RUN_STOPPED) goto cancelled;
    if (probe_rc != 0) {
        LOG_WRN("Autorange sonde1 echouee ; repli RTIA=1 kOhm");
        s_rtia_idx = CV_AUTORANGE_FALLBACK_RTIA_IDX;
        goto restore;
    }
    LOG_INF("Autorange sonde1 : RTIA=1000 ohm, Ipic=%d nA", pk);

    if (pk > 0) {
        double ipk   = (double)pk * 1e-9;                /* A */
        double ideal = RTIA_SWING_V * 0.4 / ipk;         /* viser ~40 %% pleine échelle */
        s_rtia_idx   = cv_rtia_idx_le(ideal);
    } else {
        s_rtia_idx   = RTIA_TABLE_N - 1;                 /* courant réellement nul → gain max */
    }

    /* Sonde(s) 2+ : vérifie qu'on ne sature pas, recule sinon. */
    probe_rc = cv_autorange_probe(lfosc_hz, &pk, stop_cb);
    if (probe_rc == MEAS_RUN_STOPPED) goto cancelled;
    if (probe_rc != 0) {
        LOG_WRN("Autorange sonde2 echouee ; repli RTIA=1 kOhm");
        s_rtia_idx = CV_AUTORANGE_FALLBACK_RTIA_IDX;
        goto restore;
    }
    double rmax = RTIA_SWING_V / (double)RTIA_TABLE[s_rtia_idx].ohm * 1e9;
    LOG_INF("Autorange sonde2 : RTIA=%u ohm, Ipic=%d nA (%d%% pleine echelle)",
            (unsigned)RTIA_TABLE[s_rtia_idx].ohm, pk,
            (rmax > 0.0) ? (int)((double)pk / rmax * 100.0) : 0);
    while (pk > 0.9 * rmax && s_rtia_idx > 0) {
        s_rtia_idx--;
        probe_rc = cv_autorange_probe(lfosc_hz, &pk, stop_cb);
        if (probe_rc == MEAS_RUN_STOPPED) goto cancelled;
        if (probe_rc != 0) {
            LOG_WRN("Autorange sonde de securite echouee ; repli RTIA=1 kOhm");
            s_rtia_idx = CV_AUTORANGE_FALLBACK_RTIA_IDX;
            goto restore;
        }
        rmax = RTIA_SWING_V / (double)RTIA_TABLE[s_rtia_idx].ohm * 1e9;
    }

    /* Restaure les paramètres utilisateur ; conserve le RTIA choisi. */
restore:
    s_params   = saved_params;
    s_point_cb = saved_cb;
    LOG_INF("Autorange → RTIA choisi = %u ohm", (unsigned)RTIA_TABLE[s_rtia_idx].ohm);
    return 0;

cancelled:
    s_params   = saved_params;
    s_point_cb = saved_cb;
    LOG_INF("Autorange annule par l'utilisateur");
    return MEAS_RUN_STOPPED;
}

AD5940Err CV_Init(float lfosc_hz)
{
    s_peak_ua    = 0.0f;
    s_rtia_cal_ohm = 0.0f;
    s_data_index = 0;
    s_sum_vi     = 0.0;
    s_sum_ii     = 0.0;

    /* Fige les paramètres réglables pour toute la durée de ce cycle. */
    s_run = s_params;

    cv_configure_ramp(lfosc_hz);

    AD5940Err err = AppRAMPInit(s_appbuff, CV_APPBUFF_SIZE);
    if (err != AD5940ERR_OK) {
        LOG_ERR("AppRAMPInit échoué (err=%d)", (int)err);
        return err;
    }

    AppRAMPCfg_Type *pDbg;
    AppRAMPGetCfg(&pDbg);
#if defined(CV_RTIA_OVERRIDE_OHM) && (CV_RTIA_OVERRIDE_OHM > 0)
    /* Calibration RTIA ignorée : on force une valeur connue. Utile quand la
     * RCAL externe ne correspond pas à CV_RCAL_OHM et fausse la cal. */
    pDbg->RtiaValue.Magnitude = (float)CV_RTIA_OVERRIDE_OHM;
    pDbg->RtiaValue.Phase     = 0.0f;
    LOG_INF("RTIA force a %d ohm (calibration ignoree)",
            (int)pDbg->RtiaValue.Magnitude);
#else
    LOG_INF("Rtia nominal=%u ohm, calibre=%d ohm",
            (unsigned)RTIA_TABLE[s_rtia_idx].ohm, (int)pDbg->RtiaValue.Magnitude);
#endif
    s_rtia_cal_ohm = pDbg->RtiaValue.Magnitude;

    LOG_INF("CV initialisée");
    return AD5940ERR_OK;
}

AD5940Err CV_Start(void)
{
    s_peak_ua    = 0.0f;
    s_data_index = 0;
    s_sum_vi     = 0.0;
    s_sum_ii     = 0.0;
    s_last_poll_ms = 0u;

    /* AppRAMPInit() vient de reconstruire les blocs DAC ping-pong et a déjà
     * initialisé RampState/bFirstDACSeq/bDACCodeInc. Ne pas les réinitialiser
     * ici : au premier CUSTOMINT0 le générateur repartirait au bloc 0 et
     * pourrait réécrire une séquence que l'AD5940 est en train d'exécuter.
     * Seuls les drapeaux de contrôle du nouveau cycle doivent être effacés. */
    AppRAMPCfg_Type *pCfg;
    AppRAMPGetCfg(&pCfg);
    pCfg->bTestFinished = bFALSE;
    pCfg->StopRequired  = bFALSE;

    AD5940Err err = AppRAMPCtrl(APPCTRL_START, NULL);
    if (err != AD5940ERR_OK) {
        LOG_ERR("CV_Start échoué (err=%d)", (int)err);
    } else {
        LOG_INF("Cycle CV démarré");
    }
    return err;
}

AD5940Err CV_Service(void)
{
    /* Polling toutes les 20 ms (pas de pin d'interruption) */
    uint32_t now_ms = k_uptime_get_32();
    if ((now_ms - s_last_poll_ms) < 20U) {
        return AD5940ERR_OK;
    }
    s_last_poll_ms = now_ms;

    uint32_t count = CV_APPBUFF_SIZE;
    AD5940Err err  = AppRAMPISR(s_appbuff, &count);

    if (err != AD5940ERR_OK) {
        static uint32_t err_count = 0;
        if (((++err_count) % 50U) == 1U) {
            LOG_WRN("AppRAMPISR err=%d (x%u)", (int)err, (unsigned)err_count);
        }
        return err;
    }

    if (count > 0) {
        /* The AD5940 can occasionally miss ENDSEQ after the final dynamic DAC
         * block even though every requested sample has reached the FIFO. Never
         * publish more points than the configured run, and use that exact
         * count as a safe completion criterion. */
        const uint32_t expected = cv_total_points(&s_run);
        if (s_data_index >= expected) {
            count = 0;
        } else if (count > expected - s_data_index) {
            count = expected - s_data_index;
        }

    }

    if (count > 0) {
        cv_process_results((float *)s_appbuff, count);
    }

    if (s_data_index == cv_total_points(&s_run) && !CV_IsFinished()) {
        AppRAMPCfg_Type *pCfg;
        AppRAMPGetCfg(&pCfg);
        /* Stop the wake-up timer before declaring completion. This fallback
         * is reached only after the exact number of requested samples. */
        err = AppRAMPCtrl(APPCTRL_STOPNOW, NULL);
        if (err != AD5940ERR_OK) {
            LOG_ERR("Arret WUPT en fin de comptage echoue (err=%d)", (int)err);
            return err;
        }
        pCfg->bTestFinished = bTRUE;
        LOG_WRN("CV terminee sur comptage (%u points, ENDSEQ absent)",
                (unsigned)s_data_index);
    }
    return AD5940ERR_OK;
}

void CV_Abort(void)
{
    AppRAMPCfg_Type *pCfg = NULL;

    (void)AppRAMPCtrl(APPCTRL_STOPNOW, NULL);
    if (AppRAMPGetCfg(&pCfg) == AD5940ERR_OK && pCfg) {
        pCfg->StopRequired = bFALSE;
        /* Une interruption sur erreur n'est pas une fin réussie. Le prochain
         * CV_Init reconstruira entièrement les séquences. */
        pCfg->bTestFinished = bFALSE;
        pCfg->RAMPInited = bFALSE;
        pCfg->bParaChanged = bTRUE;
    }
}

int CV_IsFinished(void)
{
    AppRAMPCfg_Type *pCfg;
    AppRAMPGetCfg(&pCfg);
    return (pCfg->bTestFinished == bTRUE) ? 1 : 0;
}

AD5940Err CV_Restart(void)
{
    AppRAMPCfg_Type *pCfg;
    AppRAMPGetCfg(&pCfg);
    pCfg->bTestFinished = bFALSE;

    s_peak_ua    = 0.0f;
    s_data_index = 0;
    s_sum_vi     = 0.0;
    s_sum_ii     = 0.0;

    AD5940_SEQCtrlS(bTRUE);
    AD5940Err err = AppRAMPCtrl(APPCTRL_START, NULL);
    if (err != AD5940ERR_OK) {
        LOG_ERR("CV_Restart échoué (err=%d)", (int)err);
    }
    return err;
}

int32_t CV_GetPeakCurrentNa(void)
{
    /* µA × 1000 → nA, stocké en int32_t */
    return (int32_t)(s_peak_ua * 1000.0f);
}

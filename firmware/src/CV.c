#include "CV.h"
#include "RampTest.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <math.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(cv, LOG_LEVEL_INF);

#define CV_APPBUFF_SIZE     1024

static uint32_t      s_appbuff[CV_APPBUFF_SIZE];
static uint32_t      s_data_index  = 0;
static float         s_peak_ua     = 0.0f;   /* pic absolu du cycle courant (µA) */
static cv_point_cb_t s_point_cb    = NULL;

/* Paramètres réglables (défauts = constantes CV.h). Modifiables via CV_SetParams(). */
static cv_params_t s_params = {
    .start_mv        = CV_START_VOLT_MV,
    .peak_mv         = CV_PEAK_VOLT_MV,
    .vzero_mv        = CV_VZERO_MV,
    .step_number     = CV_STEP_NUMBER,
    .duration_ms     = CV_DURATION_MS,
    .sample_delay_ms = CV_SAMPLE_DELAY_MS,
};

/* Instantané figé au démarrage d'un cycle (CV_Init) : isole le cycle en cours
 * d'un éventuel CV_SetParams() concurrent. */
static cv_params_t s_run = {
    .start_mv        = CV_START_VOLT_MV,
    .peak_mv         = CV_PEAK_VOLT_MV,
    .vzero_mv        = CV_VZERO_MV,
    .step_number     = CV_STEP_NUMBER,
    .duration_ms     = CV_DURATION_MS,
    .sample_delay_ms = CV_SAMPLE_DELAY_MS,
};

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
    pCfg->ADCRefVolt     = CV_ADC_REF_MV;
    pCfg->SysClkFreq     = 16000000.0f;
    pCfg->LFOSCClkFreq   = lfosc_hz;

    pCfg->RampStartVolt  = s_run.start_mv;
    pCfg->RampPeakVolt   = s_run.peak_mv;
    pCfg->VzeroStart     = s_run.vzero_mv;
    pCfg->VzeroPeak      = s_run.vzero_mv;
    pCfg->StepNumber     = s_run.step_number;
    pCfg->RampDuration   = s_run.duration_ms;
    pCfg->SampleDelay    = s_run.sample_delay_ms;

    pCfg->LPTIARtiaSel   = RTIA_TABLE[s_rtia_idx].sel;   /* gamme réglable */
    pCfg->LPTIARloadSel  = LPTIARLOAD_SHORT;
    pCfg->AdcPgaGain     = CV_ADC_PGA_GAIN;
    pCfg->FifoThresh     = 480;
    pCfg->bRampOneDir    = bFALSE;

    LOG_INF("CV configuree : Vstart=%d mV  Vpeak=%d mV  %u pas  %u ms",
            (int)s_run.start_mv, (int)s_run.peak_mv,
            (unsigned)s_run.step_number, (unsigned)s_run.duration_ms);
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
    if (in->step_number < 2u || in->step_number > 4000u) return -1;
    if ((in->step_number & 1u) != 0u)                    return -1;
    if (in->duration_ms < 200u || in->duration_ms > 600000u) return -1;
    if (in->sample_delay_ms < 0.1f || in->sample_delay_ms > 1000.0f) return -1;
    if (in->start_mv < -2200.0f || in->start_mv > 2200.0f) return -1;
    if (in->peak_mv  < -2200.0f || in->peak_mv  > 2200.0f) return -1;
    if (in->vzero_mv <     0.0f || in->vzero_mv  > 2400.0f) return -1;

    s_params = *in;
    LOG_INF("CV params: Vstart=%d Vpeak=%d Vzero=%d steps=%u dur=%u ms",
            (int)s_params.start_mv, (int)s_params.peak_mv, (int)s_params.vzero_mv,
            (unsigned)s_params.step_number, (unsigned)s_params.duration_ms);
    return 0;
}

uint32_t CV_GetActiveDurationMs(void)
{
    return s_run.duration_ms;
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

/* Un balayage de sonde rapide ; renvoie le courant de pic (nA) ou -1 si échec. */
static int32_t cv_autorange_probe(float lfosc_hz)
{
    if (CV_Init(lfosc_hz) != AD5940ERR_OK || CV_Start() != AD5940ERR_OK) {
        return -1;
    }
    int64_t t0 = k_uptime_get();
    while (!CV_IsFinished()) {
        k_sleep(K_MSEC(20));
        CV_Service();
        if (k_uptime_get() - t0 > 5000) {    /* sécurité : pas de blocage */
            break;
        }
    }
    return CV_GetPeakCurrentNa();
}

void CV_Autorange(float lfosc_hz)
{
    if (!s_autorange) {
        return;
    }

    /* La sonde modifie temporairement params/callback : on sauvegarde. */
    cv_point_cb_t saved_cb     = s_point_cb;
    cv_params_t   saved_params = s_params;
    s_point_cb = NULL;                       /* pas de streaming des points de sonde */

    /* Balayage de sonde : rapide, même plage de tension/Vzero. */
    s_params.step_number     = 60;
    s_params.duration_ms     = 1200;
    s_params.sample_delay_ms = 2.0f;

    /* Sonde 1 : gain faible (1 kΩ → jusqu'à ~900 µA sans saturer) pour estimer. */
    s_rtia_idx = 1;
    int32_t pk = cv_autorange_probe(lfosc_hz);
    LOG_INF("Autorange sonde1 : RTIA=1000 ohm, Ipic=%d nA", pk);

    if (pk > 0) {
        double ipk   = (double)pk * 1e-9;                /* A */
        double ideal = RTIA_SWING_V * 0.4 / ipk;         /* viser ~40 %% pleine échelle */
        s_rtia_idx   = cv_rtia_idx_le(ideal);
    } else {
        s_rtia_idx   = RTIA_TABLE_N - 1;                 /* indétectable → gain max */
    }

    /* Sonde(s) 2+ : vérifie qu'on ne sature pas, recule sinon. */
    pk = cv_autorange_probe(lfosc_hz);
    double rmax = RTIA_SWING_V / (double)RTIA_TABLE[s_rtia_idx].ohm * 1e9;
    LOG_INF("Autorange sonde2 : RTIA=%u ohm, Ipic=%d nA (%d%% pleine echelle)",
            (unsigned)RTIA_TABLE[s_rtia_idx].ohm, pk,
            (rmax > 0.0) ? (int)((double)pk / rmax * 100.0) : 0);
    while (pk > 0.9 * rmax && s_rtia_idx > 0) {
        s_rtia_idx--;
        pk   = cv_autorange_probe(lfosc_hz);
        rmax = RTIA_SWING_V / (double)RTIA_TABLE[s_rtia_idx].ohm * 1e9;
    }

    /* Restaure les paramètres utilisateur ; conserve le RTIA choisi. */
    s_params   = saved_params;
    s_point_cb = saved_cb;
    LOG_INF("Autorange → RTIA choisi = %u ohm", (unsigned)RTIA_TABLE[s_rtia_idx].ohm);
}

AD5940Err CV_Init(float lfosc_hz)
{
    s_peak_ua    = 0.0f;
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

    /* Remise à zéro de la machine d'état du ramp AVANT de démarrer : le
     * gestionnaire ENDSEQ (RampTest.c) laisse bTestFinished=bTRUE en fin de
     * cycle et n'est effacé nulle part ailleurs. Sans ça, seul le 1er cycle
     * après un boot tourne — tous les suivants (sondes d'autorange + la vraie
     * CV) voient CV_IsFinished()==vrai d'emblée → 0 point, pic=0. */
    AppRAMPCfg_Type *pCfg;
    AppRAMPGetCfg(&pCfg);
    pCfg->bTestFinished = bFALSE;
    pCfg->RampState     = RAMP_STATE0;
    pCfg->bFirstDACSeq  = bTRUE;
    pCfg->bDACCodeInc   = bTRUE;

    AD5940Err err = AppRAMPCtrl(APPCTRL_START, NULL);
    if (err != AD5940ERR_OK) {
        LOG_ERR("CV_Start échoué (err=%d)", (int)err);
    } else {
        LOG_INF("Cycle CV démarré");
    }
    return err;
}

void CV_Service(void)
{
    /* Polling toutes les 20 ms (pas de pin d'interruption) */
    static uint32_t s_last_poll_ms = 0;

    uint32_t now_ms = k_uptime_get_32();
    if ((now_ms - s_last_poll_ms) < 20U) {
        return;
    }
    s_last_poll_ms = now_ms;

    uint32_t count = CV_APPBUFF_SIZE;
    AD5940Err err  = AppRAMPISR(s_appbuff, &count);

    if (err != AD5940ERR_OK) {
        static uint32_t err_count = 0;
        if (((++err_count) % 50U) == 1U) {
            LOG_WRN("AppRAMPISR err=%d (x%u)", (int)err, (unsigned)err_count);
        }
        return;
    }

    if (count > 0) {
        cv_process_results((float *)s_appbuff, count);
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

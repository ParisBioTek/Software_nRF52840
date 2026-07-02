#include "DPV.h"
#include "ad5940_lp.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(dpv, LOG_LEVEL_INF);

static dpv_params_t s_p = {
    .start_mv   = -600.0f,
    .end_mv     =  600.0f,
    .estep_mv   =    5.0f,
    .epulse_mv  =   50.0f,
    .tpulse_ms  =   50,
    .tperiod_ms =  200,
    .vzero_mv   = 1300.0f,
};
static cv_point_cb_t s_cb;
static int32_t       s_peak_na;

void DPV_SetPointCallback(cv_point_cb_t cb) { s_cb = cb; }

void DPV_GetParams(dpv_params_t *out) { if (out) *out = s_p; }

int DPV_SetParams(const dpv_params_t *in)
{
    if (!in) return -1;
    if (in->start_mv < -2000.0f || in->start_mv > 2000.0f) return -1;
    if (in->end_mv   < -2000.0f || in->end_mv   > 2000.0f) return -1;
    if (in->end_mv == in->start_mv)                        return -1;
    if (in->estep_mv <= 0.0f || in->estep_mv > 200.0f)     return -1;
    if (in->epulse_mv == 0.0f || fabsf(in->epulse_mv) > 500.0f) return -1;
    if (in->tpulse_ms < 5u || in->tpulse_ms > 5000u)       return -1;
    if (in->tperiod_ms <= in->tpulse_ms || in->tperiod_ms > 20000u) return -1;
    if (in->vzero_mv < 200.0f || in->vzero_mv > 2400.0f)   return -1;
    s_p = *in;
    LOG_INF("DPV params : %d→%d mV  pas=%d  pulse=%d mV  tp=%u ms  T=%u ms",
            (int)s_p.start_mv, (int)s_p.end_mv, (int)s_p.estep_mv,
            (int)s_p.epulse_mv, (unsigned)s_p.tpulse_ms, (unsigned)s_p.tperiod_ms);
    return 0;
}

int32_t DPV_GetPeakCurrentNa(void) { return s_peak_na; }

int DPV_Run(float lfosc_hz, uint32_t rtia_ohm)
{
    dpv_params_t p = s_p;
    s_peak_na = 0;

    if (lp_setup(lfosc_hz, rtia_ohm) != 0) {
        return -1;
    }

    float    dir     = (p.end_mv >= p.start_mv) ? 1.0f : -1.0f;
    float    step    = dir * fabsf(p.estep_mv);
    uint32_t nsteps  = (uint32_t)(fabsf(p.end_mv - p.start_mv) / fabsf(p.estep_mv)) + 1u;
    uint32_t tbase   = p.tperiod_ms - p.tpulse_ms;
    float    peak    = 0.0f;

    lp_set_potential_mv(p.vzero_mv, p.start_mv);
    k_sleep(K_MSEC(50));                     /* établissement de la cellule */

    for (uint32_t k = 0; k < nsteps; k++) {
        float e_base = p.start_mv + step * (float)k;

        /* Niveau de base : on mesure I1 en fin de palier. */
        lp_set_potential_mv(p.vzero_mv, e_base);
        k_sleep(K_MSEC(tbase));
        float i1 = lp_read_current_ua();

        /* Impulsion : on mesure I2 en fin d'impulsion. */
        lp_set_potential_mv(p.vzero_mv, e_base + p.epulse_mv);
        k_sleep(K_MSEC(p.tpulse_ms));
        float i2 = lp_read_current_ua();

        float di = i2 - i1;
        if (fabsf(di) > peak) peak = fabsf(di);
        if (s_cb) s_cb(k, e_base, di);       /* X = tension base (mV), Y = ΔI */
    }

    s_peak_na = (int32_t)(peak * 1000.0f);
    lp_teardown();
    LOG_INF("DPV terminée — %u points, pic ΔI=%d nA", (unsigned)nsteps, s_peak_na);
    return 0;
}

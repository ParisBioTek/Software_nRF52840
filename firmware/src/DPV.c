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
    if (!isfinite(in->start_mv) || !isfinite(in->end_mv) ||
        !isfinite(in->estep_mv) || !isfinite(in->epulse_mv) ||
        !isfinite(in->vzero_mv))                              return -1;
    if (in->end_mv == in->start_mv)                        return -1;
    if (in->estep_mv <= 0.0f || in->estep_mv > 200.0f)     return -1;
    if (in->epulse_mv == 0.0f || fabsf(in->epulse_mv) > 500.0f) return -1;
    if (in->tpulse_ms < 5u || in->tpulse_ms > 5000u)       return -1;
    if (in->tperiod_ms <= in->tpulse_ms || in->tperiod_ms > 20000u) return -1;
    if ((uint32_t)(fabsf(in->end_mv - in->start_mv) /
                   fabsf(in->estep_mv)) + 1u > CV_MAX_TOTAL_POINTS) return -1;
    if (lp_validate_potential_mv(in->vzero_mv, in->start_mv) != 0 ||
        lp_validate_potential_mv(in->vzero_mv, in->end_mv) != 0 ||
        lp_validate_potential_mv(in->vzero_mv, in->start_mv + in->epulse_mv) != 0 ||
        lp_validate_potential_mv(in->vzero_mv, in->end_mv + in->epulse_mv) != 0) return -1;
    s_p = *in;
    LOG_INF("DPV params : %d→%d mV  pas=%d  pulse=%d mV  tp=%u ms  T=%u ms",
            (int)s_p.start_mv, (int)s_p.end_mv, (int)s_p.estep_mv,
            (int)s_p.epulse_mv, (unsigned)s_p.tpulse_ms, (unsigned)s_p.tperiod_ms);
    return 0;
}

int32_t DPV_GetPeakCurrentNa(void) { return s_peak_na; }

static bool dpv_wait_or_stop(uint32_t delay_ms, measurement_stop_cb_t stop_cb)
{
    while (delay_ms > 0u) {
        if (stop_cb && stop_cb()) return true;
        uint32_t slice_ms = (delay_ms > 20u) ? 20u : delay_ms;
        k_sleep(K_MSEC(slice_ms));
        delay_ms -= slice_ms;
    }
    return stop_cb && stop_cb();
}

int DPV_Run(float lfosc_hz, uint32_t rtia_ohm,
            measurement_stop_cb_t stop_cb)
{
    dpv_params_t p = s_p;
    s_peak_na = 0;

    if (stop_cb && stop_cb()) {
        return MEAS_RUN_STOPPED;
    }

    if (lp_setup(lfosc_hz, rtia_ohm) != 0) {
        return -1;
    }

    if (stop_cb && stop_cb()) {
        lp_teardown();
        return MEAS_RUN_STOPPED;
    }

    float    dir     = (p.end_mv >= p.start_mv) ? 1.0f : -1.0f;
    float    step    = dir * fabsf(p.estep_mv);
    uint32_t nsteps  = (uint32_t)(fabsf(p.end_mv - p.start_mv) / fabsf(p.estep_mv)) + 1u;
    uint32_t tbase   = p.tperiod_ms - p.tpulse_ms;
    float    peak    = 0.0f;

    if (lp_set_potential_mv(p.vzero_mv, p.start_mv) != 0) {
        lp_teardown();
        return -1;
    }
    if (dpv_wait_or_stop(50u, stop_cb)) {    /* établissement de la cellule */
        lp_teardown();
        return MEAS_RUN_STOPPED;
    }

    for (uint32_t k = 0; k < nsteps; k++) {
        float e_base = p.start_mv + step * (float)k;

        /* Niveau de base : on mesure I1 en fin de palier. */
        if (lp_set_potential_mv(p.vzero_mv, e_base) != 0) {
            lp_teardown();
            return -1;
        }
        if (dpv_wait_or_stop(tbase, stop_cb)) {
            s_peak_na = (int32_t)(peak * 1000.0f);
            lp_teardown();
            LOG_INF("DPV interrompue par l'utilisateur apres %u point(s)",
                    (unsigned)k);
            return MEAS_RUN_STOPPED;
        }
        float i1 = lp_read_current_ua();
        if (stop_cb && stop_cb()) {
            s_peak_na = (int32_t)(peak * 1000.0f);
            lp_teardown();
            LOG_INF("DPV interrompue par l'utilisateur apres %u point(s)",
                    (unsigned)k);
            return MEAS_RUN_STOPPED;
        }

        /* Impulsion : on mesure I2 en fin d'impulsion. */
        if (lp_set_potential_mv(p.vzero_mv, e_base + p.epulse_mv) != 0) {
            lp_teardown();
            return -1;
        }
        if (dpv_wait_or_stop(p.tpulse_ms, stop_cb)) {
            s_peak_na = (int32_t)(peak * 1000.0f);
            lp_teardown();
            LOG_INF("DPV interrompue par l'utilisateur apres %u point(s)",
                    (unsigned)k);
            return MEAS_RUN_STOPPED;
        }
        float i2 = lp_read_current_ua();

        if (!isfinite(i1) || !isfinite(i2)) {
            lp_teardown();
            return -1;
        }

        float di = i2 - i1;
        if (fabsf(di) > peak) peak = fabsf(di);
        if (s_cb) s_cb(k, e_base, di);       /* X = tension base (mV), Y = ΔI */
    }

    s_peak_na = (int32_t)(peak * 1000.0f);
    lp_teardown();
    LOG_INF("DPV terminée — %u points, pic ΔI=%d nA", (unsigned)nsteps, s_peak_na);
    return 0;
}

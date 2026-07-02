#include "CA.h"
#include "ad5940_lp.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(ca, LOG_LEVEL_INF);

static ca_params_t s_p = {
    .pot_mv      = 500.0f,
    .vzero_mv    = 1300.0f,
    .duration_ms = 10000,
    .interval_ms = 100,
};
static cv_point_cb_t s_cb;
static int32_t       s_peak_na;

void CA_SetPointCallback(cv_point_cb_t cb) { s_cb = cb; }

void CA_GetParams(ca_params_t *out) { if (out) *out = s_p; }

int CA_SetParams(const ca_params_t *in)
{
    if (!in) return -1;
    if (in->duration_ms < 100u  || in->duration_ms > 600000u) return -1;
    if (in->interval_ms < 5u    || in->interval_ms > 10000u)  return -1;
    if (in->interval_ms > in->duration_ms)                    return -1;
    if (in->pot_mv   < -2000.0f || in->pot_mv   > 2000.0f)    return -1;
    if (in->vzero_mv <   200.0f || in->vzero_mv > 2400.0f)    return -1;
    s_p = *in;
    LOG_INF("CA params : E=%d mV  Vzero=%d mV  durée=%u ms  intervalle=%u ms",
            (int)s_p.pot_mv, (int)s_p.vzero_mv,
            (unsigned)s_p.duration_ms, (unsigned)s_p.interval_ms);
    return 0;
}

int32_t CA_GetPeakCurrentNa(void) { return s_peak_na; }

int CA_Run(float lfosc_hz, uint32_t rtia_ohm)
{
    ca_params_t p = s_p;
    s_peak_na = 0;

    if (lp_setup(lfosc_hz, rtia_ohm) != 0) {
        return -1;
    }

    lp_set_potential_mv(p.vzero_mv, p.pot_mv);
    k_sleep(K_MSEC(50));                     /* établissement de la cellule */

    uint32_t n = p.duration_ms / p.interval_ms;
    if (n == 0) n = 1;

    int64_t t0   = k_uptime_get();
    float   peak = 0.0f;

    for (uint32_t k = 0; k < n; k++) {
        int64_t target = t0 + (int64_t)k * p.interval_ms;
        int64_t now    = k_uptime_get();
        if (target > now) {
            k_sleep(K_MSEC(target - now));
        }
        float i_ua = lp_read_current_ua();
        if (fabsf(i_ua) > peak) peak = fabsf(i_ua);

        uint32_t t_ms = (uint32_t)(k_uptime_get() - t0);
        if (s_cb) s_cb(k, (float)t_ms, i_ua);   /* X = temps (ms) */
    }

    s_peak_na = (int32_t)(peak * 1000.0f);
    lp_teardown();
    LOG_INF("CA terminée — %u points, pic=%d nA", (unsigned)n, s_peak_na);
    return 0;
}

#include "power_manager.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>

LOG_MODULE_REGISTER(power_mgr, LOG_LEVEL_INF);

/* ── GPIO P0.08 : contrôle oscillateur 32 MHz externe ───────────────────────
 * HIGH = cristal actif   LOW = cristal éteint (économie d'énergie)
 * Défini via l'alias "hfxo-enable" dans app.overlay.
 */
static const struct gpio_dt_spec hfxo_en =
    GPIO_DT_SPEC_GET(DT_ALIAS(hfxo_enable), gpios);

/* ── ADC SAADC : lecture tension VBUS ────────────────────────────────────────
 * Utilise l'entrée interne VDDHDIV5 du nRF52840 : mesure VDDH/5 sans pin externe.
 * VDDH est connecté à VBUS en interne → Vbus = Vadc × 5.
 * À VBUS = 5 V : VDDHDIV5 ≈ 1 V, bien dans la plage SAADC (ref 0.6 V, gain 1/6 → 3.6 V max).
 */

static const struct adc_dt_spec vbus_adc =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t       adc_raw;
static struct adc_sequence adc_seq = {
    .buffer      = &adc_raw,
    .buffer_size = sizeof(adc_raw),
};

/* ── Initialisation ──────────────────────────────────────────────────────── */

int power_mgr_init(void)
{
    int err;

    /* GPIO P0.08 */
    if (!gpio_is_ready_dt(&hfxo_en)) {
        LOG_ERR("GPIO HFXO (P0.08) non prêt");
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&hfxo_en, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("gpio_pin_configure HFXO: %d", err);
        return err;
    }
    LOG_INF("GPIO HFXO P0.08 configuré (inactif)");

    /* ADC VBUS */
    if (!adc_is_ready_dt(&vbus_adc)) {
        LOG_ERR("ADC VBUS non prêt");
        return -ENODEV;
    }
    err = adc_channel_setup_dt(&vbus_adc);
    if (err) {
        LOG_ERR("adc_channel_setup VBUS: %d", err);
        return err;
    }
    (void)adc_sequence_init_dt(&vbus_adc, &adc_seq);

    LOG_INF("Power manager initialisé");
    return 0;
}

/* ── Contrôle HFXO ───────────────────────────────────────────────────────── */

void power_mgr_hfxo_enable(void)
{
    gpio_pin_set_dt(&hfxo_en, 1);
    /* ~500 µs de stabilisation (typique pour un cristal XTAL SMD) */
    k_busy_wait(500);
    LOG_DBG("HFXO ON");
}

void power_mgr_hfxo_disable(void)
{
    gpio_pin_set_dt(&hfxo_en, 0);
    LOG_DBG("HFXO OFF");
}

/* ── Détection et lecture VBUS ───────────────────────────────────────────── */

bool power_mgr_vbus_present(void)
{
    /* Registre POWER du nRF52840 : bit VBUSDET — sans ADC, sans diviseur */
    return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
}

int32_t power_mgr_vbus_read_mv(void)
{
    int err = adc_read_dt(&vbus_adc, &adc_seq);
    if (err) {
        LOG_ERR("adc_read VBUS: %d", err);
        return err;
    }

    /* Tension sur l'entrée ADC (mV) — référence interne 0.6 V, gain 1/6 → pleine échelle 3.6 V */
    int32_t val_mv = (int32_t)adc_raw;
    err = adc_raw_to_millivolts_dt(&vbus_adc, &val_mv);
    if (err) {
        LOG_ERR("adc_raw_to_millivolts: %d", err);
        return err;
    }

    /* VDDHDIV5 = VBUS / 5 → Vbus = Vadc × 5 */
    val_mv = val_mv * 5;

    LOG_INF("VBUS: %d mV (présent: %s)",
            val_mv, power_mgr_vbus_present() ? "oui" : "non");
    return val_mv;
}

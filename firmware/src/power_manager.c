#include "power_manager.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>

LOG_MODULE_REGISTER(power_mgr, LOG_LEVEL_INF);

/* ── GPIO P0.08 : OE de l'oscillateur 32 MHz du nRF52840 ────────────────────
 * OSC1.3 alimente directement XC1 du nRF52840. Une fois HFCLK basculée sur
 * cette source, forcer OE bas fige le CPU et l'USB (le port COM reste alors
 * visible côté Windows mais n'accepte plus aucune écriture). L'application
 * utilise USB et BLE en permanence : OSC1 doit donc rester activé.
 * Défini via l'alias "hfxo-enable" dans app.overlay.
 */
static const struct gpio_dt_spec hfxo_en =
    GPIO_DT_SPEC_GET(DT_ALIAS(hfxo_enable), gpios);
static volatile bool s_hfxo_enabled;

/* ── ADC SAADC : lecture tension VDDH/VDD ───────────────────────────────────
 * Utilise l'entrée interne VDDHDIV5 du nRF52840 : mesure VDDH/5 sans pin externe.
 * Sur ce PCB, VDDH est reliée au net VDD (~3,3 V), distinct de VBUS.
 */

static const struct adc_dt_spec vddh_adc =
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
    err = gpio_pin_configure_dt(&hfxo_en, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("gpio_pin_configure HFXO: %d", err);
        return err;
    }
    s_hfxo_enabled = true;
    k_msleep(2);
    LOG_INF("OSC1 32 MHz nRF activé en permanence (P0.08=HIGH)");

    /* ADC VDDH/VDD */
    if (!adc_is_ready_dt(&vddh_adc)) {
        LOG_ERR("ADC VDDH non prêt");
        return -ENODEV;
    }
    err = adc_channel_setup_dt(&vddh_adc);
    if (err) {
        LOG_ERR("adc_channel_setup VDDH: %d", err);
        return err;
    }
    (void)adc_sequence_init_dt(&vddh_adc, &adc_seq);

    LOG_INF("Power manager initialisé");
    return 0;
}

/* ── Contrôle HFXO ───────────────────────────────────────────────────────── */

void power_mgr_hfxo_enable(void)
{
    int err = gpio_pin_set_dt(&hfxo_en, 1);
    if (err != 0) {
        s_hfxo_enabled = false;
        LOG_ERR("OSC1 OE=HIGH: %d", err);
        return;
    }
    s_hfxo_enabled = true;
    /* Marge de démarrage du module oscillateur actif avant un nouvel usage. */
    k_msleep(2);
}

bool power_mgr_hfxo_enabled(void)
{
    return s_hfxo_enabled;
}

/* ── Détection et lecture VBUS ───────────────────────────────────────────── */

bool power_mgr_vbus_present(void)
{
    /* Registre POWER du nRF52840 : bit VBUSDET — sans ADC, sans diviseur */
    return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
}

int32_t power_mgr_vddh_read_mv(void)
{
    int err = adc_read_dt(&vddh_adc, &adc_seq);
    if (err) {
        LOG_ERR("adc_read VDDH: %d", err);
        return err;
    }

    /* Tension sur l'entrée ADC (mV) — référence interne 0.6 V, gain 1/6 → pleine échelle 3.6 V */
    int32_t val_mv = (int32_t)adc_raw;
    err = adc_raw_to_millivolts_dt(&vddh_adc, &val_mv);
    if (err) {
        LOG_ERR("adc_raw_to_millivolts: %d", err);
        return err;
    }

    /* VDDHDIV5 = VDDH / 5 → VDDH = Vadc × 5 */
    val_mv = val_mv * 5;

    LOG_INF("VDDH/VDD: %d mV (VBUS présent: %s)",
            val_mv, power_mgr_vbus_present() ? "oui" : "non");
    return val_mv;
}

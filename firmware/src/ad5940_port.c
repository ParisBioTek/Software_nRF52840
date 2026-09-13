#include "ad5940_port.h"
#include "ad5940.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_gpio.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(ad5940_port, LOG_LEVEL_INF);

/* ── GPIO : CS (P0.17), RST (P0.20), load switch (P1.00) ───────────────── */
static const struct gpio_dt_spec cs_pin  = GPIO_DT_SPEC_GET(DT_ALIAS(ad5940_cs),  gpios);
static const struct gpio_dt_spec rst_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ad5940_rst), gpios);
static const struct gpio_dt_spec pwr_pin = GPIO_DT_SPEC_GET(DT_ALIAS(sensor_power), gpios);

/* ── Bus SPI1 ────────────────────────────────────────────────────────────── */
static const struct device *const spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));

/* Le pilote nrfx met en cache l'ADRESSE de spi_config. Quatre objets distincts
 * sont donc indispensables pour que le balayage reconfigure vraiment CPOL/CPHA. */
#define AD5940_SPI_BASE_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
static const struct spi_config s_spi_configs[4] = {
    { .frequency = 1000000U, .operation = AD5940_SPI_BASE_OP,
      .slave = 0 },
    { .frequency = 1000000U, .operation = AD5940_SPI_BASE_OP | SPI_MODE_CPHA,
      .slave = 0 },
    { .frequency = 1000000U, .operation = AD5940_SPI_BASE_OP | SPI_MODE_CPOL,
      .slave = 0 },
    { .frequency = 1000000U,
      .operation = AD5940_SPI_BASE_OP | SPI_MODE_CPOL | SPI_MODE_CPHA,
      .slave = 0 },
};
static const struct spi_config *s_spi_cfg = &s_spi_configs[0];

/* Ces valeurs ne sont écrites que par le thread de mesure. Le handler SMP ne
 * fait que les lire pour construire STATUS. */
static volatile uint32_t s_spi_error_count;
static volatile int32_t  s_spi_last_error;
static volatile int32_t  s_platform_last_error;
static volatile uint32_t s_platform_adiid;
static volatile uint32_t s_platform_chipid;
static volatile int32_t  s_spi_selected_mode = -1;
static volatile uint32_t s_spi_mode_ids01 = UINT32_MAX;
static volatile uint32_t s_spi_mode_ids23 = UINT32_MAX;
static volatile uint32_t s_spi_mode_tried_mask;
static volatile int32_t  s_gpio_last_error;
static volatile uint32_t s_gpio_commanded_state;
static volatile uint32_t s_platform_gpio_state;
static volatile int32_t  s_lfosc_cal_error;
static volatile uint32_t s_lfosc_measured_hz;
static volatile uint32_t s_lfosc_used_hz = 32000u;
static volatile uint32_t s_lfosc_fallback_used;

static int spi_power_action(enum pm_device_action action)
{
    int rc = pm_device_action_run(spi_dev, action);
    if (rc == -EALREADY) {
        return 0;
    }
    if (rc != 0) {
        s_spi_error_count++;
        s_spi_last_error = rc;
        LOG_ERR("SPI1 action PM %s: rc=%d",
                action == PM_DEVICE_ACTION_RESUME ? "RESUME" : "SUSPEND", rc);
    }
    return rc;
}

#define GPIO_STATE_PWR_ACTIVE BIT(0)
#define GPIO_STATE_CS_ACTIVE  BIT(1)
#define GPIO_STATE_RST_ACTIVE BIT(2)
#define GPIO_STATE_PWR_RAW_HI BIT(8)
#define GPIO_STATE_CS_RAW_HI  BIT(9)
#define GPIO_STATE_RST_RAW_HI BIT(10)

static void gpio_record_error(const char *operation, int rc)
{
    if (rc != 0) {
        s_gpio_last_error = rc;
        LOG_ERR("GPIO %s: rc=%d", operation, rc);
    }
}

static void gpio_record_command(uint32_t mask, bool active, int rc)
{
    if (rc != 0) {
        return;
    }
    if (active) {
        s_gpio_commanded_state |= mask;
    } else {
        s_gpio_commanded_state &= ~mask;
    }
}

static uint32_t gpio_state_snapshot(void)
{
    uint32_t state = s_gpio_commanded_state;
    if (nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 0)) != 0u) {
        state |= GPIO_STATE_PWR_RAW_HI;
    }
    if (nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 17)) != 0u) {
        state |= GPIO_STATE_CS_RAW_HI;
    }
    if (nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 20)) != 0u) {
        state |= GPIO_STATE_RST_RAW_HI;
    }
    return state;
}

static void spi_record_mode_id(uint8_t mode, uint32_t adiid)
{
    uint32_t id16 = adiid & 0xFFFFu;
    if (mode < 4u) {
        s_spi_mode_tried_mask |= BIT(mode);
    }
    if (mode < 2u) {
        uint32_t shift = (uint32_t)mode * 16u;
        s_spi_mode_ids01 = (s_spi_mode_ids01 & ~(0xFFFFu << shift)) |
                           (id16 << shift);
    } else if (mode < 4u) {
        uint32_t shift = ((uint32_t)mode - 2u) * 16u;
        s_spi_mode_ids23 = (s_spi_mode_ids23 & ~(0xFFFFu << shift)) |
                           (id16 << shift);
    }
}

/* ════════════════════════════════════════════════════════════════════════════
   Fonctions port requises par la librairie ADI
   ════════════════════════════════════════════════════════════════════════════ */

/* CS actif bas : ACTIVE_LOW dans le DT → set(1) = broche LOW = chip sélectionné */
void AD5940_CsClr(void)
{
    int rc = gpio_pin_set_dt(&cs_pin, 1);
    gpio_record_error("CS=LOW", rc);
    gpio_record_command(GPIO_STATE_CS_ACTIVE, true, rc);
}
void AD5940_CsSet(void)
{
    int rc = gpio_pin_set_dt(&cs_pin, 0);
    gpio_record_error("CS=HIGH", rc);
    gpio_record_command(GPIO_STATE_CS_ACTIVE, false, rc);
    /* Temps CS-high minimal garanti entre deux transactions. C'est
     * particulièrement important pendant la réécriture dynamique de la SRAM
     * du séquenceur, qui enchaîne plusieurs centaines d'accès SPI. */
    k_busy_wait(1u);
}

/* RST actif bas : set(1) = broche LOW = chip en reset */
void AD5940_RstClr(void)
{
    int rc = gpio_pin_set_dt(&rst_pin, 1);
    gpio_record_error("RESET=LOW", rc);
    gpio_record_command(GPIO_STATE_RST_ACTIVE, true, rc);
}
void AD5940_RstSet(void)
{
    int rc = gpio_pin_set_dt(&rst_pin, 0);
    gpio_record_error("RESET=HIGH", rc);
    gpio_record_command(GPIO_STATE_RST_ACTIVE, false, rc);
}

/* Délai en unités de 10 µs */
void AD5940_Delay10us(uint32_t time)
{
    k_busy_wait(time * 10U);
}

/* Transfert SPI full-duplex */
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,
                            unsigned char *pRecvBuff,
                            unsigned long  length)
{
    struct spi_buf     tx_buf = { .buf = pSendBuffer, .len = length };
    struct spi_buf     rx_buf = { .buf = pRecvBuff,   .len = length };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    int err = spi_transceive(spi_dev, s_spi_cfg, &tx, &rx);
    if (err) {
        s_spi_error_count++;
        s_spi_last_error = err;
        /* Ne jamais laisser le pilote ADI interpréter comme une réponse valide
         * les octets résiduels du buffer après un transfert échoué. */
        if (pRecvBuff && length > 0u) {
            memset(pRecvBuff, 0, length);
        }
        LOG_ERR("SPI transceive: %d", err);
    }
}

/* Pas de pin d'interruption → polling via CV_Service toutes les 20 ms */
uint32_t AD5940_GetMCUIntFlag(void) { return 0; }
uint32_t AD5940_ClrMCUIntFlag(void) { return 0; }

/* ════════════════════════════════════════════════════════════════════════════
   API propre au port Zephyr
   ════════════════════════════════════════════════════════════════════════════ */

void ad5940_hw_init(void)
{
    int rc;

    s_gpio_last_error = 0;
    s_gpio_commanded_state = 0u;
    /* Rail capteur coupe : maintenir CS/RST physiquement bas pour eviter
     * l'alimentation fantome par les diodes des entrees numeriques. Comme ces
     * GPIO sont ACTIVE_LOW, OUTPUT_ACTIVE correspond au niveau LOW. */
    if (!gpio_is_ready_dt(&cs_pin)) {
        gpio_record_error("contrôleur CS non prêt", -ENODEV);
    } else {
        rc = gpio_pin_configure_dt(&cs_pin, GPIO_OUTPUT_ACTIVE);
        gpio_record_error("configuration CS", rc);
        gpio_record_command(GPIO_STATE_CS_ACTIVE, true, rc);
    }
    if (!gpio_is_ready_dt(&rst_pin)) {
        gpio_record_error("contrôleur RESET non prêt", -ENODEV);
    } else {
        rc = gpio_pin_configure_dt(&rst_pin, GPIO_OUTPUT_ACTIVE);
        gpio_record_error("configuration RESET", rc);
        gpio_record_command(GPIO_STATE_RST_ACTIVE, true, rc);
    }
    /* Alimentation éteinte par défaut */
    if (!gpio_is_ready_dt(&pwr_pin)) {
        gpio_record_error("contrôleur PWR non prêt", -ENODEV);
    } else {
        rc = gpio_pin_configure_dt(&pwr_pin, GPIO_OUTPUT_INACTIVE);
        gpio_record_error("configuration PWR", rc);
        gpio_record_command(GPIO_STATE_PWR_ACTIVE, false, rc);
    }

    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI1 non prêt");
    } else {
        /* Rail coupé : déconnecter le périphérique et appliquer le pinctrl
         * sleep (SCLK/MOSI avec pull-down) avant toute mesure. */
        (void)spi_power_action(PM_DEVICE_ACTION_SUSPEND);
    }

    LOG_INF("GPIO AD5940 configurés (rc=%d, état=0x%03X)",
            (int)s_gpio_last_error, (unsigned)gpio_state_snapshot());
}

int ad5940_power_on(void)
{
    int rc;

    /* Effacer les instantanés du run précédent avant toute étape susceptible
     * d'échouer, y compris la commande d'alimentation elle-même. */
    ad5940_diag_reset();
    s_gpio_last_error = 0;
    s_spi_cfg = &s_spi_configs[0];
    AD5940_RstClr();
    if (s_gpio_last_error != 0) {
        return s_gpio_last_error;
    }

    AD5940_CsClr();
    if (s_gpio_last_error != 0) {
        return s_gpio_last_error;
    }

    /* Garder CS/RESET bas et le bus SPI en pinctrl sleep pendant la montée des
     * rails. RESET n'est relâché que par platform_init(). */
    rc = gpio_pin_set_dt(&pwr_pin, 1);
    gpio_record_error("PWR=HIGH", rc);
    gpio_record_command(GPIO_STATE_PWR_ACTIVE, true, rc);
    if (rc != 0) {
        return rc;
    }

    k_msleep(100);   /* marge de démarrage des deux rails LT3045 */

    rc = spi_power_action(PM_DEVICE_ACTION_RESUME);
    if (rc != 0) {
        int power_off_rc = gpio_pin_set_dt(&pwr_pin, 0);
        gpio_record_error("PWR=LOW après échec SPI RESUME", power_off_rc);
        gpio_record_command(GPIO_STATE_PWR_ACTIVE, false, power_off_rc);
        return rc;
    }

    /* Le pinctrl actif remet SCLK/MOSI en mode SPI0, rails désormais stables.
     * Désélectionner le composant avant de relâcher RESET. */
    AD5940_CsSet();
    if (s_gpio_last_error != 0) {
        int gpio_err = s_gpio_last_error;
        (void)spi_power_action(PM_DEVICE_ACTION_SUSPEND);
        int power_off_rc = gpio_pin_set_dt(&pwr_pin, 0);
        gpio_record_error("PWR=LOW après échec CS", power_off_rc);
        gpio_record_command(GPIO_STATE_PWR_ACTIVE, false, power_off_rc);
        /* Conserver l'erreur CS comme cause primaire si le rollback réussit. */
        if (power_off_rc == 0) {
            s_gpio_last_error = gpio_err;
        }
        return gpio_err;
    }

    LOG_INF("Capteur AD5940 alimenté (GPIO=0x%03X)",
            (unsigned)gpio_state_snapshot());
    return 0;
}

void ad5940_power_off(void)
{
    /* RESET/CS bas, puis vrai pinctrl sleep SCLK/MOSI, avant la chute de DVDD. */
    AD5940_RstClr();
    AD5940_CsClr();
    (void)spi_power_action(PM_DEVICE_ACTION_SUSPEND);
    int rc = gpio_pin_set_dt(&pwr_pin, 0);
    gpio_record_error("PWR=LOW", rc);
    gpio_record_command(GPIO_STATE_PWR_ACTIVE, false, rc);
    s_spi_cfg = &s_spi_configs[0];
    LOG_INF("Capteur AD5940 hors tension");
}

/* Essaie les 4 modes SPI ; logue l'ADIID de chacun et ADOPTE le premier qui
 * répond 0x4144 (chip laissé initialisé dans ce mode). Retourne true si trouvé. */
static bool ad5940_find_working_mode(void)
{
    LOG_INF("--- Recherche du mode SPI (ADIID attendu 0x4144) ---");
    for (uint8_t m = 0; m < 4; m++) {
        s_spi_cfg = &s_spi_configs[m];
        AD5940_RstClr(); k_msleep(2); AD5940_RstSet(); k_msleep(5);
        AD5940_HWReset();
        AD5940_Initialize();
        uint32_t id = AD5940_ReadReg(REG_AFECON_ADIID) & 0xFFFF;
        uint32_t chipid = AD5940_ReadReg(REG_AFECON_CHIPID) & 0xFFFF;
        s_platform_adiid = id;
        s_platform_chipid = chipid;
        spi_record_mode_id(m, id);
        LOG_INF("SPI mode %u : ADIID=0x%04X CHIPID=0x%04X%s", m,
                (unsigned)id, (unsigned)chipid,
                (id == 0x4144) ? "   <== OK, adopte" : "");
        if (id == 0x4144) {
            s_spi_selected_mode = m;
            return true;   /* chip initialisé dans ce mode, configuration conservée */
        }
    }
    s_spi_cfg = &s_spi_configs[0];
    LOG_ERR("Aucun mode SPI ne donne 0x4144 → cablage/alim AD5940 a verifier");
    return false;
}

float ad5940_platform_init(void)
{
    CLKCfg_Type       clk_cfg;
    SEQCfg_Type       seq_cfg;
    FIFOCfg_Type      fifo_cfg;
    AGPIOCfg_Type     gpio_cfg;
    LFOSCMeasure_Type lfosc_meas;
    float             lfosc_freq = 32000.0f;

    s_spi_cfg = &s_spi_configs[0];

    if (!device_is_ready(spi_dev)) {
        s_spi_error_count = 1u;
        s_spi_last_error = -ENODEV;
        s_platform_last_error = -ENODEV;
        LOG_ERR("SPI1 non prêt au démarrage de la plateforme");
        return -1.0f;
    }

    /* Reset matériel */
    AD5940_RstClr();
    k_msleep(10);
    AD5940_RstSet();
    k_msleep(100);

    s_platform_gpio_state = gpio_state_snapshot();

    AD5940_HWReset();
    AD5940_Initialize();

    /* ── Test de vie SPI : l'AD5940 doit renvoyer ADIID = 0x4144 ───────────── */
    uint32_t adiid  = AD5940_ReadReg(REG_AFECON_ADIID)  & 0xFFFF;
    uint32_t chipid = AD5940_ReadReg(REG_AFECON_CHIPID) & 0xFFFF;
    s_platform_adiid = adiid;
    s_platform_chipid = chipid;
    spi_record_mode_id(0u, adiid);
    LOG_INF("AD5940 ADIID=0x%04X CHIPID=0x%04X (ADIID attendu 0x4144)",
            (unsigned)adiid, (unsigned)chipid);
    if (adiid != 0x4144) {
        /* Mode par défaut KO : on cherche et on adopte automatiquement le bon. */
        LOG_WRN("ADIID=0x%04X en mode SPI par defaut → recherche du bon mode...",
                (unsigned)adiid);
        if (!ad5940_find_working_mode()) {
            s_platform_last_error = (s_gpio_last_error != 0) ?
                                    s_gpio_last_error :
                                    ((s_spi_last_error != 0) ?
                                     s_spi_last_error : AD5940ERR_APPERROR);
            return -1.0f;   /* aucun mode ne marche : on annule (sinon blocage) */
        }
        /* Mode trouvé : la suite de l'init s'exécute dans ce mode. */
    } else {
        s_spi_selected_mode = 0;
    }

    /* Horloge système : HFOSC interne 16 MHz */
    clk_cfg.HFOSCEn        = bTRUE;
    clk_cfg.HFXTALEn       = bFALSE;
    clk_cfg.LFOSCEn        = bTRUE;
    clk_cfg.HfOSC32MHzMode = bFALSE;
    clk_cfg.SysClkSrc      = SYSCLKSRC_HFOSC;
    clk_cfg.SysClkDiv      = SYSCLKDIV_1;
    clk_cfg.ADCCLkSrc      = ADCCLKSRC_HFOSC;
    clk_cfg.ADCClkDiv      = ADCCLKDIV_1;
    AD5940Err clk_error = AD5940_CLKCfg(&clk_cfg);
    if (clk_error != AD5940ERR_OK) {
        s_platform_last_error = clk_error;
        LOG_ERR("Initialisation des horloges AD5940 echouee (rc=%d)",
                (int)clk_error);
        return -1.0f;
    }

    /* FIFO 2 kB, source SINC3 */
    fifo_cfg.FIFOEn     = bTRUE;
    fifo_cfg.FIFOMode   = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize   = FIFOSIZE_2KB;
    fifo_cfg.FIFOSrc    = FIFOSRC_SINC3;
    fifo_cfg.FIFOThresh = 4;
    AD5940_FIFOCfg(&fifo_cfg);

    /* Séquenceur 4 kB */
    seq_cfg.SeqMemSize   = SEQMEMSIZE_4KB;
    seq_cfg.SeqBreakEn   = bFALSE;
    seq_cfg.SeqIgnoreEn  = bTRUE;
    seq_cfg.SeqCntCRCClr = bTRUE;
    seq_cfg.SeqEnable    = bFALSE;
    seq_cfg.SeqWrTimer   = 0;
    AD5940_SEQCfg(&seq_cfg);

    /* Interruptions */
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    AD5940_INTCCfg(AFEINTC_0,
                   AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0,
                   bTRUE);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    /* GPIO AD5940 : GP0=INT, GP1=SLEEP, GP2=SYNC */
    gpio_cfg.FuncSet     = GP0_INT | GP1_SLEEP | GP2_SYNC;
    gpio_cfg.InputEnSet  = 0;
    gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
    gpio_cfg.OutVal      = 0;
    gpio_cfg.PullEnSet   = 0;
    AD5940_AGPIOCfg(&gpio_cfg);

    /* Mesure LFOSC réelle */
    lfosc_meas.CalDuration   = 1000.0f;
    lfosc_meas.CalSeqAddr    = 0;
    lfosc_meas.SystemClkFreq = 16000000.0f;
    s_lfosc_cal_error = (int32_t)AD5940_LFOSCMeasure(&lfosc_meas, &lfosc_freq);
    if (s_lfosc_cal_error == AD5940ERR_OK &&
        lfosc_freq >= 20000.0f && lfosc_freq <= 50000.0f) {
        s_lfosc_measured_hz = (uint32_t)(lfosc_freq + 0.5f);
        s_lfosc_used_hz = s_lfosc_measured_hz;
        s_lfosc_fallback_used = 0u;
    } else {
        /* Une mauvaise calibration ne doit pas produire des temporisations
         * absurdes. Le LFOSC interne est nominalement à 32 kHz : repli sûr et
         * anomalie visible dans STATUS. */
        s_lfosc_measured_hz = (s_lfosc_cal_error == AD5940ERR_OK &&
                               lfosc_freq > 0.0f && lfosc_freq < 1000000.0f) ?
                              (uint32_t)(lfosc_freq + 0.5f) : 0u;
        s_lfosc_used_hz = 32000u;
        s_lfosc_fallback_used = 1u;
        lfosc_freq = 32000.0f;
        LOG_WRN("Calibration LFOSC invalide (rc=%d, mesure=%u Hz) : repli 32000 Hz",
                (int)s_lfosc_cal_error, (unsigned)s_lfosc_measured_hz);
    }
    LOG_INF("LFOSC mesure=%u Hz, utilise=%u Hz, rc=%d",
            (unsigned)s_lfosc_measured_hz, (unsigned)s_lfosc_used_hz,
            (int)s_lfosc_cal_error);

    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    return lfosc_freq;
}

uint32_t ad5940_read_adiid(void)
{
    return AD5940_ReadReg(REG_AFECON_ADIID) & 0xFFFF;
}

void ad5940_diag_reset(void)
{
    s_spi_error_count = 0u;
    s_spi_last_error = 0;
    s_platform_last_error = 0;
    s_platform_adiid = 0u;
    s_platform_chipid = 0u;
    s_spi_selected_mode = -1;
    s_spi_mode_ids01 = UINT32_MAX;
    s_spi_mode_ids23 = UINT32_MAX;
    s_spi_mode_tried_mask = 0u;
    s_platform_gpio_state = 0u;
    s_lfosc_cal_error = 0;
    s_lfosc_measured_hz = 0u;
    s_lfosc_used_hz = 32000u;
    s_lfosc_fallback_used = 0u;
}

uint32_t ad5940_spi_error_count(void)
{
    return s_spi_error_count;
}

int32_t ad5940_spi_last_error(void)
{
    return s_spi_last_error;
}

int32_t ad5940_platform_last_error(void)
{
    return s_platform_last_error;
}

uint32_t ad5940_platform_adiid(void)
{
    return s_platform_adiid;
}

uint32_t ad5940_platform_chipid(void)
{
    return s_platform_chipid;
}

int32_t ad5940_spi_selected_mode(void)
{
    return s_spi_selected_mode;
}

uint32_t ad5940_spi_mode_ids01(void)
{
    return s_spi_mode_ids01;
}

uint32_t ad5940_spi_mode_ids23(void)
{
    return s_spi_mode_ids23;
}

uint32_t ad5940_spi_mode_tried_mask(void)
{
    return s_spi_mode_tried_mask;
}

int32_t ad5940_gpio_last_error(void)
{
    return s_gpio_last_error;
}

uint32_t ad5940_platform_gpio_state(void)
{
    return s_platform_gpio_state;
}

int32_t ad5940_lfosc_cal_error(void)
{
    return s_lfosc_cal_error;
}

uint32_t ad5940_lfosc_measured_hz(void)
{
    return s_lfosc_measured_hz;
}

uint32_t ad5940_lfosc_used_hz(void)
{
    return s_lfosc_used_hz;
}

uint32_t ad5940_lfosc_fallback_used(void)
{
    return s_lfosc_fallback_used;
}

#include "ad5940_port.h"
#include "ad5940.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ad5940_port, LOG_LEVEL_INF);

/* ── GPIO : CS (P0.17), RST (P0.20), load switch (P1.00) ───────────────── */
static const struct gpio_dt_spec cs_pin  = GPIO_DT_SPEC_GET(DT_ALIAS(ad5940_cs),  gpios);
static const struct gpio_dt_spec rst_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ad5940_rst), gpios);
static const struct gpio_dt_spec pwr_pin = GPIO_DT_SPEC_GET(DT_ALIAS(sensor_power), gpios);

/* ── Bus SPI1 ────────────────────────────────────────────────────────────── */
static const struct device *const spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));

/* MSB en premier, CS géré manuellement. Mode SPI ajustable (défaut : mode 0). */
static struct spi_config spi_cfg = {
    .frequency = 1000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,   /* mode 0 (CPOL=0 CPHA=0) */
    .slave     = 0,
    /* cs : laissé à zéro → pas de CS hardware, géré manuellement via GPIO */
};

/* Construit les flags d'opération SPI pour un mode 0..3 (MSB, 8 bits). */
static uint32_t spi_op_for_mode(uint8_t mode)
{
    uint32_t op = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    if (mode & 0x2) op |= SPI_MODE_CPOL;
    if (mode & 0x1) op |= SPI_MODE_CPHA;
    return op;
}

/* ════════════════════════════════════════════════════════════════════════════
   Fonctions port requises par la librairie ADI
   ════════════════════════════════════════════════════════════════════════════ */

/* CS actif bas : ACTIVE_LOW dans le DT → set(1) = broche LOW = chip sélectionné */
void AD5940_CsClr(void) { gpio_pin_set_dt(&cs_pin, 1); }
void AD5940_CsSet(void) { gpio_pin_set_dt(&cs_pin, 0); }

/* RST actif bas : set(1) = broche LOW = chip en reset */
void AD5940_RstClr(void) { gpio_pin_set_dt(&rst_pin, 1); }
void AD5940_RstSet(void) { gpio_pin_set_dt(&rst_pin, 0); }

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

    int err = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    if (err) {
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
    /* CS inactif par défaut (broche HIGH) */
    gpio_pin_configure_dt(&cs_pin,  GPIO_OUTPUT_INACTIVE);
    /* RST inactif par défaut (broche HIGH = chip pas en reset) */
    gpio_pin_configure_dt(&rst_pin, GPIO_OUTPUT_INACTIVE);
    /* Alimentation éteinte par défaut */
    gpio_pin_configure_dt(&pwr_pin, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI1 non prêt");
    }

    LOG_INF("GPIO AD5940 configurés");
}

void ad5940_power_on(void)
{
    gpio_pin_set_dt(&pwr_pin, 1);
    k_msleep(50);   /* stabilisation alimentation */
    LOG_INF("Capteur AD5940 alimenté");
}

void ad5940_power_off(void)
{
    gpio_pin_set_dt(&pwr_pin, 0);
    LOG_INF("Capteur AD5940 hors tension");
}

/* Essaie les 4 modes SPI ; logue l'ADIID de chacun et ADOPTE le premier qui
 * répond 0x4144 (chip laissé initialisé dans ce mode). Retourne true si trouvé. */
static bool ad5940_find_working_mode(void)
{
    LOG_INF("--- Recherche du mode SPI (ADIID attendu 0x4144) ---");
    for (uint8_t m = 0; m < 4; m++) {
        spi_cfg.operation = spi_op_for_mode(m);
        AD5940_RstClr(); k_msleep(2); AD5940_RstSet(); k_msleep(5);
        AD5940_HWReset();
        AD5940_Initialize();
        uint32_t id = AD5940_ReadReg(REG_AFECON_ADIID) & 0xFFFF;
        LOG_INF("SPI mode %u : ADIID=0x%04X%s", m, (unsigned)id,
                (id == 0x4144) ? "   <== OK, adopte" : "");
        if (id == 0x4144) {
            return true;   /* chip initialisé dans ce mode, spi_cfg réglé dessus */
        }
    }
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

    /* Reset matériel */
    AD5940_RstClr();
    k_msleep(10);
    AD5940_RstSet();
    k_msleep(100);

    AD5940_HWReset();
    AD5940_Initialize();

    /* ── Test de vie SPI : l'AD5940 doit renvoyer ADIID = 0x4144 ───────────── */
    uint32_t adiid  = AD5940_ReadReg(REG_AFECON_ADIID)  & 0xFFFF;
    uint32_t chipid = AD5940_ReadReg(REG_AFECON_CHIPID) & 0xFFFF;
    LOG_INF("AD5940 ADIID=0x%04X CHIPID=0x%04X (ADIID attendu 0x4144)",
            (unsigned)adiid, (unsigned)chipid);
    if (adiid != 0x4144) {
        /* Mode par défaut KO : on cherche et on adopte automatiquement le bon. */
        LOG_WRN("ADIID=0x%04X en mode SPI par defaut → recherche du bon mode...",
                (unsigned)adiid);
        if (!ad5940_find_working_mode()) {
            return -1.0f;   /* aucun mode ne marche : on annule (sinon blocage) */
        }
        /* Mode trouvé : la suite de l'init s'exécute dans ce mode. */
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
    AD5940_CLKCfg(&clk_cfg);

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
    AD5940_LFOSCMeasure(&lfosc_meas, &lfosc_freq);
    LOG_INF("LFOSC mesuré : %d Hz", (int)lfosc_freq);

    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    return lfosc_freq;
}

uint32_t ad5940_read_adiid(void)
{
    return AD5940_ReadReg(REG_AFECON_ADIID) & 0xFFFF;
}

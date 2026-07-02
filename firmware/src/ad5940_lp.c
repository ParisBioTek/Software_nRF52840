#include "ad5940_lp.h"
#include "CV.h"          /* CV_RtiaSelNearest, CV_RCAL_OHM, CV_ADC_REF_MV, CV_ADC_PGA_GAIN */
#include "RampTest.h"    /* DAC12BITVOLT_1LSB, DAC6BITVOLT_1LSB */

#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(ad5940_lp, LOG_LEVEL_INF);

#define LP_ADC_REF_MV   CV_ADC_REF_MV
#define LP_PGA          CV_ADC_PGA_GAIN
#define LP_RCAL_OHM     CV_RCAL_OHM

static float    s_rtia_mag = 0.0f;      /* RTIA calibré (ohms)          */
static uint32_t s_rtia_nom = 0;         /* RTIA nominal retenu (ohms)   */
static uint32_t s_rtia_sel = LPTIARTIA_4K;

/* ── Config de référence (identique RampTest.c) ─────────────────────────── */
static void lp_ref_cfg(void)
{
    AFERefCfg_Type aferef_cfg;
    aferef_cfg.HpBandgapEn   = bTRUE;
    aferef_cfg.Hp1V1BuffEn   = bTRUE;
    aferef_cfg.Hp1V8BuffEn   = bTRUE;
    aferef_cfg.Disc1V1Cap    = bFALSE;
    aferef_cfg.Disc1V8Cap    = bFALSE;
    aferef_cfg.Hp1V8ThemBuff = bFALSE;
    aferef_cfg.Hp1V8Ilimit   = bFALSE;
    aferef_cfg.Lp1V1BuffEn   = bFALSE;
    aferef_cfg.Lp1V8BuffEn   = bFALSE;
    aferef_cfg.LpBandgapEn   = bTRUE;
    aferef_cfg.LpRefBufEn    = bTRUE;
    aferef_cfg.LpRefBoostEn  = bFALSE;
    AD5940_REFCfgS(&aferef_cfg);
}

/* ── Boucle LP : LPTIA0 + LPDAC0 (identique RampTest.c) ─────────────────── */
static void lp_loop_cfg(void)
{
    LPLoopCfg_Type lploop_cfg;

    lploop_cfg.LpAmpCfg.LpAmpSel    = LPAMP0;
    lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_BOOST3;
    lploop_cfg.LpAmpCfg.LpPaPwrEn   = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaPwrEn  = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaRf     = LPTIARF_20K;
    lploop_cfg.LpAmpCfg.LpTiaRload  = LPTIARLOAD_SHORT;
    lploop_cfg.LpAmpCfg.LpTiaRtia   = s_rtia_sel;
    lploop_cfg.LpAmpCfg.LpTiaSW     = LPTIASW(2) | LPTIASW(4) | LPTIASW(5);

    lploop_cfg.LpDacCfg.LpdacSel      = LPDAC0;
    lploop_cfg.LpDacCfg.DacData12Bit  = 0x800;
    lploop_cfg.LpDacCfg.DacData6Bit   = 0;
    lploop_cfg.LpDacCfg.DataRst       = bFALSE;
    lploop_cfg.LpDacCfg.LpDacSW       = LPDACSW_VBIAS2LPPA | LPDACSW_VZERO2LPTIA |
                                        LPDACSW_VBIAS2PIN;
    lploop_cfg.LpDacCfg.LpDacRef      = LPDACREF_2P5;
    lploop_cfg.LpDacCfg.LpDacSrc      = LPDACSRC_MMR;
    lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lploop_cfg.LpDacCfg.PowerEn       = bTRUE;
    AD5940_LPLoopCfgS(&lploop_cfg);
}

/* ── Chaîne ADC : mux LPTIA0, PGA, filtres (identique RampTest.c) ───────── */
static void lp_dsp_cfg(void)
{
    DSPCfg_Type dsp_cfg;
    AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
    dsp_cfg.ADCBaseCfg.ADCMuxN            = ADCMUXN_LPTIA0_N;
    dsp_cfg.ADCBaseCfg.ADCMuxP            = ADCMUXP_LPTIA0_P;
    dsp_cfg.ADCBaseCfg.ADCPga             = LP_PGA;
    dsp_cfg.ADCFilterCfg.ADCSinc3Osr      = ADCSINC3OSR_2;
    dsp_cfg.ADCFilterCfg.ADCRate          = ADCRATE_800KHZ;
    dsp_cfg.ADCFilterCfg.BpSinc3          = bFALSE;
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.BpNotch          = bTRUE;    /* notch bypassé → SINC2 */
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr      = ADCSINC2OSR_1067;
    dsp_cfg.ADCFilterCfg.ADCAvgNum        = ADCAVGNUM_2;
    AD5940_DSPCfgS(&dsp_cfg);
}

/* ── Calibration RTIA (identique CV : RCAL externe = CV_RCAL_OHM) ───────── */
static void lp_rtia_cal(void)
{
    if (s_rtia_sel == LPTIARTIA_OPEN) {
        s_rtia_mag = 20000.0f;
        return;
    }
    fImpPol_Type   cal;
    LPRTIACal_Type lprtia_cal;
    AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));
    lprtia_cal.LpAmpSel     = LPAMP0;
    lprtia_cal.bPolarResult = bTRUE;
    lprtia_cal.AdcClkFreq   = 16000000.0f;
    lprtia_cal.SysClkFreq   = 16000000.0f;
    lprtia_cal.ADCSinc3Osr  = ADCSINC3OSR_4;
    lprtia_cal.ADCSinc2Osr  = ADCSINC2OSR_22;
    lprtia_cal.DftCfg.DftNum   = DFTNUM_2048;
    lprtia_cal.DftCfg.DftSrc   = DFTSRC_SINC2NOTCH;
    lprtia_cal.DftCfg.HanWinEn = bTRUE;
    lprtia_cal.fFreq        = 16000000.0f / 4 / 22 / 2048 * 3;
    lprtia_cal.fRcal        = LP_RCAL_OHM;
    lprtia_cal.LpTiaRtia    = s_rtia_sel;
    lprtia_cal.LpAmpPwrMod  = LPAMPPWR_NORM;
    lprtia_cal.bWithCtia    = bFALSE;
    AD5940_LPRtiaCal(&lprtia_cal, &cal);
    s_rtia_mag = cal.Magnitude;
}

int lp_setup(float lfosc_hz, uint32_t rtia_ohm)
{
    (void)lfosc_hz;

    s_rtia_sel = CV_RtiaSelNearest(rtia_ohm, &s_rtia_nom);

    if (AD5940_WakeUp(10) > 10) {
        LOG_ERR("LP: réveil AD5940 échoué");
        return -1;
    }
    AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);

    lp_ref_cfg();
    lp_loop_cfg();
    lp_dsp_cfg();

    /* La cal reconfigure l'AFE (DFT, switches) : on la fait puis on ré-applique
     * la config de boucle/ADC de mesure. */
    lp_rtia_cal();
    if (s_rtia_mag <= 0.0f) {
        s_rtia_mag = (float)s_rtia_nom;   /* garde-fou si cal ratée */
    }
    lp_ref_cfg();
    lp_loop_cfg();
    lp_dsp_cfg();
    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

    LOG_INF("LP setup : RTIA nominal=%u ohm, calibré=%d ohm",
            (unsigned)s_rtia_nom, (int)s_rtia_mag);
    return 0;
}

void lp_set_potential_mv(float vzero_mv, float e_mv)
{
    if (vzero_mv < 200.0f)  vzero_mv = 200.0f;
    if (vzero_mv > 2400.0f) vzero_mv = 2400.0f;

    uint32_t VzeroCode = (uint32_t)((vzero_mv - 200.0f) / DAC6BITVOLT_1LSB + 0.5f);
    if (VzeroCode > 63) VzeroCode = 63;

    int32_t rampCode  = (int32_t)lroundf(e_mv / DAC12BITVOLT_1LSB);
    int32_t VbiasCode = (int32_t)(VzeroCode * 64) + rampCode;
    if (VbiasCode < 0)    VbiasCode = 0;
    if (VbiasCode > 4095) VbiasCode = 4095;

    AD5940_WriteReg(REG_AFE_LPDACDAT0, ((uint32_t)VzeroCode << 12) | (uint32_t)VbiasCode);
}

float lp_read_current_ua(void)
{
    uint32_t code = 0;

    /* Deux conversions : la 1re « amorce » le filtre SINC2, on garde la 2de. */
    for (int k = 0; k < 2; k++) {
        AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
        AD5940_Delay10us(25);                       /* ~250 µs power-up ADC */
        AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
        AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bTRUE);

        uint32_t guard = 0;
        while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_SINC2RDY) == bFALSE) {
            if (++guard > 400000u) {                /* garde-fou anti-blocage */
                break;
            }
        }
        AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
        code = AD5940_ReadAfeResult(AFERESULT_SINC2) & 0xFFFF;
        AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH | AFECTRL_ADCPWR, bFALSE);
    }

    /* Même conversion que RampTest : courant en µA, RTIA calibré. */
    float volt = -AD5940_ADCCode2Volt(code, LP_PGA, LP_ADC_REF_MV);
    return volt / s_rtia_mag * 1e3f;
}

uint32_t lp_rtia_nominal_ohm(void)    { return s_rtia_nom; }
uint32_t lp_rtia_calibrated_ohm(void) { return (uint32_t)(s_rtia_mag + 0.5f); }

void lp_teardown(void)
{
    AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH | AFECTRL_ADCPWR, bFALSE);
}

/*!
 *****************************************************************************
 @file:    RampTest.c
 @author:  Neo Xu – Analog Devices Inc.
 @brief:   Séquences pour le test Ramp (Voltampérométrie Cyclique).
 *
 * Source original : https://github.com/analogdevicesinc/ad5940-examples
 * Adapté pour ESP-IDF / ESP32.
 *
 * Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 *****************************************************************************/

#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "RampTest.h"

/* ── Valeurs par défaut ────────────────────────────────────────────────── */
AppRAMPCfg_Type AppRAMPCfg =
{
    .bParaChanged       = bFALSE,
    .SeqStartAddr       = 0,
    .MaxSeqLen          = 0,
    .SeqStartAddrCal    = 0,
    .MaxSeqLenCal       = 0,

    .LFOSCClkFreq       = 32000.0f,
    .SysClkFreq         = 16000000.0f,
    .AdcClkFreq         = 16000000.0f,
    .RcalVal            = 10000.0f,
    .ADCRefVolt         = 1.820f,
    .bTestFinished      = bFALSE,

    /* Signal rampe (CV : -1V → +1V → -1V) */
    .RampStartVolt      = -1000.0f,
    .RampPeakVolt       = +1000.0f,
    .VzeroStart         = 1300.0f,
    .VzeroPeak          = 1300.0f,
    .StepNumber         = 800,
    .StepsPerCycle      = 800,
    .CycleCount         = 1,
    .RampDuration       = 24000,    /* 24 s */

    /* Réception */
    .SampleDelay        = 7.0f,
    .LPTIARtiaSel       = LPTIARTIA_4K,
    .LPTIARloadSel      = LPTIARLOAD_SHORT,
    .ExternalRtiaValue  = 20000.0f,
    .AdcPgaGain         = ADCPGA_1P5,
    .ADCSinc3Osr        = ADCSINC3OSR_2,
    .FifoThresh         = 480,

    /* Privé */
    .RAMPInited         = bFALSE,
    .StopRequired       = bFALSE,
    .RampState          = RAMP_STATE0,
    .bFirstDACSeq       = bTRUE,
    .bRampOneDir        = bFALSE,
};

static AppRAMPDiag_Type s_ramp_diag;

#define RAMP_SERVICE_INT_MASK  (AFEINTSRC_DATAFIFOTHRESH | \
                                AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0)
#define RAMP_FATAL_INT_MASK    (AFEINTSRC_SEQTIMEOUTERR | \
                                AFEINTSRC_CMDFIFOOF | AFEINTSRC_CMDFIFOUF | \
                                AFEINTSRC_DATAFIFOOF | AFEINTSRC_DATAFIFOUF)

/* Valeur ecrite dans SEQSLPLOCK pour autoriser l'hibernation. Le dernier
 * journal a montre cette valeur exacte (0x000A47E5) comme pretendu INTCFLAG0,
 * alors qu'INTCFLAG1 ne confirmait aucune faute. C'est une lecture SPI
 * transitoire au moment ou l'AFE se rendort, pas un SEQTIMEOUTERR reel. */
#define RAMP_SLEEP_KEY_SIGNATURE ((uint32_t)SLPKEY_UNLOCK)
#define RAMP_INTC_READ_ATTEMPTS  3u
/* A manually triggered 20-word Init sequence normally starts within a few
 * microseconds. Do not hold the measurement thread for seconds if MMR trigger
 * is unavailable on a board: apply the same register configuration by SPI. */
#define RAMP_INITSEQ_MANUAL_POLLS 50u

/* Lire les deux controleurs en rejetant explicitement la signature du
 * registre de sommeil. L'appelant a deja verrouille le sommeil ; un nouveau
 * WakeUp+LOCK entre les tentatives ferme la fenetre de course avec SEQ_SLP(). */
static AD5940Err AppRAMPReadIntcStable(uint32_t *pFlag0, uint32_t *pFlag1)
{
    if (!pFlag0 || !pFlag1) {
        return AD5940ERR_NULLP;
    }

    for (uint32_t attempt = 0; attempt < RAMP_INTC_READ_ATTEMPTS; attempt++) {
        const uint32_t flag0 = AD5940_INTCGetFlag(AFEINTC_0);
        const uint32_t flag1 = AD5940_INTCGetFlag(AFEINTC_1);

        if (flag0 != RAMP_SLEEP_KEY_SIGNATURE &&
            flag1 != RAMP_SLEEP_KEY_SIGNATURE) {
            *pFlag0 = flag0;
            *pFlag1 = flag1;
            return AD5940ERR_OK;
        }

        s_ramp_diag.last_rejected_intc0 = flag0;
        s_ramp_diag.last_rejected_intc1 = flag1;
        s_ramp_diag.stale_intc_count++;
        if (attempt + 1u < RAMP_INTC_READ_ATTEMPTS) {
            s_ramp_diag.intc_retry_count++;
            if (AD5940_WakeUp(10) > 10) {
                return AD5940ERR_WAKEUP;
            }
            AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
            AD5940_Delay10us(1);
        }
    }

    /* Ne jamais transformer une lecture perimee repetee en interruption ou
     * en nouvelle erreur de mesure. Ignorer ce poll laisse la FIFO intacte ;
     * le poll suivant la relira, tandis que stale_intc_count rend l'anomalie
     * visible. Le watchdog de progression reste le dernier garde-fou. */
    *pFlag0 = 0u;
    *pFlag1 = 0u;
    return AD5940ERR_OK;
}

/* Keep the sequencer state before cleanup, otherwise STATUS only reports the
 * zeros written while stopping a failed initialization. */
static void AppRAMPCaptureSequencerDiag(void)
{
    s_ramp_diag.seq_timeout = AD5940_SEQTimeOutRd();
    s_ramp_diag.seq_count = AD5940_ReadReg(REG_AFE_SEQCNT);
    s_ramp_diag.seq_con = AD5940_ReadReg(REG_AFE_SEQCON);
    s_ramp_diag.seq0_info = AD5940_ReadReg(REG_AFE_SEQ0INFO);
    s_ramp_diag.seq1_info = AD5940_ReadReg(REG_AFE_SEQ1INFO);
    s_ramp_diag.seq2_info = AD5940_ReadReg(REG_AFE_SEQ2INFO);
    s_ramp_diag.seq3_info = AD5940_ReadReg(REG_AFE_SEQ3INFO);
}

/*
 * Init ends with SEQ_STOP(), which clears SEQEN and normally raises ENDSEQ.
 * ENDSEQ routing can be changed by ADI calibration code, so accept completion
 * only when either controller reports ENDSEQ or the hardware proves that
 * SEQ_STOP has run. A true running sequence still reaches the timeout.
 */
static AD5940Err AppRAMPWaitForInitSequence(void)
{
    for (uint32_t guard = 0; guard < RAMP_INITSEQ_MANUAL_POLLS; guard++) {
        uint32_t intc0 = 0u;
        uint32_t intc1 = 0u;
        AD5940Err error = AppRAMPReadIntcStable(&intc0, &intc1);

        if (error != AD5940ERR_OK) {
            s_ramp_diag.last_error = error;
            return error;
        }

        s_ramp_diag.last_intc0 = intc0;
        s_ramp_diag.last_intc1 = intc1;
        s_ramp_diag.init_wait_polls = guard + 1u;
        if (((intc0 | intc1) & AFEINTSRC_ENDSEQ) != 0u) {
            return AD5940ERR_OK;
        }

        s_ramp_diag.seq_con = AD5940_ReadReg(REG_AFE_SEQCON);
        if ((s_ramp_diag.seq_con & BITM_AFE_SEQCON_SEQEN) == 0u) {
            s_ramp_diag.init_endseq_missing++;
            return AD5940ERR_OK;
        }

        AD5940_Delay10us(1);
    }

    AppRAMPCaptureSequencerDiag();
    return AD5940ERR_TIMEOUT;
}

/* ════════════════════════════════════════════════════════════════════════
   API publique
   ════════════════════════════════════════════════════════════════════════ */

AD5940Err AppRAMPGetCfg(void *pCfg)
{
    if (pCfg) {
        *(AppRAMPCfg_Type **)pCfg = &AppRAMPCfg;
        return AD5940ERR_OK;
    }
    return AD5940ERR_PARA;
}

void AppRAMPGetDiag(AppRAMPDiag_Type *pDiag)
{
    if (pDiag) {
        *pDiag = s_ramp_diag;
    }
}

void AppRAMPResetDiag(void)
{
    memset(&s_ramp_diag, 0, sizeof(s_ramp_diag));
}

AD5940Err AppRAMPCtrl(uint32_t Command, void *pPara)
{
    switch (Command)
    {
    case APPCTRL_START:
    {
        WUPTCfg_Type wupt_cfg = {0};
        const float period_ms = (AppRAMPCfg.StepNumber > 0u)
                              ? (float)AppRAMPCfg.RampDuration /
                                (float)AppRAMPCfg.StepNumber
                              : 0.0f;
        const uint32_t sample_ticks = (uint32_t)(AppRAMPCfg.LFOSCClkFreq *
                                                  AppRAMPCfg.SampleDelay / 1000.0f);
        const uint32_t ramp_ticks = (period_ms > AppRAMPCfg.SampleDelay)
                                  ? (uint32_t)(AppRAMPCfg.LFOSCClkFreq *
                                               (period_ms - AppRAMPCfg.SampleDelay) /
                                               1000.0f)
                                  : 0u;
        if (AD5940_WakeUp(10) > 10)
            return AD5940ERR_WAKEUP;
        if (AppRAMPCfg.RAMPInited == bFALSE)
            return AD5940ERR_APPERROR;
        if (AppRAMPCfg.RampState == RAMP_STOP)
            return AD5940ERR_APPERROR;
        /* Les compteurs WUPT font 20 bits et le code retire 6 ticks de marge.
         * Refuser une cadence impossible au lieu de provoquer un underflow
         * uint32_t qui bloquerait la rampe pendant plusieurs minutes. */
        if (sample_ticks <= 6u || ramp_ticks <= 6u ||
            sample_ticks > 0xFFFFFu + 6u || ramp_ticks > 0xFFFFFu + 6u) {
            return AD5940ERR_PARA;
        }

        /* Cette carte ne route pas GP0/INT vers le nRF52840 et le service se
         * fait donc par polling. Garder SEQSLPLOCK verrouille pendant tout le
         * run empeche l'AFE de s'endormir entre WakeUp() et le premier read
         * SPI. Les commandes SEQ_SLP deviennent sans effet, mais les blocs
         * ADC/DAC se terminent normalement et le WUPT continue de les lancer.
         * L'impact est uniquement une consommation superieure pendant la CV. */
        AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
        AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

        wupt_cfg.WuptEn               = bTRUE;
        wupt_cfg.WuptEndSeq           = WUPTENDSEQ_D;
        wupt_cfg.WuptOrder[0]         = SEQID_0;
        wupt_cfg.WuptOrder[1]         = SEQID_2;
        wupt_cfg.WuptOrder[2]         = SEQID_1;
        wupt_cfg.WuptOrder[3]         = SEQID_2;
        wupt_cfg.SeqxSleepTime[SEQID_2]   = 4;
        wupt_cfg.SeqxWakeupTime[SEQID_2]  = sample_ticks - 4u - 2u;
        wupt_cfg.SeqxSleepTime[SEQID_0]   = 4;
        wupt_cfg.SeqxWakeupTime[SEQID_0]  = ramp_ticks - 4u - 2u;
        wupt_cfg.SeqxSleepTime[SEQID_1]   = wupt_cfg.SeqxSleepTime[SEQID_0];
        wupt_cfg.SeqxWakeupTime[SEQID_1]  = wupt_cfg.SeqxWakeupTime[SEQID_0];
        AD5940_WUPTCfg(&wupt_cfg);
        break;
    }
    case APPCTRL_STOPNOW:
        if (AD5940_WakeUp(10) > 10)
            return AD5940ERR_WAKEUP;
        AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
        AD5940_WUPTCtrl(bFALSE);
        AD5940_WUPTCtrl(bFALSE);
        AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
        break;
    case APPCTRL_STOPSYNC:
        AppRAMPCfg.StopRequired = bTRUE;
        break;
    case APPCTRL_SHUTDOWN:
    {
        AD5940Err stop_error = AppRAMPCtrl(APPCTRL_STOPNOW, 0);
        if (stop_error != AD5940ERR_OK)
            return stop_error;
        AD5940_ShutDownS();
        break;
    }
    default:
        break;
    }
    return AD5940ERR_OK;
}

/* ════════════════════════════════════════════════════════════════════════
   Générateurs de séquences (privés)
   ════════════════════════════════════════════════════════════════════════ */

static AD5940Err AppRAMPSeqInitGen(void)
{
    AD5940Err error = AD5940ERR_OK;
    const uint32_t *pSeqCmd;
    uint32_t SeqLen;
    AFERefCfg_Type  aferef_cfg;
    LPLoopCfg_Type  lploop_cfg;
    DSPCfg_Type     dsp_cfg;

    AD5940_SEQGenCtrl(bTRUE);
    AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);

    aferef_cfg.HpBandgapEn      = bTRUE;
    aferef_cfg.Hp1V1BuffEn      = bTRUE;
    aferef_cfg.Hp1V8BuffEn      = bTRUE;
    aferef_cfg.Disc1V1Cap       = bFALSE;
    aferef_cfg.Disc1V8Cap       = bFALSE;
    aferef_cfg.Hp1V8ThemBuff    = bFALSE;
    aferef_cfg.Hp1V8Ilimit      = bFALSE;
    aferef_cfg.Lp1V1BuffEn      = bFALSE;
    aferef_cfg.Lp1V8BuffEn      = bFALSE;
    aferef_cfg.LpBandgapEn      = bTRUE;
    aferef_cfg.LpRefBufEn       = bTRUE;
    aferef_cfg.LpRefBoostEn     = bFALSE;
    AD5940_REFCfgS(&aferef_cfg);

    lploop_cfg.LpAmpCfg.LpAmpSel      = LPAMP0;
    lploop_cfg.LpAmpCfg.LpAmpPwrMod   = LPAMPPWR_BOOST3;
    lploop_cfg.LpAmpCfg.LpPaPwrEn     = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaPwrEn    = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaRf       = LPTIARF_20K;
    lploop_cfg.LpAmpCfg.LpTiaRload    = AppRAMPCfg.LPTIARloadSel;
    lploop_cfg.LpAmpCfg.LpTiaRtia     = AppRAMPCfg.LPTIARtiaSel;
    if (AppRAMPCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
        lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(9);
    else
        lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5);

    lploop_cfg.LpDacCfg.LpdacSel       = LPDAC0;
    lploop_cfg.LpDacCfg.DacData12Bit   = 0x800;
    lploop_cfg.LpDacCfg.DacData6Bit    = 0;
    lploop_cfg.LpDacCfg.DataRst        = bFALSE;
    /* Route aussi Vbias vers la broche pour visualisation oscilloscope. */
    lploop_cfg.LpDacCfg.LpDacSW        = LPDACSW_VBIAS2LPPA | LPDACSW_VZERO2LPTIA |
                                         LPDACSW_VBIAS2PIN;
    lploop_cfg.LpDacCfg.LpDacRef       = LPDACREF_2P5;
    lploop_cfg.LpDacCfg.LpDacSrc       = LPDACSRC_MMR;
    lploop_cfg.LpDacCfg.LpDacVbiasMux  = LPDACVBIAS_12BIT;
    lploop_cfg.LpDacCfg.LpDacVzeroMux  = LPDACVZERO_6BIT;
    lploop_cfg.LpDacCfg.PowerEn        = bTRUE;
    AD5940_LPLoopCfgS(&lploop_cfg);

    AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
    dsp_cfg.ADCBaseCfg.ADCMuxN          = ADCMUXN_LPTIA0_N;
    dsp_cfg.ADCBaseCfg.ADCMuxP          = ADCMUXP_LPTIA0_P;
    dsp_cfg.ADCBaseCfg.ADCPga           = AppRAMPCfg.AdcPgaGain;
    dsp_cfg.ADCFilterCfg.ADCSinc3Osr   = AppRAMPCfg.ADCSinc3Osr;
    dsp_cfg.ADCFilterCfg.ADCRate        = ADCRATE_800KHZ;
    dsp_cfg.ADCFilterCfg.BpSinc3        = bFALSE;
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.BpNotch        = bTRUE;
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr   = ADCSINC2OSR_1067;
    dsp_cfg.ADCFilterCfg.ADCAvgNum     = ADCAVGNUM_2;
    AD5940_DSPCfgS(&dsp_cfg);

    AD5940_SEQGenInsert(SEQ_STOP());
    AD5940_SEQGenCtrl(bFALSE);
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    if (error == AD5940ERR_OK) {
        AD5940_StructInit(&AppRAMPCfg.InitSeqInfo, sizeof(AppRAMPCfg.InitSeqInfo));
        if (SeqLen >= AppRAMPCfg.MaxSeqLen)
            return AD5940ERR_SEQLEN;
        AppRAMPCfg.InitSeqInfo.SeqId       = SEQID_3;
        AppRAMPCfg.InitSeqInfo.SeqRamAddr  = AppRAMPCfg.SeqStartAddr;
        AppRAMPCfg.InitSeqInfo.pSeqCmd     = pSeqCmd;
        AppRAMPCfg.InitSeqInfo.SeqLen      = SeqLen;
        AppRAMPCfg.InitSeqInfo.WriteSRAM   = bTRUE;
        AD5940_SEQInfoCfg(&AppRAMPCfg.InitSeqInfo);
    } else {
        return error;
    }
    return AD5940ERR_OK;
}

/*
 * Execute the one-shot initialization through the MCU when the AD5940 accepts
 * SPI accesses but does not advance after an MMR sequence trigger. This is the
 * same configuration generated by AppRAMPSeqInitGen(), except that the writes
 * go immediately to the AFE. The acquisition and its ping-pong DAC sequences
 * remain in SRAM and are still started by WUPT.
 */
static void AppRAMPApplyInitDirect(void)
{
    AFERefCfg_Type aferef_cfg;
    LPLoopCfg_Type lploop_cfg;
    DSPCfg_Type dsp_cfg;

    AD5940_SEQGenCtrl(bFALSE);
    AD5940_StructInit(&aferef_cfg, sizeof(aferef_cfg));
    AD5940_StructInit(&lploop_cfg, sizeof(lploop_cfg));
    AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));

    AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);

    aferef_cfg.HpBandgapEn = bTRUE;
    aferef_cfg.Hp1V1BuffEn = bTRUE;
    aferef_cfg.Hp1V8BuffEn = bTRUE;
    aferef_cfg.Disc1V1Cap = bFALSE;
    aferef_cfg.Disc1V8Cap = bFALSE;
    aferef_cfg.Hp1V8ThemBuff = bFALSE;
    aferef_cfg.Hp1V8Ilimit = bFALSE;
    aferef_cfg.Lp1V1BuffEn = bFALSE;
    aferef_cfg.Lp1V8BuffEn = bFALSE;
    aferef_cfg.LpBandgapEn = bTRUE;
    aferef_cfg.LpRefBufEn = bTRUE;
    aferef_cfg.LpRefBoostEn = bFALSE;
    AD5940_REFCfgS(&aferef_cfg);

    lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
    lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_BOOST3;
    lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
    lploop_cfg.LpAmpCfg.LpTiaRload = AppRAMPCfg.LPTIARloadSel;
    lploop_cfg.LpAmpCfg.LpTiaRtia = AppRAMPCfg.LPTIARtiaSel;
    lploop_cfg.LpAmpCfg.LpTiaSW = (AppRAMPCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
        ? (LPTIASW(2) | LPTIASW(4) | LPTIASW(5) | LPTIASW(9))
        : (LPTIASW(2) | LPTIASW(4) | LPTIASW(5));

    lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
    lploop_cfg.LpDacCfg.DacData12Bit = 0x800;
    lploop_cfg.LpDacCfg.DacData6Bit = 0;
    lploop_cfg.LpDacCfg.DataRst = bFALSE;
    lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA |
                                      LPDACSW_VZERO2LPTIA |
                                      LPDACSW_VBIAS2PIN;
    lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lploop_cfg.LpDacCfg.PowerEn = bTRUE;
    AD5940_LPLoopCfgS(&lploop_cfg);

    dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
    dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
    dsp_cfg.ADCBaseCfg.ADCPga = AppRAMPCfg.AdcPgaGain;
    dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppRAMPCfg.ADCSinc3Osr;
    dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
    dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_1067;
    dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;
    AD5940_DSPCfgS(&dsp_cfg);
}

static AD5940Err AppRAMPSeqADCCtrlGen(void)
{
    AD5940Err error = AD5940ERR_OK;
    const uint32_t *pSeqCmd;
    uint32_t SeqLen;
    uint32_t WaitClks;
    ClksCalInfo_Type clks_cal;

    clks_cal.DataCount         = 1;
    clks_cal.DataType          = DATATYPE_SINC3;
    clks_cal.ADCSinc3Osr       = AppRAMPCfg.ADCSinc3Osr;
    clks_cal.ADCSinc2Osr       = ADCSINC2OSR_1067;
    clks_cal.ADCAvgNum         = ADCAVGNUM_2;
    clks_cal.RatioSys2AdcClk   = AppRAMPCfg.SysClkFreq / AppRAMPCfg.AdcClkFreq;
    AD5940_ClksCalculate(&clks_cal, &WaitClks);

    AD5940_SEQGenCtrl(bTRUE);
    AD5940_SEQGpioCtrlS(AGPIO_Pin2);
    AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(16 * 250));
    AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV, bFALSE);
    AD5940_SEQGpioCtrlS(0);
    AD5940_EnterSleepS();
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    AD5940_SEQGenCtrl(bFALSE);

    if (error == AD5940ERR_OK) {
        AD5940_StructInit(&AppRAMPCfg.ADCSeqInfo, sizeof(AppRAMPCfg.ADCSeqInfo));
        if ((SeqLen + AppRAMPCfg.InitSeqInfo.SeqLen) >= AppRAMPCfg.MaxSeqLen)
            return AD5940ERR_SEQLEN;
        AppRAMPCfg.ADCSeqInfo.SeqId      = SEQID_2;
        AppRAMPCfg.ADCSeqInfo.SeqRamAddr = AppRAMPCfg.InitSeqInfo.SeqRamAddr + AppRAMPCfg.InitSeqInfo.SeqLen;
        AppRAMPCfg.ADCSeqInfo.pSeqCmd    = pSeqCmd;
        AppRAMPCfg.ADCSeqInfo.SeqLen     = SeqLen;
        AppRAMPCfg.ADCSeqInfo.WriteSRAM  = bTRUE;
        AD5940_SEQInfoCfg(&AppRAMPCfg.ADCSeqInfo);
    } else {
        return error;
    }
    return AD5940ERR_OK;
}

/* ── Calcul du code DAC pas à pas ─────────────────────────────────────── */
static AD5940Err RampDacRegUpdate(uint32_t *pDACData)
{
    int32_t VbiasCode;
    uint32_t VzeroCode;
    const uint32_t steps_per_cycle = AppRAMPCfg.StepsPerCycle;
    uint32_t cycle_step_pos;

    if (!pDACData || AppRAMPCfg.CycleCount == 0u || steps_per_cycle < 2u ||
        steps_per_cycle > AppRAMPCfg.StepNumber ||
        (AppRAMPCfg.StepNumber % steps_per_cycle) != 0u ||
        AppRAMPCfg.CycleCount != AppRAMPCfg.StepNumber / steps_per_cycle) {
        return AD5940ERR_PARA;
    }

    /* CurrStepPos is global over the complete run. Reset the waveform state
     * at each elementary-cycle boundary, while StepNumber still controls the
     * final SEQ_STOP generated by AppRAMPSeqDACCtrlGen(). */
    cycle_step_pos = AppRAMPCfg.CurrStepPos % steps_per_cycle;
    if (AppRAMPCfg.CurrStepPos != 0u && cycle_step_pos == 0u) {
        AppRAMPCfg.CurrRampCode  = AppRAMPCfg.RampStartVolt / DAC12BITVOLT_1LSB;
        AppRAMPCfg.CurrVzeroCode = (uint32_t)lroundf(
            (AppRAMPCfg.VzeroStart - 200.0f) / DAC6BITVOLT_1LSB);
        AppRAMPCfg.bDACCodeInc   = (AppRAMPCfg.RampPeakVolt >= AppRAMPCfg.RampStartVolt)
                                   ? bTRUE : bFALSE;
        AppRAMPCfg.RampState     = RAMP_STATE0;
    }

    if (AppRAMPCfg.bRampOneDir) {
        switch (AppRAMPCfg.RampState) {
        case RAMP_STATE0:
            AppRAMPCfg.CurrVzeroCode = (uint32_t)lroundf((AppRAMPCfg.VzeroStart - 200.0f) / DAC6BITVOLT_1LSB);
            AppRAMPCfg.RampState = RAMP_STATE1;
            break;
        case RAMP_STATE1:
            if (cycle_step_pos >= steps_per_cycle / 2) {
                AppRAMPCfg.RampState     = RAMP_STATE4;
                AppRAMPCfg.CurrVzeroCode = (uint32_t)lroundf((AppRAMPCfg.VzeroPeak - 200.0f) / DAC6BITVOLT_1LSB);
            }
            break;
        case RAMP_STATE4:
            if (AppRAMPCfg.CurrStepPos >= AppRAMPCfg.StepNumber)
                AppRAMPCfg.RampState = RAMP_STOP;
            break;
        case RAMP_STOP:
            break;
        default:
            break;
        }
    } else {
        switch (AppRAMPCfg.RampState) {
        case RAMP_STATE0:
            AppRAMPCfg.CurrVzeroCode = (uint32_t)lroundf((AppRAMPCfg.VzeroStart - 200.0f) / DAC6BITVOLT_1LSB);
            AppRAMPCfg.RampState = RAMP_STATE1;
            break;
        case RAMP_STATE1:
            if (cycle_step_pos >= steps_per_cycle / 4) {
                AppRAMPCfg.RampState     = RAMP_STATE2;
                AppRAMPCfg.CurrVzeroCode = (uint32_t)lroundf((AppRAMPCfg.VzeroPeak - 200.0f) / DAC6BITVOLT_1LSB);
            }
            break;
        case RAMP_STATE2:
            if (cycle_step_pos >= (steps_per_cycle * 2) / 4) {
                AppRAMPCfg.RampState    = RAMP_STATE3;
                AppRAMPCfg.bDACCodeInc  = AppRAMPCfg.bDACCodeInc ? bFALSE : bTRUE;
            }
            break;
        case RAMP_STATE3:
            if (cycle_step_pos >= (steps_per_cycle * 3) / 4) {
                AppRAMPCfg.RampState     = RAMP_STATE4;
                AppRAMPCfg.CurrVzeroCode = (uint32_t)lroundf((AppRAMPCfg.VzeroStart - 200.0f) / DAC6BITVOLT_1LSB);
            }
            break;
        case RAMP_STATE4:
            if (AppRAMPCfg.CurrStepPos >= AppRAMPCfg.StepNumber)
                AppRAMPCfg.RampState = RAMP_STOP;
            break;
        case RAMP_STOP:
            break;
        default:
            break;
        }
    }

    AppRAMPCfg.CurrStepPos++;
    if (AppRAMPCfg.bDACCodeInc)
        AppRAMPCfg.CurrRampCode += AppRAMPCfg.DACCodePerStep;
    else
        AppRAMPCfg.CurrRampCode -= AppRAMPCfg.DACCodePerStep;

    VzeroCode = AppRAMPCfg.CurrVzeroCode;
    if (VzeroCode > 63)   VzeroCode = 63;
    VbiasCode = (int32_t)(VzeroCode * 64) + (int32_t)lroundf(AppRAMPCfg.CurrRampCode);
    if (VbiasCode < 0) VbiasCode = 0;
    if (VbiasCode > 4095) VbiasCode = 4095;
    *pDACData = (VzeroCode << 12) | (uint32_t)VbiasCode;
    return AD5940ERR_OK;
}

/* ── Génération des séquences DAC (ping-pong buffer) ─────────────────── */
static AD5940Err AppRAMPSeqDACCtrlGen(void)
{
#define SEQLEN_ONESTEP  4L
#define CURRBLK_BLK0    0
#define CURRBLK_BLK1    1

    AD5940Err error = AD5940ERR_OK;
    uint32_t  BlockStartSRAMAddr;
    uint32_t  DACData, SRAMAddr;
    uint32_t  i;
    uint32_t  StepsThisBlock;
    BoolFlag  bIsFinalBlk;
    uint32_t  SeqCmdBuff[SEQLEN_ONESTEP];

    static BoolFlag  bCmdForSeq0 = bTRUE;
    static uint32_t  DACSeqBlk0Addr, DACSeqBlk1Addr;
    static uint32_t  StepsRemainning, StepsPerBlock, DACSeqCurrBlk;

    if (AppRAMPCfg.StepNumber < 2u || AppRAMPCfg.CycleCount == 0u ||
        AppRAMPCfg.StepsPerCycle < 2u ||
        AppRAMPCfg.StepsPerCycle > AppRAMPCfg.StepNumber ||
        (AppRAMPCfg.StepNumber % AppRAMPCfg.StepsPerCycle) != 0u ||
        AppRAMPCfg.CycleCount != AppRAMPCfg.StepNumber / AppRAMPCfg.StepsPerCycle) {
        return AD5940ERR_PARA;
    }

    if (AppRAMPCfg.bFirstDACSeq == bTRUE) {
        int32_t DACSeqLenMax;
        StepsRemainning = AppRAMPCfg.StepNumber;

        DACSeqLenMax = (int32_t)AppRAMPCfg.MaxSeqLen
                     - (int32_t)AppRAMPCfg.InitSeqInfo.SeqLen
                     - (int32_t)AppRAMPCfg.ADCSeqInfo.SeqLen;
        if (DACSeqLenMax < SEQLEN_ONESTEP * 4)
            return AD5940ERR_SEQLEN;
        DACSeqLenMax -= SEQLEN_ONESTEP * 2;
        StepsPerBlock = DACSeqLenMax / SEQLEN_ONESTEP / 2;

        DACSeqBlk0Addr = AppRAMPCfg.ADCSeqInfo.SeqRamAddr + AppRAMPCfg.ADCSeqInfo.SeqLen;
        DACSeqBlk1Addr = DACSeqBlk0Addr + StepsPerBlock * SEQLEN_ONESTEP;
        DACSeqCurrBlk  = CURRBLK_BLK0;

        s_ramp_diag.steps_per_block = StepsPerBlock;

        if (AppRAMPCfg.bRampOneDir) {
            AppRAMPCfg.DACCodePerStep = ((AppRAMPCfg.RampPeakVolt - AppRAMPCfg.RampStartVolt)
                                          / AppRAMPCfg.StepsPerCycle) / DAC12BITVOLT_1LSB;
        } else {
            AppRAMPCfg.DACCodePerStep = ((AppRAMPCfg.RampPeakVolt - AppRAMPCfg.RampStartVolt)
                                          / AppRAMPCfg.StepsPerCycle * 2) / DAC12BITVOLT_1LSB;
        }
#if ALIGIN_VOLT2LSB
        AppRAMPCfg.DACCodePerStep = (int32_t)AppRAMPCfg.DACCodePerStep;
#endif
        if (AppRAMPCfg.DACCodePerStep > 0) {
            AppRAMPCfg.bDACCodeInc = bTRUE;
        } else {
            AppRAMPCfg.DACCodePerStep = -AppRAMPCfg.DACCodePerStep;
            AppRAMPCfg.bDACCodeInc = bFALSE;
        }
        AppRAMPCfg.CurrRampCode  = AppRAMPCfg.RampStartVolt / DAC12BITVOLT_1LSB;
        AppRAMPCfg.RampState     = RAMP_STATE0;
        AppRAMPCfg.CurrStepPos   = 0;
        bCmdForSeq0              = bTRUE;
    }

    if (StepsRemainning == 0) return AD5940ERR_OK;

    bIsFinalBlk = (StepsRemainning <= StepsPerBlock) ? bTRUE : bFALSE;
    StepsThisBlock = bIsFinalBlk ? StepsRemainning : StepsPerBlock;
    StepsRemainning -= StepsThisBlock;
    s_ramp_diag.last_block_steps = StepsThisBlock;
    s_ramp_diag.steps_remaining = StepsRemainning;
    s_ramp_diag.blocks_generated++;

    BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk0Addr : DACSeqBlk1Addr;
    SRAMAddr = BlockStartSRAMAddr;

    for (i = 0; i < StepsThisBlock - 1; i++) {
        uint32_t CurrAddr = SRAMAddr;
        SRAMAddr += SEQLEN_ONESTEP;
        error = RampDacRegUpdate(&DACData);
        if (error != AD5940ERR_OK) return error;
        SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
        SeqCmdBuff[1] = SEQ_WAIT(10);
        SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                                (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) |
                                (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
        SeqCmdBuff[3] = SEQ_SLP();
        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
        bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
    }

    if (bIsFinalBlk) {
        uint32_t CurrAddr = SRAMAddr;
        SRAMAddr += SEQLEN_ONESTEP;
        error = RampDacRegUpdate(&DACData);
        if (error != AD5940ERR_OK) return error;
        SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
        SeqCmdBuff[1] = SEQ_WAIT(10);
        SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                                (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) |
                                (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
        SeqCmdBuff[3] = SEQ_SLP();
        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
        CurrAddr += SEQLEN_ONESTEP;
        /* Séquence de fin : désactive le séquenceur */
        SeqCmdBuff[0] = SEQ_NOP();
        SeqCmdBuff[1] = SEQ_NOP();
        SeqCmdBuff[2] = SEQ_NOP();
        SeqCmdBuff[3] = SEQ_STOP();
        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    } else {
        uint32_t CurrAddr = SRAMAddr;
        SRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk1Addr : DACSeqBlk0Addr;
        error = RampDacRegUpdate(&DACData);
        if (error != AD5940ERR_OK) return error;
        SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
        SeqCmdBuff[1] = SEQ_WAIT(10);
        SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                                (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) |
                                (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
        SeqCmdBuff[3] = SEQ_INT0(); /* Interruption custom → recharge du buffer */
        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
        bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
    }

    DACSeqCurrBlk = (DACSeqCurrBlk == CURRBLK_BLK0) ? CURRBLK_BLK1 : CURRBLK_BLK0;

    if (AppRAMPCfg.bFirstDACSeq) {
        AppRAMPCfg.bFirstDACSeq = bFALSE;
        if (bIsFinalBlk == bFALSE) {
            error = AppRAMPSeqDACCtrlGen();
            if (error != AD5940ERR_OK) return error;
        }
        AppRAMPCfg.DACSeqInfo.SeqId       = SEQID_0;
        AppRAMPCfg.DACSeqInfo.SeqLen      = SEQLEN_ONESTEP;
        AppRAMPCfg.DACSeqInfo.SeqRamAddr  = BlockStartSRAMAddr;
        AppRAMPCfg.DACSeqInfo.WriteSRAM   = bFALSE;
        AD5940_SEQInfoCfg(&AppRAMPCfg.DACSeqInfo);
    }
    return AD5940ERR_OK;
}

/* ── Calibration Rtia ─────────────────────────────────────────────────── */
static AD5940Err AppRAMPRtiaCal(void)
{
    fImpPol_Type  RtiaCalValue;
    LPRTIACal_Type lprtia_cal;
    AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

    lprtia_cal.LpAmpSel       = LPAMP0;
    lprtia_cal.bPolarResult   = bTRUE;
    lprtia_cal.AdcClkFreq     = AppRAMPCfg.AdcClkFreq;
    lprtia_cal.SysClkFreq     = AppRAMPCfg.SysClkFreq;
    lprtia_cal.ADCSinc3Osr    = ADCSINC3OSR_4;
    lprtia_cal.ADCSinc2Osr    = ADCSINC2OSR_22;
    lprtia_cal.DftCfg.DftNum  = DFTNUM_2048;
    lprtia_cal.DftCfg.DftSrc  = DFTSRC_SINC2NOTCH;
    lprtia_cal.DftCfg.HanWinEn = bTRUE;
    lprtia_cal.fFreq          = AppRAMPCfg.AdcClkFreq / 4 / 22 / 2048 * 3;
    lprtia_cal.fRcal          = AppRAMPCfg.RcalVal;
    lprtia_cal.LpTiaRtia      = AppRAMPCfg.LPTIARtiaSel;
    lprtia_cal.LpAmpPwrMod    = LPAMPPWR_NORM;
    lprtia_cal.bWithCtia      = bFALSE;
    AD5940Err error = AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
    if (error != AD5940ERR_OK || !isfinite(RtiaCalValue.Magnitude) ||
        RtiaCalValue.Magnitude <= 0.0f) {
        return (error != AD5940ERR_OK) ? error : AD5940ERR_APPERROR;
    }
    AppRAMPCfg.RtiaValue = RtiaCalValue;
    return AD5940ERR_OK;
}

/* ════════════════════════════════════════════════════════════════════════
   AppRAMPInit
   ════════════════════════════════════════════════════════════════════════ */
AD5940Err AppRAMPInit(uint32_t *pBuffer, uint32_t BufferSize)
{
    AD5940Err error = AD5940ERR_OK;
    FIFOCfg_Type fifo_cfg;
    SEQCfg_Type  seq_cfg;

    AppRAMPResetDiag();

    if (AD5940_WakeUp(10) > 10)
        return AD5940ERR_WAKEUP;

    seq_cfg.SeqMemSize   = SEQMEMSIZE_4KB;
    seq_cfg.SeqBreakEn   = bFALSE;
    seq_cfg.SeqIgnoreEn  = bFALSE;
    seq_cfg.SeqCntCRCClr = bTRUE;
    seq_cfg.SeqEnable    = bFALSE;
    seq_cfg.SeqWrTimer   = 0;
    AD5940_SEQCfg(&seq_cfg);

    if ((AppRAMPCfg.RAMPInited == bFALSE) || (AppRAMPCfg.bParaChanged == bTRUE)) {
        if (pBuffer == 0 || BufferSize == 0) return AD5940ERR_PARA;

        if (AppRAMPCfg.LPTIARtiaSel == LPTIARTIA_OPEN) {
            AppRAMPCfg.RtiaValue.Magnitude = AppRAMPCfg.ExternalRtiaValue;
            AppRAMPCfg.RtiaValue.Phase     = 0;
        } else {
            error = AppRAMPRtiaCal();
            if (error != AD5940ERR_OK) return error;
        }

        AppRAMPCfg.RAMPInited = bFALSE;
        AD5940_SEQGenInit(pBuffer, BufferSize);

        error = AppRAMPSeqInitGen();
        if (error != AD5940ERR_OK) return error;
        error = AppRAMPSeqADCCtrlGen();
        if (error != AD5940ERR_OK) return error;

        AppRAMPCfg.bParaChanged = bFALSE;
    }

    AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);
    fifo_cfg.FIFOEn     = bTRUE;
    fifo_cfg.FIFOSrc    = FIFOSRC_SINC3;
    fifo_cfg.FIFOThresh = AppRAMPCfg.FifoThresh;
    fifo_cfg.FIFOMode   = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize   = FIFOSIZE_2KB;
    AD5940_FIFOCfg(&fifo_cfg);

    /* Les calibrations ADI modifient temporairement les contrôleurs
     * d'interruptions. Réarmer explicitement les sources indispensables juste
     * avant de construire/démarrer le run évite une perte intermittente du
     * CUSTOMINT0 qui recharge les blocs DAC. INTC1 fournit une seconde vue des
     * mêmes drapeaux en mode polling. */
    /* Repartir de masques connus : les calibrations ADI peuvent laisser des
     * sources residuelles actives dans les deux controleurs. */
    AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_ALLINT, bFALSE);
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bFALSE);
    AD5940_INTCCfg(AFEINTC_0, RAMP_SERVICE_INT_MASK | RAMP_FATAL_INT_MASK, bTRUE);
    AD5940_INTCCfg(AFEINTC_1, RAMP_SERVICE_INT_MASK | RAMP_FATAL_INT_MASK, bTRUE);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    AppRAMPCfg.bFirstDACSeq = bTRUE;
    error = AppRAMPSeqDACCtrlGen();
    if (error != AD5940ERR_OK) return error;

    AppRAMPCfg.InitSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppRAMPCfg.InitSeqInfo);

    AD5940_SEQCtrlS(bTRUE);
    AD5940_SEQMmrTrig(AppRAMPCfg.InitSeqInfo.SeqId);
    error = AppRAMPWaitForInitSequence();
    if (error == AD5940ERR_TIMEOUT) {
        /*
         * This board answers every SPI register access but its manual MMR
         * trigger does not make SEQ3 advance (SEQCNT stays at zero).  Do not
         * fail the CV for this one-shot setup: stop the inactive sequencer
         * and apply the exact generated setup directly through SPI.  ADC/DAC
         * acquisitions remain hardware sequences launched by WUPT below.
         */
        AD5940_SEQCtrlS(bFALSE);
        AppRAMPApplyInitDirect();
        s_ramp_diag.init_direct_fallback = 1u;
    } else if (error != AD5940ERR_OK) {
        s_ramp_diag.last_error = error;
        AD5940_SEQCtrlS(bFALSE);
        return error;
    }
    /* Aucun drapeau de la sequence d'initialisation ne doit contaminer la
     * premiere lecture de service de la rampe. */
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    AppRAMPCfg.ADCSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppRAMPCfg.ADCSeqInfo);
    AppRAMPCfg.DACSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppRAMPCfg.DACSeqInfo);

    AD5940_SEQCtrlS(bFALSE);
    AD5940_WriteReg(REG_AFE_SEQCNT, 0);
    AD5940_SEQCtrlS(bTRUE);
    AD5940_ClrMCUIntFlag();

    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
    AppRAMPCfg.RAMPInited = bTRUE;
    return AD5940ERR_OK;
}

/* ════════════════════════════════════════════════════════════════════════
   ISR – appelée depuis la tâche principale quand l'interruption GP0 est active
   ════════════════════════════════════════════════════════════════════════ */
static int32_t AppRAMPRegModify(int32_t *const pData, uint32_t *pDataCount)
{
    if (AppRAMPCfg.StopRequired == bTRUE) {
        AD5940_WUPTCtrl(bFALSE);
    }
    return AD5940ERR_OK;
}

static int32_t AppRAMPDataProcess(int32_t *const pData, uint32_t *pDataCount)
{
    uint32_t i, datacount;
    datacount = *pDataCount;
    float *pOut = (float *)pData;
    float temp;
    for (i = 0; i < datacount; i++) {
        pData[i] &= 0xffff;
        temp = -AD5940_ADCCode2Volt(pData[i], AppRAMPCfg.AdcPgaGain, AppRAMPCfg.ADCRefVolt);
        /* AD5940_ADCCode2Volt renvoie des volts : V / ohm = A, puis A -> µA. */
        pOut[i] = temp / AppRAMPCfg.RtiaValue.Magnitude * 1e6f;
    }
    return 0;
}

AD5940Err AppRAMPISR(void *pBuff, uint32_t *pCount)
{
    uint32_t BuffCount = *pCount;
    uint32_t FifoCnt = 0;
    uint32_t IntFlag;
    uint32_t IntFlag0;
    uint32_t IntFlag1;
    uint32_t written = 0;
    AD5940Err error = AD5940ERR_OK;

    s_ramp_diag.service_calls++;
    if (AD5940_WakeUp(10) > 10) {
        s_ramp_diag.last_error = AD5940ERR_WAKEUP;
        return AD5940ERR_WAKEUP;
    }
    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
    *pCount  = 0;
    error = AppRAMPReadIntcStable(&IntFlag0, &IntFlag1);
    if (error != AD5940ERR_OK) {
        s_ramp_diag.last_error = error;
        goto done;
    }
    IntFlag  = IntFlag0 | IntFlag1;
    s_ramp_diag.last_intc0 = IntFlag0;
    s_ramp_diag.last_intc1 = IntFlag1;
    s_ramp_diag.latched_flags |= IntFlag;

    if (IntFlag & RAMP_FATAL_INT_MASK) {
        uint32_t verify0 = 0u;
        uint32_t verify1 = 0u;
        uint32_t confirmed;

        /* Une faute FIFO/sequenceur reste latchee jusqu'a INTCCLR. Elle doit
         * donc etre presente lors de deux lectures successives du meme
         * controleur avant d'interrompre la mesure. */
        s_ramp_diag.fatal_recheck_count++;
        AD5940_Delay10us(1);
        error = AppRAMPReadIntcStable(&verify0, &verify1);
        if (error != AD5940ERR_OK) {
            s_ramp_diag.last_error = error;
            goto done;
        }
        confirmed = ((IntFlag0 & verify0) | (IntFlag1 & verify1)) &
                    RAMP_FATAL_INT_MASK;
        if (confirmed == 0u) {
            s_ramp_diag.last_rejected_intc0 = IntFlag0;
            s_ramp_diag.last_rejected_intc1 = IntFlag1;
            s_ramp_diag.unconfirmed_fatal_count++;
            IntFlag0 = verify0;
            IntFlag1 = verify1;
            IntFlag = verify0 | verify1;
            s_ramp_diag.last_intc0 = verify0;
            s_ramp_diag.last_intc1 = verify1;
            s_ramp_diag.latched_flags |= IntFlag;
        } else {
            s_ramp_diag.last_intc0 = verify0;
            s_ramp_diag.last_intc1 = verify1;
            s_ramp_diag.latched_flags |= verify0 | verify1;
            s_ramp_diag.fatal_flags |= confirmed;
            AppRAMPCaptureSequencerDiag();
            s_ramp_diag.last_error = AD5940ERR_APPERROR;
            AD5940_INTCClrFlag(confirmed);
            error = AD5940ERR_APPERROR;
            goto done;
        }
    }

    if (IntFlag & AFEINTSRC_CUSTOMINT0) {
        s_ramp_diag.custom_irq_count++;
        /* Conserver le contexte avant d'acquitter. En cas d'échec de refill,
         * l'erreur est maintenant propagée et le run sera arrêté par l'appelant
         * au lieu de continuer avec un bloc périmé. */
        AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
        error = AppRAMPSeqDACCtrlGen();
        if (error != AD5940ERR_OK) {
            s_ramp_diag.last_error = error;
            goto done;
        }
        s_ramp_diag.refill_count++;
    }
    if (IntFlag & AFEINTSRC_DATAFIFOTHRESH) {
        FifoCnt = AD5940_FIFOGetCnt();
        s_ramp_diag.last_fifo_count = FifoCnt;
        if (FifoCnt > s_ramp_diag.max_fifo_count) {
            s_ramp_diag.max_fifo_count = FifoCnt;
        }
        if (FifoCnt > BuffCount - written) {
            /* Buffer trop petit – on lit ce qu'on peut */
            FifoCnt = BuffCount - written;
        }
        AD5940_FIFORd((uint32_t *)pBuff + written, FifoCnt);
        AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
        AppRAMPRegModify((int32_t *)pBuff + written, &FifoCnt);
        AppRAMPDataProcess((int32_t *)pBuff + written, &FifoCnt);
        written += FifoCnt;
    }
    if (IntFlag & AFEINTSRC_ENDSEQ) {
        FifoCnt = AD5940_FIFOGetCnt();
        s_ramp_diag.last_fifo_count = FifoCnt;
        if (FifoCnt > s_ramp_diag.max_fifo_count) {
            s_ramp_diag.max_fifo_count = FifoCnt;
        }
        if (FifoCnt > BuffCount - written) {
            FifoCnt = BuffCount - written;
        }
        AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
        AD5940_FIFORd((uint32_t *)pBuff + written, FifoCnt);
        AppRAMPDataProcess((int32_t *)pBuff + written, &FifoCnt);
        written += FifoCnt;
        error = AppRAMPCtrl(APPCTRL_STOPNOW, 0);
        if (error != AD5940ERR_OK) {
            s_ramp_diag.last_error = error;
            goto done;
        }

        /* Réinitialiser pour permettre un nouveau cycle */
        AppRAMPCfg.bTestFinished = bTRUE;
        AppRAMPCfg.RampState     = RAMP_STATE0;
        AppRAMPCfg.bFirstDACSeq  = bTRUE;
        AppRAMPCfg.bDACCodeInc   = bTRUE;
    }

    /* Il n'y a pas de ligne d'interruption matérielle sur ce port :
     * AppRAMPISR() est appelé par polling. Lire aussi les quelques mots qui
     * restent dans le FIFO sous le seuil configuré. Cela rend la fin d'une CV
     * indépendante de ENDSEQ, qui peut manquer après le dernier bloc DAC
     * dynamique, et permet à CV_Service() de terminer sur le comptage exact. */
    if ((IntFlag & AFEINTSRC_ENDSEQ) == 0u && written < BuffCount) {
        FifoCnt = AD5940_FIFOGetCnt();
        s_ramp_diag.last_fifo_count = FifoCnt;
        if (FifoCnt > s_ramp_diag.max_fifo_count) {
            s_ramp_diag.max_fifo_count = FifoCnt;
        }
        if (FifoCnt > BuffCount - written) {
            FifoCnt = BuffCount - written;
        }
        if (FifoCnt > 0u) {
            AD5940_FIFORd((uint32_t *)pBuff + written, FifoCnt);
            AppRAMPRegModify((int32_t *)pBuff + written, &FifoCnt);
            AppRAMPDataProcess((int32_t *)pBuff + written, &FifoCnt);
            written += FifoCnt;
        }
    }
done:
    if (error != AD5940ERR_OK) {
        s_ramp_diag.last_error = error;
    }
    *pCount = written;
    /* Le sommeil reste verrouille pendant toute la CV en mode polling. Il est
     * de nouveau autorise seulement apres l'arret du WUPT dans STOPNOW. */
    return error;
}

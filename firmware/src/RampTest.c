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
    .ADCRefVolt         = 1820.0f,
    .bTestFinished      = bFALSE,

    /* Signal rampe (CV : -1V → +1V → -1V) */
    .RampStartVolt      = -1000.0f,
    .RampPeakVolt       = +1000.0f,
    .VzeroStart         = 1300.0f,
    .VzeroPeak          = 1300.0f,
    .StepNumber         = 800,
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

AD5940Err AppRAMPCtrl(uint32_t Command, void *pPara)
{
    switch (Command)
    {
    case APPCTRL_START:
    {
        WUPTCfg_Type wupt_cfg;
        if (AD5940_WakeUp(10) > 10)
            return AD5940ERR_WAKEUP;
        if (AppRAMPCfg.RAMPInited == bFALSE)
            return AD5940ERR_APPERROR;
        if (AppRAMPCfg.RampState == RAMP_STOP)
            return AD5940ERR_APPERROR;

        wupt_cfg.WuptEn               = bTRUE;
        wupt_cfg.WuptEndSeq           = WUPTENDSEQ_D;
        wupt_cfg.WuptOrder[0]         = SEQID_0;
        wupt_cfg.WuptOrder[1]         = SEQID_2;
        wupt_cfg.WuptOrder[2]         = SEQID_1;
        wupt_cfg.WuptOrder[3]         = SEQID_2;
        wupt_cfg.SeqxSleepTime[SEQID_2]   = 4;
        wupt_cfg.SeqxWakeupTime[SEQID_2]  =
            (uint32_t)(AppRAMPCfg.LFOSCClkFreq * AppRAMPCfg.SampleDelay / 1000.0f) - 4 - 2;
        wupt_cfg.SeqxSleepTime[SEQID_0]   = 4;
        wupt_cfg.SeqxWakeupTime[SEQID_0]  =
            (uint32_t)(AppRAMPCfg.LFOSCClkFreq *
                       (AppRAMPCfg.RampDuration / AppRAMPCfg.StepNumber - AppRAMPCfg.SampleDelay) /
                       1000.0f) - 4 - 2;
        wupt_cfg.SeqxSleepTime[SEQID_1]   = wupt_cfg.SeqxSleepTime[SEQID_0];
        wupt_cfg.SeqxWakeupTime[SEQID_1]  = wupt_cfg.SeqxWakeupTime[SEQID_0];
        AD5940_WUPTCfg(&wupt_cfg);
        break;
    }
    case APPCTRL_STOPNOW:
        if (AD5940_WakeUp(10) > 10)
            return AD5940ERR_WAKEUP;
        AD5940_WUPTCtrl(bFALSE);
        AD5940_WUPTCtrl(bFALSE);
        break;
    case APPCTRL_STOPSYNC:
        AppRAMPCfg.StopRequired = bTRUE;
        break;
    case APPCTRL_SHUTDOWN:
        AppRAMPCtrl(APPCTRL_STOPNOW, 0);
        AD5940_ShutDownS();
        break;
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
    uint32_t VbiasCode, VzeroCode;

    if (AppRAMPCfg.bRampOneDir) {
        switch (AppRAMPCfg.RampState) {
        case RAMP_STATE0:
            AppRAMPCfg.CurrVzeroCode = (uint32_t)((AppRAMPCfg.VzeroStart - 200.0f) / DAC6BITVOLT_1LSB);
            AppRAMPCfg.RampState = RAMP_STATE1;
            break;
        case RAMP_STATE1:
            if (AppRAMPCfg.CurrStepPos >= AppRAMPCfg.StepNumber / 2) {
                AppRAMPCfg.RampState     = RAMP_STATE4;
                AppRAMPCfg.CurrVzeroCode = (uint32_t)((AppRAMPCfg.VzeroPeak - 200.0f) / DAC6BITVOLT_1LSB);
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
            AppRAMPCfg.CurrVzeroCode = (uint32_t)((AppRAMPCfg.VzeroStart - 200.0f) / DAC6BITVOLT_1LSB);
            AppRAMPCfg.RampState = RAMP_STATE1;
            break;
        case RAMP_STATE1:
            if (AppRAMPCfg.CurrStepPos >= AppRAMPCfg.StepNumber / 4) {
                AppRAMPCfg.RampState     = RAMP_STATE2;
                AppRAMPCfg.CurrVzeroCode = (uint32_t)((AppRAMPCfg.VzeroPeak - 200.0f) / DAC6BITVOLT_1LSB);
            }
            break;
        case RAMP_STATE2:
            if (AppRAMPCfg.CurrStepPos >= (AppRAMPCfg.StepNumber * 2) / 4) {
                AppRAMPCfg.RampState    = RAMP_STATE3;
                AppRAMPCfg.bDACCodeInc  = AppRAMPCfg.bDACCodeInc ? bFALSE : bTRUE;
            }
            break;
        case RAMP_STATE3:
            if (AppRAMPCfg.CurrStepPos >= (AppRAMPCfg.StepNumber * 3) / 4) {
                AppRAMPCfg.RampState     = RAMP_STATE4;
                AppRAMPCfg.CurrVzeroCode = (uint32_t)((AppRAMPCfg.VzeroStart - 200.0f) / DAC6BITVOLT_1LSB);
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
    VbiasCode = (uint32_t)(VzeroCode * 64 + AppRAMPCfg.CurrRampCode);
    if (VbiasCode < (VzeroCode * 64))
        VbiasCode--;
    if (VbiasCode > 4095) VbiasCode = 4095;
    if (VzeroCode > 63)   VzeroCode = 63;
    *pDACData = (VzeroCode << 12) | VbiasCode;
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

        if (AppRAMPCfg.bRampOneDir) {
            AppRAMPCfg.DACCodePerStep = ((AppRAMPCfg.RampPeakVolt - AppRAMPCfg.RampStartVolt)
                                          / AppRAMPCfg.StepNumber) / DAC12BITVOLT_1LSB;
        } else {
            AppRAMPCfg.DACCodePerStep = ((AppRAMPCfg.RampPeakVolt - AppRAMPCfg.RampStartVolt)
                                          / AppRAMPCfg.StepNumber * 2) / DAC12BITVOLT_1LSB;
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

    BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk0Addr : DACSeqBlk1Addr;
    SRAMAddr = BlockStartSRAMAddr;

    for (i = 0; i < StepsThisBlock - 1; i++) {
        uint32_t CurrAddr = SRAMAddr;
        SRAMAddr += SEQLEN_ONESTEP;
        RampDacRegUpdate(&DACData);
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
        RampDacRegUpdate(&DACData);
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
        RampDacRegUpdate(&DACData);
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
    AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
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
            AppRAMPRtiaCal();
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

    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    AppRAMPCfg.bFirstDACSeq = bTRUE;
    error = AppRAMPSeqDACCtrlGen();
    if (error != AD5940ERR_OK) return error;

    AppRAMPCfg.InitSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppRAMPCfg.InitSeqInfo);

    AD5940_SEQCtrlS(bTRUE);
    AD5940_SEQMmrTrig(AppRAMPCfg.InitSeqInfo.SeqId);
    while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

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
        pOut[i] = temp / AppRAMPCfg.RtiaValue.Magnitude * 1e3f;  /* µA */
    }
    return 0;
}

AD5940Err AppRAMPISR(void *pBuff, uint32_t *pCount)
{
    uint32_t BuffCount = *pCount;
    uint32_t FifoCnt;
    uint32_t IntFlag;

    if (AD5940_WakeUp(10) > 10)
        return AD5940ERR_WAKEUP;
    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
    *pCount  = 0;
    IntFlag  = AD5940_INTCGetFlag(AFEINTC_0);

    if (IntFlag & AFEINTSRC_CUSTOMINT0) {
        AD5940Err error;
        AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
        error = AppRAMPSeqDACCtrlGen();
        if (error != AD5940ERR_OK) return error;
        AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    }
    if (IntFlag & AFEINTSRC_DATAFIFOTHRESH) {
        FifoCnt = AD5940_FIFOGetCnt();
        if (FifoCnt > BuffCount) {
            /* Buffer trop petit – on lit ce qu'on peut */
            FifoCnt = BuffCount;
        }
        AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
        AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
        AppRAMPRegModify(pBuff, &FifoCnt);
        AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
        AppRAMPDataProcess((int32_t *)pBuff, &FifoCnt);
        *pCount = FifoCnt;
        return 0;
    }
    if (IntFlag & AFEINTSRC_ENDSEQ) {
        FifoCnt = AD5940_FIFOGetCnt();
        AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
        AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
        AppRAMPDataProcess((int32_t *)pBuff, &FifoCnt);
        *pCount = FifoCnt;
        AppRAMPCtrl(APPCTRL_STOPNOW, 0);

        /* Réinitialiser pour permettre un nouveau cycle */
        AppRAMPCfg.bTestFinished = bTRUE;
        AppRAMPCfg.RampState     = RAMP_STATE0;
        AppRAMPCfg.bFirstDACSeq  = bTRUE;
        AppRAMPCfg.bDACCodeInc   = bTRUE;
        AppRAMPSeqDACCtrlGen();
    }
    return 0;
}

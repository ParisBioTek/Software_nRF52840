/*!
 *****************************************************************************
 @file:    RampTest.h
 @author:  Neo Xu – Analog Devices Inc.
 @brief:   Ramp Test header file (Cyclic Voltammetry).
 *
 * Source original : https://github.com/analogdevicesinc/ad5940-examples
 * Adapté pour ESP-IDF / ESP32.
 *
 * Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 *****************************************************************************/
#ifndef _RAMPTEST_H_
#define _RAMPTEST_H_

#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"

/* Ne pas modifier ces paramètres internes */
#define ALIGIN_VOLT2LSB     0               /* 1 = aligner chaque pas au LSB du DAC */
#define DAC12BITVOLT_1LSB   (2200.0f/4095)  /* mV par LSB (12 bits) */
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  /* mV par LSB (6 bits) */

/**
 * @brief Structure de configuration du test Ramp (Voltampérométrie Cyclique).
 */
typedef struct
{
    /* Paramètres communs à toutes les applications */
    BoolFlag  bParaChanged;         /**< Forcer la regénération des séquences */
    uint32_t  SeqStartAddr;         /**< Adresse de départ des séquences en SRAM AD5940 */
    uint32_t  MaxSeqLen;            /**< Longueur max des séquences */
    uint32_t  SeqStartAddrCal;      /**< Non utilisé pour Ramp */
    uint32_t  MaxSeqLenCal;         /**< Non utilisé pour Ramp */

    /* Paramètres temporels */
    float     LFOSCClkFreq;         /**< Fréquence LFOSC mesurée (Hz), typiquement ~32 kHz */
    float     SysClkFreq;           /**< Fréquence horloge système (Hz) */
    float     AdcClkFreq;           /**< Fréquence horloge ADC (Hz) */
    float     RcalVal;              /**< Valeur du Rcal en Ohm */
    float     ADCRefVolt;           /**< Tension de référence ADC réelle en V */
    BoolFlag  bTestFinished;        /**< Indicateur de fin de test */

    /* Paramètres du signal rampe (Voltampérométrie Cyclique) */
    float     RampStartVolt;        /**< Tension de départ en mV */
    float     RampPeakVolt;         /**< Tension crête (max/min) en mV */
    float     VzeroStart;           /**< Tension Vzero initiale en mV */
    float     VzeroPeak;            /**< Tension Vzero au pic en mV */
    uint32_t  StepNumber;           /**< Nombre total de pas du run (≤ 4095) */
    uint32_t  StepsPerCycle;        /**< Nombre de pas d'un cycle élémentaire */
    uint32_t  CycleCount;           /**< Nombre de cycles élémentaires */
    uint32_t  RampDuration;         /**< Durée totale du run en ms */

    /* Chemin de réception */
    float     SampleDelay;          /**< Délai entre MAJ DAC et déclenchement ADC (ms) */
    uint32_t  LPTIARtiaSel;        /**< Sélection Rtia interne (LPTIARTIA_xxx) */
    uint32_t  LPTIARloadSel;       /**< Sélection Rload (LPTIARLOAD_xxx) */
    float     ExternalRtiaValue;    /**< Valeur Rtia externe en Ohm (si LPTIARTIA_OPEN) */
    uint32_t  AdcPgaGain;           /**< Gain PGA ADC */
    uint8_t   ADCSinc3Osr;         /**< Sur-échantillonnage filtre SINC3 */

    /* Numérique */
    uint32_t  FifoThresh;           /**< Seuil FIFO */

    /* Variables privées internes */
    BoolFlag      RAMPInited;
    fImpPol_Type  RtiaValue;
    SEQInfo_Type  InitSeqInfo;
    SEQInfo_Type  ADCSeqInfo;
    BoolFlag      bFirstDACSeq;
    SEQInfo_Type  DACSeqInfo;
    uint32_t      CurrStepPos;
    float         DACCodePerStep;
    float         CurrRampCode;
    uint32_t      CurrVzeroCode;
    BoolFlag      bDACCodeInc;
    BoolFlag      StopRequired;
    enum _RampState {
        RAMP_STATE0 = 0,
        RAMP_STATE1,
        RAMP_STATE2,
        RAMP_STATE3,
        RAMP_STATE4,
        RAMP_STOP
    } RampState;
    BoolFlag  bRampOneDir;          /**< Rampe unidirectionnelle (LSV) si bTRUE */
} AppRAMPCfg_Type;

/* Instantané de diagnostic du moteur ping-pong. Il permet de distinguer une
 * perte de CUSTOMINT0, une faute FIFO et une erreur de régénération du bloc. */
typedef struct
{
    uint32_t service_calls;
    uint32_t custom_irq_count;
    uint32_t refill_count;
    uint32_t blocks_generated;
    uint32_t steps_per_block;
    uint32_t steps_remaining;
    uint32_t last_block_steps;
    uint32_t last_intc0;
    uint32_t last_intc1;
    /* Une lecture INTC0 peut exceptionnellement renvoyer la derniere valeur
     * du registre SEQSLPLOCK (0xA47E5) si l'AFE se rendort entre le reveil SPI
     * et la lecture. Ces champs rendent le rejet/retry visible dans STATUS. */
    uint32_t last_rejected_intc0;
    uint32_t last_rejected_intc1;
    uint32_t intc_retry_count;
    uint32_t stale_intc_count;
    uint32_t fatal_recheck_count;
    uint32_t unconfirmed_fatal_count;
    uint32_t latched_flags;
    uint32_t fatal_flags;
    /* Instantane des registres du sequenceur, capture uniquement lorsqu'une
     * faute fatale est confirmee par deux lectures successives. */
    uint32_t seq_timeout;
    uint32_t seq_count;
    uint32_t seq_con;
    uint32_t seq0_info;
    uint32_t seq1_info;
    uint32_t seq2_info;
    uint32_t seq3_info;
    /* SEQ_STOP terminates Init by clearing SEQEN. This remains a reliable
     * completion proof when the ENDSEQ interrupt is not observed. */
    uint32_t init_endseq_missing;
    uint32_t init_wait_polls;
    uint32_t init_direct_fallback;
    uint32_t last_fifo_count;
    uint32_t max_fifo_count;
    int32_t  last_error;
} AppRAMPDiag_Type;

/* Commandes de contrôle */
#define APPCTRL_START       0
#define APPCTRL_STOPNOW     1
#define APPCTRL_STOPSYNC    2
#define APPCTRL_SHUTDOWN    3

/* API publique */
AD5940Err AppRAMPInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppRAMPGetCfg(void *pCfg);
void      AppRAMPGetDiag(AppRAMPDiag_Type *pDiag);
void      AppRAMPResetDiag(void);
AD5940Err AppRAMPISR(void *pBuff, uint32_t *pCount);
AD5940Err AppRAMPCtrl(uint32_t Command, void *pPara);

#endif /* _RAMPTEST_H_ */

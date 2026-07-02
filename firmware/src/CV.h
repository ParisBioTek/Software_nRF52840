#ifndef _CV_H_
#define _CV_H_

#include <stdint.h>
#include "ad5940.h"

/* ═══════════════════════════════════════════════════════════════════════════
   PARAMÈTRES UTILISATEUR
   ═══════════════════════════════════════════════════════════════════════════ */
#define CV_START_VOLT_MV        -1000.0f
#define CV_PEAK_VOLT_MV         +1000.0f
#define CV_VZERO_MV              1300.0f

#define CV_STEP_NUMBER            800
#define CV_DURATION_MS          24000
#define CV_SAMPLE_DELAY_MS        7.0f

#define CV_RTIA_SEL             LPTIARTIA_4K
#define CV_ADC_PGA_GAIN         ADCPGA_1P5

#define CV_RCAL_OHM             100.0f
#define CV_ADC_REF_MV           1820.0f

/* Forçage du RTIA en ohms : si > 0, la calibration RTIA est ignorée et cette
 * valeur est utilisée pour convertir le courant. À utiliser quand la
 * calibration (qui dépend de la RCAL externe = CV_RCAL_OHM) est fausse.
 * Mettre 0 pour calibrer normalement.
 * Valeur empirique mesurée avec une résistance étalon de 10 kΩ : ~4600 Ω. */
#define CV_RTIA_OVERRIDE_OHM    0

/* ═══════════════════════════════════════════════════════════════════════════
   API PUBLIQUE
   ═══════════════════════════════════════════════════════════════════════════ */

AD5940Err CV_Init(float lfosc_hz);
AD5940Err CV_Start(void);
void      CV_Service(void);
int       CV_IsFinished(void);
AD5940Err CV_Restart(void);

/* Courant de pic mesuré lors du dernier cycle, en nanoampères (nA).
 * Valeur absolue maximale parmi tous les points du cycle. */
int32_t CV_GetPeakCurrentNa(void);

/* Résistance estimée du dernier cycle (Ω), pente V/I par moindres carrés
 * passant par l'origine. Pertinent pour tester une simple résistance.
 * Retourne 0 si pas assez de points. */
float CV_GetResistanceOhm(void);

/* Callback appelé pour CHAQUE point traité pendant le cycle CV.
 * @param index       index de l'échantillon dans le cycle (0..StepNumber)
 * @param volt_mv     tension appliquée à cet échantillon (mV)
 * @param current_ua  courant mesuré (µA)
 */
typedef void (*cv_point_cb_t)(uint32_t index, float volt_mv, float current_ua);

/* Enregistre (ou retire avec NULL) le callback par point. */
void CV_SetPointCallback(cv_point_cb_t cb);

/* ═══════════════════════════════════════════════════════════════════════════
   PARAMÈTRES DE BALAYAGE RÉGLABLES AU RUNTIME
   Initialisés aux valeurs CV_* ci-dessus ; surchargeables via CV_SetParams()
   (commande "SET" reçue sur le serial). Les gains matériels (RTIA, PGA, RCAL,
   Vref ADC) restent fixes/calibrés et ne sont PAS exposés.
   ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float    start_mv;        /* tension de départ du balayage (mV)        */
    float    peak_mv;         /* tension de sommet du balayage (mV)        */
    float    vzero_mv;        /* tension de polarisation Vzero (mV)        */
    uint32_t step_number;     /* nombre de pas (pair, >= 2)                */
    uint32_t duration_ms;     /* durée totale du cycle (ms)                */
    float    sample_delay_ms; /* délai d'échantillon par pas (ms)          */
} cv_params_t;

/* Copie les paramètres courants (réglables) dans *out. */
void CV_GetParams(cv_params_t *out);

/* Valide puis applique de nouveaux paramètres. Retourne 0 si OK, <0 si une
 * valeur est hors plage (dans ce cas les paramètres ne sont pas modifiés).
 * La prise d'effet est au prochain CV_Init() : un cycle en cours n'est pas
 * perturbé. */
int CV_SetParams(const cv_params_t *in);

/* Durée (ms) du cycle EN COURS (instantané pris au CV_Init), pour le timeout. */
uint32_t CV_GetActiveDurationMs(void);

/* ═══════════════════════════════════════════════════════════════════════════
   GAMME DE MESURE (RTIA) — réglable au runtime + autorange
   Le RTIA fixe la fenêtre de courant : I_max ≈ ~0,9 V / RTIA.
   ═══════════════════════════════════════════════════════════════════════════ */

/* Règle le RTIA (ohms) : la valeur de table la plus proche est retenue.
 * ohm == 0  → active l'autorange (choix automatique de la gamme).
 * Retourne l'ohm effectivement retenu (0 si autorange). */
int CV_SetRtia(uint32_t ohm);

/* RTIA nominal courant (ohms), ou 0 si autorange actif (pour le réglage). */
uint32_t CV_GetRtiaOhm(void);

/* RTIA nominal effectivement sélectionné (ohms), même en autorange (pour le
 * rapport après choix). */
uint32_t CV_GetActiveRtiaOhm(void);

/* 1 si l'autorange est actif, 0 sinon. */
int CV_GetAutorange(void);

/* RTIA réellement calibré au dernier CV_Init (ohms). */
uint32_t CV_GetCalibratedRtiaOhm(void);

/* Renvoie la sélection matérielle LPTIARTIA_* de la valeur de table la plus
 * proche de `ohm` ; *ohm_out (si non NULL) reçoit l'ohm nominal correspondant.
 * Utilisé par les autres méthodes (ad5940_lp) pour partager la même table RTIA. */
uint32_t CV_RtiaSelNearest(uint32_t ohm, uint32_t *ohm_out);

/* Sélection automatique de gamme : lance des balayages de sonde rapides pour
 * choisir le RTIA. À appeler après l'init AD5940 et AVANT le CV_Init du vrai
 * cycle. Ne fait rien si l'autorange n'est pas actif. */
void CV_Autorange(float lfosc_hz);

#endif /* _CV_H_ */

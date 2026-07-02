#ifndef DPV_H_
#define DPV_H_

#include <stdint.h>
#include "ad5940.h"
#include "CV.h"          /* cv_point_cb_t (callback commun aux méthodes) */

/* Voltampérométrie à impulsions différentielles (DPV) : escalier start→end par
 * pas estep ; à chaque marche, un niveau de base puis une impulsion epulse.
 * On enregistre ΔI = I(fin d'impulsion) − I(fin de base) au potentiel de base.
 * Logiciel-timé (LPDAC + lectures ADC via ad5940_lp). X = tension (mV), Y = ΔI. */
typedef struct {
    float    start_mv;     /* potentiel de départ (mV)                */
    float    end_mv;       /* potentiel de fin (mV)                   */
    float    estep_mv;     /* hauteur de marche (mV, > 0)             */
    float    epulse_mv;    /* amplitude d'impulsion (mV)              */
    uint32_t tpulse_ms;    /* durée d'impulsion (ms)                  */
    uint32_t tperiod_ms;   /* période par marche (ms, > tpulse_ms)    */
    float    vzero_mv;     /* point de polarisation Vzero (mV)        */
} dpv_params_t;

void    DPV_SetPointCallback(cv_point_cb_t cb);
void    DPV_GetParams(dpv_params_t *out);
int     DPV_SetParams(const dpv_params_t *in);   /* 0 si OK, <0 si hors plage */

/* Lance la DPV (bloquant). rtia_ohm = gamme (ohms, >0). 0 si OK. */
int     DPV_Run(float lfosc_hz, uint32_t rtia_ohm);

/* Pic de |ΔI| du dernier run (nA). */
int32_t DPV_GetPeakCurrentNa(void);

#endif /* DPV_H_ */

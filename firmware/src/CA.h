#ifndef CA_H_
#define CA_H_

#include <stdint.h>
#include "ad5940.h"
#include "CV.h"          /* cv_point_cb_t (callback commun aux méthodes) */

/* Chronoampérométrie (i–t) : potentiel fixe, courant échantillonné dans le temps.
 * Logiciel-timé (LPDAC + lectures ADC ponctuelles via ad5940_lp). X = temps (ms). */
typedef struct {
    float    pot_mv;       /* potentiel appliqué (mV, électrode de travail) */
    float    vzero_mv;     /* point de polarisation Vzero (mV)              */
    uint32_t duration_ms;  /* durée totale de la mesure (ms)                */
    uint32_t interval_ms;  /* période d'échantillonnage (ms)                */
} ca_params_t;

void    CA_SetPointCallback(cv_point_cb_t cb);
void    CA_GetParams(ca_params_t *out);
int     CA_SetParams(const ca_params_t *in);   /* 0 si OK, <0 si hors plage */

/* Lance la chronoampérométrie (bloquant). rtia_ohm = gamme (ohms, >0). 0 si OK. */
int     CA_Run(float lfosc_hz, uint32_t rtia_ohm);

/* Pic de courant absolu du dernier run (nA). */
int32_t CA_GetPeakCurrentNa(void);

#endif /* CA_H_ */

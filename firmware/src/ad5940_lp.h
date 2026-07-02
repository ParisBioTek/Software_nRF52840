#ifndef AD5940_LP_H_
#define AD5940_LP_H_

#include <stdint.h>
#include "ad5940.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Primitives « boucle basse-puissance » (LPDAC0 + LPTIA0 → CE0/RE0/SE0) pour
 * les mesures LOGICIEL-TIMÉES (chronoampérométrie, DPV). On règle le potentiel
 * directement sur le LPDAC puis on lit le courant par conversions ADC ponctuelles
 * (pas de séquenceur/FIFO). La config analogique et la conversion en courant sont
 * calquées sur RampTest.c / CV.c pour que l'échelle de courant reste identique à
 * la CV (RTIA calibré avec la même RCAL externe).
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Configure la boucle LP et calibre le RTIA. rtia_ohm = gamme demandée (ohms) ;
 * la valeur de table la plus proche est retenue. Retourne 0 si OK, <0 sinon.
 * À appeler après ad5940_platform_init() (AD5940 réveillé, horloges prêtes). */
int lp_setup(float lfosc_hz, uint32_t rtia_ohm);

/* Applique un potentiel de cellule e_mv (électrode de travail vs référence),
 * polarisé autour de vzero_mv. Écrit le registre LPDAC (12 bits Vbias / 6 bits
 * Vzero). L'appelant gère le délai d'établissement. */
void lp_set_potential_mv(float vzero_mv, float e_mv);

/* Une lecture de courant (µA) — même formule/échelle que la CV. */
float lp_read_current_ua(void);

/* Gammes du dernier lp_setup (pour le rapport). */
uint32_t lp_rtia_nominal_ohm(void);
uint32_t lp_rtia_calibrated_ohm(void);

/* Coupe l'ADC (l'alim capteur est coupée par l'appelant via ad5940_power_off). */
void lp_teardown(void);

#endif /* AD5940_LP_H_ */

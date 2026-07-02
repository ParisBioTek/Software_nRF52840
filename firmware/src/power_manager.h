#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialise le gestionnaire d'alimentation.
 *        Configure le GPIO P0.08 (contrôle HFXO) et le canal ADC VBUS.
 * @return 0 si succès, code d'erreur négatif sinon.
 */
int power_mgr_init(void);

/**
 * @brief Active le cristal 32 MHz externe (P0.08 = HIGH).
 *        À appeler avant toute opération nécessitant l'HFXO (mesure, RF dense…).
 *        Inclut un délai de stabilisation de ~500 µs.
 */
void power_mgr_hfxo_enable(void);

/**
 * @brief Éteint le cristal 32 MHz externe (P0.08 = LOW).
 *        À appeler après la mesure, avant la mise en veille du CPU.
 */
void power_mgr_hfxo_disable(void);

/**
 * @brief Détecte la présence de VBUS (USB branché ≥ ~4 V).
 *        Lecture via le registre POWER du nRF52840 — aucun ADC requis.
 * @return true si VBUS est présent.
 */
bool power_mgr_vbus_present(void);

/**
 * @brief Lit la tension VBUS en millivolts via l'ADC (SAADC AIN0 / P0.02).
 *
 *        Utilise l'entrée interne VDDHDIV5 du nRF52840 (VBUS/5).
 *        Aucun pin externe ni pont diviseur requis.
 *
 * @return Tension VBUS en mV (>0), ou valeur négative si erreur ADC.
 */
int32_t power_mgr_vbus_read_mv(void);

#endif /* POWER_MANAGER_H */

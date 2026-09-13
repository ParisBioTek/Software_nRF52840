#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialise le gestionnaire d'alimentation.
 *        Active durablement OSC1 via P0.08 et configure le canal ADC VDDH.
 * @return 0 si succès, code d'erreur négatif sinon.
 */
int power_mgr_init(void);

/**
 * @brief Garantit que l'oscillateur actif 32 MHz du nRF est alimenté
 *        (P0.08 = HIGH).
 */
void power_mgr_hfxo_enable(void);

/** @return true si la dernière commande OSC1 OE=HIGH a réussi. */
bool power_mgr_hfxo_enabled(void);

/**
 * @brief Détecte la présence de VBUS (USB branché ≥ ~4 V).
 *        Lecture via le registre POWER du nRF52840 — aucun ADC requis.
 * @return true si VBUS est présent.
 */
bool power_mgr_vbus_present(void);

/**
 * @brief Lit la tension VDDH en millivolts via le SAADC interne.
 *
 *        Utilise l'entrée interne VDDHDIV5 du nRF52840 (VDDH/5). Sur ce PCB,
 *        VDDH est reliée au rail VDD, pas directement à VBUS.
 *        Aucun pin externe ni pont diviseur requis.
 *
 * @return Tension VDDH/VDD en mV (>0), ou valeur négative si erreur ADC.
 */
int32_t power_mgr_vddh_read_mv(void);

#endif /* POWER_MANAGER_H */

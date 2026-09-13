#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <zephyr/kernel.h>
#include <stdbool.h>

/**
 * Sémaphore signalé quand le téléphone envoie CMD_REQUEST_DATA via BLE.
 * Utilisé en mode développement pour déclencher une mesure à la demande.
 */
extern struct k_sem data_request_sem;

/* Réserve une acquisition BLE seulement si aucune acquisition SMP/BLE n'est
 * déjà en attente ou en cours. */
bool measurement_request_submit(void);

/**
 * Sémaphore signalé quand la liaison BLE est authentifiée (LE SC niveau L2+).
 * Utilisé en mode produit pour démarrer le cycle de mesure autonome.
 */
extern struct k_sem paired_sem;

#endif /* APP_EVENTS_H */

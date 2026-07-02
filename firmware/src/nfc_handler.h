#ifndef NFC_HANDLER_H
#define NFC_HANDLER_H

#include <stdint.h>

/** Callback appelé quand un lecteur NFC lit le tag (field ON). */
typedef void (*nfc_tap_cb_t)(void);

/**
 * @brief Initialise le tag NFC T2T, encode le message NDEF OOB BLE
 *        et enregistre le callback de tap.
 * @param cb  Fonction appelée lors d'un tap NFC (peut être NULL).
 * @return 0 si succès, code d'erreur négatif sinon.
 */
int nfc_handler_init(nfc_tap_cb_t cb);

/**
 * @brief Démarre l'émulation NFC (appelé après bt_enable).
 * @return 0 si succès, code d'erreur négatif sinon.
 */
int nfc_handler_start(void);

/**
 * @brief Met à jour l'adresse BLE encodée dans le tag NDEF.
 *        À appeler si l'adresse change (RPA rotation).
 * @param addr  Tableau de 6 octets (little-endian).
 */
void nfc_handler_set_addr(const uint8_t addr[6]);

#endif /* NFC_HANDLER_H */
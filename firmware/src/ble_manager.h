#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

/**
 * Callback appelé lorsque la liaison BLE est authentifiée (LE SC niveau L2+).
 * Permet à main.c de mettre à jour les paramètres de connexion selon le mode.
 */
typedef void (*ble_secured_cb_t)(struct bt_conn *conn);

/**
 * @brief Initialise le BLE manager : GATT, sécurité, callbacks.
 * @return 0 si succès.
 */
int ble_manager_init(void);

/**
 * @brief Enregistre un callback appelé après authentification BLE réussie.
 *        À appeler avant ble_manager_init.
 * @param cb  Fonction appelée avec la connexion sécurisée en argument.
 */
void ble_manager_set_secured_cb(ble_secured_cb_t cb);

/**
 * @brief Démarre l'advertising BLE connectable.
 *        Appelé depuis le callback NFC tap.
 * @return 0 si succès.
 */
int ble_manager_start_adv(void);

/**
 * @brief Stoppe l'advertising BLE.
 */
void ble_manager_stop_adv(void);

/**
 * @brief Déverrouille l'accès aux données (appelé après authentification BLE réussie).
 */
void ble_manager_unlock(void);

/**
 * @brief Retourne une référence à la connexion BLE active, ou NULL si non connecté.
 *        L'appelant doit appeler bt_conn_unref() après usage.
 */
struct bt_conn *ble_manager_get_conn(void);

#endif /* BLE_MANAGER_H */

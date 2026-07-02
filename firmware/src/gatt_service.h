#ifndef GATT_SERVICE_H
#define GATT_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

/* ── UUIDs custom 128 bits ───────────────────────────────────────────────── */
/*
 * Générés une fois, fixes dans le firmware ET dans l'app Android.
 * Format : BT_UUID_128_ENCODE(u32, u16, u16, u16, u48)
 */
#define BT_UUID_CUSTOM_SVC_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_CUSTOM_NOTIF_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_CUSTOM_WRITE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BT_UUID_CUSTOM_SVC   BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SVC_VAL)
#define BT_UUID_CUSTOM_NOTIF BT_UUID_DECLARE_128(BT_UUID_CUSTOM_NOTIF_VAL)
#define BT_UUID_CUSTOM_WRITE BT_UUID_DECLARE_128(BT_UUID_CUSTOM_WRITE_VAL)

/* ── Opcodes commandes BLE → wearable ────────────────────────────────────── */
#define CMD_REQUEST_DATA  0x01
#define CMD_ADD_DOCTOR    0x02
#define CMD_REMOVE_DOCTOR 0x03
#define CMD_RESET         0xFF

/* ── API publique ────────────────────────────────────────────────────────── */

/**
 * @brief Initialise le service GATT custom.
 * @return 0 si succès.
 */
int gatt_service_init(void);

/**
 * @brief Active ou désactive l'accès aux données chiffrées.
 *        Appelé depuis ble_manager après pairing réussi.
 * @param unlocked  true = accès autorisé.
 */
void gatt_service_set_unlocked(bool unlocked);

/**
 * @brief Envoie une notification brute sur la caractéristique NOTIF.
 * @param conn  Connexion BLE cible.
 * @param data  Données à envoyer.
 * @param len   Longueur en octets.
 * @return 0 si succès.
 */
int gatt_service_notify(struct bt_conn *conn,
                        const uint8_t *data, uint16_t len);

/**
 * @brief Envoie une série de mesures (timestamp + valeur float) par notifications.
 * @param conn        Connexion BLE cible.
 * @param timestamps  Tableau de timestamps (ms epoch).
 * @param values      Tableau de valeurs float.
 * @param count       Nombre de points.
 * @return 0 si succès.
 */
int gatt_service_send_burst(struct bt_conn *conn,
                             const int64_t *timestamps,
                             const float   *values,
                             uint16_t       count);

#endif /* GATT_SERVICE_H */
#include "gatt_service.h"

#include "app_events.h" 
#include <zephyr/kernel.h>
#include <zephyr/types.h>                   /* ssize_t */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>          /* BT_GATT_*, bt_gatt_notify, BT_GATT_CCC_NOTIFY */
#include <zephyr/bluetooth/att.h>           /* BT_ATT_ERR_* */
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(gatt_service, LOG_LEVEL_INF);

/* ── État interne ────────────────────────────────────────────────────────── */
#define MAX_DOCTORS 8
static char    authorized_doctors[MAX_DOCTORS][7];
static uint8_t doctor_count = 0;
static bool    unlocked     = false;

static uint8_t notif_buf[13];

/* ── Handlers GATT ───────────────────────────────────────────────────────── */

static ssize_t read_notif(struct bt_conn *c, const struct bt_gatt_attr *a,
                           void *buf, uint16_t len, uint16_t offset)
{
    if (!unlocked) {
        return BT_GATT_ERR(BT_ATT_ERR_READ_NOT_PERMITTED);
    }
    return bt_gatt_attr_read(c, a, buf, len, offset,
                             notif_buf, sizeof(notif_buf));
}

static void ccc_changed(const struct bt_gatt_attr *a, uint16_t val)
{
    ARG_UNUSED(a);
    LOG_INF("CCC: notifications %s",
            val == BT_GATT_CCC_NOTIFY ? "ON" : "OFF");
}

static ssize_t write_cmd(struct bt_conn *c, const struct bt_gatt_attr *a,
                          const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(c);
    ARG_UNUSED(a);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    const uint8_t *data = buf;

    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    switch (data[0]) {

    case CMD_REQUEST_DATA:
        LOG_INF("CMD: REQUEST_DATA");
        k_sem_give(&data_request_sem);
        break;

    case CMD_ADD_DOCTOR:
        if (len >= 7 && doctor_count < MAX_DOCTORS) {
            memcpy(authorized_doctors[doctor_count], &data[1], 6);
            authorized_doctors[doctor_count][6] = '\0';
            LOG_INF("Médecin ajouté: %s", authorized_doctors[doctor_count]);
            doctor_count++;
        } else {
            LOG_WRN("ADD_DOCTOR: liste pleine ou trame trop courte");
        }
        break;

    case CMD_REMOVE_DOCTOR: {
        if (len < 7) {
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
        char code[7];
        memcpy(code, &data[1], 6);
        code[6] = '\0';
        for (int i = 0; i < doctor_count; i++) {
            if (strncmp(authorized_doctors[i], code, 6) == 0) {
                memmove(&authorized_doctors[i],
                        &authorized_doctors[i + 1],
                        (doctor_count - i - 1) * sizeof(authorized_doctors[0]));
                doctor_count--;
                LOG_INF("Médecin retiré: %s", code);
                break;
            }
        }
        break;
    }

    case CMD_RESET:
        LOG_INF("CMD: RESET");
        doctor_count = 0;
        memset(authorized_doctors, 0, sizeof(authorized_doctors));
        break;

    default:
        LOG_WRN("CMD inconnue: 0x%02x", data[0]);
        break;
    }

    return (ssize_t)len;
}

/* ── Définition du service GATT ──────────────────────────────────────────── */
/*
 * BT_GATT_PERM_READ_ENCRYPT / WRITE_ENCRYPT :
 * la stack refuse l'accès si la session n'est pas chiffrée (LE SC).
 */
BT_GATT_SERVICE_DEFINE(custom_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SVC),

    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_NOTIF,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        read_notif, NULL, notif_buf),

    BT_GATT_CCC(ccc_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_WRITE,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE_ENCRYPT,
        NULL, write_cmd, NULL),
);

/* ── API publique ────────────────────────────────────────────────────────── */

int gatt_service_init(void)
{
    memset(notif_buf, 0, sizeof(notif_buf));
    unlocked     = false;
    doctor_count = 0;
    LOG_INF("Service GATT initialisé");
    return 0;
}

void gatt_service_set_unlocked(bool state)
{
    unlocked = state;
    LOG_INF("GATT unlocked: %d", state);
}

int gatt_service_notify(struct bt_conn *conn,
                        const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &custom_svc.attrs[1];
    return bt_gatt_notify(conn, attr, data, len);
}

int gatt_service_send_burst(struct bt_conn *conn,
                             const int64_t *timestamps,
                             const float   *values,
                             uint16_t       count)
{
    const struct bt_gatt_attr *attr = &custom_svc.attrs[1];
    uint8_t pkt[13];
    int err = 0;

    for (uint16_t i = 0; i < count; i++) {
        pkt[0] = 0x10;
        sys_put_be64((uint64_t)timestamps[i], &pkt[1]);

        union { float f; uint32_t u; } conv = { .f = values[i] };
        sys_put_be32(conv.u, &pkt[9]);

        err = bt_gatt_notify(conn, attr, pkt, sizeof(pkt));
        if (err) {
            LOG_WRN("notify err %d idx=%u", err, i);
            return err;
        }
        k_msleep(10);
    }

    /* Trame de fin de burst */
    uint8_t end = 0xFF;
    return bt_gatt_notify(conn, attr, &end, sizeof(end));
}
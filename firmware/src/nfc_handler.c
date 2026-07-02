#include "nfc_handler.h"
#include <nfc/ndef/msg.h>
#include <nfc/ndef/le_oob_rec.h>
#include <nfc_t2t_lib.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(nfc_handler, LOG_LEVEL_INF);

/* ── État interne ────────────────────────────────────────────────────────── */

static uint8_t       ndef_msg_buf[256];
static nfc_tap_cb_t  tap_callback;
static bt_addr_le_t  oob_addr;

static struct nfc_ndef_le_oob_rec_payload_desc rec_payload;

/* ── Callback NFC T2T ────────────────────────────────────────────────────── */

static void nfc_callback(void *context, nfc_t2t_event_t event,
                         const uint8_t *data, size_t data_length)
{
    ARG_UNUSED(context);
    ARG_UNUSED(data);
    ARG_UNUSED(data_length);

    switch (event) {
    case NFC_T2T_EVENT_FIELD_ON:
        LOG_INF("NFC field ON");
        if (tap_callback) {
            tap_callback();
        }
        break;
    case NFC_T2T_EVENT_FIELD_OFF:
        LOG_INF("NFC field OFF");
        break;
    default:
        break;
    }
}

/* ── Encodage interne ────────────────────────────────────────────────────── */

/* Déclarations static au niveau fichier — une seule fois */
NFC_NDEF_LE_OOB_RECORD_DESC_DEF(oob_rec, 0, &rec_payload);
NFC_NDEF_MSG_DEF(nfc_le_oob_msg, 1);

static int encode_and_set_payload(void)
{
    struct bt_le_oob oob_local;
    int err = bt_le_oob_get_local(BT_ID_DEFAULT, &oob_local);
    if (err) {
        LOG_ERR("bt_le_oob_get_local error: %d", err);
        return err;
    }

    struct nfc_ndef_le_oob_rec_payload_desc rec_payload;
    memset(&rec_payload, 0, sizeof(rec_payload));

    /* Tag minimal : seules l'adresse et le rôle (obligatoire) sont nécessaires.
     * L'app n'exploite que l'adresse (structure AD 0x1B) et le pairing se fait
     * en LE SC "Just Works" (aucun callback OOB côté firmware) → inutile
     * d'embarquer le_sc_data / nom / appearance. Un tag plus petit se lit
     * nettement plus vite en T2T. */
    rec_payload.addr       = &oob_local.addr;
    rec_payload.le_role    = NFC_NDEF_LE_OOB_REC_LE_ROLE(
                                 NFC_NDEF_LE_OOB_REC_LE_ROLE_PERIPH_ONLY);
    rec_payload.flags      = NFC_NDEF_LE_OOB_REC_FLAGS(BT_LE_AD_NO_BREDR);

    NFC_NDEF_MSG_DEF(nfc_le_oob_msg, 1);
    NFC_NDEF_LE_OOB_RECORD_DESC_DEF(nfc_le_oob_rec, 0, &rec_payload);

    err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_le_oob_msg),
                                  &NFC_NDEF_LE_OOB_RECORD_DESC(nfc_le_oob_rec));
    if (err) {
        LOG_ERR("record_add error: %d", err);
        return err;
    }

    uint32_t len = sizeof(ndef_msg_buf);
    err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_le_oob_msg),
                              ndef_msg_buf, &len);
    if (err) {
        LOG_ERR("msg_encode error: %d", err);
        return err;
    }

    err = nfc_t2t_payload_set(ndef_msg_buf, len);
    if (err) {
        LOG_ERR("payload_set error: %d", err);
    }
    return err;
}
/* ── API publique ────────────────────────────────────────────────────────── */

int nfc_handler_init(nfc_tap_cb_t cb)
{
    tap_callback = cb;

    int err = nfc_t2t_setup(nfc_callback, NULL);
    if (err) {
        LOG_ERR("nfc_t2t_setup error: %d", err);
        return err;
    }

    LOG_INF("NFC handler initialized");
    return 0;
}

int nfc_handler_start(void)
{
    struct bt_le_oob oob;
    int err = bt_le_oob_get_local(BT_ID_DEFAULT, &oob);
    if (err) {
        LOG_ERR("bt_le_oob_get_local error: %d", err);
        return err;
    }
    memcpy(&oob_addr, &oob.addr, sizeof(oob_addr));

    err = encode_and_set_payload();
    if (err) {
        return err;
    }

    err = nfc_t2t_emulation_start();
    if (err) {
        LOG_ERR("nfc_t2t_emulation_start error: %d", err);
        return err;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&oob_addr, addr_str, sizeof(addr_str));
    LOG_INF("NFC OOB tag started — adresse publiée: %s", addr_str);
    return 0;
}

void nfc_handler_set_addr(const uint8_t addr[6])
{
    oob_addr.type = BT_ADDR_LE_RANDOM;
    memcpy(oob_addr.a.val, addr, 6);

    int err = encode_and_set_payload();
    if (err) {
        LOG_ERR("nfc_handler_set_addr re-encode error: %d", err);
    }
}
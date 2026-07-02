#include "ble_manager.h"
#include "gatt_service.h"
#include "app_events.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_manager, LOG_LEVEL_INF);

/* ── Paramètres d'advertising ────────────────────────────────────────────── */
#define ADV_PARAM \
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY, \
                    BT_GAP_ADV_FAST_INT_MIN_2, \
                    BT_GAP_ADV_FAST_INT_MAX_2, \
                    NULL)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ── État interne ────────────────────────────────────────────────────────── */
static struct bt_conn    *current_conn;
static ble_secured_cb_t   secured_cb;

/* ── Relance robuste de l'advertising ────────────────────────────────────────
 * Différée sur la workqueue système (et pas appelée directement dans les
 * callbacks BT) + retry : sinon, si bt_le_adv_start échoue ponctuellement
 * (déconnexion encore en cours de traitement par le contrôleur), le device
 * reste muet jusqu'au reboot — d'où "il faut reflasher pour reconnecter".
 */
static struct k_work_delayable adv_work;

static void adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (current_conn) {
        return;  /* déjà connecté : ne pas advertiser */
    }

    int err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == -EALREADY) {
        return;
    }
    if (err) {
        LOG_WRN("adv_start échoué (%d) — nouvel essai dans 200 ms", err);
        k_work_reschedule(&adv_work, K_MSEC(200));
        return;
    }
    LOG_INF("Advertising (re)démarré");
}

static void restart_adv(void)
{
    k_work_reschedule(&adv_work, K_NO_WAIT);
}

/* ── Callbacks de connexion ──────────────────────────────────────────────── */

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (err) {
        /* Échec d'établissement : le contrôleur a déjà stoppé l'advertising
         * ET disconnected() ne sera PAS appelé. On DOIT relancer l'adv,
         * sinon le device n'est plus jamais joignable. */
        LOG_ERR("Connexion échouée avec %s: err %d → relance advertising", addr_str, err);
        restart_adv();
        return;
    }
    current_conn = bt_conn_ref(conn);
    LOG_INF("BLE connecté à %s", addr_str);
    bt_le_adv_stop();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE déconnecté, raison: %d → relance advertising", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    gatt_service_set_unlocked(false);

    restart_adv();
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
                              enum bt_security_err err)
{
    if (err) {
        LOG_ERR("Changement de sécurité échoué: niveau %d err %d", level, err);
        return;
    }
    LOG_INF("Niveau de sécurité BLE: %d (chiffrement actif si >= 2)", level);

    if (level >= BT_SECURITY_L2) {
        LOG_INF("Session chiffrée → déverrouillage GATT");
        ble_manager_unlock();

        /* Signale à main.c que le pairing est terminé */
        k_sem_give(&paired_sem);

        /* Notifie la couche applicative (ex. mise à jour des paramètres de connexion) */
        if (secured_cb) {
            secured_cb(conn);
        }
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .security_changed = security_changed,
};

/* ── Callbacks d'authentification ────────────────────────────────────────── */

static void pairing_complete_cb(struct bt_conn *conn, bool bonded)
{
    LOG_INF("Pairing complet, lié: %d", bonded);
}

static void pairing_failed_cb(struct bt_conn *conn,
                               enum bt_security_err reason)
{
    LOG_WRN("Pairing échoué, raison: %d", reason);
}

static struct bt_conn_auth_info_cb auth_info = {
    .pairing_complete = pairing_complete_cb,
    .pairing_failed   = pairing_failed_cb,
};

/* ── API publique ────────────────────────────────────────────────────────── */

void ble_manager_set_secured_cb(ble_secured_cb_t cb)
{
    secured_cb = cb;
}

int ble_manager_init(void)
{
    k_work_init_delayable(&adv_work, adv_work_handler);

    int err = bt_conn_auth_info_cb_register(&auth_info);
    if (err) {
        LOG_ERR("auth_info_cb_register: %d", err);
        return err;
    }

    err = gatt_service_init();
    if (err) {
        LOG_ERR("gatt_service_init: %d", err);
        return err;
    }

    LOG_INF("BLE manager initialisé");
    return 0;
}

int ble_manager_start_adv(void)
{
    int err = bt_le_adv_start(ADV_PARAM, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == -EALREADY) {
        LOG_WRN("Advertising déjà actif");
        return 0;
    }
    if (err) {
        LOG_ERR("bt_le_adv_start: %d", err);
        return err;
    }
    LOG_INF("Advertising démarré");
    return 0;
}

void ble_manager_stop_adv(void)
{
    bt_le_adv_stop();
    LOG_INF("Advertising arrêté");
}

void ble_manager_unlock(void)
{
    gatt_service_set_unlocked(true);
    LOG_INF("Données GATT déverrouillées");
}

struct bt_conn *ble_manager_get_conn(void)
{
    return current_conn ? bt_conn_ref(current_conn) : NULL;
}

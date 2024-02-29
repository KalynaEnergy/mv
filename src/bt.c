
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/lbs.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt, CONFIG_BT_LOG_LEVEL);

#include <math.h>
#include "arm_math.h"


/* interfaces */

extern struct statechange_work_data {
    struct k_work work;
    bool newstate;
} statechange_work_data;

extern void trip_off();

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)


// custom parameters for advertising, 
struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_NONE, 800, 801, NULL);

// code for including custom dynamic data in advertising packet
#define COMPANY_ID_CODE 0x0059 // use Nordic's code for development, must apply to Bluetooth SIG for a unique ID
typedef struct adv_mfg_data {
	uint16_t company_code;	    /* Company Identifier Code. */
	uint16_t count;      
} adv_mfg_data_type;
adv_mfg_data_type adv_mfg_data = {COMPANY_ID_CODE,0x00};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA,(unsigned char *)&adv_mfg_data, sizeof(adv_mfg_data)),
};

// currently UUID for Nordic LBS service
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)),
};
// static unsigned char url_data[] ={0x17,'/','/','k','a','l','n','a','.','e','n',
//                                  'e','r','g','y'};
// static const struct bt_data sd[] = {
// 	BT_DATA(BT_DATA_URI, url_data,sizeof(url_data)),
// };

void count_handler(struct k_work *work) {
	adv_mfg_data.count++;
	bt_le_adv_update_data(ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
}

struct bt_conn_info info;
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)\n", err);
		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}
	LOG_INF("Connected");
	float32_t connection_interval = info.le.interval*1.25f; // in ms
	uint16_t supervision_timeout = info.le.timeout*10; // in ms
	LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", 
		connection_interval, info.le.latency, supervision_timeout);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	trip_off();
	LOG_INF("Disconnected (reason %u)", reason);
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    float32_t connection_interval = interval*1.25f;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms
    LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.le_param_updated = on_le_param_updated,
};

/* interrupt callback */
static void statechange_cb(bool newstate)
{
    /* Set the state in your work data structure */
    statechange_work_data.newstate = newstate;
	/* submit work */
    k_work_submit(&statechange_work_data.work);
}

static struct bt_lbs_cb lbs_callbacks = {
	.led_cb    = statechange_cb,
	.button_cb = NULL,
};

K_WORK_DEFINE(count_work, count_handler);

void count_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&count_work);
}

K_TIMER_DEFINE(count_timer, count_timer_handler, NULL);

void init_bt() {
	int err;
	LOG_INF("Starting Bluetooth");

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_lbs_init(&lbs_callbacks);
	if (err) {
		LOG_ERR("Failed to init LBS (err:%d)", err);
		return;
	}

	// XXX use adv_param custom parameters?
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	k_timer_start(&count_timer, K_USEC(0U), K_MSEC(1000U));
	LOG_INF("Advertising successfully started");
}
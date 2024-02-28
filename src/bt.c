
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

/* interfaces */

extern struct statechange_work_data {
    struct k_work work;
    bool newstate;
} statechange_work_data;

extern void trip_off();

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_NONE, 800, 801, NULL);
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

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
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


static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}
	printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	trip_off();
	printk("Disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
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

void init_lbs() {
	int err;
	printk("Starting Bluetooth\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_lbs_init(&lbs_callbacks);
	if (err) {
		printk("Failed to init LBS (err:%d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	k_timer_start(&count_timer, K_USEC(0U), K_MSEC(1000U));


	printk("Advertising successfully started\n");
}
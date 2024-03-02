#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "mv.h"
#include "bt_mv.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_mv, LOG_LEVEL_DBG);

static int32_t system_value;
static struct bt_mv_cb bt_mv_cb;

static ssize_t read_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	//get a pointer to system_value which is passed in the BT_GATT_CHARACTERISTIC() and stored in attr->user_data
	const int32_t *value = attr->user_data;
	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle,
		(void *)conn);
	if (bt_mv_cb.readval_cb) {
		// Call the application callback function to update the get the current value of the button
		system_value = bt_mv_cb.readval_cb();
		return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
					 sizeof(*value));
	}
	return 0;
}

static ssize_t write_statechange(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle,
		(void *)conn);
	if (len != 1U) {
		LOG_DBG("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (offset != 0) {
		LOG_DBG("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (bt_mv_cb.statechange_cb) {
		//Read the received value 
		uint8_t val = *((uint8_t *)buf);
		if (val == 0x00 || val == 0x01) {
			//Call the application callback function to update the LED state
			bt_mv_cb.statechange_cb(val ? true : false);
		} else {
			LOG_DBG("Write led: Incorrect value");
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
	}
	return len;
}

BT_GATT_SERVICE_DEFINE(bt_mv_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_MV),
		       BT_GATT_CHARACTERISTIC(BT_UUID_MV_READVAL, BT_GATT_CHRC_READ,
					      BT_GATT_PERM_READ, read_state, NULL, &system_value),
		       BT_GATT_CHARACTERISTIC(BT_UUID_MV_STATECHANGE, BT_GATT_CHRC_WRITE,
					      BT_GATT_PERM_WRITE, NULL, write_statechange, NULL),

);

/* A function to register application callbacks for the service characteristics  */
int bt_mv_init(struct bt_mv_cb *callbacks)
{
	if (callbacks) {
		bt_mv_cb.statechange_cb = callbacks->statechange_cb;
		bt_mv_cb.readval_cb = callbacks->readval_cb;
	}
    LOG_DBG("bt_mv_init complete");

	return 0;
}
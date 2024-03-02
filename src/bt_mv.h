#ifndef BT_MV_H_
#define BT_MV_H_

/*
	BT MV service API
	based on https://github.com/NordicDeveloperAcademy/bt-fund/blob/main/lesson4/blefund_less4_exer1/src/my_lbs.h
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

// Define the 128 bit UUIDs for the GATT service and its characteristics
// we're mostly stealing values from LBS. XXX pick our own UUIDs
#define BT_UUID_MV_VAL \
	BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd124)
// read characteristic
#define BT_UUID_MV_READVAL_VAL \
	BT_UUID_128_ENCODE(0x00011524, 0x1212, 0xefde, 0x1523, 0x785feabcd124)
// statechange characteristic
#define BT_UUID_MV_STATECHANGE_VAL \
	BT_UUID_128_ENCODE(0x00011526, 0x1212, 0xefde, 0x1523, 0x785feabcd124)
#define BT_UUID_MV           BT_UUID_DECLARE_128(BT_UUID_MV_VAL)
#define BT_UUID_MV_READVAL    BT_UUID_DECLARE_128(BT_UUID_MV_READVAL_VAL)
#define BT_UUID_MV_STATECHANGE       BT_UUID_DECLARE_128(BT_UUID_MV_STATECHANGE_VAL)

/** @brief Callback type for when a state change is received. */
typedef void (*statechange_cb_t)(const bool newstate);

/** @brief Callback type for when the button state is pulled. */
typedef int32_t (*readval_cb_t)(void);

struct bt_mv_cb {
	statechange_cb_t statechange_cb;
	readval_cb_t readval_cb;
};

int bt_mv_init(struct bt_mv_cb *callbacks);

#ifdef __cplusplus
}
#endif

#endif /* BT_MV_H_ */
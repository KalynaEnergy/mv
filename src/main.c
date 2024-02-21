#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>


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

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000

#define USER_LED                DK_LED3
#define USER_BUTTON             DK_BTN1_MSK


static const struct pwm_dt_spec custompwm0 = PWM_DT_SPEC_GET(DT_ALIAS(mycustompwm));

#define PWM_FREQ 10000 // Hz
#define WAVEFORM_FREQ 3 // Hz

#define STEPS 180
float levels[STEPS]; // holds duty cycles, duty = what fraction of time HS switch is on
#define DUTY_AVG 0.5f // 0. to 1.
#define DUTY_RANGE 0.90f // 0. to 1.
#define DEADTIME_NS 500U

#define TWO_PI 6.28318530718f

void step_handler(struct k_work *work)
{
	static uint32_t count = 0;
	static uint32_t oldpulsewidth_ns = 0;

	uint32_t pulsewidth_ns = PWM_HZ(PWM_FREQ)*levels[count++ % STEPS]; 
	uint32_t ret = pwm_set_dt(&custompwm0, 
		PWM_HZ(PWM_FREQ), 
		pulsewidth_ns);
	if (pulsewidth_ns > oldpulsewidth_ns) {
		ret |= pwm_set(custompwm0.dev, 1,
			PWM_HZ(PWM_FREQ), 
			pulsewidth_ns + DEADTIME_NS, PWM_POLARITY_INVERTED);
		ret |= pwm_set(custompwm0.dev, 2,
			PWM_HZ(PWM_FREQ), 
			pulsewidth_ns - DEADTIME_NS, PWM_POLARITY_NORMAL);
	} else {
		ret |= pwm_set(custompwm0.dev, 2,
			PWM_HZ(PWM_FREQ), 
			pulsewidth_ns - DEADTIME_NS, PWM_POLARITY_NORMAL);
		ret |= pwm_set(custompwm0.dev, 1,
			PWM_HZ(PWM_FREQ), 
			pulsewidth_ns + DEADTIME_NS, PWM_POLARITY_INVERTED);
	}
	oldpulsewidth_ns = pulsewidth_ns;
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
	}
}

K_WORK_DEFINE(step_work, step_handler);

void step_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&step_work);
}

K_TIMER_DEFINE(step_timer, step_timer_handler, NULL);

struct mvstate_t {
	bool poweron;
	float32_t duty_avg;

};

struct mvstate_t mvstate = {
	.poweron = false,
	.duty_avg = DUTY_AVG,
};

void log_state_handler(struct k_work* work) 
{
	if (!mvstate.poweron) {
		k_timer_start(&step_timer, K_USEC(0U), K_USEC(1000000U/(WAVEFORM_FREQ*STEPS)));
		mvstate.poweron = true;
		printk("Power state turned on\n");
	} else {
		k_timer_stop(&step_timer);
		k_msleep(1u); // otherwise it gets stuck sometimes
		uint32_t ret = pwm_set_dt(&custompwm0, PWM_HZ(PWM_FREQ), 0);
		ret |= pwm_set(custompwm0.dev, 2,
			PWM_HZ(PWM_FREQ), 
			0, PWM_POLARITY_NORMAL);
		ret |= pwm_set(custompwm0.dev, 1,
			PWM_HZ(PWM_FREQ), 
			0, PWM_POLARITY_NORMAL);

		if (ret) {
			printk("Error %d: failed to set pulse width in log_state_handler\n", ret);
		}

		mvstate.poweron = false;
		printk("Power state turned off\n");
	}
	
}
// struct k_work log_state_work = {
// 	.handler = log_state_handler,
// };
K_WORK_DEFINE(log_state_work, log_state_handler);




void pwm_init() {
	if (!device_is_ready(custompwm0.dev)) {
		printk("Error: PWM device %s is not ready\n",
		       custompwm0.dev->name);
	} 
	uint32_t ret = pwm_set_dt(&custompwm0, PWM_HZ(PWM_FREQ), 0);
	ret |= pwm_set(custompwm0.dev, 2,
		PWM_HZ(PWM_FREQ), 
		0, PWM_POLARITY_NORMAL);
	ret |= pwm_set(custompwm0.dev, 1,
		PWM_HZ(PWM_FREQ), 
		0, PWM_POLARITY_NORMAL);
	if (ret) {
		printk("failed pwm_set in pwm_init\n");
	}
	printk("pwm_init complete\n");
}

void waveform_init() {
	for (int i =0; i < STEPS; i++) {
		levels[i] = DUTY_AVG*(1 + DUTY_RANGE*sin(TWO_PI*i/STEPS));
	}
}

void console_init() {
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		// return 0;
	}

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	printk("Console_init complete\n");

}

int main(void)
{
	console_init();
	pwm_init();
	waveform_init();

	//k_timer_start(&step_timer, K_USEC(0U), K_USEC(1000000U/(WAVEFORM_FREQ*STEPS)));

	init_lbs();

	k_sleep(K_FOREVER);
	return 0;
}

static bool app_button_state;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

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
	printk("Disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
};

static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;

static void app_led_cb(bool led_state)
{
//	printk("Received set LED %d\n", led_state); // note this causes resets 
	k_work_submit(&log_state_work);
	dk_set_led(USER_LED, led_state);
}

static bool app_button_cb(void)
{
	return app_button_state;
}

static struct bt_lbs_cb lbs_callbacks = {
	.led_cb    = app_led_cb,
	.button_cb = NULL,
};

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & USER_BUTTON) {
		uint32_t user_button_state = button_state & USER_BUTTON;

		bt_lbs_send_button_state(user_button_state);
		app_button_state = user_button_state ? true : false;
	}
}

static int init_button(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err: %d)\n", err);
	}

	return err;
}

void init_lbs() {
	int err;
	printk("Starting Bluetooth Minverter\n");

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

	printk("Advertising successfully started\n");
}
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);// , CONFIG_MAIN_LOG_LEVEL);

#include "mv.h"



static const struct pwm_dt_spec custompwm0 = PWM_DT_SPEC_GET(DT_ALIAS(mycustompwm));

float levels[STEPS]; // holds duty cycles, duty = what fraction of time HS switch is on

#define TWO_PI 6.28318530718f

struct mv_param_t {
	/* IEEE 1547 10.6, tables 30-40 */
	/* ... */
	bool PermitService;
	/* .... */
	

	float32_t duty_avg;
	float32_t duty_range;
};

struct mv_param_t mv_param = {
	.PermitService = false,
	.duty_avg = DUTY_AVG,
	.duty_range = DUTY_RANGE,
};

struct mv_nameplate_t mv_nameplate = {
	.HardwareID = "unsupported",
	.VoltageNominal = -999.f,
	.Current = -999.f,
	.ActivePower = -999.f,
	.ApparentPower = -999.f,
	.ReactivePower = -999.f,
};

float32_t sysdata[7] = {0.f};

void step_handler(struct k_work *work)
{
	static uint32_t count = 0;
	static uint32_t oldpulsewidth_ns = 0;

	if (!mv_param.PermitService) {
		LOG_INF("step_handler called when not powered"); // this is normal: we may have steps left in the workqueue even if the output has been turned off
																// catching and ignoring them in the handler is recc https://docs.zephyrproject.org/latest/kernel/services/threads/workqueue.html#workqueue-best-practices
		return;
	}
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
		LOG_ERR("Error %d: failed to set pulse width", ret);
		// XXX
		trip_off(); // shouldn't happen, but if it does, trip?
	}
	if (!mv_param.PermitService) {
		LOG_ERR("ERR: step_handler finished when not powered"); // should not happen
		trip_off(); // but if it does let's power off
	}
}

K_WORK_DEFINE(step_work, step_handler);

void step_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&step_work);
}

void step_timer_off()
{
	LOG_DBG("step_timer_off called\n");
}

K_TIMER_DEFINE(step_timer, step_timer_handler, step_timer_off);


/* Create an instance of your custom structure */
static void statechange_handler(struct k_work *work);

struct statechange_work_data statechange_work_data = {
    .work = Z_WORK_INITIALIZER(statechange_handler),
};

/*
  turn everything off and set to known state
*/
void trip_off() {
	/* set state */
	mv_param.PermitService = false;
	/* stop any running timers */
	k_timer_stop(&step_timer);
	/* set all PWM to zero pulse width */
	uint32_t ret = pwm_set_dt(&custompwm0, PWM_HZ(PWM_FREQ), 0);
	ret |= pwm_set(custompwm0.dev, 2,
		PWM_HZ(PWM_FREQ), 
		0, PWM_POLARITY_NORMAL);
	ret |= pwm_set(custompwm0.dev, 1,
		PWM_HZ(PWM_FREQ), 
		0, PWM_POLARITY_NORMAL);
	if (ret) {
		LOG_ERR("Error %d: failed to set pulse width in trip_off", ret);
		// XXX
	}
}

void statechange_handler(struct k_work* work) {
    struct statechange_work_data *work_data = CONTAINER_OF(work, struct statechange_work_data, work);
 	bool newstate = work_data->newstate;
	if (newstate != !mv_param.PermitService) {
		LOG_INF("State change to %d despite existing state %d", newstate, mv_param.PermitService);
	}
	
	if (newstate) {
		k_timer_start(&step_timer, K_USEC(0U), K_USEC(1000000U/(WAVEFORM_FREQ*STEPS)));
		mv_param.PermitService = true;
		LOG_INF("Power state turned on");
	} else {
		trip_off();
		LOG_INF("Power state turned off");
	}	
}

void pwm_init() {
	if (!device_is_ready(custompwm0.dev)) {
		LOG_ERR("Error: PWM device %s is not ready",
		       custompwm0.dev->name);
	} 
	trip_off();
	LOG_INF("pwm_init complete");
}

void waveform_init() {
	for (int i =0; i < STEPS; i++) {
		levels[i] = mv_param.duty_avg*(1 + mv_param.duty_range*sin(TWO_PI*i/STEPS));
	}
}

void console_init() {
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		printk("USB enable failed\n"); // won't be seen if using usb console
		LOG_ERR("USB enable failed"); // again, not likely to be seen
		// return 0;
	}

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	LOG_INF("Console_init complete");

}

int init_hw_id() {
	int err = hw_id_get(mv_nameplate.HardwareID, ARRAY_SIZE(mv_nameplate.HardwareID));
	if (err) {
		LOG_ERR("hw_id_get failed (err %d)\n", err);
		return err;
	}
	LOG_INF("hw_id: %s\n", mv_nameplate.HardwareID);
	return 0;
}

int main(void)
{
	console_init();
	pwm_init();
	waveform_init();
	init_bt();
	adc_init();
	init_hw_id();

	while (1) {
		adc_mainloop();
		k_sleep(K_MSEC(1));
	}
	k_sleep(K_FOREVER);
	return 0;
}


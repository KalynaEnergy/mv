#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

/* interfaces */

void init_lbs();
void adc_init();
void adc_mainloop();


static const struct pwm_dt_spec custompwm0 = PWM_DT_SPEC_GET(DT_ALIAS(mycustompwm));

#define PWM_FREQ 10000 // Hz
#define WAVEFORM_FREQ 3 // Hz

#define STEPS 180
float levels[STEPS]; // holds duty cycles, duty = what fraction of time HS switch is on
#define DUTY_AVG 0.5f // 0. to 1.
#define DUTY_RANGE 0.90f // 0. to 1.
#define DEADTIME_NS 500U

#define TWO_PI 6.28318530718f

struct mvstate_t {
	bool poweron;
	float32_t duty_avg;
	float32_t duty_range;

};

struct mvstate_t mvstate = {
	.poweron = false,
	.duty_avg = DUTY_AVG,
	.duty_range = DUTY_RANGE,
	/* ... other parts of system state that can be modified ... */
};


void step_handler(struct k_work *work)
{
	static uint32_t count = 0;
	static uint32_t oldpulsewidth_ns = 0;

	if (!mvstate.poweron) {
		printk("INFO: step_handler called when not powered\n"); // this is normal: we may have steps left in the workqueue even if the output has been turned off
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
		printk("Error %d: failed to set pulse width\n", ret);
	}
	if (!mvstate.poweron) {
		printk("ERR: step_handler finished when not powered\n"); // should not happen
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
	printk("step_timer_off called\n");
}

K_TIMER_DEFINE(step_timer, step_timer_handler, step_timer_off);

struct statechange_work_data {
    struct k_work work;
    bool newstate;
};

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
	mvstate.poweron = false;
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
		printk("Error %d: failed to set pulse width in trip_off\n", ret);
	}
}

void statechange_handler(struct k_work* work) {
    struct statechange_work_data *work_data = CONTAINER_OF(work, struct statechange_work_data, work);
 	bool newstate = work_data->newstate;
	if (newstate != !mvstate.poweron) {
		printk("State change to %d despite existing state %d\n", newstate, mvstate.poweron);
	}
	
	if (newstate) {
		k_timer_start(&step_timer, K_USEC(0U), K_USEC(1000000U/(WAVEFORM_FREQ*STEPS)));
		mvstate.poweron = true;
		printk("Power state turned on\n");
	} else {
		trip_off();
		printk("Power state turned off\n");
	}	
}

void pwm_init() {
	if (!device_is_ready(custompwm0.dev)) {
		printk("Error: PWM device %s is not ready\n",
		       custompwm0.dev->name);
	} 
	trip_off();
	printk("pwm_init complete\n");
}

void waveform_init() {
	for (int i =0; i < STEPS; i++) {
		levels[i] = mvstate.duty_avg*(1 + mvstate.duty_range*sin(TWO_PI*i/STEPS));
	}
}

void console_init() {
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		printk("USB enable failed\n"); // won't be seen if using usb console
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
	init_lbs();
	adc_init();

	while (1) {
		adc_mainloop();
		k_sleep(K_MSEC(1));
	}
	k_sleep(K_FOREVER);
	return 0;
}


#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

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
	/* ... other parts of system state that can be modified ... */
};

struct statechange_work_data {
    struct k_work work;
    bool newstate;
};

/* Create an instance of your custom structure */
static void statechange_handler(struct k_work *work);

struct statechange_work_data statechange_work_data = {
    .work = Z_WORK_INITIALIZER(statechange_handler),
};

void statechange_handler(struct k_work* work) 
{

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
			printk("Error %d: failed to set pulse width in statechange_handler\n", ret);
		}

		mvstate.poweron = false;
		printk("Power state turned off\n");
	}
	
}

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

void init_lbs();


int main(void)
{
	console_init();
	pwm_init();
	waveform_init();
	init_lbs();

	k_sleep(K_FOREVER);
	return 0;
}


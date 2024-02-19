#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>

static const struct pwm_dt_spec custompwm0 = PWM_DT_SPEC_GET(DT_ALIAS(mycustompwm));


#define PWM_FREQ 10000 // Hz
#define CYCLE_FREQ 0.3 // Hz

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

int main(void)
{
	uint32_t ret;

	if (!device_is_ready(custompwm0.dev)) {
		printk("Error: PWM device %s is not ready\n",
		       custompwm0.dev->name);
		return 0;
	}

	for (int i =0; i < STEPS; i++) {
		levels[i] = DUTY_AVG*(1 + DUTY_RANGE*sin(TWO_PI*i/STEPS));
	}

	k_timer_start(&step_timer, K_USEC(0U), K_USEC(1000000U/(CYCLE_FREQ*STEPS)));

	k_sleep(K_FOREVER);
	return 0;
}


/* intermodule interfaces, system configuration */

#ifndef BT_H_
#define BT_H_
#include <arm_math_types.h>
#include <hw_id.h>

/* interfaces */

void init_bt();
void adc_init();
void adc_mainloop();

extern float32_t sysdata[];


struct statechange_work_data {
    struct k_work work;
    bool newstate;
};

extern struct statechange_work_data statechange_work_data;

extern void trip_off();

/*
 * system configuration
*/

#define PWM_FREQ 10000 // Hz
#define WAVEFORM_FREQ 3 // Hz

#define STEPS 180
extern float levels[]; // holds duty cycles, duty = what fraction of time HS switch is on
#define DUTY_AVG 0.5f // 0. to 1.
#define DUTY_RANGE 0.90f // 0. to 1.
#define DEADTIME_NS 500U

//////

// nameplate ratings: Nominal voltage (V), current (A), maximum active power (kW), apparent power
// (kVA), and reactive power (kvar) at which a DER is capable of sustained operation.
struct mv_nameplate_t {
    uint8_t HardwareID[HW_ID_LEN];
    float32_t VoltageNominal;
    float32_t Current;
    float32_t ActivePower;
    float32_t ApparentPower;
    float32_t ReactivePower;

};

#endif /* BT_H_ */

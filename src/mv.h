/* intermodule interfaces */

#ifndef BT_H_
#define BT_H_
#include <arm_math_types.h>

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


#endif /* BT_H_ */

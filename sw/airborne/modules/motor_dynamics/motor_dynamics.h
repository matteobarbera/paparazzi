//
// Created by matteo on 28/06/2021.
//

#ifndef PAPARAZZI_MOTOR_DYNAMICS_H
#define PAPARAZZI_MOTOR_DYNAMICS_H

#include "generated/airframe.h"

struct motor_dynamics_t {
    int32_t values[ACTUATORS_NB];
};
extern motor_dynamics_t actuator_values;

// GCS variables
extern int8_t actuator_idx;
extern uint32_t step_min;
extern uint32_t step_max;
extern uint8_t start_test;

extern void motor_dynamics_init();
extern void motor_dynamics_periodic();

extern void set_actuator_values();

// GCS settings handlers
extern void step_min_handler(uint32_t value);
extern void step_max_handler(uint32_t value);
extern void actuator_idx_handler(uint8_t value);
extern void start_test_handler(uint8_t value);

#endif //PAPARAZZI_MOTOR_DYNAMICS_H

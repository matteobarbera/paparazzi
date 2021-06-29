//
// Created by matteo on 28/06/2021.
//

#ifndef MOTOR_DYNAMICS_H
#define MOTOR_DYNAMICS_H

#include "generated/airframe.h"

struct motor_dynamics_t {
    int32_t commands[ACTUATORS_NB];
    int8_t actuator_idx;
    int32_t step_min;
    int32_t step_max;
    uint8_t start_test;
    uint8_t mode;
};
extern struct motor_dynamics_t motor_dynamics;

extern void motor_dynamics_init(void);
extern void motor_dynamics_run(void);

// GCS settings handlers
extern void motor_dynamics_step_min_handler(int32_t value);
extern void motor_dynamics_step_max_handler(int32_t value);
extern void motor_dynamics_actuator_idx_handler(int8_t value);
extern void motor_dynamics_start_test_handler(uint8_t value);
extern void motor_dynamics_mode_select_handler(uint8_t value);

#endif // MOTOR_DYNAMICS_H

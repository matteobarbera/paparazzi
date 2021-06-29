//
// Created by matteo on 28/06/2021.
//

#include "paparazzi.h"
#include "mcu_periph/sys_time.h"

#include "motor_dynamics.h"

/*
 * Requires that the following lines be added to your airframe file:
 * With the other modules:
 * <define name="SERVO_HZ" value="400"/>
 * <module name="motor_dynamics"/>
 *
 * At the end of command laws:
 * <command_laws>
 *    ...
 *    Already existing defines
 *    ...
 *
 *    <call fun="motor_dynamics_srun()"/>
 *    <set servo="YOUR_SERVO_0_NAME" value="motor_dynamics.commands[0]"/>
 *    <set servo="YOUR_SERVO_1_NAME" value="motor_dynamics.commands[1]"/>
 *    ... Repeat for all servo names, NOTE the change in array index !!!
 * </command_laws>
 *
 */

// Start/Stop definitions
#define STOP 0
#define START 1

// Mode definitions
#define NOMINAL 0
#define BODE 1

// For nominal mode
#define STEP_TIME 1  // Duration of step input in seconds
#define N_STEPS 20  // Number of step changes
// For bode mode
#define DELTA_FREQUENCY 0.25  // Increase in chirp frequency between steps
#define BODE_STEP_DURATION 2  // Duration of each chirp
#define MAX_TEST_FREQUENCY 15  // Max frequency to test

struct motor_dynamics_t motor_dynamics = {
    .commands = {0},
    .actuator_idx = -1,
    .step_min = 1000,  // should not be 0 as it needs to make sound
    .step_max = 9600,
    .start_test = STOP,
    .mode = NOMINAL
};

float start_t = 0;
uint8_t ctr = 0;
// For nominal mode
uint32_t old_value = 0;
uint32_t new_value = 0;
// For bode mode
float chirp_f = 1 / BODE_STEP_DURATION;  // Initialize such that you see min/max in first step
float bode_t = 0;
float half_chirp_dur;  // Duration of step during chirp

void motor_dynamics_init() {
  half_chirp_dur = 1 / chirp_f * 0.5;
  old_value = motor_dynamics.step_min;
  new_value = motor_dynamics.step_max;
}

void motor_dynamics_run() {
  // Actuator index must be manually set to correct value in GCS to start
  // Start must be given by user in GCS
  if (motor_dynamics.actuator_idx == -1 || motor_dynamics.start_test == STOP) {
    return;
  }

  if (motor_dynamics.mode == NOMINAL) {
    if (get_sys_time_float() - start_t > STEP_TIME) {
      // Reset time
      start_t = get_sys_time_float();

      // Switch step value
      uint32_t tmp = old_value;
      old_value = new_value;
      new_value = tmp;

      // Count step
      ctr++;
      // If number of steps exceeded end test
      if (ctr >= N_STEPS) {
        motor_dynamics_start_test_handler(STOP);
      }
    }
  } else if (motor_dynamics.mode == BODE) {
    if (get_sys_time_float() - start_t > BODE_STEP_DURATION) {
      // Reset time
      start_t = get_sys_time_float();
      bode_t = get_sys_time_float();
      // Increase chirp frequency
      chirp_f += DELTA_FREQUENCY;
      half_chirp_dur = 1 / chirp_f;
      // If exceeding max test frequency end test
      if (chirp_f > MAX_TEST_FREQUENCY) {
        motor_dynamics_start_test_handler(STOP);
        return;
      }
    } else if (get_sys_time_float() - bode_t > half_chirp_dur) {
      // Reset time
      bode_t = get_sys_time_float();

      // Switch step value
      uint32_t tmp = old_value;
      old_value = new_value;
      new_value = tmp;
    }
  }
  // Set all servo values to 0
  for (int i = 0; i < ACTUATORS_NB; i++) {
    motor_dynamics.commands[i] = 0;
  }
  motor_dynamics.commands[motor_dynamics.actuator_idx] = new_value;
}

void motor_dynamics_step_min_handler(int32_t value) {
  // Cannot change value mid test
  if (motor_dynamics.start_test == START) {
    return;
  }
  motor_dynamics.step_min = value;
  old_value = motor_dynamics.step_min;
  new_value = motor_dynamics.step_max;
}

void motor_dynamics_step_max_handler(int32_t value) {
  // Cannot change value mid test
  if (motor_dynamics.start_test == START) {
    return;
  }
  motor_dynamics.step_max = value;
  old_value = motor_dynamics.step_min;
  new_value = motor_dynamics.step_max;
}

void motor_dynamics_actuator_idx_handler(int8_t value) {
  // Cannot change actuator mid test
  if (motor_dynamics.start_test == START) {
    return;
  }
  // Ensure index is not larger than array
  // Do not start test if it is
  if (value < ACTUATORS_NB) {
    motor_dynamics.actuator_idx = value;
  } else {
    motor_dynamics.actuator_idx = -1;
  }
}

void motor_dynamics_start_test_handler(uint8_t value) {
  // Cannot set start_test to true if actuator index hasn't been set
  if (motor_dynamics.actuator_idx == -1) {
    return;
  }
  // If stop, reset the test
  if (value == STOP) {
    ctr = 0;
    chirp_f = DELTA_FREQUENCY;
    half_chirp_dur = 1 / chirp_f * 0.5;
    motor_dynamics.start_test = STOP;
  }
  // If not already started, start timer
  else if (value == START && motor_dynamics.start_test == STOP) {
    start_t = get_sys_time_float();
    bode_t = get_sys_time_float();
    motor_dynamics.start_test = START;
  }
}

void motor_dynamics_mode_select_handler(uint8_t value) {
  // Cannot change mode mid test
  if (motor_dynamics.start_test == START) {
    return;
  }
  motor_dynamics.mode = value;
}


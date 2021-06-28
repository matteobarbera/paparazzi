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
 *    <call fun="motor_dynamics_set_actuator_values()"/>
 *    <set servo="YOUR_SERVO_0_NAME" value="motor_dynamics.values[0]"/>
 *    <set servo="YOUR_SERVO_1_NAME" value="motor_dynamics.values[1]"/>
 *    ... Repeat for all servo names, NOTE the change in array index !!!
 * </command_laws>
 *
 */

#define STEP_TIME 1  // Duration of step input in seconds
#define N_STEPS 20  // Number of step changes

struct motor_dynamics_t motor_dynamics;
struct motor_dynamics_t current_actuator_values;

float start_t = 0;
uint32_t old_value = 0;
uint32_t new_value = 0;
uint8_t ctr = 0;

// GCS variables
int8_t actuator_idx = -1;
uint32_t step_min = 1000;  // should not be 0 as it needs to make sound
uint32_t step_max = 9600;
uint8_t start_test = false;

void motor_dynamics_init() {
  for (int i = 0; i < ACTUATORS_NB, i++) {
    motor_dynamics.values[i] = 0;
    current_actuator_values.values[i] = 0;
  }
  old_value = step_min;
  new_value = step_max;
}

void motor_dynamics_periodic() {
  // Manually set idx to correct value in GCS to start
  if (actuator_idx == -1 || !start_test) {
    return;
  }
  if (get_sys_time_float() - start_t > STEP_TIME) {
    // Reset time
    start_t = get_sys_time_float();

    // Switch step value
    uint32_t tmp = old_value;
    old_value = new_value;
    new_value = tmp;

    // Count step
    ctr++;
  }
  // Set all servo values to 0
  for (int i = 0; i < ACTUATORS_NB, i++) {
    current_actuator_values.values[i] = 0;
  }
  // If test still ongoing, set chosen servo values
  if (ctr < N_STEPS) {
    current_actuator_values.values[actuator_idx] = new_value;
  }
}

void set_actuator_values() {
  for (int i = 0; i < ACTUATORS_NB, i++) {
    motor_dynamics.values[i] = BoundAbs(current_actuator_values.values[i], MAX_PPRZ);
  }
}

void step_min_handler(uint32_t value) {
  step_min = value;
  old_value = step_min;
  new_value = step_max;
}

void step_max_handler(uint32_t value) {
  step_max = value;
  old_value = step_min;
  new_value = step_max;
}

void actuator_idx_handler(int8_t value) {
  // Cannot change actuator mid test
  if (start_test) {
    return;
  }
  // Ensure index is not larger than array
  // Do not start test if it is
  if (value < ACTUATORS_NB) {
    actuator_idx = value;
  } else {
    actuator_idx = -1;
  }
}

void start_test_handler(uint8_t value) {
  start_test = value;
  // If stop, reset the test
  if (value == 0) {
    ctr = 0;
    start_t = 0;
  }
}


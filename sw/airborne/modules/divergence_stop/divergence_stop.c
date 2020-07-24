#include "modules/divergence_stop/divergence_stop.h"

//#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/radio_control.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

#ifndef DIVERGENCE_STOP_THRESHOLD
#error Error! divergence_stop threshold not defined
#endif
float size_divergence;

#ifndef OFH_OPTICAL_FLOW_ID
#define OFH_OPTICAL_FLOW_ID ABI_BROADCAST
#endif

#define PRINT(string, ...) fprintf(stderr, "[divergence_stop->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

float divergence_stop_threshold = DIVERGENCE_STOP_THRESHOLD;
bool obstacle_encountered = false;

static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y,
                            int16_t flow_der_x, int16_t flow_der_y, float quality, float size_div) {
    size_divergence = size_div;
}

void divergence_stop_init(void) {
    // Initialize by flying forward
    navigation_state = DIVERGENCE_MODE_FORWARD;

    // Subscribe to the optical flow estimator:
    AbiBindMsgOPTICAL_FLOW(OFH_OPTICAL_FLOW_ID, &optical_flow_ev, optical_flow_cb);

}

void divergence_stop_periodic(void) {
    if (!autopilot_in_flight()) && (autopilot_get_mode() != AP_MODE_ATTITUDE_Z_HOLD) {
        return;
    }
    if (is_divergence_stop_active()){
        PRINT("Running!\n");
        if (size_divergence > divergence_stop_threshold) {
            PRINT("THRESHOLD EXCEEDED\n");
            obstacle_encountered = true;
        }
        PRINT("%f\n", size_divergence);
    } else {
        obstacle_encountered = false;
    }
    return;
}

bool is_divergence_stop_active() {
    return (radio_control.values[RADIO_AUX2] > 4500) ? true : false;
}

void divergence_stop_add_values(bool motors_on, bool override_on, pprz_t in_cmd[]) {
    if (motors_on) {
        if (is_divergence_stop_active()) {
            in_cmd[COMMAND_ROLL] = obstacle_encountered ? 0 : 2500;
        }
    }
}

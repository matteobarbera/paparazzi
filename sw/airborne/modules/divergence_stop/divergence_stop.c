#include "modules/divergence_stop/divergence_stop.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include "autopilot_static.h"
#include <stdio.h>
#include <time.h>

#ifndef DIVERGENCE_STOP_THRESHOLD
#error Error! divergence_stop threshold not defined
#endif
float size_divergence;

#ifndef OFH_OPTICAL_FLOW_ID
#define OFH_OPTICAL_FLOW_ID ABI_BROADCAST
#endif

#define PRINT(string,...) fprintf(stderr, "[divergence_stop->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

int navigation_state = DIVERGENCE_MODE_FORWARD;
float divergence_stop_threshold = DIVERGENCE_STOP_THRESHOLD;

static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y,
                         int16_t flow_der_x, int16_t flow_der_y, float quality, float size_div)
{
  size_divergence = size_div;
}

void divergence_stop_init(void)
{
  // Initialize by flying forward
  navigation_state = DIVERGENCE_MODE_FORWARD;

  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(OFH_OPTICAL_FLOW_ID, &optical_flow_ev, optical_flow_cb);

}

void divergence_stop_periodic(void)
{
  if (size_divergence > divergence_stop_threshold) {
//	  navigation_state = DIVERGENCE_MODE_STOP;
      PRINT("THRESHOLD EXCEEDED: New Mode\n");
      autopilot_static_set_mode(AP_MODE_ATTITUDE_Z_HOLD);
  }
//  else {
//	  navigation_state = DIVERGENCE_MODE_FORWARD;
//  }
//  if (navigation_state == DIVERGENCE_MODE_STOP) {
//	  // guidance_h_set_guided_body_vel(0, 0);
//	  PRINT("THRESHOLD EXCEEDED: New Mode\n");
//	  autopilot_static_set_mode(AP_MODE_CARE_FREE_DIRECT);
//  } //else {
	  // guidance_h_set_guided_body_vel(1.0, 0);
  //}
  PRINT("%f\n", size_divergence);
  return;
}

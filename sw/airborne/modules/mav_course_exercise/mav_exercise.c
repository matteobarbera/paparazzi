//
// Created by matteo on 31/05/2020.
//

#include "mav_exercise.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot_static.h"

#include "generated/flight_plan.h"

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);

enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND
};

// define and initialise global variables
float oa_color_count_frac = 0.18f;
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float moveDistance = 2;                 // waypoint displacement [m]

// needed to receive output from a separate module running on a parallel process
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
    color_count = quality;
}

void mav_exercise_init(void)
{
    // bind our colorfilter callbacks to receive the color filter outputs
    AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

void mav_exercise_periodic(void)
{
    // only evaluate our state machine if we are flying
    if(!autopilot_in_flight()){
        return;
    }

    // compute current color thresholds
    // front_camera defined in airframe xml, with the video_capture module
    int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

    // update our safe confidence using color threshold
    if(color_count < color_count_threshold){
        obstacle_free_confidence++;
    } else {
        obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
    }

    switch (navigation_state) {
        case SAFE:
            if (obstacle_free_confidence == 0) {
                navigation_state = OBSTACLE_FOUND;
            } else {
                moveWaypointForward(WP_GOAL, moveDistance);
            }
            break;
        case OBSTACLE_FOUND:
            // TODO Change behavior
            // Land as soon as an obstacle is found
            waypoint_move_here_2d(WP_HOME);
            autopilot_static_set_mode(AP_MODE_HOME);
            break;
        default:
            break;
    }
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
    float heading  = stateGetNedToBodyEulers_f()->psi;

    // Now determine where to place the waypoint you want to go to
    new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
    new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
    return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
    waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
    return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
    struct EnuCoor_i new_coor;
    calculateForwards(&new_coor, distanceMeters);
    moveWaypoint(waypoint, &new_coor);
    return false;
}

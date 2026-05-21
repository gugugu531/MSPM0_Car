#ifndef GIMBAL_TRACKING_H
#define GIMBAL_TRACKING_H

#include "bsp_common.h"
#include "canmv_uart.h"
#include "common/core_types.h"
#include "pid/pid.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_TRACKING_DEFAULT_IMAGE_HEIGHT 480.0f
#define GIMBAL_TRACKING_DEFAULT_PAPER_WIDTH 315.0f
#define GIMBAL_TRACKING_DEFAULT_PAPER_HEIGHT 212.0f
#define GIMBAL_TRACKING_DEFAULT_CIRCLE_RADIUS 60.0f

typedef struct {
    PID_CONFIG yaw_pid;
    PID_CONFIG pitch_pid;
    float image_height;
    float paper_width;
    float paper_height;
    float circle_radius;
    float yaw_output_sign;
    float pitch_output_sign;
} GIMBAL_TRACKING_CONFIG;

typedef struct {
    CORE_POINT2F target;
    CORE_POINT2F laser;
    CORE_POINT2F error;
    float yaw_speed;
    float pitch_speed;
    CANMV_STATUS laser_status;
    CANMV_STATUS rect_status;
    bool target_valid;
    bool laser_valid;
} GIMBAL_TRACKING_STATE;

void GimbalTracking_Init(const GIMBAL_TRACKING_CONFIG *config);
void GimbalTracking_Reset(void);
BSP_STATUS GimbalTracking_UpdateLaserCenter(float dt_s);
BSP_STATUS GimbalTracking_UpdateRectCircle(int32_t edge_index,
                                           float angle_offset_deg,
                                           float dt_s);
BSP_STATUS GimbalTracking_TrackPoints(CORE_POINT2F target,
                                      CORE_POINT2F laser,
                                      float dt_s);
BSP_STATUS GimbalTracking_Stop(void);
GIMBAL_TRACKING_STATE GimbalTracking_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_TRACKING_H */

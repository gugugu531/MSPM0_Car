#ifndef TURN_DRIVE_H
#define TURN_DRIVE_H

#include "bsp_common.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TURN_DRIVE_BASE_DUTY_PERCENT                  80.0f
#define TURN_DRIVE_LEFT_DECELERATION_DURATION_MS       250U
#define TURN_DRIVE_LEFT_DECELERATION_END_DUTY_PERCENT  (-80.0f)
#define TURN_DRIVE_FULL_BASE_DUTY_PERCENT              100.0f
#define TURN_DRIVE_FULL_LEFT_DECELERATION_DURATION_MS  300U
#define TURN_DRIVE_FULL_LEFT_DECELERATION_END_DUTY_PERCENT (-100.0f)
#define TURN_DRIVE_PRE_TURN_DISTANCE_M                 2.0f
#define TURN_DRIVE_POST_TURN_DISTANCE_M                1.0f
#define TURN_DRIVE_LEFT_YAW_DELTA_DEG                  (-90.0f)
#define TURN_DRIVE_INTEGRATED_PHASE_DURATION_MS        500U
#define TURN_DRIVE_IMU_MAX_AGE_MS                      60U
#define TURN_DRIVE_YAW_SIGN                            (-1.0f)
#define TURN_DRIVE_GYRO_Z_SIGN                         (1.0f)
#define TURN_DRIVE_TURN_COMPLETE_TOLERANCE_DEG         3.0f
#define TURN_DRIVE_TURN_TIMEOUT_MS                     5000U
#define TURN_DRIVE_HEADING_KP                          1.0f
#define TURN_DRIVE_HEADING_OUTPUT_LIMIT                20.0f
#define TURN_DRIVE_FULL_HEADING_OUTPUT_LIMIT           10.0f

typedef enum {
    TURN_DRIVE_MODE_STANDARD = 0,
    TURN_DRIVE_MODE_FULL
} TURN_DRIVE_MODE;

typedef enum {
    TURN_DRIVE_PHASE_WAIT_IMU = 0,
    TURN_DRIVE_PHASE_INTEGRATED_STRAIGHT,
    TURN_DRIVE_PHASE_STRAIGHT,
    TURN_DRIVE_PHASE_LEFT_DECELERATE,
    TURN_DRIVE_PHASE_LEFT_TURN_BRAKED,
    TURN_DRIVE_PHASE_POST_TURN_STRAIGHT,
    TURN_DRIVE_PHASE_COMPLETE
} TURN_DRIVE_PHASE;

typedef struct {
    TURN_DRIVE_MODE mode;
    TURN_DRIVE_PHASE phase;
    bool imu_ready;
    float yaw_deg;
    float corrected_yaw_deg;
    float yaw_correction_deg;
    float gyro_z_deg_s;
    float integrated_heading_deg;
    float heading_reference_deg;
    float turn_target_deg;
    float distance_m;
    float post_turn_distance_m;
    float duty_left_percent;
    float duty_right_percent;
    float turn_reduction_percent;
} TURN_DRIVE_OUTPUT;

void TurnDrive_Init(TURN_DRIVE_MODE mode);
BSP_STATUS TurnDrive_Update(float dt_s);
TURN_DRIVE_OUTPUT TurnDrive_GetOutput(void);
bool TurnDrive_IsComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* TURN_DRIVE_H */

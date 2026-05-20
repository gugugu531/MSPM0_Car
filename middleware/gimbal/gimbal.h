/**
 * @file  gimbal.h
 * @brief Middleware gimbal system interface.
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "bsp_common.h"
#include "step_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GIMBAL_PITCH_MIN_DEG
#define GIMBAL_PITCH_MIN_DEG (-45.0f)
#endif

#ifndef GIMBAL_PITCH_MAX_DEG
#define GIMBAL_PITCH_MAX_DEG 45.0f
#endif

typedef enum {
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_PITCH,
    GIMBAL_AXIS_MAX
} GIMBAL_AXIS;

typedef struct {
    float yaw_deg;
    float pitch_deg;
} GIMBAL_ANGLE;

typedef struct {
    float yaw_deg_s;
    float pitch_deg_s;
} GIMBAL_SPEED;

typedef struct {
    float pitch_min_deg;
    float pitch_max_deg;
} GIMBAL_LIMIT;

typedef struct {
    GIMBAL_ANGLE angle;
    GIMBAL_SPEED speed;
    GIMBAL_LIMIT limit;
} GIMBAL_STATUS;

BSP_STATUS Gimbal_Init(void);

BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s);
BSP_STATUS Gimbal_Stop(void);
BSP_STATUS Gimbal_Update(void);

BSP_STATUS Gimbal_GetStatus(GIMBAL_STATUS *out);
GIMBAL_ANGLE Gimbal_GetAngle(void);
GIMBAL_SPEED Gimbal_GetSpeed(void);

void Gimbal_ResetPosition(void);
void Gimbal_ResetAxisPosition(GIMBAL_AXIS axis);

BSP_STATUS Gimbal_SetLimit(const GIMBAL_LIMIT *limit);
GIMBAL_LIMIT Gimbal_GetLimit(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_H */

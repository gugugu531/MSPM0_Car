/**
 * @file  gimbal.c
 * @brief Middleware gimbal system implementation.
 */
#include "gimbal.h"
#include "bsp_time.h"
#include <stdbool.h>
#include <stddef.h>

static GIMBAL_LIMIT s_gimbal_limit = {
    .pitch_min_deg = GIMBAL_PITCH_MIN_DEG,
    .pitch_max_deg = GIMBAL_PITCH_MAX_DEG,
};

static bool Gimbal_IsValidAxis(GIMBAL_AXIS axis){
    return axis < GIMBAL_AXIS_MAX;
}

static BSP_STATUS Gimbal_CombineStatus(BSP_STATUS current, BSP_STATUS next){
    return (current == BSP_STATUS_OK) ? next : current;
}

static float Gimbal_ApplyPitchLimit(float pitch_speed_deg_s){
    float pitch_deg = StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH);

    if (pitch_deg >= s_gimbal_limit.pitch_max_deg && pitch_speed_deg_s > 0.0f){
        return 0.0f;
    }

    if (pitch_deg <= s_gimbal_limit.pitch_min_deg && pitch_speed_deg_s < 0.0f){
        return 0.0f;
    }

    return pitch_speed_deg_s;
}

BSP_STATUS Gimbal_Init(void){
    s_gimbal_limit.pitch_min_deg = GIMBAL_PITCH_MIN_DEG;
    s_gimbal_limit.pitch_max_deg = GIMBAL_PITCH_MAX_DEG;
    return StepMotor_Init();
}

BSP_STATUS Gimbal_SetSpeed(float yaw_deg_s, float pitch_deg_s){
    (void)Gimbal_Update();

    float limited_pitch_speed = Gimbal_ApplyPitchLimit(pitch_deg_s);
    BSP_STATUS status = StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_YAW, yaw_deg_s);
    BSP_STATUS pitch_status = StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_PITCH, limited_pitch_speed);

    return Gimbal_CombineStatus(status, pitch_status);
}

BSP_STATUS Gimbal_Stop(void){
    return StepMotor_StopAll();
}

BSP_STATUS Gimbal_Update(void){
    return StepMotor_UpdateAllState(BSP_Time_GetMs());
}

BSP_STATUS Gimbal_GetStatus(GIMBAL_STATUS *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    out->angle = Gimbal_GetAngle();
    out->speed = Gimbal_GetSpeed();
    out->limit = Gimbal_GetLimit();
    return BSP_STATUS_OK;
}

GIMBAL_ANGLE Gimbal_GetAngle(void){
    GIMBAL_ANGLE angle = {
        .yaw_deg = StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW),
        .pitch_deg = StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH),
    };

    return angle;
}

GIMBAL_SPEED Gimbal_GetSpeed(void){
    GIMBAL_SPEED speed = {
        .yaw_deg_s = StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_YAW),
        .pitch_deg_s = StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_PITCH),
    };

    return speed;
}

void Gimbal_ResetPosition(void){
    StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW);
    StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH);
}

void Gimbal_ResetAxisPosition(GIMBAL_AXIS axis){
    if (!Gimbal_IsValidAxis(axis)){
        return;
    }

    STEP_MOTOR_CHANNEL channel = (axis == GIMBAL_AXIS_YAW) ?
        STEP_MOTOR_CHANNEL_YAW : STEP_MOTOR_CHANNEL_PITCH;

    StepMotor_ResetEstimatedPosition(channel);
}

BSP_STATUS Gimbal_SetLimit(const GIMBAL_LIMIT *limit){
    if (limit == NULL){
        return BSP_STATUS_NULL;
    }

    if (limit->pitch_min_deg > limit->pitch_max_deg){
        return BSP_STATUS_INVALID_ARG;
    }

    s_gimbal_limit = *limit;
    return BSP_STATUS_OK;
}

GIMBAL_LIMIT Gimbal_GetLimit(void){
    return s_gimbal_limit;
}

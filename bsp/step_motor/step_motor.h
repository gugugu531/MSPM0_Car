/**
 * @file  step_motor.h
 * @brief BSP 步进电机开环控制接口。
 */
#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef STEP_MOTOR_YAW_DIR_PORT
#define STEP_MOTOR_YAW_DIR_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_YAW_DIR_PIN
#define STEP_MOTOR_YAW_DIR_PIN SMotor_IO_DIR2_PIN
#endif

#ifndef STEP_MOTOR_YAW_PWM_TIMER
#define STEP_MOTOR_YAW_PWM_TIMER SMotor_2_INST
#endif

#ifndef STEP_MOTOR_YAW_PWM_CHANNEL
#define STEP_MOTOR_YAW_PWM_CHANNEL DL_TIMER_CC_1_INDEX
#endif

#ifndef STEP_MOTOR_YAW_POSITIVE_DIR_HIGH
#define STEP_MOTOR_YAW_POSITIVE_DIR_HIGH 1U
#endif

#ifndef STEP_MOTOR_PITCH_DIR_PORT
#define STEP_MOTOR_PITCH_DIR_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_PITCH_DIR_PIN
#define STEP_MOTOR_PITCH_DIR_PIN SMotor_IO_DIR1_PIN
#endif

#ifndef STEP_MOTOR_PITCH_PWM_TIMER
#define STEP_MOTOR_PITCH_PWM_TIMER SMotor_1_INST
#endif

#ifndef STEP_MOTOR_PITCH_PWM_CHANNEL
#define STEP_MOTOR_PITCH_PWM_CHANNEL DL_TIMER_CC_0_INDEX
#endif

#ifndef STEP_MOTOR_PITCH_POSITIVE_DIR_HIGH
#define STEP_MOTOR_PITCH_POSITIVE_DIR_HIGH 0U
#endif

#ifndef STEP_MOTOR_STEP_ANGLE_DEG
#define STEP_MOTOR_STEP_ANGLE_DEG 1.8f
#endif

#ifndef STEP_MOTOR_MICROSTEP
#define STEP_MOTOR_MICROSTEP 32.0f
#endif

#ifndef STEP_MOTOR_TIMER_CLOCK_HZ
#define STEP_MOTOR_TIMER_CLOCK_HZ 32000000U
#endif

#ifndef STEP_MOTOR_TIMER_PRESCALER_FACTOR
#define STEP_MOTOR_TIMER_PRESCALER_FACTOR (32U * 8U * 2U)
#endif

#ifndef STEP_MOTOR_MAX_ARR
#define STEP_MOTOR_MAX_ARR 65535U
#endif

typedef enum {
    STEP_MOTOR_CHANNEL_YAW = 0,
    STEP_MOTOR_CHANNEL_PITCH,
    STEP_MOTOR_CHANNEL_MAX
} STEP_MOTOR_CHANNEL;

BSP_STATUS StepMotor_Init(void);

BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s);
BSP_STATUS StepMotor_RunFor(STEP_MOTOR_CHANNEL channel,
                            float speed_deg_per_s,
                            uint32_t duration_ms);

BSP_STATUS StepMotor_Stop(STEP_MOTOR_CHANNEL channel);
BSP_STATUS StepMotor_StopAll(void);

BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms);
BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms);

float StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel);
float StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel);
void StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel);

#ifdef __cplusplus
}
#endif

#endif /* STEP_MOTOR_H */

/**
 * @file  chassis.h
 * @brief Middleware chassis system interface.
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "bsp_common.h"
#include "hall_encoder.h"
#include "tb6612fng.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHASSIS_STOP_MODE_COAST = 0,
    CHASSIS_STOP_MODE_BRAKE
} CHASSIS_STOP_MODE;

typedef struct {
    float left_percent;
    float right_percent;
} CHASSIS_DUTY;

typedef struct {
    CHASSIS_DUTY duty;
    float speed_mps;
    float distance_m;
    HALL_ENCODER_DIR encoder_dir;
    TB6612FNG_OUTPUT left_output;
    TB6612FNG_OUTPUT right_output;
} CHASSIS_STATUS;

BSP_STATUS Chassis_Init(void);

BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent);

BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode);
BSP_STATUS Chassis_Brake(void);
BSP_STATUS Chassis_Coast(void);

BSP_STATUS Chassis_Update(void);
void Chassis_HandleEncoderIrq(uint32_t gpio_status);
void Chassis_HandleEncoderTimerIrq(void);

BSP_STATUS Chassis_GetStatus(CHASSIS_STATUS *out);
CHASSIS_DUTY Chassis_GetDuty(void);
float Chassis_GetSpeed(void);
float Chassis_GetDistance(void);
HALL_ENCODER_DIR Chassis_GetEncoderDir(void);
void Chassis_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */

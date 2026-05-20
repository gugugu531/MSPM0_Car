/**
 * @file  hall_encoder.h
 * @brief BSP 霍尔编码器接口。
 */
#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HALL_ENCODER_A_PORT
#define HALL_ENCODER_A_PORT Motor_IO_E1A_PORT
#endif

#ifndef HALL_ENCODER_A_PIN
#define HALL_ENCODER_A_PIN Motor_IO_E1A_PIN
#endif

#ifndef HALL_ENCODER_B_PORT
#define HALL_ENCODER_B_PORT Motor_IO_E1A_PORT
#endif

#ifndef HALL_ENCODER_B_PIN
#define HALL_ENCODER_B_PIN Motor_IO_E2A_PIN
#endif

#ifndef HALL_ENCODER_GPIO_IRQN
#define HALL_ENCODER_GPIO_IRQN GPIOA_INT_IRQn
#endif

#ifndef HALL_ENCODER_SAMPLE_TIMER
#define HALL_ENCODER_SAMPLE_TIMER TIMER_0_INST
#endif

#ifndef HALL_ENCODER_SAMPLE_TIMER_IRQN
#define HALL_ENCODER_SAMPLE_TIMER_IRQN TIMER_0_INST_INT_IRQN
#endif

#ifndef HALL_ENCODER_PI
#define HALL_ENCODER_PI 3.1415926f
#endif

#ifndef HALL_ENCODER_PPR
#define HALL_ENCODER_PPR 13.0f
#endif

#ifndef HALL_ENCODER_REDUCTION_RATIO
#define HALL_ENCODER_REDUCTION_RATIO 28.0f
#endif

#ifndef HALL_ENCODER_WHEEL_DIAMETER_M
#define HALL_ENCODER_WHEEL_DIAMETER_M 0.065f
#endif

#ifndef HALL_ENCODER_SAMPLE_PERIOD_S
#define HALL_ENCODER_SAMPLE_PERIOD_S 0.01f
#endif

#ifndef HALL_ENCODER_DISTANCE_SCALE
#define HALL_ENCODER_DISTANCE_SCALE 1.05f
#endif

typedef enum {
    HALL_ENCODER_DIR_FORWARD = 0,
    HALL_ENCODER_DIR_REVERSE
} HALL_ENCODER_DIR;

BSP_STATUS HallEncoder_Init(void);
void HallEncoder_HandleGpioIrq(uint32_t gpio_status);
void HallEncoder_UpdateSample(void);

int32_t HallEncoder_GetCount(void);
HALL_ENCODER_DIR HallEncoder_GetDir(void);
float HallEncoder_GetSpeed(void);
float HallEncoder_GetDistance(void);

void HallEncoder_Reset(void);
void HallEncoder_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* HALL_ENCODER_H */

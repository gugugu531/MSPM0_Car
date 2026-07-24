/**
 * @file  chassis.h
 * @brief Middleware 层底盘组合服务接口。
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

/**
 * @brief 底盘停止模式。
 */
typedef enum {
    /** 关闭 PWM，让电机自然滑行。 */
    CHASSIS_STOP_MODE_COAST = 0,
    /** IN1/IN2 同时制动，主动刹车。 */
    CHASSIS_STOP_MODE_BRAKE
} CHASSIS_STOP_MODE;

/**
 * @brief 左右轮占空比输出。
 */
typedef struct {
    /** 左轮占空比，范围通常为 [-100, 100]。 */
    float left_percent;
    /** 右轮占空比，范围通常为 [-100, 100]。 */
    float right_percent;
} CHASSIS_DUTY;

/**
 * @brief 初始化底盘相关 BSP 外设。
 */
BSP_STATUS Chassis_Init(void);

/**
 * @brief 设置左右轮占空比并立即输出到底盘电机。
 * @param left_percent 左轮占空比，正负表示方向。
 * @param right_percent 右轮占空比，正负表示方向。
 */
BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent);

/**
 * @brief 按指定模式停止底盘。
 */
BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode);

/**
 * @brief 主动刹停左右电机。
 */
BSP_STATUS Chassis_Brake(void);

/**
 * @brief 关闭左右电机输出，使其自然滑行。
 */
BSP_STATUS Chassis_Coast(void);

/**
 * @brief 获取最近设置的左右轮占空比。
 */
CHASSIS_DUTY Chassis_GetDuty(void);

/**
 * @brief 获取车体估算线速度(左右轮均值)，单位 m/s。
 */
float Chassis_GetSpeed(void);

/**
 * @brief 获取指定轮估算线速度，单位 m/s。
 */
float Chassis_GetWheelSpeed(HALL_ENCODER_ID wheel);

/**
 * @brief 获取车体累计距离(左右轮均值)，单位 m。
 */
float Chassis_GetDistance(void);

/**
 * @brief 获取指定轮累计距离，单位 m。
 */
float Chassis_GetWheelDistance(HALL_ENCODER_ID wheel);

/**
 * @brief 清零两轮编码器累计距离。
 */
void Chassis_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */

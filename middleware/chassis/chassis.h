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

/*
 * 每轮速度环 PID 默认增益(位置式: setpoint=目标轮速 m/s, feedback=实测轮速, output=占空比 %)。
 * 起始值, 必须上板整定(用 Device Check「Speed PID」)。integral_limit 使 ki*积分饱和值≈输出上限,
 * 提供抗积分饱和; output_limit=100 即满占空比。
 */
#define CHASSIS_SPEED_KP             200.00f
#define CHASSIS_SPEED_KI             17.0f
#define CHASSIS_SPEED_KD             0.0f
#define CHASSIS_SPEED_INTEGRAL_LIMIT 96.0f
#define CHASSIS_SPEED_OUTPUT_LIMIT   100.0f

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
 * @brief 底盘控制模式。
 */
typedef enum {
    /** 开环：直接下发占空比(默认)。 */
    CHASSIS_CONTROL_DUTY = 0,
    /** 闭环：每轮速度环 PID 跟踪目标轮速。 */
    CHASSIS_CONTROL_SPEED
} CHASSIS_CONTROL_MODE;

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
 * @brief 设置左右轮占空比并立即输出到底盘电机(开环)。切回 CHASSIS_CONTROL_DUTY 模式。
 * @param left_percent 左轮占空比，正负表示方向。
 * @param right_percent 右轮占空比，正负表示方向。
 */
BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent);

/**
 * @brief 设置左右轮目标线速度并进入速度闭环模式(CHASSIS_CONTROL_SPEED)。
 * @param left_mps  左轮目标线速度，m/s(负为倒转)。
 * @param right_mps 右轮目标线速度，m/s。
 * @note 仅设定目标; 实际出力由 Chassis_UpdateSpeedControl() 周期驱动。从开环切入时复位 PID。
 */
void Chassis_SetWheelSpeed(float left_mps, float right_mps);

/**
 * @brief 设置车体直行目标线速度(两轮同速), 进入速度闭环。
 * @param body_mps 目标线速度，m/s。
 * @note 差速转向需 track_width(暂缺), 故此处仅直行; 后续可用 core/kinematics BodyToWheel 扩展。
 */
void Chassis_SetSpeed(float body_mps);

/**
 * @brief 速度闭环单步更新: 闭环模式下跑两轮 PID 并出力; 开环模式为空操作。
 * @param dt_s 控制周期，s。
 * @return 出力状态; 开环模式恒返回 BSP_STATUS_OK。
 * @note 须由控制周期任务周期调用(当前接在 App_ControlTick, 20ms)。
 */
BSP_STATUS Chassis_UpdateSpeedControl(float dt_s);

/**
 * @brief 获取当前底盘控制模式。
 */
CHASSIS_CONTROL_MODE Chassis_GetControlMode(void);

/**
 * @brief 获取指定轮当前目标线速度(m/s), 仅闭环模式有意义。
 */
float Chassis_GetWheelSpeedTarget(HALL_ENCODER_ID wheel);

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

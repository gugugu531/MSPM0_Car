/**
 * @file  straight_drive.h
 * @brief Middleware 层直行测试控制器。
 */
#ifndef STRAIGHT_DRIVE_H
#define STRAIGHT_DRIVE_H

#include "bsp_common.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 指令范围 ============================ */

/** 开环及姿态闭环模式的默认基础占空比，单位 %。 */
#define STRAIGHT_DRIVE_DEFAULT_DUTY                 50.0f
/** 占空比模式每次调整量，单位 %。 */
#define STRAIGHT_DRIVE_DUTY_STEP                    2.0f
/** 基础占空比指令绝对值上限，单位 %。 */
#define STRAIGHT_DRIVE_DUTY_LIMIT                   80.0f
/** 速度闭环模式每次调整量，单位 m/s。 */
#define STRAIGHT_DRIVE_SPEED_STEP                   0.05f
/** 速度闭环默认目标，单位 m/s；当前约对应 80% 稳态占空比。 */
#define STRAIGHT_DRIVE_DEFAULT_SPEED                1.06f
/** 速度目标绝对值上限，单位 m/s。 */
#define STRAIGHT_DRIVE_SPEED_LIMIT                  3.0f
/** 差速混控后的单轮占空比绝对值上限，单位 %。 */
#define STRAIGHT_DRIVE_OUTPUT_LIMIT                 100.0f
/** 姿态样本最大允许时延；超时后姿态闭环立即停止电机。 */
#define STRAIGHT_DRIVE_IMU_MAX_AGE_MS               60U

/* ========================= 角速度闭环 PID ========================= */

/** 目标 gz 固定为 0 deg/s；PID 输出为左右轮差速修正量。 */
#define STRAIGHT_DRIVE_RATE_PID_KP                  0.20f
#define STRAIGHT_DRIVE_RATE_PID_KI                  0.0f
#define STRAIGHT_DRIVE_RATE_PID_KD                  0.0f
#define STRAIGHT_DRIVE_RATE_PID_INTEGRAL_LIMIT      100.0f
#define STRAIGHT_DRIVE_RATE_PID_OUTPUT_LIMIT        20.0f

/* ========================== 巡航角闭环 PID ========================= */

/** 目标角为每次从停车启动时的 yaw，误差按 [-180, 180) 取最短角。 */
#define STRAIGHT_DRIVE_HEADING_PID_KP               1.0f
#define STRAIGHT_DRIVE_HEADING_PID_KI               0.0f
#define STRAIGHT_DRIVE_HEADING_PID_KD               0.0f
#define STRAIGHT_DRIVE_HEADING_PID_INTEGRAL_LIMIT   100.0f
#define STRAIGHT_DRIVE_HEADING_PID_OUTPUT_LIMIT     20.0f

/** JY61P z 轴角速度反馈符号；实车验证应保持原始方向。 */
#define STRAIGHT_DRIVE_RATE_GYRO_SIGN               (1.0f)
/** JY61P yaw 反馈符号；当前安装方向下需反相以构成负反馈。 */
#define STRAIGHT_DRIVE_HEADING_YAW_SIGN             (-1.0f)

typedef enum {
    STRAIGHT_DRIVE_MODE_DUTY_OPEN = 0,
    STRAIGHT_DRIVE_MODE_SPEED,
    STRAIGHT_DRIVE_MODE_GYRO_RATE,
    STRAIGHT_DRIVE_MODE_GYRO_HEADING
} STRAIGHT_DRIVE_MODE;

/** 控制器状态快照，供 app 显示和遥测使用。 */
typedef struct {
    STRAIGHT_DRIVE_MODE mode;
    float command;
    float duty_left_percent;
    float duty_right_percent;
    float speed_left_mps;
    float speed_right_mps;
    float distance_left_m;
    float distance_right_m;
    float yaw_deg;
    float gyro_z_deg_s;
    float correction_percent;
    float heading_reference_deg;
    bool heading_reference_valid;
    bool imu_ready;
} STRAIGHT_DRIVE_OUTPUT;

/** 初始化指定直行模式、控制器状态和里程，并装载该模式默认指令。 */
void StraightDrive_Init(STRAIGHT_DRIVE_MODE mode);
/** 按模式规定步长调整指令；steps 可为正数或负数。 */
void StraightDrive_AdjustCommand(int8_t steps);
/** 将运动指令归零；下次启动巡航角模式时会重新捕获基准角。 */
void StraightDrive_ZeroCommand(void);
/** 读取缓存传感器、执行一次控制并更新状态快照。 */
BSP_STATUS StraightDrive_Update(float dt_s);
/** 获取最近一次控制更新后的完整状态快照。 */
STRAIGHT_DRIVE_OUTPUT StraightDrive_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* STRAIGHT_DRIVE_H */

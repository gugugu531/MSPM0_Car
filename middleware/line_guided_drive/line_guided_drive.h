/**
 * @file  line_guided_drive.h
 * @brief 80% 直接起步的 Yahboom 循线外环 + 航向内环控制器。
 */
#ifndef LINE_GUIDED_DRIVE_H
#define LINE_GUIDED_DRIVE_H

#include "bsp_common.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 直接起步基础占空比。 */
#define LINE_GUIDED_BASE_DUTY_PERCENT             80.0f
/** 起步角速度闭环持续时间。 */
#define LINE_GUIDED_STARTUP_RATE_DURATION_MS      1000U
/** JY61P 完整样本最大允许年龄。 */
#define LINE_GUIDED_IMU_MAX_AGE_MS                60U
/** 当前安装方向下的 JY61P 反馈符号，须分别上板验证。 */
#define LINE_GUIDED_RATE_GYRO_SIGN                (1.0f)
#define LINE_GUIDED_HEADING_YAW_SIGN              (-1.0f)
/** Yahboom 中间 X4/X5（bit3/4）；只有其余通道命中才进入循线外环。 */
#define LINE_GUIDED_CENTER_SENSOR_MASK            0x18U
#define LINE_GUIDED_OUTER_SENSOR_MASK             0xE7U

/** Yahboom 质心误差到即时航向偏置的比例及偏置限幅。 */
#define LINE_GUIDED_OUTER_HEADING_KP              0.8f
#define LINE_GUIDED_OUTER_HEADING_LIMIT_DEG       30.0f
#define LINE_GUIDED_OUTER_ERROR_LPF_ALPHA         0.5f

/** 起步角速度内环 PID。 */
#define LINE_GUIDED_RATE_PID_KP                   0.20f
#define LINE_GUIDED_RATE_PID_KI                   0.0f
#define LINE_GUIDED_RATE_PID_KD                   0.0f
#define LINE_GUIDED_RATE_PID_INTEGRAL_LIMIT       100.0f
#define LINE_GUIDED_RATE_PID_OUTPUT_LIMIT         20.0f

/** 巡航/灰度外环共用的航向内环 PID。 */
#define LINE_GUIDED_HEADING_PID_KP                1.0f
#define LINE_GUIDED_HEADING_PID_KI                0.0f
#define LINE_GUIDED_HEADING_PID_KD                0.0f
#define LINE_GUIDED_HEADING_PID_INTEGRAL_LIMIT    100.0f
#define LINE_GUIDED_HEADING_PID_OUTPUT_LIMIT      20.0f

typedef enum {
    LINE_GUIDED_PHASE_WAIT_IMU = 0,
    LINE_GUIDED_PHASE_STARTUP_RATE,
    LINE_GUIDED_PHASE_HEADING_HOLD,
    LINE_GUIDED_PHASE_LINE_OUTER
} LINE_GUIDED_PHASE;

typedef struct {
    LINE_GUIDED_PHASE phase;
    uint8_t level_mask;
    uint8_t black_mask;
    uint8_t black_count;
    bool line_lost;
    bool line_sensor_ready;
    bool imu_ready;
    float line_error;
    float yaw_deg;
    float gyro_z_deg_s;
    float integrated_heading_deg;
    float startup_fusion_offset_deg;
    float heading_reference_deg;
    float correction_percent;
    float left_duty_percent;
    float right_duty_percent;
} LINE_GUIDED_OUTPUT;

/** 初始化状态、PID、里程和 A/B 航向估计器。 */
void LineGuidedDrive_Init(void);
/**
 * @brief 使用 app 已采集的 Yahboom 黑线掩码，推进状态机并写入底盘。
 * @param detected_mask bit0=X1 ... bit7=X8；1=检测到黑线。
 * @param sensor_ready 本次掩码是否存在且未超时。
 * @param dt_s 控制周期，单位 s。
 */
BSP_STATUS LineGuidedDrive_Update(uint8_t detected_mask,
                                  bool sensor_ready,
                                  float dt_s);
/** 立即将电机指令归零并复位控制动态状态。 */
void LineGuidedDrive_Stop(void);
/** 获取最近控制输出，供 app 显示和遥测。 */
LINE_GUIDED_OUTPUT LineGuidedDrive_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_GUIDED_DRIVE_H */

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
#define STRAIGHT_DRIVE_DEFAULT_DUTY                 80.0f
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

/* ======================== 启动与闭环切换测试 ======================== */

/** 航向闭环斜坡模式的占空比变化率，单位 %/s；默认约 1.6 s 升至 80%。 */
#define STRAIGHT_DRIVE_RAMP_RATE_PERCENT_S          50.0f
/** 定时切换测试启动时直接使用的基础占空比，单位 %。 */
#define STRAIGHT_DRIVE_SWITCH_HEADING_DUTY_PERCENT  80.0f
/** 极限速度积分/融合切换测试的基础占空比，单位 %。 */
#define STRAIGHT_DRIVE_FULL_HEADING_DUTY_PERCENT    100.0f
/** 定时切换测试使用角速度闭环的启动阶段时长，单位 s。 */
#define STRAIGHT_DRIVE_RATE_PHASE_DURATION_S        1.0f
/** 定时切换测试使用编码器等路径闭环的启动阶段时长，单位 s。 */
#define STRAIGHT_DRIVE_ENCODER_PHASE_DURATION_S     1.0f
/** 纯角速度积分航向阶段持续时间，单位 ms。 */
#define STRAIGHT_DRIVE_INTEGRATED_PHASE_DURATION_MS 500U

/* ========================= 角速度闭环 PID ========================= */

/** 目标 gz 固定为 0 deg/s；PID 输出为左右轮差速修正量。 */
#define STRAIGHT_DRIVE_RATE_PID_KP                  0.20f
#define STRAIGHT_DRIVE_RATE_PID_KI                  0.0f
#define STRAIGHT_DRIVE_RATE_PID_KD                  0.0f
#define STRAIGHT_DRIVE_RATE_PID_INTEGRAL_LIMIT      100.0f
#define STRAIGHT_DRIVE_RATE_PID_OUTPUT_LIMIT        20.0f

/* ======================== 编码器等路径闭环 PID ======================= */

/** 目标为左右轮累计路程差 0 m；PID 输出为左右轮差速修正量。 */
#define STRAIGHT_DRIVE_ENCODER_PID_KP               200.0f
#define STRAIGHT_DRIVE_ENCODER_PID_KI               0.0f
#define STRAIGHT_DRIVE_ENCODER_PID_KD               0.0f
#define STRAIGHT_DRIVE_ENCODER_PID_INTEGRAL_LIMIT   0.1f
#define STRAIGHT_DRIVE_ENCODER_PID_OUTPUT_LIMIT     20.0f

/* ========================== 巡航角闭环 PID ========================= */

/** 目标角为每次从停车启动时的 yaw，误差按 [-180, 180) 取最短角。 */
#define STRAIGHT_DRIVE_HEADING_PID_KP               1.0f
#define STRAIGHT_DRIVE_HEADING_PID_KI               0.0f
#define STRAIGHT_DRIVE_HEADING_PID_KD               0.0f
#define STRAIGHT_DRIVE_HEADING_PID_INTEGRAL_LIMIT   100.0f
#define STRAIGHT_DRIVE_HEADING_PID_OUTPUT_LIMIT     20.0f
/** 100%模式的差速修正上限；速度优先混控下快侧 100%、最慢侧 80%。 */
#define STRAIGHT_DRIVE_FULL_HEADING_OUTPUT_LIMIT    10.0f

/** JY61P z 轴角速度闭环反馈符号；实车验证应保持原始方向。不得用于航向积分。 */
#define STRAIGHT_DRIVE_RATE_GYRO_SIGN               (1.0f)
/** 纯积分航向使用的 gz 符号；须与修正后的融合 yaw 采用同一正方向。 */
#define STRAIGHT_DRIVE_INTEGRATION_GYRO_SIGN        (-1.0f)
/** JY61P yaw 反馈符号；当前安装方向下需反相以构成负反馈。 */
#define STRAIGHT_DRIVE_HEADING_YAW_SIGN             (-1.0f)

typedef enum {
    STRAIGHT_DRIVE_MODE_DUTY_OPEN = 0,
    STRAIGHT_DRIVE_MODE_SPEED,
    STRAIGHT_DRIVE_MODE_GYRO_RATE,
    STRAIGHT_DRIVE_MODE_GYRO_HEADING,
    /** 占空比斜坡与巡航阶段全程使用启动角闭环。 */
    STRAIGHT_DRIVE_MODE_RAMP_HEADING,
    /** 直接输出 80%，前 1 s 角速度闭环，随后保持切换时刻的航向角。 */
    STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING,
    /** 直接输出 80%，前 1 s 编码器等路径闭环，随后保持切换时刻的航向角。 */
    STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING,
    /** 80% 修正 yaw 版：参考角固定 B0，前 500 ms 动态更新 A-B 修正量，随后冻结。 */
    STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING,
    /** 100%速度优先版：参考角固定 B0，按新样本更新 A-B 修正量并在 500 ms 后冻结。 */
    STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING
} STRAIGHT_DRIVE_MODE;

/** 控制器状态快照，供 app 显示和遥测使用。 */
typedef struct {
    STRAIGHT_DRIVE_MODE mode;
    /** 用户设定的最终占空比或速度目标。 */
    float command;
    /** 本拍实际使用的指令；仅斜坡航向模式与 command 不同。 */
    float applied_command;
    float duty_left_percent;
    float duty_right_percent;
    float speed_left_mps;
    float speed_right_mps;
    float distance_left_m;
    float distance_right_m;
    /** JY61P 输出并按安装方向修正后的融合 yaw，单位 deg。 */
    float yaw_deg;
    /** 添加 yaw_correction_deg 后供积分/yaw 模式航向 PID 使用的 yaw，单位 deg。 */
    float corrected_yaw_deg;
    /** 积分航向 A 与融合 yaw B 的差值；积分/yaw 模式在 500 ms 后冻结，单位 deg。 */
    float yaw_correction_deg;
    /** 经角速度闭环极性校正的 gz；不代表航向积分坐标系。 */
    float gyro_z_deg_s;
    /** 启动阶段由 B0 起算、再对 gz 纯积分得到的绝对航向角 A，单位 deg。 */
    float integrated_heading_deg;
    float correction_percent;
    float heading_reference_deg;
    /** 启动阶段已完成：斜坡已到目标，或定时切换模式已进入巡航阶段。 */
    bool startup_complete;
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

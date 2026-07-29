/**
 * @file  step_motor.h
 * @brief BSP 步进电机开环控制接口（摆杆执行器）。
 *
 * 移植自 NUEDC_2026/2026H 的双轴（YAW/PITCH）云台驱动。本工程只有一路步进电机
 * 用于驱动摆杆，故裁剪为单通道；原 PITCH 通道的位置限位逻辑正好是摆杆需要的
 * 机械限位保护，予以保留并改名为通用的 Position 限位。
 *
 * ⚠ 定时器时钟必须为 32MHz / STEP_MOTOR_TIMER_PRESCALER_FACTOR = 62500 Hz，
 *   即 SysConfig 中 SMotor 的 clockDivider=8、clockPrescale=64。
 *   两者不一致会导致 speed(deg/s) → ARR 的换算整体偏离。
 */
#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef STEP_MOTOR_BEAM_DIR_PORT
#define STEP_MOTOR_BEAM_DIR_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_BEAM_DIR_PIN
#define STEP_MOTOR_BEAM_DIR_PIN SMotor_IO_DIR1_PIN
#endif

#ifndef STEP_MOTOR_BEAM_EN_PORT
#define STEP_MOTOR_BEAM_EN_PORT SMotor_IO_PORT
#endif

#ifndef STEP_MOTOR_BEAM_EN_PIN
#define STEP_MOTOR_BEAM_EN_PIN SMotor_IO_EN1_PIN
#endif

/** 使能脚高电平有效时为 1；驱动器为低有效则改为 0。 */
#ifndef STEP_MOTOR_BEAM_ENABLE_HIGH
#define STEP_MOTOR_BEAM_ENABLE_HIGH 1U
#endif

#ifndef STEP_MOTOR_BEAM_PWM_TIMER
#define STEP_MOTOR_BEAM_PWM_TIMER SMotor_INST
#endif

#ifndef STEP_MOTOR_BEAM_PWM_CHANNEL
#define STEP_MOTOR_BEAM_PWM_CHANNEL DL_TIMER_CC_0_INDEX
#endif

/** 正方向对应 DIR 脚高电平时为 1；上板确认转向后按需翻转。 */
#ifndef STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH
#define STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH 1U
#endif

/** 电机固有步距角，单位 deg。 */
#ifndef STEP_MOTOR_STEP_ANGLE_DEG
#define STEP_MOTOR_STEP_ANGLE_DEG 1.8f
#endif

/** 驱动器细分数，须与驱动器拨码一致。 */
#ifndef STEP_MOTOR_MICROSTEP
#define STEP_MOTOR_MICROSTEP 32.0f
#endif

#ifndef STEP_MOTOR_TIMER_CLOCK_HZ
#define STEP_MOTOR_TIMER_CLOCK_HZ 32000000U
#endif

/** = clockDivider(8) × clockPrescale(64)，见文件头说明。 */
#ifndef STEP_MOTOR_TIMER_PRESCALER_FACTOR
#define STEP_MOTOR_TIMER_PRESCALER_FACTOR (32U * 8U * 2U)
#endif

#ifndef STEP_MOTOR_MAX_ARR
#define STEP_MOTOR_MAX_ARR 65535U
#endif

#ifndef STEP_MOTOR_MAX_SPEED_DEG_S
#define STEP_MOTOR_MAX_SPEED_DEG_S 240.0f
#endif

/**
 * 开环估计位置的机械限位默认值，单位 deg（电机轴角，非摆杆角）。
 * ⚠ 必须按实际连杆/丝杆减速比与摆杆行程标定后用
 *   StepMotor_SetPositionLimit() 覆盖，默认值仅防止无限制走飞。
 */
#ifndef STEP_MOTOR_MIN_POSITION_DEG
#define STEP_MOTOR_MIN_POSITION_DEG (-30.0f)
#endif

#ifndef STEP_MOTOR_MAX_POSITION_DEG
#define STEP_MOTOR_MAX_POSITION_DEG 30.0f
#endif

/**
 * @brief 步进电机通道。当前只有摆杆一路，保留枚举便于后续扩展。
 */
typedef enum {
    STEP_MOTOR_CHANNEL_BEAM = 0,
    STEP_MOTOR_CHANNEL_MAX
} STEP_MOTOR_CHANNEL;

/**
 * @brief 步进电机开环估计位置限位。
 */
typedef struct {
    /** 最小估计位置，单位 deg。 */
    float min_deg;
    /** 最大估计位置，单位 deg。 */
    float max_deg;
} STEP_MOTOR_POSITION_LIMIT;

/**
 * @brief 初始化步进电机输出并使能驱动器。
 * @note PWM/GPIO 外设本体由 SysConfig(SYSCFG_DL_init) 初始化，本函数只做上电状态设置。
 */
BSP_STATUS StepMotor_Init(void);

/**
 * @brief 设置指定通道开环速度。
 * @param channel 步进电机通道。
 * @param speed_deg_per_s 速度，单位 deg/s；正负表示方向，0 表示停止脉冲。
 * @note 速度会被限制到 [-STEP_MOTOR_MAX_SPEED_DEG_S, STEP_MOTOR_MAX_SPEED_DEG_S]，
 *       并在触及位置限位时归零。
 */
BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s);

/**
 * @brief 阻塞运行指定时间后停止。
 * @param channel 步进电机通道。
 * @param speed_deg_per_s 速度，单位 deg/s。
 * @param duration_ms 运行时间，单位 ms；会被位置限位裁短。
 * @warning 阻塞式接口，不可在控制拍内调用，仅供自检/标定使用。
 */
BSP_STATUS StepMotor_RunFor(STEP_MOTOR_CHANNEL channel,
                            float speed_deg_per_s,
                            uint32_t duration_ms);

/**
 * @brief 停止指定通道脉冲输出。
 */
BSP_STATUS StepMotor_Stop(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 停止全部通道脉冲输出。
 */
BSP_STATUS StepMotor_StopAll(void);

/**
 * @brief 按当前速度和时间差更新开环估计位置。
 * @param now_ms 当前毫秒时间戳。
 */
BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms);

/**
 * @brief 更新全部通道开环估计位置。
 */
BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms);

/**
 * @brief 获取指定通道最近设置速度，单位 deg/s。
 */
float StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 获取指定通道开环估计位置，单位 deg。
 * @note 该值由速度积分得到，不是编码器反馈位置；丢步不会被察觉。
 */
float StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 将指定通道开环估计位置清零。
 * @note 不执行物理归零，也不会停止电机。
 */
void StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 设置开环估计位置限位。
 */
BSP_STATUS StepMotor_SetPositionLimit(const STEP_MOTOR_POSITION_LIMIT *limit);

/**
 * @brief 获取开环估计位置限位。
 */
STEP_MOTOR_POSITION_LIMIT StepMotor_GetPositionLimit(void);

#ifdef __cplusplus
}
#endif

#endif /* STEP_MOTOR_H */

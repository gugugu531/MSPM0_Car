/**
 * @file  line_follow.h
 * @brief Middleware 层巡线闭环控制器，将标准化灰度观测转换为底盘输出。
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "bsp_common.h"
#include "pid/pid.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 传感器与输出默认参数 ======================== */

/** 巡线算法固定处理 8 个逻辑通道；硬件源由上层适配。 */
#define LINE_FOLLOW_SENSOR_COUNT                       8U
/** 参与巡线的灰度通道掩码；bit0..7 分别对应逻辑通道 0..7。 */
#define LINE_FOLLOW_ACTIVE_SENSOR_MASK                 0xFFU
/** 传感器位置到控制误差的缩放系数；当前 10 表示横向偏移一格约产生 10 单位误差。 */
#define LINE_FOLLOW_DEFAULT_POSITION_SCALE             10.0f
/** 无转向修正时左右轮共同使用的基础占空比，单位 %。 */
#define LINE_FOLLOW_DEFAULT_BASE_DUTY                  34.0f
/** 左右轮最终占空比绝对值上限，单位 %。 */
#define LINE_FOLLOW_DEFAULT_OUTPUT_LIMIT               100.0f
/**
 * 左右轮占空比差值上限，单位百分点。
 *
 * 差速混控满足 left-right=2*correction，因此当前 16 会把 correction 最终限制为 ±8。
 */
#define LINE_FOLLOW_DEFAULT_DIFFERENTIAL_LIMIT         16.0f

/* =========================== 误差预处理参数 =========================== */

/** 灰度误差一阶低通 EMA 系数；范围 (0,1]，越小越平滑但滞后越大。 */
#define LINE_FOLLOW_ERROR_LPF_ALPHA                    0.5f
/**
 * 滤波误差的中心死区半宽。
 *
 * 当前 POSITION_SCALE=10 时，10 约等于一个传感器间距；绝对误差小于该值时不修正。
 */
#define LINE_FOLLOW_ERROR_DEADBAND                     10.0f

/* ======================= 纯位置 PID 退化路径参数 ====================== */

/**
 * 以下参数仅在 gyro_stab_enabled=false 时生效。
 *
 * 默认 Kp=1、Ki=Kd=0，实际为位置式 P 控制；保留完整 PID 参数宏，便于后续独立整定。
 * 数字灰度误差已经过 EMA 与死区，Kd 默认关闭以避免离散质心跳变被微分放大。
 */
#define LINE_FOLLOW_DEFAULT_PID_KP                     1.0f
#define LINE_FOLLOW_DEFAULT_PID_KI                     0.0f
#define LINE_FOLLOW_DEFAULT_PID_KD                     0.0f
/** 积分累计值绝对限幅；限制 integral 本身，不是 Ki*integral。 */
#define LINE_FOLLOW_DEFAULT_PID_INTEGRAL_LIMIT         500.0f
/** PID 原始修正量绝对限幅；之后仍会受到 DIFFERENTIAL_LIMIT 的进一步限制。 */
#define LINE_FOLLOW_DEFAULT_PID_OUTPUT_LIMIT           60.0f
#define LINE_FOLLOW_DEFAULT_PID_MODE                   PID_MODE_POSITION

/* ========================== 陀螺串级默认参数 ========================== */

/**
 * 默认控制路径不是上述 PID，而是“灰度偏差 P 外环 + 角速度 P 内环”：
 *   omega_ref = clamp(gyro_line_kp * error, ±omega_ref_limit)
 *   correction = gyro_stab_kp * (omega_ref - gyro_z_sign * gz)
 *
 * 它利用 gz 反馈补偿左右电机/轮径不对称，并为航向变化提供阻尼。
 */
#define LINE_FOLLOW_DEFAULT_GYRO_STAB_ENABLED          true
/** 灰度误差到期望航向角速度的比例系数；误差 10 对应约 30 deg/s。 */
#define LINE_FOLLOW_DEFAULT_GYRO_LINE_KP               3.0f
/** 航向角速度误差到占空比修正量的比例系数。 */
#define LINE_FOLLOW_DEFAULT_GYRO_STAB_KP               0.20f
/** 期望航向角速度绝对值上限，单位 deg/s。 */
#define LINE_FOLLOW_DEFAULT_OMEGA_REF_LIMIT            60.0f
/** 陀螺 z 轴符号校正；若实机表现为正反馈，应改为 -1.0f。 */
#define LINE_FOLLOW_DEFAULT_GYRO_Z_SIGN                (1.0f)

/** @brief 一次巡线数字电平观测；当前实机语义为 0=黑线，1=非黑线。 */
typedef struct {
    uint8_t level[LINE_FOLLOW_SENSOR_COUNT];
} LINE_FOLLOW_INPUT;

/** 灰度阵列的纯观测结果，不包含 PID、IMU 或电机语义。 */
typedef struct {
    uint8_t level_mask;
    uint8_t black_mask;
    uint8_t black_count;
    float error;
    bool line_lost;
} LINE_FOLLOW_OBSERVATION;

/**
 * @brief 巡线控制配置。
 */
typedef struct {
    /** 基础占空比，左右轮在无偏差时使用该输出。 */
    float base_duty;
    /** 传感器横向位置到控制误差的缩放系数。 */
    float sensor_position_scale;
    /** 左右轮最终输出绝对值限幅。 */
    float output_limit;
    /** 左右轮占空比差值绝对值限幅；<= 0 时不启用。 */
    float differential_limit;
    /** 巡线偏差 PID 配置（陀螺增稳关闭时的纯位置控制路径使用）。 */
    PID_CONFIG pid_config;
    /** 陀螺增稳串级使能；true 时灰度出 omega_ref，false 时使用纯位置 PID。 */
    bool gyro_stab_enabled;
    /** 外环增益：灰度偏差（缩放后）到期望航向角速度 omega_ref。 */
    float gyro_line_kp;
    /** 内环增益：角速度误差（omega_ref-gz）到差速修正。 */
    float gyro_stab_kp;
    /** omega_ref 限幅，单位 deg/s。 */
    float omega_ref_limit;
    /** gz 符号系数：使陀螺内环成为负反馈。 */
    float gyro_z_sign;
} LINE_FOLLOW_CONFIG;

/**
 * @brief 巡线控制计算结果和本拍观测摘要。
 */
typedef struct {
    /** 本拍数字电平掩码；bit i=1 表示通道 i 未检测到黑线。 */
    uint8_t level_mask;
    /** 启用通道中检测到黑线的数量。 */
    uint8_t black_count;
    /** 缩放后的原始巡线误差，负值表示线偏左，正值表示线偏右。 */
    float error;
    /** 当前控制路径计算得到的转向修正量；可能来自陀螺串级或纯位置 PID。 */
    float correction;
    /** 左轮占空比输出。 */
    float left_duty;
    /** 右轮占空比输出。 */
    float right_duty;
    /** 是否没有任何启用通道检测到线。 */
    bool line_lost;
} LINE_FOLLOW_OUTPUT;

/** @brief 获取默认巡线配置，供上层复制后覆盖个别字段。 */
LINE_FOLLOW_CONFIG LineFollow_GetDefaultConfig(void);

/**
 * @brief 初始化巡线控制器。
 * @param config 配置指针；传入 NULL 时使用默认配置。
 */
void LineFollow_Init(const LINE_FOLLOW_CONFIG *config);

/** @brief 重置 PID、滤波状态和最近输出。 */
void LineFollow_Reset(void);

/**
 * @brief 使用归一化黑线检测掩码推进完整巡线闭环并驱动底盘。
 * @param detected_mask bit0..7 对应 X1..X8；1=检测到黑线。
 * @param dt_s 控制周期，单位 s。
 * @note 这是 Yahboom 等 I2C 观测源应使用的唯一入口，不在本层访问总线。
 *       JY61P 的初始化和 JY61P_I2C_Poll() 调度由 app 任务负责。
 */
BSP_STATUS LineFollow_UpdateDetectedMask(uint8_t detected_mask, float dt_s);

/**
 * @brief 将标准化数字电平转换为黑线掩码和质心误差，不使用控制器运行状态。
 * @param input 灰度观测，0=黑线、1=非黑线。
 * @param position_scale 传感器位置到误差的缩放系数。
 * @param out 纯观测结果。
 */
BSP_STATUS LineFollow_Observe(const LINE_FOLLOW_INPUT *input,
                              float position_scale,
                              LINE_FOLLOW_OBSERVATION *out);

/**
 * @brief 从归一化黑线检测掩码计算质心观测。
 * @param detected_mask bit0..7 对应逻辑通道 0..7；1=检测到黑线。
 */
BSP_STATUS LineFollow_ObserveDetectedMask(uint8_t detected_mask,
                                          float position_scale,
                                          LINE_FOLLOW_OBSERVATION *out);

/**
 * @brief 基于给定标准化观测计算巡线输出，不读取硬件、不驱动底盘。
 * @param input 巡线观测；当前实机上 level[i]==0 表示通道 i 检测到黑线。
 * @param dt_s 控制周期，单位 s。
 * @param omega_deg_s 已完成符号校正的车体航向角速度，单位 deg/s。
 * @param out 输出结果。
 */
BSP_STATUS LineFollow_Compute(const LINE_FOLLOW_INPUT *input,
                              float dt_s,
                              float omega_deg_s,
                              LINE_FOLLOW_OUTPUT *out);

/** @brief 获取最近一次巡线输出和观测摘要。 */
LINE_FOLLOW_OUTPUT LineFollow_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_H */

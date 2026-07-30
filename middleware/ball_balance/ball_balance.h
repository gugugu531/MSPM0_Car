/**
 * @file  ball_balance.h
 * @brief 一维滚球位置/速度外环（纯计算，不直接访问硬件）。
 *
 * 正方向约定：水管正倾角使球向正方向加速。调用方必须保证视觉 x/v 与该方向一致。
 */
#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float reference_speed_limit_mm_s;
    float position_scale_mm;
    float braking_acceleration_mm_s2;
    float braking_distance_weight;
    float velocity_angle_limit_deg;
    float velocity_scale_mm_s;
    float acceleration_gain_deg_per_mm_s2;
    float acceleration_filter_tau_s;
    float acceleration_limit_mm_s2;
    float zero_trim_ki_deg_per_mm_s;
    float zero_trim_limit_deg;
    float angle_min_deg;
    float angle_max_deg;
    float control_angle_limit_deg;
    float angle_rate_near_deg_s;
    float angle_rate_far_deg_s;
    float angle_rate_brake_deg_s;
    float angle_rate_transition_mm;
    float stuck_min_error_mm;
    float stuck_max_position_change_mm;
    float stuck_min_command_deg;
    float stuck_time_s;
    float stuck_angle_limit_deg;
    float trim_min_position_mm;
    float trim_max_position_mm;
    float trim_min_speed_mm_s;
    float trim_max_speed_mm_s;
    float settled_position_mm;
    float settled_speed_mm_s;
    float settled_time_s;
} BALL_BALANCE_CONFIG;

typedef struct {
    float target_x_mm;
    float zero_trim_deg;
    float output_deg;
    float previous_velocity_mm_s;
    float previous_position_mm;
    float filtered_acceleration_mm_s2;
    float stagnant_elapsed_s;
    float settled_elapsed_s;
    bool  have_previous_velocity;
    bool  have_previous_position;
    bool  intercepted;
    bool  settled;
} BALL_BALANCE_CONTROLLER;

typedef struct {
    float x_mm;
    float velocity_mm_s;
    float dt_s;
    bool  moving;
    bool  allow_zero_trim;
} BALL_BALANCE_INPUT;

typedef struct {
    float angle_deg;
    float feedback_deg;
    float stopping_error_mm;
    float reference_velocity_mm_s;
    float velocity_error_mm_s;
    float speed_feedback_deg;
    float acceleration_feedback_deg;
    float acceleration_mm_s2;
    float angle_rate_limit_deg_s;
    float zero_trim_deg;
    bool  saturated;
    bool  braking;
    bool  intercepted;
    bool  settled;
} BALL_BALANCE_OUTPUT;

/** 初始化并清空控制器状态。 */
void BallBalance_Init(BALL_BALANCE_CONTROLLER *controller);

/** 清空动态状态，并令下一拍从给定实际水管角开始做斜率限制。 */
void BallBalance_Reset(BALL_BALANCE_CONTROLLER *controller,
                       float current_angle_deg);

/** 设置目标位置，单位 mm。H3 守中任务固定取 0。 */
void BallBalance_SetTarget(BALL_BALANCE_CONTROLLER *controller,
                           float target_x_mm);

/** 执行一次非线性位置到速度外环 + 速度到倾角内环更新。 */
bool BallBalance_Update(BALL_BALANCE_CONTROLLER *controller,
                        const BALL_BALANCE_CONFIG *config,
                        const BALL_BALANCE_INPUT *input,
                        BALL_BALANCE_OUTPUT *output);

#ifdef __cplusplus
}
#endif

#endif /* BALL_BALANCE_H */

/**
 * @file  ball_balance.c
 * @brief 一维滚球位置/速度外环实现。
 */
#include "ball_balance.h"

#include <math.h>
#include <stddef.h>

static float BallBalance_Clamp(float value, float low, float high){
    if (value < low){
        return low;
    }
    if (value > high){
        return high;
    }
    return value;
}

static bool BallBalance_ConfigValid(const BALL_BALANCE_CONFIG *config){
    return (config != NULL) &&
           (config->reference_speed_limit_mm_s > 0.0f) &&
           (config->position_scale_mm > 0.0f) &&
           (config->braking_acceleration_mm_s2 > 0.0f) &&
           (config->braking_distance_weight >= 0.0f) &&
           (config->velocity_angle_limit_deg >= 0.0f) &&
           (config->velocity_scale_mm_s > 0.0f) &&
           (config->acceleration_gain_deg_per_mm_s2 >= 0.0f) &&
           (config->acceleration_filter_tau_s > 0.0f) &&
           (config->acceleration_limit_mm_s2 > 0.0f) &&
           (config->zero_trim_ki_deg_per_mm_s >= 0.0f) &&
           (config->zero_trim_limit_deg >= 0.0f) &&
           (config->angle_min_deg < config->angle_max_deg) &&
           (config->control_angle_limit_deg > 0.0f) &&
           (config->control_angle_limit_deg <= config->angle_max_deg) &&
           (config->control_angle_limit_deg <= -config->angle_min_deg) &&
           (config->angle_rate_near_deg_s > 0.0f) &&
           (config->angle_rate_far_deg_s > 0.0f) &&
           (config->angle_rate_brake_deg_s >= config->angle_rate_near_deg_s) &&
           (config->angle_rate_near_deg_s >= config->angle_rate_far_deg_s) &&
           (config->angle_rate_transition_mm > 0.0f) &&
           (config->stuck_min_error_mm >= 0.0f) &&
           (config->stuck_max_position_change_mm >= 0.0f) &&
           (config->stuck_min_command_deg >= 0.0f) &&
           (config->stuck_time_s >= 0.0f) &&
           (config->stuck_angle_limit_deg > 0.0f) &&
           (config->stuck_angle_limit_deg <= config->control_angle_limit_deg) &&
           (config->settled_position_mm >= 0.0f) &&
           (config->settled_speed_mm_s >= 0.0f) &&
           (config->settled_time_s >= 0.0f);
}

void BallBalance_Init(BALL_BALANCE_CONTROLLER *controller){
    if (controller == NULL){
        return;
    }
    controller->target_x_mm = 0.0f;
    controller->zero_trim_deg = 0.0f;
    controller->output_deg = 0.0f;
    controller->previous_velocity_mm_s = 0.0f;
    controller->previous_position_mm = 0.0f;
    controller->filtered_acceleration_mm_s2 = 0.0f;
    controller->stagnant_elapsed_s = 0.0f;
    controller->settled_elapsed_s = 0.0f;
    controller->have_previous_velocity = false;
    controller->have_previous_position = false;
    controller->intercepted = false;
    controller->settled = false;
}

void BallBalance_Reset(BALL_BALANCE_CONTROLLER *controller,
                       float current_angle_deg){
    if (controller == NULL){
        return;
    }
    controller->zero_trim_deg = 0.0f;
    controller->output_deg = current_angle_deg;
    controller->previous_velocity_mm_s = 0.0f;
    controller->previous_position_mm = 0.0f;
    controller->filtered_acceleration_mm_s2 = 0.0f;
    controller->stagnant_elapsed_s = 0.0f;
    controller->settled_elapsed_s = 0.0f;
    controller->have_previous_velocity = false;
    controller->have_previous_position = false;
    controller->intercepted = false;
    controller->settled = false;
}

void BallBalance_SetTarget(BALL_BALANCE_CONTROLLER *controller,
                           float target_x_mm){
    if (controller == NULL){
        return;
    }
    controller->target_x_mm = target_x_mm;
    controller->settled_elapsed_s = 0.0f;
    controller->settled = false;
}

bool BallBalance_Update(BALL_BALANCE_CONTROLLER *controller,
                        const BALL_BALANCE_CONFIG *config,
                        const BALL_BALANCE_INPUT *input,
                        BALL_BALANCE_OUTPUT *output){
    if ((controller == NULL) || (input == NULL) || (output == NULL) ||
        !BallBalance_ConfigValid(config) || (input->dt_s <= 0.0f)){
        return false;
    }

    float error_mm = input->x_mm - controller->target_x_mm;
    float abs_error = fabsf(error_mm);
    float abs_speed = fabsf(input->velocity_mm_s);

    /*
     * 加速度由速度差分得到，先做物理限幅再一阶低通。它不是滚球二阶模型的独立
     * 状态，默认只作诊断；实机确认噪声和相位延迟后才应把 Ka 从 0 往上调。
     */
    if (controller->have_previous_velocity){
        float raw_acceleration =
            (input->velocity_mm_s - controller->previous_velocity_mm_s) / input->dt_s;
        raw_acceleration = BallBalance_Clamp(
            raw_acceleration,
            -config->acceleration_limit_mm_s2,
            config->acceleration_limit_mm_s2);
        float acceleration_alpha = input->dt_s /
            (config->acceleration_filter_tau_s + input->dt_s);
        controller->filtered_acceleration_mm_s2 += acceleration_alpha *
            (raw_acceleration - controller->filtered_acceleration_mm_s2);
    } else{
        controller->have_previous_velocity = true;
    }
    controller->previous_velocity_mm_s = input->velocity_mm_s;

    /*
     * 只在球确实运动且仍处于中心控制区时慢速学习零偏。球静止时保留 PPR 管的
     * 被动黏滞，不让积分项为“脱困”不断蓄力而形成守位极限环。
     */
    bool trim_window = input->allow_zero_trim &&
                       (abs_error >= config->trim_min_position_mm) &&
                       (abs_error <= config->trim_max_position_mm) &&
                       (abs_speed >= config->trim_min_speed_mm_s) &&
                       (abs_speed <= config->trim_max_speed_mm_s) &&
                       (controller->output_deg >
                        (config->angle_min_deg +
                         0.1f * (config->angle_max_deg - config->angle_min_deg))) &&
                       (controller->output_deg <
                        (config->angle_max_deg -
                         0.1f * (config->angle_max_deg - config->angle_min_deg)));
    if (trim_window){
        controller->zero_trim_deg -=
            config->zero_trim_ki_deg_per_mm_s * error_mm * input->dt_s;
        controller->zero_trim_deg = BallBalance_Clamp(
            controller->zero_trim_deg,
            -config->zero_trim_limit_deg,
            config->zero_trim_limit_deg);
    }

    /*
     * 相平面停止坐标把当前动能折算为制动距离。球快速靠近中心且剩余距离不足时，
     * stopping_error 会提前越过零点，外环立即请求反向速度，而不是等位置过零后才刹车。
     */
    float stopping_distance_mm = input->velocity_mm_s * abs_speed /
        (2.0f * config->braking_acceleration_mm_s2);
    float stopping_error_mm = error_mm +
        config->braking_distance_weight * stopping_distance_mm;
    float reference_velocity_mm_s = -config->reference_speed_limit_mm_s *
        tanhf(stopping_error_mm / config->position_scale_mm);
    float velocity_error_mm_s = input->velocity_mm_s - reference_velocity_mm_s;

    /* 速度内环同样用 tanh 软饱和：近端细腻，强扰动时平滑趋近最大控制角。 */
    float speed_feedback_deg = -config->velocity_angle_limit_deg *
        tanhf(velocity_error_mm_s / config->velocity_scale_mm_s);
    float acceleration_feedback_deg =
        -config->acceleration_gain_deg_per_mm_s2 *
         controller->filtered_acceleration_mm_s2;
    float feedback_deg = speed_feedback_deg + acceleration_feedback_deg;
    float requested_deg = feedback_deg + controller->zero_trim_deg;
    float unrestricted_deg = requested_deg;
    requested_deg = BallBalance_Clamp(requested_deg,
                                      -config->control_angle_limit_deg,
                                      config->control_angle_limit_deg);

    /*
     * 工具截停或球被黏滞卡住时，禁止控制器持续维持大倾角。位置重新变化或 MOVING
     * 恢复后立即解除；由于基础调参阶段 Ki=0，此状态也不会积累隐藏的积分能量。
     */
    float position_change = controller->have_previous_position
        ? fabsf(input->x_mm - controller->previous_position_mm) : 0.0f;
    bool stagnant = controller->have_previous_position && !input->moving &&
                    (position_change <= config->stuck_max_position_change_mm) &&
                    (abs_error >= config->stuck_min_error_mm) &&
                    (fabsf(requested_deg) >= config->stuck_min_command_deg);
    if (stagnant){
        controller->stagnant_elapsed_s += input->dt_s;
        if (controller->stagnant_elapsed_s >= config->stuck_time_s){
            controller->intercepted = true;
        }
    } else{
        controller->stagnant_elapsed_s = 0.0f;
        controller->intercepted = false;
    }
    controller->previous_position_mm = input->x_mm;
    controller->have_previous_position = true;
    if (controller->intercepted){
        requested_deg = BallBalance_Clamp(requested_deg,
                                          -config->stuck_angle_limit_deg,
                                          config->stuck_angle_limit_deg);
    }

    float limited_deg = BallBalance_Clamp(requested_deg,
                                          config->angle_min_deg,
                                          config->angle_max_deg);
    bool saturated = (limited_deg != unrestricted_deg) ||
                     controller->intercepted;

    /*
     * 普通纠偏仍满足“远端慢、近端快”；若目标角与当前速度反向，则判为制动，
     * 临时使用更高变化率，以便外界拨动后先消掉动能。
     */
    float rate_ratio = abs_error / config->angle_rate_transition_mm;
    float scheduled_rate_deg_s = config->angle_rate_far_deg_s +
        (config->angle_rate_near_deg_s - config->angle_rate_far_deg_s) /
        (1.0f + rate_ratio * rate_ratio);
    bool braking = (abs_speed > config->settled_speed_mm_s) &&
                   ((input->velocity_mm_s * limited_deg) < 0.0f);
    if (braking){
        scheduled_rate_deg_s = config->angle_rate_brake_deg_s;
    }
    float max_step_deg = scheduled_rate_deg_s * input->dt_s;
    float change_deg = limited_deg - controller->output_deg;
    change_deg = BallBalance_Clamp(change_deg, -max_step_deg, max_step_deg);
    controller->output_deg += change_deg;

    if ((abs_error <= config->settled_position_mm) &&
        (abs_speed <= config->settled_speed_mm_s)){
        controller->settled_elapsed_s += input->dt_s;
        if (controller->settled_elapsed_s >= config->settled_time_s){
            controller->settled = true;
        }
    } else{
        controller->settled_elapsed_s = 0.0f;
        controller->settled = false;
    }

    output->angle_deg = controller->output_deg;
    output->feedback_deg = feedback_deg;
    output->stopping_error_mm = stopping_error_mm;
    output->reference_velocity_mm_s = reference_velocity_mm_s;
    output->velocity_error_mm_s = velocity_error_mm_s;
    output->speed_feedback_deg = speed_feedback_deg;
    output->acceleration_feedback_deg = acceleration_feedback_deg;
    output->acceleration_mm_s2 = controller->filtered_acceleration_mm_s2;
    output->angle_rate_limit_deg_s = scheduled_rate_deg_s;
    output->zero_trim_deg = controller->zero_trim_deg;
    output->saturated = saturated;
    output->braking = braking;
    output->intercepted = controller->intercepted;
    output->settled = controller->settled;
    return true;
}

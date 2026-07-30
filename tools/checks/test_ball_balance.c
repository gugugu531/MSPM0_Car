#include "ball_balance.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static const BALL_BALANCE_CONFIG CONFIG = {
    .reference_speed_limit_mm_s = 80.0f,
    .position_scale_mm = 45.0f,
    .braking_acceleration_mm_s2 = 200.0f,
    .braking_distance_weight = 1.0f,
    .velocity_angle_limit_deg = 3.0f,
    .velocity_scale_mm_s = 55.0f,
    .acceleration_gain_deg_per_mm_s2 = 0.0f,
    .acceleration_filter_tau_s = 0.15f,
    .acceleration_limit_mm_s2 = 1500.0f,
    .zero_trim_ki_deg_per_mm_s = 0.0f,
    .zero_trim_limit_deg = 0.5f,
    .angle_min_deg = -6.7f,
    .angle_max_deg = 4.7f,
    .control_angle_limit_deg = 3.0f,
    .angle_rate_near_deg_s = 32.0f,
    .angle_rate_far_deg_s = 10.0f,
    .angle_rate_brake_deg_s = 48.0f,
    .angle_rate_transition_mm = 30.0f,
    .stuck_min_error_mm = 5.0f,
    .stuck_max_position_change_mm = 0.5f,
    .stuck_min_command_deg = 0.5f,
    .stuck_time_s = 0.4f,
    .stuck_angle_limit_deg = 0.7f,
    .trim_min_position_mm = 1.0f,
    .trim_max_position_mm = 100.0f,
    .trim_min_speed_mm_s = 0.5f,
    .trim_max_speed_mm_s = 300.0f,
    .settled_position_mm = 2.0f,
    .settled_speed_mm_s = 5.0f,
    .settled_time_s = 1.0f,
};

static BALL_BALANCE_INPUT Input(float x_mm, float velocity_mm_s, bool moving){
    BALL_BALANCE_INPUT input = {
        .x_mm = x_mm,
        .velocity_mm_s = velocity_mm_s,
        .dt_s = 0.02f,
        .moving = moving,
        .allow_zero_trim = false,
    };
    return input;
}

int main(void){
    BALL_BALANCE_CONTROLLER controller;
    BALL_BALANCE_OUTPUT output;

    /* 静止位置误差先转换为朝中心的目标速度，再由速度内环给出小角度。 */
    BallBalance_Init(&controller);
    BallBalance_SetTarget(&controller, 0.0f);
    BALL_BALANCE_INPUT input = Input(10.0f, 0.0f, false);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(output.stopping_error_mm == 10.0f);
    assert(output.reference_velocity_mm_s < 0.0f);
    assert(output.velocity_error_mm_s > 0.0f);
    assert(output.speed_feedback_deg < 0.0f);
    assert(output.angle_deg < -0.58f && output.angle_deg > -0.61f);

    /* 远端目标反馈更大，但普通纠偏角速度更低。 */
    BALL_BALANCE_CONTROLLER near_controller;
    BALL_BALANCE_CONTROLLER far_controller;
    BALL_BALANCE_OUTPUT near_output;
    BALL_BALANCE_OUTPUT far_output;
    BallBalance_Init(&near_controller);
    BallBalance_Init(&far_controller);
    input = Input(5.0f, 0.0f, false);
    assert(BallBalance_Update(&near_controller, &CONFIG, &input, &near_output));
    input = Input(100.0f, 0.0f, false);
    assert(BallBalance_Update(&far_controller, &CONFIG, &input, &far_output));
    assert(fabsf(far_output.feedback_deg) > fabsf(near_output.feedback_deg));
    assert(far_output.angle_rate_limit_deg_s < near_output.angle_rate_limit_deg_s);

    /* 快速靠近时，停止距离使外环提前翻向，进入高变化率制动。 */
    BallBalance_Init(&controller);
    input = Input(20.0f, -100.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(output.stopping_error_mm < 0.0f);
    assert(output.reference_velocity_mm_s > 0.0f);
    assert(output.feedback_deg > 0.0f);
    assert(output.braking);
    assert(output.angle_rate_limit_deg_s == CONFIG.angle_rate_brake_deg_s);

    /* 速度差分只形成诊断加速度；Ka=0 时不直接污染控制输出。 */
    input = Input(20.0f, -80.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(output.acceleration_mm_s2 > 100.0f);
    assert(output.acceleration_feedback_deg == 0.0f);

    /* 工具截停：静止超时后把倾角压到小值；位置恢复变化时立即解除。 */
    BallBalance_Init(&controller);
    input = Input(20.0f, 0.0f, false);
    for (unsigned i = 0U; i < 30U; i++){
        assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    }
    assert(output.intercepted);
    assert(fabsf(output.angle_deg) <= CONFIG.stuck_angle_limit_deg + 0.001f);
    input = Input(18.0f, -10.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(!output.intercepted);

    /* 基础阶段关闭零偏积分，即使调用方误开许可也不能污染长期状态。 */
    BallBalance_Init(&controller);
    input = Input(5.0f, 10.0f, true);
    input.allow_zero_trim = true;
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(output.zero_trim_deg == 0.0f);

    BallBalance_Init(&controller);
    input = Input(1.0f, 2.0f, true);
    for (unsigned i = 0U; i < 51U; i++){
        assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    }
    assert(output.settled);
    input = Input(3.0f, 2.0f, true);
    assert(BallBalance_Update(&controller, &CONFIG, &input, &output));
    assert(!output.settled);

    puts("ball_balance tests passed");
    return 0;
}

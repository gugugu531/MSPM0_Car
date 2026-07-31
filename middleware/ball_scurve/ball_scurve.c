/**
 * @file  ball_scurve.c
 * @brief 纯五次 S 曲线滚球点到点控制的实现。
 */
#include "ball_scurve.h"

#include <math.h>

#define BALL_SCURVE_RAD_TO_DEG 57.29577951308232f

/*
 * 静止到静止的五次剖面峰值加速度 = PEAK_ACC_FACTOR·Δx/T²。
 * 推导：s = t/T 时 a(s) ∝ 60s − 180s² + 120s³，极值在 s = (1−1/√3)/2 = 0.21132，
 * 代回得 5.7735 = 10/√3。
 */
#define BALL_SCURVE_PEAK_ACC_FACTOR 5.773502691896258f
/* 同一剖面的峰值速度 = 1.875·Δx/T（s = 0.5 处 30s²(1−s)² 的极值 15/8）。 */
#define BALL_SCURVE_PEAK_VEL_FACTOR 1.875f
/* 时长迭代次数：起点带初速时闭式时长只是近似，需按真实峰值加速度收紧。 */
#define BALL_SCURVE_DURATION_ITERATIONS 4U
/* 峰值加速度校核的采样点数（含端点）。 */
#define BALL_SCURVE_ACC_SAMPLES 21U

static float BallScurve_Clamp(float value, float low, float high){
    if (value < low){ return low; }
    if (value > high){ return high; }
    return value;
}

static float BallScurve_Abs(float value){
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 由归一化时间求剖面的位置/速度/加速度。
 *
 * x(t) = x0 + v0·t + a0·t²/2 + c0·s³ + c1·s⁴ + c2·s⁵,  s = t/T
 *
 * 三个系数由末端 x=target、v=0、a=0 三个条件唯一确定（见 PlanTo）。
 */
static void BallScurve_Evaluate(const BALL_SCURVE_CONTROLLER *controller,
                                float time_s,
                                float *x_mm,
                                float *v_mm_s,
                                float *a_mm_s2){
    float duration = controller->duration_s;
    float s = time_s / duration;
    float s2 = s * s;
    float s3 = s2 * s;
    float s4 = s3 * s;
    float s5 = s4 * s;
    const float *c = controller->c;

    *x_mm = controller->x0_mm + controller->v0_mm_s * time_s +
            0.5f * controller->a0_mm_s2 * time_s * time_s +
            c[0] * s3 + c[1] * s4 + c[2] * s5;

    *v_mm_s = controller->v0_mm_s + controller->a0_mm_s2 * time_s +
              (3.0f * c[0] * s2 + 4.0f * c[1] * s3 + 5.0f * c[2] * s4) / duration;

    *a_mm_s2 = controller->a0_mm_s2 +
               (6.0f * c[0] * s + 12.0f * c[1] * s2 + 20.0f * c[2] * s3) /
               (duration * duration);
}

/** 解出给定时长下的三个归一化系数。 */
static void BallScurve_SolveCoefficients(BALL_SCURVE_CONTROLLER *controller,
                                         float target_mm,
                                         float duration_s){
    /*
     * 记 T = duration，剩余量为末端条件减去"惯性外推"的部分：
     *   dx = target − (x0 + v0·T + a0·T²/2)
     *   dv = (0 − (v0 + a0·T))·T          （已乘 T，与 s 的求导对齐）
     *   da = (0 − a0)·T²
     * 则 c0 = 10dx − 4dv + 0.5da, c1 = −15dx + 7dv − da, c2 = 6dx − 3dv + 0.5da。
     * 校核：c0+c1+c2 = dx，3c0+4c1+5c2 = dv，6c0+12c1+20c2 = da。
     */
    float t2 = duration_s * duration_s;
    float dx = target_mm - (controller->x0_mm +
                            controller->v0_mm_s * duration_s +
                            0.5f * controller->a0_mm_s2 * t2);
    float dv = -(controller->v0_mm_s + controller->a0_mm_s2 * duration_s) * duration_s;
    float da = -controller->a0_mm_s2 * t2;

    controller->duration_s = duration_s;
    controller->c[0] = 10.0f * dx - 4.0f * dv + 0.5f * da;
    controller->c[1] = -15.0f * dx + 7.0f * dv - da;
    controller->c[2] = 6.0f * dx - 3.0f * dv + 0.5f * da;
}

/** 采样求剖面全程的峰值 |a|，用于时长迭代校核。 */
static float BallScurve_PeakAcceleration(const BALL_SCURVE_CONTROLLER *controller){
    float peak = 0.0f;
    for (unsigned i = 0U; i < BALL_SCURVE_ACC_SAMPLES; i++){
        float time_s = controller->duration_s *
                       ((float)i / (float)(BALL_SCURVE_ACC_SAMPLES - 1U));
        float x;
        float v;
        float a;
        BallScurve_Evaluate(controller, time_s, &x, &v, &a);
        a = BallScurve_Abs(a);
        if (a > peak){ peak = a; }
    }
    return peak;
}

void BallScurve_Init(BALL_SCURVE_CONTROLLER *controller){
    if (controller == 0){ return; }
    controller->x0_mm = 0.0f;
    controller->v0_mm_s = 0.0f;
    controller->a0_mm_s2 = 0.0f;
    for (unsigned i = 0U; i < BALL_SCURVE_COEFFICIENTS; i++){
        controller->c[i] = 0.0f;
    }
    controller->duration_s = 1.0f;
    controller->elapsed_s = 0.0f;
    controller->target_mm = 0.0f;
    controller->active = false;
    controller->last_angle_deg = 0.0f;
    controller->have_last_angle = false;
    controller->settled_elapsed_s = 0.0f;
    controller->settled = false;
    controller->dither_phase_rad = 0.0f;
    controller->dither_stuck_elapsed_s = 0.0f;
    controller->dither_on = false;
    controller->breakout_angle_deg = 0.0f;
    controller->breakout_stuck_elapsed_s = 0.0f;
    controller->breakout_release_elapsed_s = 0.0f;
    controller->breakout_on = false;
}

void BallScurve_Reset(BALL_SCURVE_CONTROLLER *controller, float current_angle_deg){
    if (controller == 0){ return; }
    float target = controller->target_mm;
    BallScurve_Init(controller);
    controller->target_mm = target;
    controller->last_angle_deg = current_angle_deg;
    controller->have_last_angle = true;
}

float BallScurve_EstimateDuration(const BALL_SCURVE_CONFIG *config, float distance_mm){
    if ((config == 0) || (config->max_acceleration_mm_s2 <= 0.0f) ||
        (config->max_velocity_mm_s <= 0.0f)){
        return 0.0f;
    }
    float distance = BallScurve_Abs(distance_mm);
    float by_acceleration = sqrtf(BALL_SCURVE_PEAK_ACC_FACTOR * distance /
                                  config->max_acceleration_mm_s2);
    float by_velocity = BALL_SCURVE_PEAK_VEL_FACTOR * distance / config->max_velocity_mm_s;
    float duration = (by_acceleration > by_velocity) ? by_acceleration : by_velocity;
    if (duration < config->min_duration_s){ duration = config->min_duration_s; }
    if ((config->max_duration_s > 0.0f) && (duration > config->max_duration_s)){
        duration = config->max_duration_s;
    }
    return duration;
}

bool BallScurve_PlanTo(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       float x_now_mm,
                       float v_now_mm_s,
                       float target_mm){
    if ((controller == 0) || (config == 0)){ return false; }
    if ((config->rolling_acceleration_gain_mm_s2 <= 0.0f) ||
        (config->max_acceleration_mm_s2 <= 0.0f) ||
        (config->max_velocity_mm_s <= 0.0f) ||
        (config->min_duration_s <= 0.0f)){
        return false;
    }

    controller->x0_mm = x_now_mm;
    controller->v0_mm_s = v_now_mm_s;
    /*
     * 规划起点加速度取 0 而不是实测加速度：视觉只给位置，二阶差分的噪声
     * 远大于信号，把它塞进边界条件只会让剖面开头出现虚假的大加速度。
     * 代价是剖面首拍的 a_ref 与实际有阶跃，由输出斜率限制吸收。
     */
    controller->a0_mm_s2 = 0.0f;
    controller->target_mm = target_mm;
    controller->elapsed_s = 0.0f;

    float duration = BallScurve_EstimateDuration(config, target_mm - x_now_mm);
    if (duration <= 0.0f){ return false; }
    BallScurve_SolveCoefficients(controller, target_mm, duration);

    /*
     * 起点带初速时，闭式时长公式（按静止到静止导出）会低估峰值加速度，
     * 因此按实际采样峰值迭代收紧：T ← T·sqrt(peak/limit)。
     * a ∝ 1/T²，一次迭代即接近收敛，取 4 次留足余量。
     */
    for (unsigned i = 0U; i < BALL_SCURVE_DURATION_ITERATIONS; i++){
        float peak = BallScurve_PeakAcceleration(controller);
        if (peak <= config->max_acceleration_mm_s2){ break; }
        duration *= sqrtf(peak / config->max_acceleration_mm_s2);
        if ((config->max_duration_s > 0.0f) && (duration > config->max_duration_s)){
            duration = config->max_duration_s;
            BallScurve_SolveCoefficients(controller, target_mm, duration);
            break;
        }
        BallScurve_SolveCoefficients(controller, target_mm, duration);
    }

    controller->active = true;
    controller->settled_elapsed_s = 0.0f;
    controller->settled = false;
    return true;
}

bool BallScurve_IsProfileFinished(const BALL_SCURVE_CONTROLLER *controller){
    if (controller == 0){ return true; }
    return !controller->active;
}

bool BallScurve_Update(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       const BALL_SCURVE_INPUT *input,
                       BALL_SCURVE_OUTPUT *output){
    if ((controller == 0) || (config == 0) || (input == 0) || (output == 0)){
        return false;
    }
    if ((input->dt_s <= 0.0f) || (config->rolling_acceleration_gain_mm_s2 <= 0.0f)){
        return false;
    }

    /* --- 1. 推进剖面时钟并取参考量 --- */
    float x_ref;
    float v_ref;
    float a_ref;
    if (controller->active){
        controller->elapsed_s += input->dt_s;
        if (controller->elapsed_s >= controller->duration_s){
            controller->elapsed_s = controller->duration_s;
            controller->active = false;
        }
        BallScurve_Evaluate(controller, controller->elapsed_s, &x_ref, &v_ref, &a_ref);
    } else{
        /*
         * 剖面走完后参考量钉在目标点，控制律退化为绕目标的纯 PD。
         * 这里刻意不做任何低速捕获处理——本模块就是要暴露这个基线的落点误差。
         */
        x_ref = controller->target_mm;
        v_ref = 0.0f;
        a_ref = 0.0f;
    }

    /* --- 2. 可选重规划（默认关闭，replan_error_mm = 0 即纯 S 曲线）--- */
    if (controller->active && (config->replan_error_mm > 0.0f) &&
        (BallScurve_Abs(x_ref - input->x_mm) > config->replan_error_mm)){
        if (BallScurve_PlanTo(controller, config, input->x_mm, input->velocity_mm_s,
                              controller->target_mm)){
            BallScurve_Evaluate(controller, 0.0f, &x_ref, &v_ref, &a_ref);
        }
    }

    /* --- 3. 加速度前馈 asin(a_ref/K_G) --- */
    float ratio = BallScurve_Clamp(a_ref / config->rolling_acceleration_gain_mm_s2,
                                   -0.999f, 0.999f);
    float feedforward_deg = asinf(ratio) * BALL_SCURVE_RAD_TO_DEG;

    /* --- 4. 滚阻前馈，按参考速度方向施加 --- */
    float rolling_ff_deg = 0.0f;
    if ((config->rolling_resistance_deg != 0.0f) &&
        (BallScurve_Abs(v_ref) > config->rolling_ff_speed_deadband_mm_s)){
        rolling_ff_deg = (v_ref > 0.0f) ? config->rolling_resistance_deg
                                        : -config->rolling_resistance_deg;
    }

    /* --- 5. 剖面跟踪反馈（只对剖面误差，不对目标误差）--- */
    float position_error = x_ref - input->x_mm;
    float velocity_error = v_ref - input->velocity_mm_s;
    float raw_feedback_deg = config->kp_deg_per_mm * position_error +
                             config->kd_deg_per_mm_s * velocity_error;
    float feedback_deg = raw_feedback_deg;
    if (config->feedback_limit_deg > 0.0f){
        feedback_deg = BallScurve_Clamp(feedback_deg,
                                        -config->feedback_limit_deg,
                                        config->feedback_limit_deg);
    }
    bool feedback_clipped = (feedback_deg != raw_feedback_deg);

    /* --- 6. 抖动：只在球确实被静摩擦钉住时注入 --- */
    /*
     * 触发条件三选三：剖面已走完（HOLD 段）、误差仍超判据、球基本不动。
     * 前两条保证"确实需要"，第三条保证"确实是卡住而不是还在滚"。
     * 误差进入 dither_min_error_mm 后自动停振，让静摩擦把球钉在那里——
     * 这同时提供了捕获：抖动破摩擦让球爬过去，停振让摩擦把它锁住。
     */
    float dither_deg = 0.0f;
    float target_error_now = controller->target_mm - input->x_mm;
    if ((config->dither_amplitude_deg > 0.0f) && !controller->active){
        bool needed =
            (BallScurve_Abs(target_error_now) > config->dither_min_error_mm) &&
            (BallScurve_Abs(input->velocity_mm_s) < config->dither_max_speed_mm_s);
        if (needed){
            controller->dither_stuck_elapsed_s += input->dt_s;
        } else{
            controller->dither_stuck_elapsed_s = 0.0f;
        }
        controller->dither_on =
            (controller->dither_stuck_elapsed_s >= config->dither_dwell_s);
    } else{
        controller->dither_stuck_elapsed_s = 0.0f;
        controller->dither_on = false;
    }
    if (controller->dither_on){
        controller->dither_phase_rad +=
            2.0f * 3.14159265358979f * config->dither_frequency_hz * input->dt_s;
        /* 相位取模，避免长时间累加后单精度失去分辨率。 */
        while (controller->dither_phase_rad > 2.0f * 3.14159265358979f){
            controller->dither_phase_rad -= 2.0f * 3.14159265358979f;
        }
        dither_deg = config->dither_amplitude_deg * sinf(controller->dither_phase_rad);
    } else{
        /* 从零相位重新起振，保证第一拍不产生角度阶跃。 */
        controller->dither_phase_rad = 0.0f;
    }

    /* --- 6b. 单向渐增脱困：只朝目标方向加倾角，球一动就撤 --- */
    /*
     * 与抖动**互斥**（见下方 breakout_enabled 判断），因为两者解决同一个问题
     * 但代价不同：
     *
     *   抖动   —— 双向往复。能脱困，但向系统注入往复能量，末端不安静；
     *              且经步进位置环衰减后（τ=1/SERVO_KP=0.333 s @ KP=3），
     *              2 Hz 时管身只剩指令的 23%，实际破不了 0.62° 的脱离角。
     *   单向脱困 —— 只朝目标方向加倾角。是直流量，**不受位置环频率衰减**，
     *              指令 1.0° 就是管身 1.0°，权限真实可用。
     *
     * ⚠ 刻意**不用积分器**做这件事。积分在静摩擦期间会持续蓄力，突破瞬间
     *   已经攒下远超所需的角度，必然过冲，并演化成黏滑极限环。单向脱困的
     *   全部意义就是「推过阈值就立刻松手」，所以释放速率(8°/s)比爬升速率
     *   (0.8°/s)快一个数量级。
     */
    float breakout_deg = 0.0f;
    bool breakout_enabled = (config->breakout_max_angle_deg > 0.0f);
    if (breakout_enabled){
        /* 互斥：脱困接管后抖动必须闭嘴，否则两个机构互相打架。 */
        dither_deg = 0.0f;
        controller->dither_on = false;
        controller->dither_phase_rad = 0.0f;

        float abs_err = BallScurve_Abs(target_error_now);
        float abs_v = BallScurve_Abs(input->velocity_mm_s);

        if (!controller->breakout_on){
            /*
             * 触发三条同时满足并持续 dwell：剖面已结束、误差仍大、球基本不动。
             * 必须看速度——只看误差会在移动末段（球高速穿过目标附近）误触发。
             */
            bool stuck = !controller->active &&
                         (abs_err > config->breakout_min_error_mm) &&
                         (abs_v < config->breakout_max_speed_mm_s);
            if (stuck){
                controller->breakout_stuck_elapsed_s += input->dt_s;
                if (controller->breakout_stuck_elapsed_s >= config->breakout_dwell_s){
                    controller->breakout_on = true;
                    controller->breakout_release_elapsed_s = 0.0f;
                }
            } else{
                controller->breakout_stuck_elapsed_s = 0.0f;
            }
        }

        if (controller->breakout_on){
            /*
             * 释放判据不能用 v != 0：视觉速度有量化（1 mm/s 台阶）和延迟，
             * 噪声就能满足。要求「速度方向确实朝向目标」**且**「超过可靠阈值」，
             * 再持续 release_dwell 才认定脱困成功。
             */
            bool moving_to_target =
                ((input->velocity_mm_s * target_error_now) > 0.0f) &&
                (abs_v > config->breakout_release_speed_mm_s);
            if (moving_to_target){
                controller->breakout_release_elapsed_s += input->dt_s;
            } else{
                controller->breakout_release_elapsed_s = 0.0f;
            }

            /*
             * ⚠ 必须带上 moving_to_target 这一项。少了它，当
             *   breakout_release_dwell_s = 0 时 `elapsed >= 0` 恒真，release
             *   首拍就成立，倾角永远爬不起来——脱困功能**静默失效**，而遥测里
             *   brk 一直是 0，看不出是被这个判据卡死的。
             */
            bool release = (moving_to_target &&
                            (controller->breakout_release_elapsed_s >=
                             config->breakout_release_dwell_s)) ||
                           (abs_err <= config->breakout_min_error_mm);
            if (release){
                /* 快速斜坡归零而非瞬间置 0——瞬间撤销就是一个角度阶跃。 */
                float step = config->breakout_release_rate_deg_s * input->dt_s;
                if (controller->breakout_angle_deg > step){
                    controller->breakout_angle_deg -= step;
                } else if (controller->breakout_angle_deg < -step){
                    controller->breakout_angle_deg += step;
                } else{
                    controller->breakout_angle_deg = 0.0f;
                    controller->breakout_on = false;
                    controller->breakout_stuck_elapsed_s = 0.0f;
                    controller->breakout_release_elapsed_s = 0.0f;
                }
            } else{
                /* 朝目标方向按斜坡加倾角，钳在上限。 */
                float dir = (target_error_now >= 0.0f) ? 1.0f : -1.0f;
                controller->breakout_angle_deg +=
                    dir * config->breakout_ramp_rate_deg_s * input->dt_s;
                controller->breakout_angle_deg =
                    BallScurve_Clamp(controller->breakout_angle_deg,
                                     -config->breakout_max_angle_deg,
                                     config->breakout_max_angle_deg);
            }
        }
        breakout_deg = controller->breakout_angle_deg;
    } else{
        controller->breakout_angle_deg = 0.0f;
        controller->breakout_stuck_elapsed_s = 0.0f;
        controller->breakout_release_elapsed_s = 0.0f;
        controller->breakout_on = false;
    }

    /* --- 7. 合成、限幅、斜率限制 --- */
    float command = config->level_bias_deg + feedforward_deg + rolling_ff_deg +
                    feedback_deg + dither_deg + breakout_deg;
    float limited = BallScurve_Clamp(command, config->angle_min_deg, config->angle_max_deg);
    bool saturated = (limited != command);

    bool rate_limited = false;
    if (config->angle_rate_limit_deg_s > 0.0f){
        /*
         * 斜率限制的基准是**上一拍下发的指令**，不是编码器实测角。用实测角会
         * 把步进的跟踪滞后叠进来，形成"指令追不上、滞后越积越多"的正反馈。
         * 实测角只在 Reset 时用来续接一次。
         */
        if (!controller->have_last_angle){
            controller->last_angle_deg = input->actual_angle_deg;
            controller->have_last_angle = true;
        }
        float max_step = config->angle_rate_limit_deg_s * input->dt_s;
        float delta = limited - controller->last_angle_deg;
        if (delta > max_step){
            limited = controller->last_angle_deg + max_step;
            rate_limited = true;
        } else if (delta < -max_step){
            limited = controller->last_angle_deg - max_step;
            rate_limited = true;
        }
    }
    controller->last_angle_deg = limited;
    controller->have_last_angle = true;

    /* --- 8. 稳定判据（纯遥测）--- */
    float target_error = target_error_now;
    if (!controller->active &&
        (BallScurve_Abs(target_error) <= config->settled_position_mm) &&
        (BallScurve_Abs(input->velocity_mm_s) <= config->settled_speed_mm_s)){
        controller->settled_elapsed_s += input->dt_s;
        if (controller->settled_elapsed_s >= config->settled_time_s){
            controller->settled = true;
        }
    } else{
        controller->settled_elapsed_s = 0.0f;
        controller->settled = false;
    }

    output->angle_deg = limited;
    output->x_ref_mm = x_ref;
    output->v_ref_mm_s = v_ref;
    output->a_ref_mm_s2 = a_ref;
    output->feedforward_deg = feedforward_deg;
    output->rolling_ff_deg = rolling_ff_deg;
    output->feedback_deg = feedback_deg;
    output->dither_deg = dither_deg;
    output->breakout_deg = breakout_deg;
    output->breakout_on = controller->breakout_on;
    output->breakout_stuck_s = controller->breakout_stuck_elapsed_s;
    output->breakout_release_s = controller->breakout_release_elapsed_s;
    output->position_error_mm = position_error;
    output->velocity_error_mm_s = velocity_error;
    output->profile_time_s = controller->elapsed_s;
    output->profile_duration_s = controller->duration_s;
    output->profile_active = controller->active;
    output->saturated = saturated;
    output->feedback_clipped = feedback_clipped;
    output->rate_limited = rate_limited;
    output->dither_on = controller->dither_on;
    output->settled = controller->settled;
    return true;
}

/**
 * @file  step_motor.c
 * @brief BSP 摆杆步进电机开环控制 + QEI 位置读取实现。
 *
 * 脉冲由定时器 PWM 产生:速度(deg/s)先换算成步进频率,再由定时器时钟推出重载值 ARR,
 * 占空比固定取 ARR/2(方波)。方向与使能是普通 GPIO。
 *
 * 位置有两套来源,互为校验:
 *   - 开环估计:速度对时间积分,恒有值但丢步不可见;
 *   - QEI 实测:硬件正交计数,反映真实轴角。
 * 二者持续偏离即说明丢步或机械打滑。
 *
 * ⚠ QEI 硬件计数器是 16 位(load 65535),必须以「相邻两次采样计数变化 < 32767」的
 *   频率调用 StepMotor_UpdateEncoder() 累加,否则会丢圈。
 */
#include "step_motor.h"

#include "bsp_time.h"

#include <stdbool.h>
#include <stddef.h>

/* ===== 硬件描述与运行状态 ===== */

typedef struct {
    GPIO_Regs *dir_port;
    uint32_t dir_pin;
    GPIO_Regs *en_port;
    uint32_t en_pin;
    GPTIMER_Regs *pwm_timer;
    DL_TIMER_CC_INDEX pwm_channel;
    bool positive_dir_high;
    bool enable_high;
} STEP_MOTOR_HW_CONFIG;

typedef struct {
    float speed_deg_per_s;
    float estimated_position_deg;
    uint32_t last_update_ms;
    bool update_started;
    bool enabled;               /* 驱动器 EN 当前状态。 */
    uint32_t step_freq_hz;      /* 最近一次实际应用的步进频率(限幅后)。 */
    uint32_t timer_load;        /* 最近一次写入的定时器装载值。 */
} STEP_MOTOR_STATE;

static const STEP_MOTOR_HW_CONFIG motor_hw[STEP_MOTOR_CHANNEL_MAX] = {
    [STEP_MOTOR_CHANNEL_BEAM] = {
        .dir_port = STEP_MOTOR_BEAM_DIR_PORT,
        .dir_pin = STEP_MOTOR_BEAM_DIR_PIN,
        .en_port = STEP_MOTOR_BEAM_EN_PORT,
        .en_pin = STEP_MOTOR_BEAM_EN_PIN,
        .pwm_timer = STEP_MOTOR_BEAM_PWM_TIMER,
        .pwm_channel = STEP_MOTOR_BEAM_PWM_CHANNEL,
        .positive_dir_high = (STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH != 0U),
        .enable_high = (STEP_MOTOR_BEAM_ENABLE_HIGH != 0U),
    },
};

static STEP_MOTOR_STATE motor_state[STEP_MOTOR_CHANNEL_MAX];

/* 开环估计位置的软限位,由 StepMotor_SetPositionLimit() 按实际机械行程覆盖。 */
static STEP_MOTOR_POSITION_LIMIT position_limit = {
    .min_deg = STEP_MOTOR_MIN_POSITION_DEG,
    .max_deg = STEP_MOTOR_MAX_POSITION_DEG,
};

/* ===== QEI 编码器状态 ===== */

static uint16_t encoder_last_raw;   /* 上次采样到的硬件 16 位计数。 */
static int32_t  encoder_accum;      /* 累计计数,已消除 16 位环绕。 */

/* ===== 内部工具 ===== */

static void StepMotor_DisablePulse(const STEP_MOTOR_HW_CONFIG *hw);

static bool StepMotor_IsValidChannel(STEP_MOTOR_CHANNEL channel){
    return channel < STEP_MOTOR_CHANNEL_MAX;
}

static float StepMotor_Abs(float value){
    return (value < 0.0f) ? -value : value;
}

static float StepMotor_LimitSpeed(float speed_deg_per_s){
    if (speed_deg_per_s > STEP_MOTOR_MAX_SPEED_DEG_S){
        return STEP_MOTOR_MAX_SPEED_DEG_S;
    }

    if (speed_deg_per_s < -STEP_MOTOR_MAX_SPEED_DEG_S){
        return -STEP_MOTOR_MAX_SPEED_DEG_S;
    }

    return speed_deg_per_s;
}

/* 已抵限位且仍向限位方向运动时,把速度指令归零。 */
static float StepMotor_LimitSpeedByPosition(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s){
    float position_deg = motor_state[channel].estimated_position_deg;

    if (position_deg >= position_limit.max_deg && speed_deg_per_s > 0.0f){
        return 0.0f;
    }

    if (position_deg <= position_limit.min_deg && speed_deg_per_s < 0.0f){
        return 0.0f;
    }

    return speed_deg_per_s;
}

/* 把 RunFor 的时长裁短到「按当前速度走到限位」所需时间。 */
static uint32_t StepMotor_LimitDurationMs(STEP_MOTOR_CHANNEL channel,
                                          float speed_deg_per_s,
                                          uint32_t duration_ms){
    if (speed_deg_per_s == 0.0f){
        return 0U;
    }

    float position_deg = motor_state[channel].estimated_position_deg;
    float remain_deg = (speed_deg_per_s > 0.0f) ?
        (position_limit.max_deg - position_deg) :
        (position_deg - position_limit.min_deg);

    if (remain_deg <= 0.0f){
        return 0U;
    }

    uint32_t allowed_ms = (uint32_t)((remain_deg / StepMotor_Abs(speed_deg_per_s)) * 1000.0f);
    return (allowed_ms < duration_ms) ? allowed_ms : duration_ms;
}

static void StepMotor_ClampPosition(STEP_MOTOR_CHANNEL channel){
    STEP_MOTOR_STATE *state = &motor_state[channel];

    if (state->estimated_position_deg > position_limit.max_deg){
        state->estimated_position_deg = position_limit.max_deg;
        StepMotor_DisablePulse(&motor_hw[channel]);
        state->speed_deg_per_s = 0.0f;
    } else if (state->estimated_position_deg < position_limit.min_deg){
        state->estimated_position_deg = position_limit.min_deg;
        StepMotor_DisablePulse(&motor_hw[channel]);
        state->speed_deg_per_s = 0.0f;
    }
}

/* ===== 脉冲输出 ===== */

static uint32_t StepMotor_GetStepFrequency(float speed_deg_per_s){
    float abs_speed = StepMotor_Abs(speed_deg_per_s);

    if (abs_speed <= 0.0f || STEP_MOTOR_STEP_ANGLE_DEG <= 0.0f || STEP_MOTOR_MICROSTEP <= 0.0f){
        return 0U;
    }

    return (uint32_t)((abs_speed / STEP_MOTOR_STEP_ANGLE_DEG) * STEP_MOTOR_MICROSTEP);
}

static void StepMotor_SetDirection(const STEP_MOTOR_HW_CONFIG *hw, float speed_deg_per_s){
    bool set_high = hw->positive_dir_high;

    if (speed_deg_per_s < 0.0f){
        set_high = !set_high;
    }

    if (set_high){
        DL_GPIO_setPins(hw->dir_port, hw->dir_pin);
    } else{
        DL_GPIO_clearPins(hw->dir_port, hw->dir_pin);
    }
}

static void StepMotor_SetEnable(const STEP_MOTOR_HW_CONFIG *hw, bool enable){
    bool set_high = hw->enable_high ? enable : !enable;

    if (set_high){
        DL_GPIO_setPins(hw->en_port, hw->en_pin);
    } else{
        DL_GPIO_clearPins(hw->en_port, hw->en_pin);
    }
}

/*
 * 停止脉冲：停计数器 + 用 ODIS 强制 CCP 输出为低。
 *
 * ⚠ 不能只把 CC 写 0 了事。EDGE_ALIGN_UP 的输出动作是
 *   ZACT_CCP_HIGH | CUACT_CCP_LOW（计数为 0 拉高、计数等于 CC 拉低），
 *   CC=0 会让这两个事件落在同一计数点上，每个定时器周期可能挤出一个窄毛刺，
 *   被驱动器当成有效 STEP 边沿。WHEELTEC 官方例程同样用 ODIS 强制低电平
 *   来规避（见其 Motor_ForceLow()）。
 */
static void StepMotor_DisablePulse(const STEP_MOTOR_HW_CONFIG *hw){
    DL_TimerG_stopCounter(hw->pwm_timer);
    DL_TimerG_setCCPOutputDisabled(hw->pwm_timer,
                                   DL_TIMER_CCP_DIS_OUT_LOW,
                                   DL_TIMER_CCP_DIS_OUT_LOW);
    DL_TimerG_setCaptureCompareValue(hw->pwm_timer, 0U, hw->pwm_channel);
}

/* 恢复 CCP 由定时器输出控制。 */
static void StepMotor_EnablePulseOutput(const STEP_MOTOR_HW_CONFIG *hw){
    DL_TimerG_setCCPOutputDisabled(hw->pwm_timer,
                                   DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
                                   DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

static BSP_STATUS StepMotor_ApplySpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s){
    const STEP_MOTOR_HW_CONFIG *hw = &motor_hw[channel];
    uint32_t step_frequency = StepMotor_GetStepFrequency(speed_deg_per_s);

    /* 低于硬件可产生的最低频率就不出脉冲，改为保持——否则会被迫跑得比指令快。 */
    if (step_frequency < STEP_MOTOR_MIN_STEP_FREQ_HZ){
        StepMotor_DisablePulse(hw);
        motor_state[channel].step_freq_hz = 0U;
        motor_state[channel].timer_load = 0U;
        return BSP_STATUS_OK;
    }

    if (step_frequency > STEP_MOTOR_MAX_STEP_FREQ_HZ){
        step_frequency = STEP_MOTOR_MAX_STEP_FREQ_HZ;
    }

    StepMotor_SetDirection(hw, speed_deg_per_s);

    /* 四舍五入取周期，减少低速端的量化偏差。 */
    uint32_t period = (STEP_MOTOR_STEP_TIMER_CLK_HZ + (step_frequency / 2U)) / step_frequency;
    if (period > (STEP_MOTOR_MAX_ARR + 1U)){
        period = STEP_MOTOR_MAX_ARR + 1U;
    }
    if (period < 2U){
        period = 2U;
    }

    DL_TimerG_stopCounter(hw->pwm_timer);
    DL_TimerG_setLoadValue(hw->pwm_timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(hw->pwm_timer, period / 2U, hw->pwm_channel);
    /* 先配好再放开输出，避免起停瞬间挤出半个周期的毛刺。 */
    StepMotor_EnablePulseOutput(hw);
    DL_TimerG_startCounter(hw->pwm_timer);

    motor_state[channel].step_freq_hz = step_frequency;
    motor_state[channel].timer_load = period - 1U;
    return BSP_STATUS_OK;
}

/* ===== 初始化 ===== */

BSP_STATUS StepMotor_Init(void){
    for (uint8_t i = 0U; i < (uint8_t)STEP_MOTOR_CHANNEL_MAX; i++){
        const STEP_MOTOR_HW_CONFIG *hw = &motor_hw[i];

        motor_state[i].speed_deg_per_s = 0.0f;
        motor_state[i].estimated_position_deg = 0.0f;
        motor_state[i].last_update_ms = 0U;
        motor_state[i].update_started = false;

        DL_GPIO_clearPins(hw->dir_port, hw->dir_pin);
        /*
         * 上电即使能,使电机始终具备保持力矩(摆杆有重力负载,失能会自重回落)。
         * 代价:静止时有保持电流的嗡嗡声与发热,且电机轴无法用手转动——
         * 标定编码器时需先调 StepMotor_SetEnabled(ch, false) 手动断电。
         */
        StepMotor_SetEnable(hw, true);
        motor_state[i].enabled = true;
        DL_TimerG_startCounter(hw->pwm_timer);
        StepMotor_DisablePulse(hw);
    }

    /* SysConfig 生成的 SYSCFG_DL_SMotor_QEI_init() 只做到 configQEI + enableClock,
     * 未调 startCounter;按 DL_Timer_configQEI 的文档要求在此补齐,否则不会计数。 */
    DL_TimerG_startCounter(STEP_MOTOR_QEI_TIMER);
    StepMotor_ResetEncoder();

    return BSP_STATUS_OK;
}

/* ===== 位置更新 ===== */

BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms){
    if (!StepMotor_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    StepMotor_UpdateEncoder();

    STEP_MOTOR_STATE *state = &motor_state[channel];

    if (!state->update_started){
        state->last_update_ms = now_ms;
        state->update_started = true;
        return BSP_STATUS_OK;
    }

    state->estimated_position_deg += state->speed_deg_per_s *
        (float)(now_ms - state->last_update_ms) * 1e-3f;
    state->last_update_ms = now_ms;

    StepMotor_ClampPosition(channel);
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms){
    BSP_STATUS status = BSP_STATUS_OK;

    for (uint8_t i = 0U; i < (uint8_t)STEP_MOTOR_CHANNEL_MAX; i++){
        BSP_STATUS channel_status = StepMotor_UpdateState((STEP_MOTOR_CHANNEL)i, now_ms);

        if (channel_status != BSP_STATUS_OK){
            status = channel_status;
        }
    }

    return status;
}

/* ===== 速度控制 ===== */

BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s){
    if (!StepMotor_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    (void)StepMotor_UpdateState(channel, BSP_Time_GetMs());

    float limited_speed_deg_per_s = StepMotor_LimitSpeed(speed_deg_per_s);
    limited_speed_deg_per_s = StepMotor_LimitSpeedByPosition(channel, limited_speed_deg_per_s);

    /* 要动就先通电;调用方无需记得手动使能。停止时不自动断电,保留保持力矩。 */
    if (limited_speed_deg_per_s != 0.0f){
        (void)StepMotor_SetEnabled(channel, true);
    }

    BSP_STATUS status = StepMotor_ApplySpeed(channel, limited_speed_deg_per_s);
    if (status != BSP_STATUS_OK){
        return status;
    }

    motor_state[channel].speed_deg_per_s = limited_speed_deg_per_s;
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_RunFor(STEP_MOTOR_CHANNEL channel,
                            float speed_deg_per_s,
                            uint32_t duration_ms){
    BSP_STATUS status = StepMotor_SetSpeed(channel, speed_deg_per_s);
    if (status != BSP_STATUS_OK){
        return status;
    }

    uint32_t limited_duration_ms = StepMotor_LimitDurationMs(channel,
                                                             StepMotor_GetSpeed(channel),
                                                             duration_ms);
    BSP_DelayMs(limited_duration_ms);
    (void)StepMotor_UpdateState(channel, BSP_Time_GetMs());
    return StepMotor_Stop(channel);
}

BSP_STATUS StepMotor_Stop(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    (void)StepMotor_UpdateState(channel, BSP_Time_GetMs());
    StepMotor_DisablePulse(&motor_hw[channel]);
    motor_state[channel].speed_deg_per_s = 0.0f;
    motor_state[channel].step_freq_hz = 0U;
    motor_state[channel].timer_load = 0U;
    return BSP_STATUS_OK;
}

BSP_STATUS StepMotor_StopAll(void){
    BSP_STATUS status = BSP_STATUS_OK;

    for (uint8_t i = 0U; i < (uint8_t)STEP_MOTOR_CHANNEL_MAX; i++){
        BSP_STATUS channel_status = StepMotor_Stop((STEP_MOTOR_CHANNEL)i);

        if (channel_status != BSP_STATUS_OK){
            status = channel_status;
        }
    }

    return status;
}

/* ===== 驱动器使能 ===== */

BSP_STATUS StepMotor_SetEnabled(STEP_MOTOR_CHANNEL channel, bool enable){
    if (!StepMotor_IsValidChannel(channel)){
        return BSP_STATUS_INVALID_ARG;
    }

    StepMotor_SetEnable(&motor_hw[channel], enable);
    motor_state[channel].enabled = enable;
    return BSP_STATUS_OK;
}

bool StepMotor_IsEnabled(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return false;
    }

    return motor_state[channel].enabled;
}

/* ===== 开环状态读取 ===== */

float StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return 0.0f;
    }

    return motor_state[channel].speed_deg_per_s;
}

uint32_t StepMotor_GetStepFrequencyHz(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return 0U;
    }

    return motor_state[channel].step_freq_hz;
}

uint32_t StepMotor_GetTimerLoad(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return 0U;
    }

    return motor_state[channel].timer_load;
}

float StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return 0.0f;
    }

    return motor_state[channel].estimated_position_deg;
}

void StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel){
    if (!StepMotor_IsValidChannel(channel)){
        return;
    }

    motor_state[channel].estimated_position_deg = 0.0f;
    motor_state[channel].last_update_ms = BSP_Time_GetMs();
    motor_state[channel].update_started = true;
}

/* ===== QEI 编码器 ===== */

void StepMotor_UpdateEncoder(void){
    uint16_t raw = (uint16_t)DL_TimerG_getTimerCount(STEP_MOTOR_QEI_TIMER);
    /* 无符号相减后按有符号解释,自动处理 16 位环绕(要求单次变化 < 32768)。 */
    int16_t delta = (int16_t)(uint16_t)(raw - encoder_last_raw);

    encoder_accum += (int32_t)delta;
    encoder_last_raw = raw;
}

int32_t StepMotor_GetEncoderCount(void){
#if (STEP_MOTOR_ENCODER_INVERT != 0U)
    return -encoder_accum;
#else
    return encoder_accum;
#endif
}

uint16_t StepMotor_GetEncoderRaw(void){
    return (uint16_t)DL_TimerG_getTimerCount(STEP_MOTOR_QEI_TIMER);
}

float StepMotor_GetMeasuredPosition(void){
    if (STEP_MOTOR_ENCODER_COUNTS_PER_REV <= 0.0f){
        return 0.0f;
    }

    return ((float)StepMotor_GetEncoderCount() * 360.0f) /
           STEP_MOTOR_ENCODER_COUNTS_PER_REV;
}

void StepMotor_ResetEncoder(void){
    DL_TimerG_setTimerCount(STEP_MOTOR_QEI_TIMER, 0U);
    encoder_last_raw = 0U;
    encoder_accum = 0;
}

bool StepMotor_IsEncoderCountingUp(void){
    bool up = (DL_TimerG_getQEIDirection(STEP_MOTOR_QEI_TIMER) == DL_TIMER_QEI_DIR_UP);

#if (STEP_MOTOR_ENCODER_INVERT != 0U)
    return !up;
#else
    return up;
#endif
}

/* ===== 限位配置 ===== */

BSP_STATUS StepMotor_SetPositionLimit(const STEP_MOTOR_POSITION_LIMIT *limit){
    if (limit == NULL){
        return BSP_STATUS_NULL;
    }

    if (limit->min_deg > limit->max_deg){
        return BSP_STATUS_INVALID_ARG;
    }

    position_limit = *limit;
    (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_BEAM, BSP_Time_GetMs());
    StepMotor_ClampPosition(STEP_MOTOR_CHANNEL_BEAM);
    return BSP_STATUS_OK;
}

STEP_MOTOR_POSITION_LIMIT StepMotor_GetPositionLimit(void){
    return position_limit;
}

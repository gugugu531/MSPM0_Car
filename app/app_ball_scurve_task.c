/**
 * @file  app_ball_scurve_task.c
 * @brief 纯 S 曲线滚球点到点任务：O → +5cm → −5cm（要求 3 序列）。
 *
 * ================== 关于"物理换算不准" ==================
 *
 * 本任务**不假设**下面任何一个换算是准的：
 *
 *   1. 滚球增益 K_G = (5/7)g —— 管身扭转、球非实心、接触点偏移都会让它偏小；
 *   2. 曲柄滑槽查表 cnt→deg —— 尺寸测量误差直接进入这张表；
 *   3. **水平位计数** —— 上电参考位是"断电自重靠住那一端"，其重复性、机构装配
 *      公差和水管自身弯曲都会让真实动力学水平点偏离查表的 0°（当前表为 180 cnt）。
 *
 * 第 3 条最要命：它是一个**常值倾角偏置**，在 T 秒的移动里造成
 * 0.5·K_G·Δθ·T² 的落点偏差——Δθ = 0.45°（15 cnt）、T = 1.2 s 时就是 40 mm。
 *
 * 应对方式是**不去猜，而是让它可辨识**：
 *
 *   - 所有换算都集中在下面的 H3S_* 宏和 SCURVE_CONFIG 里，一处可改；
 *   - `H3S_LEVEL_BIAS_DEG` 默认 0，即"承认没标定"，其真值会以单向漂移暴露出来；
 *   - 遥测同时输出**实际水管角 beam**（编码器经查表）和**球加速度估计 aest**，
 *     二者做一次线性回归 aest = K·(beam − θ0) 就同时得到真实增益 K 和真实
 *     水平角 θ0。S 曲线移动本身在 ±3° 内扫过倾角，**这趟移动就是辨识实验**，
 *     不需要另做标定动作。
 *
 * 因此本任务里的所有"精确公式"都只当**近似控制关系**用：前馈负责大致把球送过去，
 * 剩下的偏差由反馈吃掉，并由遥测暴露出来供离线辨识。
 *
 * ⚠ 本任务刻意**不含**低速捕获逻辑（抖动/蠕进/单向逼近/俘获偏置补偿）。
 *   末端落点误差就是这条纯 S 曲线基线的固有下限，是留作对照的。
 */
#include "app_ball_scurve_task.h"

#include "app_ball_config.h"
#include "app_ball_tune.h"
#include "app_fmt.h"
#include "ball_scurve.h"
#include "bsp_time.h"
#include "chassis.h"
#include "debug_uart.h"
#include "rpi_uart.h"
#include "step_motor.h"
#include "ui.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define H3S_UI_PERIOD_MS         100U
#define H3S_TELEMETRY_PERIOD_MS   50U
#define H3S_ARM_POSITION_MM      120.0f
/*
 * 步进转速上限。仿真表明限速不是巡航段的瓶颈（60 与 120 deg/s 结果几乎相同），
 * 真正的瓶颈是位置环 STEP_MOTOR_SERVO_KP；这里取 120 只是为了不让限速成为第二个约束。
 */
#define H3S_ACTUATOR_SPEED_DEG_S 120.0f
/*
 * 本任务使用的步进位置环增益。位置环是一阶滞后，时间常数 1/KP：
 * KP=3 → τ=333ms，在振铃频率 3.3 rad/s 上独占 47° 相位；KP=10 → τ=100ms。
 * 受 STEP_MOTOR_SERVO_KP_MAX 上限约束。
 */
#define H3S_SERVO_KP_TUNED 10.0f
/* 进入任务后等视觉稳定、再开始第一段移动的静默时间。 */
#define H3S_ARM_DWELL_MS         800U
/* 每个航点到达后的驻留时间：留给观察落点，也给下一段一个静止起点。 */
#define H3S_WAYPOINT_DWELL_MS   2000U

/*
 * 动力学水平点相对查表 0° 的偏置，deg。**默认 0 = 承认未标定**。
 * 标定方法见文件头：跑一趟本任务，对遥测的 (beam, aest) 做线性回归，
 * 零交点即真实水平角，填到这里。1 cnt ≈ 0.03°，15 cnt ≈ 0.45°。
 */
#define H3S_LEVEL_BIAS_DEG 0.0f

/* 要求 3 的航点序列：O → +5cm → −5cm。 */
static const float H3S_WAYPOINT_MM[] = { 0.0f, 50.0f, -50.0f };
#define H3S_WAYPOINT_COUNT ((uint8_t)(sizeof(H3S_WAYPOINT_MM) / sizeof(H3S_WAYPOINT_MM[0])))

/*
 * ⚠ 刻意**不是** const —— app_ball_tune 会在运行期改写这里的字段。
 *   掉电即恢复本文件里的编译期默认值；整定出来的数必须回填源码才算数。
 */
static BALL_SCURVE_CONFIG H3S_SCURVE_CONFIG = {
    /*
     * K_G 名义值 (5/7)·g = 7004.75 mm/s² per rad。**这是近似值**：只用来把
     * a_ref 换成前馈倾角，偏 25% 也只是让前馈少给/多给同比例的驱动，
     * 由剖面跟踪反馈补上（仿真 gain 场景：巡航 RMS 1.63 mm、末端 0.03 mm）。
     */
    .rolling_acceleration_gain_mm_s2 = 7004.75f,

    /* 峰值 200 mm/s² 对应峰值倾角 asin(200/7004.75) ≈ 1.64°，权限充裕。 */
    .max_acceleration_mm_s2 = 200.0f,
    .max_velocity_mm_s = 90.0f,
    .min_duration_s = 0.40f,
    .max_duration_s = 3.00f,

    /* wn = 2.4 rad/s、ζ = 0.9：Kp = wn²/K_G、Kd = 2ζwn/K_G，再由 rad/m 转 deg/mm。 */
    .kp_deg_per_mm = 0.04711f,
    .kd_deg_per_mm_s = 0.03533f,
    .feedback_limit_deg = 2.0f,

    /*
     * 滚阻前馈默认 0：先跑一趟测出实际欠冲量 d，再按 θ_roll = 2d/(K_G·T²) 反解填回。
     * 拍脑袋填值会把一个可测量的系统性偏差换成一个不可测的。
     */
    .rolling_resistance_deg = 0.0f,
    .rolling_ff_speed_deadband_mm_s = 3.0f,
    .level_bias_deg = H3S_LEVEL_BIAS_DEG,

    /* 查表在软限位 20..430 cnt 上的角度范围是 −5.345°..+5.236°，两端各留 0.1°。 */
    .angle_min_deg = -5.2f,
    .angle_max_deg = 5.1f,
    .angle_rate_limit_deg_s = 60.0f,

    /*
     * MOVE 段积分（PID for MOVE, PD for HOLD）。
     *
     * 为什么积分只放 MOVE：球在滚时误差来自**常值角度偏置**（水平点未标定
     * 约 0.25°、滚阻约 0.05°），那正是积分该干的活；球静止时误差来自
     * **静摩擦死区**（实测 θ_stick≈0.62°），对它积分会顶到脱离角然后窜出去，
     * 是教科书级的黏滑。所以 MOVE 累积、HOLD 冻结。
     *
     * Ki 由相位代价定：积分在穿越频率 ω_c≈3.2 rad/s 处引入
     * atan(Ki/Kp/ω_c) 的滞后。取 Ki/Kp = 0.42 → 约 7.5°，在 MOVE/HOLD
     * 调度挣回的裕度之内。Ki = 0.42 × 0.04711 ≈ 0.020。
     *
     * ⚠ 积分收敛值**就是自动标定出的水平角**，应与离线用 (beam, aest) 回归
     *   得到的 +0.254° 一致。遥测字段 ki 直接读它——这是本轮最该盯的数。
     */
    /* H3 车体静止，车加速度前馈默认关闭；H4/H5/H6 接入时置 1。 */
    .car_feedforward_gain = 0.0f,

    .move_ki_deg_per_mm_s = 0.020f,
    .move_integral_limit_deg = 0.6f,      /* 覆盖水平点 0.25° + 滚阻 0.05° + 余量 */
    .move_integral_leak_tau_s = 0.0f,     /* 辨识期间不泄漏，便于读收敛值 */
    .move_integral_apply_in_hold = true,  /* HOLD 沿用学到的偏置（可 A/B 关掉） */
    .move_integral_min_speed_mm_s = 8.0f, /* 球没滚起来就别积分 */
    /*
     * 种子默认**关闭**。原设想是用规划时刻的保持角做积分初值来掐掉起步
     * overshoot，但仿真否定了它：
     *
     *   real 场景 wp1 误差   无种子 −0.65 mm  →  有种子 +17.04 mm
     *   real 场景 峰值|a|    无种子 401       →  有种子 342（起步确实更平顺）
     *
     * 原因是**保持角不等于平衡角**——它可以偏离真实平衡角达 ±θ_stick(0.62°)。
     * 用被静摩擦污染的值做种子，等于把那个误差直接注入成指令偏置。
     * 起步平顺换落点精度，在 ±10 mm 判据下不划算。
     *
     * 而种子想要的那个效果，「积分器跨航点不清零」已经免费提供了：
     * 第一段移动学到的偏置会一路带给后续各段，且没有 θ_stick 污染。
     * 代价只是第一段移动仍有起步 overshoot。
     */
    .move_integral_seed_from_angle = false,

    /*
     * MOVE/HOLD 增益调度。2026-07-31 实测：剖面走完（ff=0，已是纯 PD 守点）后
     * 仍有 6.42 rad/s、峰峰 17~28 mm 的持续极限环，饱和的是 Kd 项
     * （Kd·v = 0.03533×40 = 1.41°，而 Kp·e 只有 0.57°，合计顶到 ±2° 限幅）。
     * 那个 Kd 是为跟踪 90 mm/s 参考轨迹推的，守一个不动的点不需要。
     *
     * Kd_hold 由「切入瞬间不许饱和」定：最坏 v≈50 mm/s 时 Kd×50 < 1.0°。
     * Kp 不调度——降 Kp 会让静摩擦残差 θ_stick/Kp 变大，方向相反。
     */
    .hold_kd_deg_per_mm_s = 0.020f,   /* ζ_eff ≈ 0.51，当前 0.03533 砍 43% */
    .hold_enter_error_mm = 15.0f,
    .hold_enter_speed_mm_s = 20.0f,   /* 必须同时看速度：球可能"很近"但正高速穿过 */
    .hold_enter_dwell_s = 0.3f,
    .hold_exit_error_mm = 30.0f,      /* 滞回，防边界抽搐 */
    .hold_blend_tau_s = 0.25f,        /* 增益连续过渡，指令角不跳变 */

    /*
     * 抖动：破静摩擦死区。这是当前**唯一**能让落点进 ±10 mm 的机制——
     * 2026-07-31 实测 θ_stick≈0.62°，纯 PD 残差下限 θ_stick/Kp = 13.2 mm，
     * 与 75 s 无扰动实测的 13.1 mm 完全吻合，靠整定消不掉。
     *
     * 指令 ±0.97° @ 2 Hz，经步进位置环（τ=1/SERVO_KP=0.1 s）衰减 0.62 倍后
     * 实际水管抖 ±0.6°，刚好覆盖脱离角；电机峰值 74 deg/s（限速 120 内），
     * 球的纹波约 0.46 mm——用 0.46 mm 换掉 13.1 mm。
     */
    .dither_amplitude_deg = 0.97f,
    .dither_frequency_hz = 2.0f,
    .dither_min_error_mm = 3.0f,   /* 与 settled_position_mm 对齐：进判据即停振 */
    .dither_max_speed_mm_s = 8.0f,
    .dither_dwell_s = 0.5f,

    .settled_position_mm = 3.0f,
    .settled_speed_mm_s = 5.0f,
    .settled_time_s = 0.5f,

    .replan_error_mm = 0.0f,   /* 0 = 纯 S 曲线，不重规划 */
};

/* 抖动的角速率需求必须留在输出斜率限制之内，否则会被削顶成三角波。 */
#if 0 /* 编译期无法算 2πfA，这里以注释留下判据 */
2π × dither_frequency_hz × dither_amplitude_deg = 12.2 deg/s < angle_rate_limit_deg_s(60)
#endif

typedef enum {
    H3S_STATE_WAIT_VISION = 0,
    H3S_STATE_ARMING,
    H3S_STATE_MOVING,
    H3S_STATE_DWELL,
    H3S_STATE_DONE,
    H3S_STATE_DEGRADED,
    H3S_STATE_ACTUATOR_FAULT
} H3S_STATE;

static H3S_STATE h3s_state;
static BALL_SCURVE_CONTROLLER h3s_controller;
static BALL_SCURVE_OUTPUT h3s_output;
static RPI_UART_PREDICTION h3s_prediction;
static bool h3s_have_prediction;
static uint8_t h3s_waypoint;
static int32_t h3s_target_count;
static uint32_t h3s_start_ms;
static uint32_t h3s_phase_ms;
static uint32_t h3s_last_ui;
static uint32_t h3s_last_telemetry;
static H3S_STATE h3s_rendered_state;

/* 球加速度估计：只用于遥测与离线辨识，**不进入控制律**。 */
static float h3s_prev_velocity_mm_s;
static float h3s_acceleration_est_mm_s2;
static bool  h3s_have_prev_velocity;

/* ===== 热更参数表 =====
 *
 * 每一项的上下界都由物理约束推出，不是拍脑袋（推导见各行注释）。
 * 越界一律拒绝而非钳位——静默钳位会让人以为设进去了。
 */
static bool H3S_SetServoGain(float kp){
    return StepMotor_SetServoGain(kp) == BSP_STATUS_OK;
}
static float H3S_GetServoGain(void){
    return StepMotor_GetServoGain();
}

static const APP_BALL_TUNE_ENTRY H3S_TUNE_TABLE[] = {
    /* --- 反馈 --- */
    /* 上界 0.15：θ_stick/Kp 残差已降到 4mm，再高环路裕度不够。 */
    { "kp",       &H3S_SCURVE_CONFIG.kp_deg_per_mm,        0.005f, 0.150f, "deg/mm",   NULL, NULL },
    /* MOVE 段 Kd。上界 0.08：再高 Kd·v 在巡航速度下就顶穿限幅。 */
    { "kd",       &H3S_SCURVE_CONFIG.kd_deg_per_mm_s,      0.000f, 0.080f, "deg/mm/s", NULL, NULL },
    /* HOLD 段 Kd。上界 0.040：切入 v=50mm/s 时 Kd·v 已达 2.0° 限幅。 */
    { "holdkd",   &H3S_SCURVE_CONFIG.hold_kd_deg_per_mm_s, 0.000f, 0.040f, "deg/mm/s", NULL, NULL },
    /* 上界 4.0：与前馈峰值相加须留在物理 ±5.1° 内。 */
    { "fblim",    &H3S_SCURVE_CONFIG.feedback_limit_deg,   0.200f, 4.000f, "deg",      NULL, NULL },

    /* --- 积分（PID for MOVE）--- */
    /* 上界 0.060：Ki/Kp 超过 1.3 rad/s 时相位代价 >20°，吃光调度挣回的裕度。 */
    { "ki",       &H3S_SCURVE_CONFIG.move_ki_deg_per_mm_s, 0.000f, 0.060f, "deg/mm/s", NULL, NULL },
    { "kilim",    &H3S_SCURVE_CONFIG.move_integral_limit_deg, 0.000f, 2.000f, "deg",   NULL, NULL },
    { "kileak",   &H3S_SCURVE_CONFIG.move_integral_leak_tau_s, 0.000f, 300.0f, "s",    NULL, NULL },
    { "kiminv",   &H3S_SCURVE_CONFIG.move_integral_min_speed_mm_s, 0.0f, 60.0f, "mm/s", NULL, NULL },

    /* --- MOVE/HOLD 调度 --- */
    { "henter",   &H3S_SCURVE_CONFIG.hold_enter_error_mm,  1.000f, 60.00f, "mm",       NULL, NULL },
    { "hentv",    &H3S_SCURVE_CONFIG.hold_enter_speed_mm_s, 1.000f, 80.00f, "mm/s",    NULL, NULL },
    { "hdwell",   &H3S_SCURVE_CONFIG.hold_enter_dwell_s,   0.000f, 3.000f, "s",        NULL, NULL },
    { "hexit",    &H3S_SCURVE_CONFIG.hold_exit_error_mm,   2.000f, 120.0f, "mm",       NULL, NULL },
    { "htau",     &H3S_SCURVE_CONFIG.hold_blend_tau_s,     0.000f, 2.000f, "s",        NULL, NULL },

    /* --- 抖动 --- */
    /* 上界 2.0：2πf·A 须小于执行器角速率上限 19.8 deg/s，f=2Hz 时 A<1.58。 */
    { "dith",     &H3S_SCURVE_CONFIG.dither_amplitude_deg, 0.000f, 2.000f, "deg",      NULL, NULL },
    /* 上界 4.0：3Hz 已让电机峰值 146 deg/s 超过 120 限速，留一档余量。 */
    { "dithhz",   &H3S_SCURVE_CONFIG.dither_frequency_hz,  0.500f, 4.000f, "Hz",       NULL, NULL },
    { "dithe",    &H3S_SCURVE_CONFIG.dither_min_error_mm,  0.500f, 30.00f, "mm",       NULL, NULL },
    { "dithv",    &H3S_SCURVE_CONFIG.dither_max_speed_mm_s, 1.000f, 40.00f, "mm/s",    NULL, NULL },
    { "dithd",    &H3S_SCURVE_CONFIG.dither_dwell_s,       0.000f, 3.000f, "s",        NULL, NULL },

    /* --- 前馈与标定 --- */
    { "bias",     &H3S_SCURVE_CONFIG.level_bias_deg,      -2.000f, 2.000f, "deg",      NULL, NULL },
    { "roll",     &H3S_SCURVE_CONFIG.rolling_resistance_deg, 0.000f, 0.500f, "deg",    NULL, NULL },
    { "carff",    &H3S_SCURVE_CONFIG.car_feedforward_gain, 0.000f, 1.500f, "-",        NULL, NULL },

    /* --- 剖面 --- */
    /* 上界 350：asin(350/K_G)=2.87°，加 fblim 2.0° 仍在物理 5.1° 内。 */
    { "amax",     &H3S_SCURVE_CONFIG.max_acceleration_mm_s2, 30.0f, 350.0f, "mm/s2",   NULL, NULL },
    { "vmax",     &H3S_SCURVE_CONFIG.max_velocity_mm_s,    10.00f, 200.0f, "mm/s",     NULL, NULL },
    { "arate",    &H3S_SCURVE_CONFIG.angle_rate_limit_deg_s, 5.00f, 90.00f, "deg/s",   NULL, NULL },

    /* --- 步进位置环（走驱动的运行期接口，任务局部，不动全局宏）--- */
    { "servokp",  NULL, 1.000f, STEP_MOTOR_SERVO_KP_MAX, "1/s",
      H3S_GetServoGain, H3S_SetServoGain },
};
#define H3S_TUNE_COUNT \
    ((uint8_t)(sizeof(H3S_TUNE_TABLE) / sizeof(H3S_TUNE_TABLE[0])))

typedef struct {
    int16_t count;
    float angle_deg;
} H3S_LINKAGE_POINT;

/*
 * 由 pygame_linkage_visualizer.py 的槽约束模型离线生成，与 app_ball_task.c 同一张表。
 * ⚠ 表的 0°（180 cnt）是**几何名义水平**，不是动力学水平点。二者之差由
 *   H3S_LEVEL_BIAS_DEG 承担，未标定时为 0。
 */
static const H3S_LINKAGE_POINT H3S_LINKAGE_TABLE[] = {
    {  20, -5.345027f }, {  40, -4.635128f }, {  60, -3.933998f },
    {  80, -3.243158f }, { 100, -2.564173f }, { 120, -1.898660f },
    { 140, -1.248293f }, { 160, -0.614805f }, { 180, +0.000000f },
    { 200, +0.594243f }, { 220, +1.165966f }, { 240, +1.713122f },
    { 260, +2.233576f }, { 280, +2.725092f }, { 300, +3.185334f },
    { 320, +3.611860f }, { 340, +4.002121f }, { 360, +4.353460f },
    { 380, +4.663114f }, { 400, +4.928215f }, { 420, +5.145804f },
    { 430, +5.235833f },
};
#define H3S_LINKAGE_POINT_COUNT \
    ((uint8_t)(sizeof(H3S_LINKAGE_TABLE) / sizeof(H3S_LINKAGE_TABLE[0])))

static uint8_t H3S_PutStr(char *dst, const char *src){
    uint8_t n = 0U;
    while (src[n] != '\0'){
        dst[n] = src[n];
        n++;
    }
    return n;
}

static int32_t H3S_RoundToCount(float value){
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static int32_t H3S_ClampCount(int32_t count){
    if (count < STEP_MOTOR_ENC_SOFT_MIN_COUNTS){ return STEP_MOTOR_ENC_SOFT_MIN_COUNTS; }
    if (count > STEP_MOTOR_ENC_SOFT_MAX_COUNTS){ return STEP_MOTOR_ENC_SOFT_MAX_COUNTS; }
    return count;
}

static float H3S_LinkageAngleFromCount(int32_t count){
    count = H3S_ClampCount(count);
    for (uint8_t i = 1U; i < H3S_LINKAGE_POINT_COUNT; i++){
        const H3S_LINKAGE_POINT *low = &H3S_LINKAGE_TABLE[i - 1U];
        const H3S_LINKAGE_POINT *high = &H3S_LINKAGE_TABLE[i];
        if (count <= high->count){
            float fraction = (float)(count - low->count) /
                             (float)(high->count - low->count);
            return low->angle_deg + fraction * (high->angle_deg - low->angle_deg);
        }
    }
    return H3S_LINKAGE_TABLE[H3S_LINKAGE_POINT_COUNT - 1U].angle_deg;
}

static int32_t H3S_LinkageCountFromAngle(float angle_deg){
    if (angle_deg <= H3S_LINKAGE_TABLE[0].angle_deg){ return H3S_LINKAGE_TABLE[0].count; }
    for (uint8_t i = 1U; i < H3S_LINKAGE_POINT_COUNT; i++){
        const H3S_LINKAGE_POINT *low = &H3S_LINKAGE_TABLE[i - 1U];
        const H3S_LINKAGE_POINT *high = &H3S_LINKAGE_TABLE[i];
        if (angle_deg <= high->angle_deg){
            float fraction = (angle_deg - low->angle_deg) /
                             (high->angle_deg - low->angle_deg);
            float count = (float)low->count + fraction * (float)(high->count - low->count);
            return H3S_ClampCount(H3S_RoundToCount(count));
        }
    }
    return H3S_LINKAGE_TABLE[H3S_LINKAGE_POINT_COUNT - 1U].count;
}

static float H3S_ActualBeamAngleDeg(void){
    return H3S_LinkageAngleFromCount(StepMotor_GetEncoderCount());
}

static BSP_STATUS H3S_CommandAngle(float angle_deg){
    int32_t requested = H3S_LinkageCountFromAngle(angle_deg);
    if (requested == h3s_target_count){ return BSP_STATUS_OK; }
    h3s_target_count = requested;
    return StepMotor_MoveToCount(requested);
}

static const char *H3S_StateName(void){
    switch (h3s_state){
        case H3S_STATE_WAIT_VISION:    return "WAIT VISION";
        case H3S_STATE_ARMING:         return "ARMING";
        case H3S_STATE_MOVING:         return "S-CURVE MOVE";
        case H3S_STATE_DWELL:          return "DWELL";
        case H3S_STATE_DONE:           return "SEQUENCE DONE";
        case H3S_STATE_DEGRADED:       return "HOLD ANGLE";
        case H3S_STATE_ACTUATOR_FAULT: return "ACTUATOR ERR";
        default:                       return "UNKNOWN";
    }
}

static void H3S_Render(void){
    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    n = H3S_PutStr(l1, H3S_StateName());
    l1[n] = '\0';

    n = H3S_PutStr(l2, "x ");
    AppFmt_Fixed(&l2[n], h3s_have_prediction ? h3s_prediction.x_mm : 0.0f, 1U);
    while (l2[n] != '\0'){ n++; }
    n += H3S_PutStr(&l2[n], " > ");
    AppFmt_Fixed(&l2[n], H3S_WAYPOINT_MM[h3s_waypoint], 0U);

    n = H3S_PutStr(l3, "ref ");
    AppFmt_Fixed(&l3[n], h3s_output.x_ref_mm, 1U);
    while (l3[n] != '\0'){ n++; }
    n += H3S_PutStr(&l3[n], " t ");
    AppFmt_Fixed(&l3[n], h3s_output.profile_time_s, 1U);

    n = H3S_PutStr(l4, "u ");
    AppFmt_Fixed(&l4[n], h3s_output.angle_deg, 2U);
    while (l4[n] != '\0'){ n++; }
    n += H3S_PutStr(&l4[n], " b ");
    AppFmt_Fixed(&l4[n], H3S_ActualBeamAngleDeg(), 2U);

    n = H3S_PutStr(l5, "cnt ");
    AppFmt_I32(&l5[n], StepMotor_GetEncoderCount());
    while (l5[n] != '\0'){ n++; }
    n += H3S_PutStr(&l5[n], " >");
    AppFmt_I32(&l5[n], h3s_target_count);

    Ui_RenderLines("H3 S-Curve", l1, l2, l3, l4, l5, "BACK: safe exit");
}

static bool H3S_ReadPrediction(void){
    h3s_have_prediction = RpiUart_Observe(&h3s_prediction);
    return h3s_have_prediction;
}

/** 由视觉速度差分估计球加速度。仅供遥测与离线辨识，不进入控制律。 */
static void H3S_UpdateAccelerationEstimate(float velocity_mm_s, float dt){
    if (dt <= 0.0f){ return; }
    if (h3s_have_prev_velocity){
        float raw = (velocity_mm_s - h3s_prev_velocity_mm_s) / dt;
        /* 0.15 的一阶低通只为让读数可看；辨识时应当用原始 v 序列在主机侧重算。 */
        h3s_acceleration_est_mm_s2 += 0.15f * (raw - h3s_acceleration_est_mm_s2);
    } else{
        h3s_have_prev_velocity = true;
    }
    h3s_prev_velocity_mm_s = velocity_mm_s;
}

static void H3S_PlanCurrentWaypoint(void){
    BallScurve_PlanTo(&h3s_controller, &H3S_SCURVE_CONFIG,
                      h3s_prediction.x_mm, h3s_prediction.velocity_mm_s,
                      H3S_WAYPOINT_MM[h3s_waypoint]);
    h3s_state = H3S_STATE_MOVING;
    h3s_phase_ms = BSP_Time_GetMs();
    DebugUart_Printf("[SCV] plan wp=%u target=%.1f from x=%.2f v=%.2f T=%.3f\r\n",
                     (unsigned)h3s_waypoint,
                     (double)H3S_WAYPOINT_MM[h3s_waypoint],
                     (double)h3s_prediction.x_mm,
                     (double)h3s_prediction.velocity_mm_s,
                     (double)h3s_controller.duration_s);
}

static void H3S_Enter(void){
    (void)Chassis_Brake();
    StepMotor_AbortStartup();
    (void)StepMotor_Stop();
    (void)StepMotor_SetSpeedLimit(H3S_ACTUATOR_SPEED_DEG_S);
    /*
     * 步进位置环增益走**运行期接口**而不是改全局宏：这样只在本任务内生效，
     * 退出时恢复，不影响 H3 Ball Static / 上电抬升 / 越界纠正。
     * 依据：实测 |spd| = SERVO_KP × |perr|，KP=3 时转速被卡死在 58.9 deg/s，
     * 限速 120 从未触及；提到 10 后 lag RMS 0.92° → 0.441°。
     */
    (void)StepMotor_SetServoGain(H3S_SERVO_KP_TUNED);
    AppBallTune_Init(H3S_TUNE_TABLE, H3S_TUNE_COUNT);

    BallScurve_Init(&h3s_controller);
    h3s_output.angle_deg = 0.0f;
    h3s_output.x_ref_mm = 0.0f;
    h3s_output.v_ref_mm_s = 0.0f;
    h3s_output.a_ref_mm_s2 = 0.0f;
    h3s_output.feedforward_deg = 0.0f;
    h3s_output.rolling_ff_deg = 0.0f;
    h3s_output.feedback_deg = 0.0f;
    h3s_output.integral_deg = 0.0f;
    h3s_output.integral_active = false;
    h3s_output.dither_deg = 0.0f;
    h3s_output.position_error_mm = 0.0f;
    h3s_output.velocity_error_mm_s = 0.0f;
    h3s_output.profile_time_s = 0.0f;
    h3s_output.profile_duration_s = 0.0f;
    h3s_output.kd_effective_deg_per_mm_s = H3S_SCURVE_CONFIG.kd_deg_per_mm_s;
    h3s_output.hold_blend = 0.0f;
    h3s_output.hold_mode = false;
    h3s_output.profile_active = false;
    h3s_output.saturated = false;
    h3s_output.feedback_clipped = false;
    h3s_output.rate_limited = false;
    h3s_output.dither_on = false;
    h3s_output.settled = false;

    h3s_state = H3S_STATE_WAIT_VISION;
    h3s_waypoint = 0U;
    h3s_target_count = StepMotor_GetEncoderCount();
    h3s_have_prediction = false;
    h3s_prev_velocity_mm_s = 0.0f;
    h3s_acceleration_est_mm_s2 = 0.0f;
    h3s_have_prev_velocity = false;
    h3s_start_ms = BSP_Time_GetMs();
    h3s_phase_ms = h3s_start_ms;
    h3s_last_ui = 0U;
    h3s_last_telemetry = 0U;
    h3s_rendered_state = (H3S_STATE)0xFF;
    RpiUart_ResetStats();

    /* 进入时把全部换算与整定参数打一遍，日志自带上下文，事后不必猜用的是哪版参数。 */
    DebugUart_Printf(
        "[SCVCFG] mode=pure-scurve linkage=pygame-lut lut_level=180 "
        "lut_zero_is_nominal=1 level_bias=%.3fdeg interp_err=%.4fdeg\r\n",
        (double)H3S_SCURVE_CONFIG.level_bias_deg,
        (double)APP_BALL_LINKAGE_MODEL_MAX_INTERP_ERROR_DEG);
    DebugUart_Printf(
        "[SCVCFG] kg=%.1f amax=%.1f vmax=%.1f tmin=%.2f tmax=%.2f\r\n",
        (double)H3S_SCURVE_CONFIG.rolling_acceleration_gain_mm_s2,
        (double)H3S_SCURVE_CONFIG.max_acceleration_mm_s2,
        (double)H3S_SCURVE_CONFIG.max_velocity_mm_s,
        (double)H3S_SCURVE_CONFIG.min_duration_s,
        (double)H3S_SCURVE_CONFIG.max_duration_s);
    DebugUart_Printf(
        "[SCVCFG] kp=%.5f kd=%.5f fblim=%.2f roll=%.3f rolldb=%.1f replan=%.1f\r\n",
        (double)H3S_SCURVE_CONFIG.kp_deg_per_mm,
        (double)H3S_SCURVE_CONFIG.kd_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.feedback_limit_deg,
        (double)H3S_SCURVE_CONFIG.rolling_resistance_deg,
        (double)H3S_SCURVE_CONFIG.rolling_ff_speed_deadband_mm_s,
        (double)H3S_SCURVE_CONFIG.replan_error_mm);
    DebugUart_Printf(
        "[SCVCFG] alim=%.2f..%.2f arate=%.1f speed=%.1f servo_kp=%.1f "
        "tol=%d resume=%d minspd=%.1f tick=%ums\r\n",
        (double)H3S_SCURVE_CONFIG.angle_min_deg,
        (double)H3S_SCURVE_CONFIG.angle_max_deg,
        (double)H3S_SCURVE_CONFIG.angle_rate_limit_deg_s,
        (double)H3S_ACTUATOR_SPEED_DEG_S,
        (double)STEP_MOTOR_SERVO_KP,
        (int)STEP_MOTOR_POSITION_TOLERANCE_COUNTS,
        (int)STEP_MOTOR_SERVO_RESUME_COUNTS,
        (double)STEP_MOTOR_SERVO_MIN_SPEED_DEG_S,
        (unsigned)STEP_MOTOR_TICK_PERIOD_MS);
    DebugUart_Printf(
        "[SCVCFG] move-integral ki=%.5f lim=%.2fdeg leak=%.1fs "
        "holdapply=%u minspd=%.1f seed=%u\r\n",
        (double)H3S_SCURVE_CONFIG.move_ki_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.move_integral_limit_deg,
        (double)H3S_SCURVE_CONFIG.move_integral_leak_tau_s,
        (unsigned)(H3S_SCURVE_CONFIG.move_integral_apply_in_hold ? 1U : 0U),
        (double)H3S_SCURVE_CONFIG.move_integral_min_speed_mm_s,
        (unsigned)(H3S_SCURVE_CONFIG.move_integral_seed_from_angle ? 1U : 0U));
    DebugUart_Printf(
        "[SCVCFG] hold kd=%.5f(move %.5f) enter=%.1fmm,%.1fmm/s,%.2fs "
        "exit=%.1fmm tau=%.2fs\r\n",
        (double)H3S_SCURVE_CONFIG.hold_kd_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.kd_deg_per_mm_s,
        (double)H3S_SCURVE_CONFIG.hold_enter_error_mm,
        (double)H3S_SCURVE_CONFIG.hold_enter_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.hold_enter_dwell_s,
        (double)H3S_SCURVE_CONFIG.hold_exit_error_mm,
        (double)H3S_SCURVE_CONFIG.hold_blend_tau_s);
    DebugUart_Printf(
        "[SCVCFG] dither amp=%.2fdeg f=%.1fHz minerr=%.1fmm maxspd=%.1f "
        "dwell=%.2fs rate_need=%.1fdeg/s\r\n",
        (double)H3S_SCURVE_CONFIG.dither_amplitude_deg,
        (double)H3S_SCURVE_CONFIG.dither_frequency_hz,
        (double)H3S_SCURVE_CONFIG.dither_min_error_mm,
        (double)H3S_SCURVE_CONFIG.dither_max_speed_mm_s,
        (double)H3S_SCURVE_CONFIG.dither_dwell_s,
        (double)(6.2831853f * H3S_SCURVE_CONFIG.dither_frequency_hz *
                 H3S_SCURVE_CONFIG.dither_amplitude_deg));
    DebugUart_Printf("[SCVCFG] waypoints=%.0f,%.0f,%.0f dwell=%ums\r\n",
                     (double)H3S_WAYPOINT_MM[0], (double)H3S_WAYPOINT_MM[1],
                     (double)H3S_WAYPOINT_MM[2], (unsigned)H3S_WAYPOINT_DWELL_MS);
}

static void H3S_Telemetry(uint32_t now, bool usable, STEP_MOTOR_GUARD_STATE guard){
    RPI_UART_STATS stats;
    RpiUart_GetStats(&stats);
    int32_t encoder = StepMotor_GetEncoderCount();
    float beam = H3S_LinkageAngleFromCount(encoder);
    float target_mm = H3S_WAYPOINT_MM[h3s_waypoint];

    /*
     * 一行输出全部中间量，key=value 便于主机侧按名解析。分组顺序 =
     * 数据流方向：视觉 → 剖面 → 控制分量 → 执行器 → 链路健康。
     *
     * 辨识用的两个关键量是 beam（实际水管角）和 aest（球加速度估计）：
     * 对 (beam, aest) 做线性回归，斜率 = 真实 K_G，零交点 = 真实水平角。
     */
    DebugUart_Printf(
        /* --- 时基与状态 --- */
        "[SCV] t=%lu st=%u wp=%u ok=%u "
        /* --- 视觉层：原始 / 使用值 / 龄期 / 质量 --- */
        "xr=%.2f x=%.2f vr=%.2f v=%.2f aest=%.1f age=%.1f q=%u "
        /* --- 剖面层 --- */
        "tgt=%.1f xref=%.2f vref=%.2f aref=%.1f tp=%.3f tpd=%.3f act=%u "
        /* --- MOVE/HOLD 调度：hm=已进 HOLD，hb=增益混合比，kde=生效的 Kd --- */
        "hm=%u hb=%.2f kde=%.5f "
        /* --- 控制分量：合成前每一项 --- */
        /* ki = 积分分量；其收敛值即自动标定出的水平角偏置。iact = 本拍在累积。 */
        "bias=%.3f ki=%.4f iact=%u ff=%.3f rff=%.3f fb=%.3f dith=%.3f u=%.3f "
        /* --- 跟踪误差 --- */
        "ex=%.2f ev=%.2f etgt=%.2f "
        /* --- 执行器层：指令角 vs 实际角是滞后的直接指标 --- */
        "beam=%.3f lag=%.3f cnt=%ld cmd=%ld perr=%ld spd=%.1f frq=%lu at=%u "
        /*
         * --- 标志与链路健康 ---
         * fbc = PD 分量被 feedback_limit_deg 夹住（sat 只反映物理角度范围，
         * 早期版本缺这一位，45% 的反馈限幅在遥测里是隐形的）。
         * dth = 抖动正在注入。
         */
        "sat=%u fbc=%u rl=%u dth=%u set=%u mv=%u vv=%u px=%u edge=%u deg=%u hold=%u "
        "guard=%u fps=%.1f gap=%lu inv=%lu crc=%lu drop=%lu\r\n",
        (unsigned long)(now - h3s_start_ms), (unsigned)h3s_state,
        (unsigned)h3s_waypoint, (unsigned)(usable ? 1U : 0U),

        (double)(h3s_have_prediction ? h3s_prediction.measured_x_mm : 0.0f),
        (double)(h3s_have_prediction ? h3s_prediction.x_mm : 0.0f),
        (double)(h3s_have_prediction ? h3s_prediction.measured_velocity_mm_s : 0.0f),
        (double)(h3s_have_prediction ? h3s_prediction.velocity_mm_s : 0.0f),
        (double)h3s_acceleration_est_mm_s2,
        (double)(h3s_have_prediction ? h3s_prediction.age_ms : 0.0f),
        (unsigned)(h3s_have_prediction ? h3s_prediction.quality : 0U),

        (double)target_mm,
        (double)h3s_output.x_ref_mm, (double)h3s_output.v_ref_mm_s,
        (double)h3s_output.a_ref_mm_s2,
        (double)h3s_output.profile_time_s, (double)h3s_output.profile_duration_s,
        (unsigned)(h3s_output.profile_active ? 1U : 0U),

        (unsigned)(h3s_output.hold_mode ? 1U : 0U),
        (double)h3s_output.hold_blend,
        (double)h3s_output.kd_effective_deg_per_mm_s,

        (double)H3S_SCURVE_CONFIG.level_bias_deg,
        (double)h3s_output.integral_deg,
        (unsigned)(h3s_output.integral_active ? 1U : 0U),
        (double)h3s_output.feedforward_deg, (double)h3s_output.rolling_ff_deg,
        (double)h3s_output.feedback_deg, (double)h3s_output.dither_deg,
        (double)h3s_output.angle_deg,

        (double)h3s_output.position_error_mm, (double)h3s_output.velocity_error_mm_s,
        (double)(target_mm - (h3s_have_prediction ? h3s_prediction.x_mm : 0.0f)),

        (double)beam, (double)(h3s_output.angle_deg - beam),
        (long)encoder, (long)h3s_target_count,
        (long)StepMotor_GetPositionErrorCount(),
        (double)StepMotor_GetSpeed(),
        (unsigned long)StepMotor_GetStepFrequencyHz(),
        (unsigned)(StepMotor_IsAtTarget() ? 1U : 0U),

        (unsigned)(h3s_output.saturated ? 1U : 0U),
        (unsigned)(h3s_output.feedback_clipped ? 1U : 0U),
        (unsigned)(h3s_output.rate_limited ? 1U : 0U),
        (unsigned)(h3s_output.dither_on ? 1U : 0U),
        (unsigned)(h3s_output.settled ? 1U : 0U),
        (unsigned)(h3s_have_prediction && h3s_prediction.moving),
        (unsigned)(h3s_have_prediction && h3s_prediction.velocity_trusted),
        (unsigned)(h3s_have_prediction && h3s_prediction.position_extrapolated),
        (unsigned)(h3s_have_prediction && h3s_prediction.edge),
        (unsigned)(h3s_have_prediction && h3s_prediction.degraded),
        (unsigned)(h3s_have_prediction && h3s_prediction.hold_output),
        (unsigned)guard, (double)stats.frame_rate_x10 * 0.1,
        (unsigned long)stats.seq_gap, (unsigned long)stats.invalid,
        (unsigned long)stats.crc_fail,
        (unsigned long)DebugUart_GetDroppedBytes());
}

static APP_TASK_STATUS H3S_Tick(float dt){
    uint32_t now = BSP_Time_GetMs();
    STEP_MOTOR_GUARD_STATE guard = StepMotor_GetGuardState();
    bool usable = H3S_ReadPrediction();
    /* 热更命令轮询：非阻塞，单拍最多处理一整行。 */
    AppBallTune_Poll();

    if (guard == STEP_MOTOR_GUARD_FAULT){
        h3s_state = H3S_STATE_ACTUATOR_FAULT;
        return APP_TASK_FAULT;
    }

    if (usable){
        H3S_UpdateAccelerationEstimate(h3s_prediction.velocity_mm_s, dt);
    }

    switch (h3s_state){
        case H3S_STATE_WAIT_VISION:
            if (usable && (fabsf(h3s_prediction.x_mm) <= H3S_ARM_POSITION_MM)){
                BallScurve_Reset(&h3s_controller, H3S_ActualBeamAngleDeg());
                h3s_state = H3S_STATE_ARMING;
                h3s_phase_ms = now;
                DebugUart_Printf("[SCV] armed x=%.2f v=%.2f age=%.1f beam=%.3f\r\n",
                                 (double)h3s_prediction.x_mm,
                                 (double)h3s_prediction.velocity_mm_s,
                                 (double)h3s_prediction.age_ms,
                                 (double)H3S_ActualBeamAngleDeg());
            }
            break;

        case H3S_STATE_ARMING:
            if (!usable){
                h3s_state = H3S_STATE_WAIT_VISION;
                break;
            }
            if ((now - h3s_phase_ms) >= H3S_ARM_DWELL_MS){
                H3S_PlanCurrentWaypoint();
            }
            break;

        case H3S_STATE_MOVING:
        case H3S_STATE_DWELL:
        case H3S_STATE_DONE: {
            if (!usable){
                /* 失去视觉时就地保持水管角，绝不向可能错误的标定水平位回归。 */
                (void)StepMotor_Stop();
                h3s_target_count = StepMotor_GetEncoderCount();
                h3s_state = H3S_STATE_DEGRADED;
                break;
            }
            BALL_SCURVE_INPUT input = {
                .x_mm = h3s_prediction.x_mm,
                .velocity_mm_s = h3s_prediction.velocity_mm_s,
                .actual_angle_deg = H3S_ActualBeamAngleDeg(),
                .dt_s = dt,
            };
            if (!BallScurve_Update(&h3s_controller, &H3S_SCURVE_CONFIG, &input,
                                   &h3s_output) ||
                (H3S_CommandAngle(h3s_output.angle_deg) != BSP_STATUS_OK)){
                h3s_state = H3S_STATE_ACTUATOR_FAULT;
                return APP_TASK_FAULT;
            }

            if ((h3s_state == H3S_STATE_MOVING) && !h3s_output.profile_active){
                h3s_state = H3S_STATE_DWELL;
                h3s_phase_ms = now;
                DebugUart_Printf("[SCV] arrive wp=%u target=%.1f x=%.2f err=%.2f\r\n",
                                 (unsigned)h3s_waypoint,
                                 (double)H3S_WAYPOINT_MM[h3s_waypoint],
                                 (double)h3s_prediction.x_mm,
                                 (double)(h3s_prediction.x_mm -
                                          H3S_WAYPOINT_MM[h3s_waypoint]));
            }
            if ((h3s_state == H3S_STATE_DWELL) &&
                ((now - h3s_phase_ms) >= H3S_WAYPOINT_DWELL_MS)){
                if ((h3s_waypoint + 1U) < H3S_WAYPOINT_COUNT){
                    h3s_waypoint++;
                    H3S_PlanCurrentWaypoint();
                } else{
                    /*
                     * 序列跑完后**保持**在最后一个航点，不返回 DONE：纯 S 曲线的
                     * 末端落点误差正是要观察的对象，任务自动退出会把它冲掉。
                     */
                    h3s_state = H3S_STATE_DONE;
                }
            }
            break;
        }

        case H3S_STATE_DEGRADED:
            if (usable && (fabsf(h3s_prediction.x_mm) <= H3S_ARM_POSITION_MM)){
                BallScurve_Reset(&h3s_controller, H3S_ActualBeamAngleDeg());
                h3s_state = H3S_STATE_DONE;
            }
            break;

        case H3S_STATE_ACTUATOR_FAULT:
        default:
            return APP_TASK_FAULT;
    }

    if ((now - h3s_last_telemetry) >= H3S_TELEMETRY_PERIOD_MS){
        h3s_last_telemetry = now;
        H3S_Telemetry(now, usable, guard);
    }

    bool render_changed = (h3s_state != h3s_rendered_state);
    bool render_periodic = (now - h3s_last_ui) >= H3S_UI_PERIOD_MS;
    if ((render_changed || render_periodic) && !Ui_IsFlushBusy()){
        h3s_last_ui = now;
        H3S_Render();
        h3s_rendered_state = h3s_state;
    }
    return APP_TASK_RUNNING;
}

static void H3S_Exit(void){
    (void)StepMotor_Stop();
    /* 把步进位置环增益还原成全局默认，避免影响后续任务。 */
    (void)StepMotor_SetServoGain(STEP_MOTOR_SERVO_KP);
    h3s_target_count = StepMotor_GetEncoderCount();
    DebugUart_Printf("[SCV] exit wp=%u hold-count=%ld beam=%.3f\r\n",
                     (unsigned)h3s_waypoint, (long)h3s_target_count,
                     (double)H3S_ActualBeamAngleDeg());
}

const APP_TASK_DESC APP_H3_BALL_SCURVE = {
    "H3 Ball SCurve", H3S_Enter, H3S_Tick, H3S_Exit
};

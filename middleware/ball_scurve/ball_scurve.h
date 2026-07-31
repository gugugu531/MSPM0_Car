/**
 * @file  ball_scurve.h
 * @brief 纯五次 S 曲线滚球点到点控制（纯计算，不访问硬件）。
 *
 * 与 `middleware/ball_balance` 的区别是**控制哲学**，不是参数：
 *
 *   ball_balance : 反馈主导。每拍由当前误差直接算倾角，用非线性调度和制动
 *                  截停来收敛。视觉时延直接进入闭环相位。
 *   ball_scurve  : 前馈主导。进入时按当前实测状态一次性规划一条五次剖面，
 *                  之后倾角主要由 `a_ref(t)/K_G` 给出，反馈只纠正剖面跟踪误差。
 *
 * 选择前馈主导的物理依据：球对倾角是**无阻尼双积分器**，位置扰动响应
 * 幅值 = K_G·α/ω²。球以速度 v 通过波长 λ 的表面起伏时 ω = 2πv/λ，故
 * 表面缺陷的影响按 **1/v²** 衰减——v = 60 mm/s 时 λ = 20 mm、幅值 0.5° 的
 * 起伏只造成 0.17 mm 位置扰动。**滚动中的球基本不受凹槽缺陷影响**，
 * 模型可信，所以应当把权重放在前馈而不是高增益反馈上。
 *
 * ⚠ 本模块**刻意不含**低速捕获逻辑（抖动、蠕进、单向逼近、俘获偏置）。
 *   剖面末端速度二阶趋零，球会在 v < v_esc(约 20~40 mm/s) 的低速段被局部
 *   凹陷俘获，落点误差就是本模块的固有下限。这是刻意留出的对照基线。
 *
 * 正方向约定：水管正倾角使球向正方向加速。调用方须保证视觉 x/v 与之一致。
 */
#ifndef BALL_SCURVE_H
#define BALL_SCURVE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 五次剖面归一化系数个数，仅供内部使用。 */
#define BALL_SCURVE_COEFFICIENTS 3U

typedef struct {
    /* ===== 对象模型 ===== */
    /**
     * 滚球增益 K_G = (5/7)·g，单位 mm/s² per rad。
     * 纯滚动实心球 1 + I/(mr²) = 7/5，故 ẍ = (5/7)·g·sinθ。
     * 该值是物理常数而非整定参数；只有管身扭转会让它偏小几个百分点。
     */
    float rolling_acceleration_gain_mm_s2;

    /* ===== 剖面规划 ===== */
    /** 剖面峰值加速度上限，mm/s²。决定所需峰值倾角 asin(a/K_G)。 */
    float max_acceleration_mm_s2;
    /** 剖面峰值速度上限，mm/s。取太小会拉长低速段，反而更容易被俘获。 */
    float max_velocity_mm_s;
    /** 规划时长的下限/上限，s。上限同时兜住"目标极近时时长趋零"。 */
    float min_duration_s;
    float max_duration_s;

    /* ===== 剖面跟踪反馈 ===== */
    /** 位置反馈增益，deg/mm。由 wn²/K_G 换算。 */
    float kp_deg_per_mm;
    /** 速度反馈增益，deg/(mm/s)。由 2ζwn/K_G 换算。 */
    float kd_deg_per_mm_s;
    /**
     * 反馈分量的限幅，deg。**只限反馈，不限前馈**——前馈是模型算出的
     * 必要驱动，把它一起夹掉会让剖面根本走不动。
     */
    float feedback_limit_deg;

    /* ===== 前馈修正 ===== */
    /**
     * 滚动阻力前馈幅值，deg，按 sign(v_ref) 施加。
     *
     * 不加这一项会有**系统性欠冲**：滚阻在整个行程内恒定反向，欠冲量
     * = 0.5·K_G·θ_roll·T²。θ_roll = 0.06°、T = 1.2 s 时是 5.3 mm，
     * 占 ±10 mm 判据的一半以上。
     *
     * 默认 0：首次上板时先测出实际欠冲量，再反解 θ_roll 填回来
     * （θ_roll = 2·欠冲量/(K_G·T²)），比拍脑袋填一个值可靠。
     */
    float rolling_resistance_deg;
    /** 施加滚阻前馈所需的最小 |v_ref|，mm/s。低于此值不施加，避免末端换号抖动。 */
    float rolling_ff_speed_deadband_mm_s;
    /**
     * 动力学水平点相对查表 0° 的偏置，deg。
     *
     * 查表的 0° 是**几何名义水平**，不是"球不动"的那个角。二者之差来自
     * 上电参考位重复性、机构装配和水管弯曲。全部倾角指令都叠加本值。
     * 未标定前填 0，此时它的真实值会表现为剖面全程的单向漂移。
     */
    float level_bias_deg;

    /* ===== 输出限制 ===== */
    /** 水管角物理可达范围，deg（由编码器软限位经查表换算）。 */
    float angle_min_deg;
    float angle_max_deg;
    /** 输出斜率限制，deg/s。须覆盖剖面所需的 dθ_ff/dt，否则前馈会被削顶。 */
    float angle_rate_limit_deg_s;

    /* ===== MOVE / HOLD 增益调度 ===== */
    /*
     * 为什么要分两段——**两段的敌人根本不同**：
     *
     *            MOVE                        HOLD
     *   球速     60~90 mm/s（远超逃逸速度）    ≈0
     *   主敌     轨迹跟踪误差、执行器带宽      静摩擦死区、饱和极限环
     *   缺陷     按 1/v² 衰减，可忽略          主导
     *   速度信号 有效信号                      树莓派给整数 mm/s，多为量化噪声
     *   Kd       提供阻尼，**必需**            **有害**：放大量化噪声、驱动饱和
     *   Kp       跟踪刚度                      直接决定静摩擦残差 θ_stick/Kp
     *
     * 2026-07-31 实测：剖面走完后（act=0、ff=0，控制律已退化成绕固定目标的
     * 纯 PD）仍出现 6.42 rad/s、峰峰 17~28 mm 的持续极限环。分解指令角：
     *
     *     Kp·e = 0.04711 × 12 mm   = 0.57°
     *     Kd·v = 0.03533 × 40 mm/s = 1.41°   ← 饱和的是这一项
     *     合计 1.98° ≈ 限幅 2.000°
     *
     * 而 fbc（反馈限幅命中率）与极限环振幅单调相关（0%→17.4mm，30%→27.7mm）。
     * **极限环由 Kd 项饱和驱动，而那个 Kd 只有移动段才需要。**
     *
     * ⚠ 只做增益调度，不做两套算法：结构相同才能无扰切换。两个独立算法会带来
     *   两套状态、两处限幅和切换瞬间的角度阶跃，都是新的失效面。
     *
     * ⚠ **Kp 不参与调度**。降 Kp 会让静摩擦残差 θ_stick/Kp 变大（实测
     *   θ_stick≈0.62°，Kp=0.04711 时残差 13.2 mm），方向正好反了。
     */
    /**
     * HOLD 段的速度反馈增益，deg/(mm/s)。**0 = 关闭调度**，全程用
     * kd_deg_per_mm_s。
     *
     * 取值约束是「切入瞬间不许饱和」，最坏情况 v≈50 mm/s：
     *     Kd_hold × 50 < 1.0°  →  Kd_hold < 0.020
     */
    float hold_kd_deg_per_mm_s;
    /**
     * 进入 HOLD 的判据：误差和速度**同时**满足并持续 hold_enter_dwell_s。
     *
     * ⚠ 只看距离是不够的——极限环里球在 −45 mm、目标 −50 mm，只差 5 mm
     *   已经"很近"，却正以 40~70 mm/s 高速穿过。此时切到低 Kd 会直接飞过去。
     */
    float hold_enter_error_mm;
    float hold_enter_speed_mm_s;
    float hold_enter_dwell_s;
    /** 退出 HOLD 的误差阈值（滞回），须显著大于 hold_enter_error_mm。 */
    float hold_exit_error_mm;
    /** 增益过渡时间常数，s。让 Kd 连续变化，切换瞬间指令角不跳变。 */
    float hold_blend_tau_s;

    /* ===== 抖动：破静摩擦死区 ===== */
    /*
     * 为什么需要它——纯 PD 的稳态残差有一个**与整定无关的数学下限**：
     *
     *     残差 = θ_stick / Kp
     *
     * 球停住的条件是 |净倾角| < 脱离角，此时控制器再怎么推也推不动。
     * 2026-07-31 实测（COM16，75 s 无扰动）：球停在 −36.9 mm、目标 −50 mm，
     * 控制器持续给出 −0.620° 而球一动不动 ⇒ **θ_stick ≈ 0.62°**，
     * 代入 Kp=0.04711 °/mm 得下限 13.2 mm，与实测 13.1 mm 完全吻合。
     * 也就是说：**不破静摩擦，±10 mm 判据在这台机器上数学上不可达。**
     *
     * 抖动为什么代价极低——球对倾角是双积分器，位置响应幅值 = K_G·A/ω²，
     * 随频率按 ω⁻² 衰减。管子实打实抖 ±0.6°，球只晃约 0.5 mm。
     *
     * 幅值必须按**实际**水管角定，而指令要考虑步进位置环的一阶衰减
     * （τ = 1/STEP_MOTOR_SERVO_KP，当前 0.1 s）：
     *
     *     实际幅值 = 指令幅值 / sqrt(1 + (2πf·τ)²)
     *
     *   f      指令      实际     电机峰值转速    球纹波
     *   1.5Hz  ±0.82°   ±0.6°     47 deg/s      0.83 mm
     *   2.0Hz  ±0.97°   ±0.6°     74 deg/s      0.46 mm   ← 当前取值
     *   3.0Hz  ±1.28°   ±0.6°    146 deg/s ✗   0.21 mm    （超 120 限速）
     *
     * ⚠ angle_rate_limit_deg_s 必须 > 2πf·A，否则抖动会被斜率限制削顶。
     *   2 Hz / ±0.97° 需要 12.2 deg/s。
     */
    /** 抖动指令幅值，deg。**0 = 关闭**。 */
    float dither_amplitude_deg;
    float dither_frequency_hz;
    /**
     * 触发抖动所需的最小目标误差，mm。误差已经够小就别抖——
     * 抖动会带来约 0.5 mm 纹波，在判据内注入纹波是净亏损。
     */
    float dither_min_error_mm;
    /** 触发抖动所需的最大球速，mm/s。球还在滚就不需要破静摩擦。 */
    float dither_max_speed_mm_s;
    /** 上述条件需连续满足多久才起振，s。防止移动末段的瞬时低速误触发。 */
    float dither_dwell_s;

    /* ===== 稳定判据（只影响遥测与状态，不参与控制律）===== */
    float settled_position_mm;
    float settled_speed_mm_s;
    float settled_time_s;

    /**
     * 跟踪误差超过本值时按当前实测状态重新规划，mm。
     * **0 = 关闭 = 纯 S 曲线**，这是本模块的默认语义。
     */
    float replan_error_mm;
} BALL_SCURVE_CONFIG;

typedef struct {
    /* 规划起点状态（规划时刻冻结）。 */
    float x0_mm;
    float v0_mm_s;
    float a0_mm_s2;
    /* 归一化五次系数，作用于 s = t/T 的 s³/s⁴/s⁵ 项。 */
    float c[BALL_SCURVE_COEFFICIENTS];
    float duration_s;
    float elapsed_s;
    float target_mm;
    bool  active;

    /* 输出斜率限制与稳定判据的状态。 */
    float last_angle_deg;
    bool  have_last_angle;
    float settled_elapsed_s;
    bool  settled;

    /* 抖动状态。相位连续累加，避免起停时产生角度阶跃。 */
    float dither_phase_rad;
    float dither_stuck_elapsed_s;
    bool  dither_on;

    /* MOVE/HOLD 调度状态。blend 0=MOVE 增益，1=HOLD 增益，一阶过渡。 */
    bool  hold_mode;
    float hold_enter_elapsed_s;
    float hold_blend;
} BALL_SCURVE_CONTROLLER;

typedef struct {
    float x_mm;
    float velocity_mm_s;
    /** 由编码器经查表得到的**实际**水管角，用于斜率限制的续接。 */
    float actual_angle_deg;
    float dt_s;
} BALL_SCURVE_INPUT;

typedef struct {
    /** 最终下发的水管角指令，deg（已含 level_bias、限幅与斜率限制）。 */
    float angle_deg;

    /* 剖面参考量。 */
    float x_ref_mm;
    float v_ref_mm_s;
    float a_ref_mm_s2;

    /* 指令分解，便于遥测中直接看清每一项的贡献。 */
    float feedforward_deg;   /**< asin(a_ref/K_G) */
    float rolling_ff_deg;    /**< 滚阻前馈 */
    float feedback_deg;      /**< 限幅后的 PD 分量 */
    float dither_deg;        /**< 本拍注入的抖动量（已含符号） */

    float position_error_mm;
    float velocity_error_mm_s;
    float profile_time_s;
    float profile_duration_s;
    /** 本拍实际生效的速度反馈增益（MOVE 与 HOLD 之间的过渡值）。 */
    float kd_effective_deg_per_mm_s;
    /** 增益混合比，0=MOVE，1=HOLD。 */
    float hold_blend;

    bool  profile_active;    /**< 剖面尚未走完 */
    bool  saturated;         /**< 总指令被物理角度范围夹住 */
    bool  feedback_clipped;  /**< PD 分量被 feedback_limit_deg 夹住 */
    bool  rate_limited;      /**< 本拍被输出斜率限制削过 */
    bool  dither_on;         /**< 抖动正在注入 */
    bool  hold_mode;         /**< 已判定进入 HOLD 段 */
    bool  settled;
} BALL_SCURVE_OUTPUT;

/** 初始化并清空全部状态；不产生任何输出。 */
void BallScurve_Init(BALL_SCURVE_CONTROLLER *controller);

/**
 * @brief 清空动态状态，并令下一拍的斜率限制从给定实际水管角续接。
 * @note 进入任务或视觉恢复后必须调用，否则第一拍会从上一次的陈旧角度做斜率限制。
 */
void BallScurve_Reset(BALL_SCURVE_CONTROLLER *controller, float current_angle_deg);

/**
 * @brief 按当前实测状态规划一条到 target_mm 的五次剖面（末端 v=0、a=0）。
 * @return 规划成功为 true；参数非法（K_G/时长/加速度上限非正）为 false。
 *
 * 时长由「峰值加速度」「峰值速度」「时长下限」三者取严，再迭代收紧到
 * 真实峰值加速度不超上限——起点带初速时闭式公式只是近似，必须校核。
 */
bool BallScurve_PlanTo(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       float x_now_mm,
                       float v_now_mm_s,
                       float target_mm);

/** 执行一次剖面推进 + 前馈/反馈合成。剖面走完后退化为绕目标点的纯 PD。 */
bool BallScurve_Update(BALL_SCURVE_CONTROLLER *controller,
                       const BALL_SCURVE_CONFIG *config,
                       const BALL_SCURVE_INPUT *input,
                       BALL_SCURVE_OUTPUT *output);

/** 剖面是否已走完（不代表球已停稳，稳定看 output.settled）。 */
bool BallScurve_IsProfileFinished(const BALL_SCURVE_CONTROLLER *controller);

/**
 * @brief 按剖面时长公式估算一次移动需要的时间，s。
 * @note 供上层做序列时间预算，不改变控制器状态。
 */
float BallScurve_EstimateDuration(const BALL_SCURVE_CONFIG *config,
                                  float distance_mm);

#ifdef __cplusplus
}
#endif

#endif /* BALL_SCURVE_H */


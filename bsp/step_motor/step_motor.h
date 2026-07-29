/**
 * @file  step_motor.h
 * @brief BSP 摆杆步进电机开环控制 + QEI 位置读取。
 *
 * 硬件:STEP 脉冲 = SMotor(TIMG6_CCP0, PA29),DIR = PB14,EN = PB11;
 *      角度反馈 = SMotor_QEI(TIMG8, PHA=PA26 / PHB=PA27)。
 *
 * 位置有两套来源,互为校验:StepMotor_GetEstimatedPosition() 是速度积分的开环值,
 * 恒有输出但丢步不可见;StepMotor_GetMeasuredPosition() 来自 QEI 硬件计数,反映
 * 真实轴角。二者持续偏离即说明丢步或机械打滑。
 *
 * STEP 波形:50% 占空比方波,装载值四舍五入;停止时用 ODIS 强制 CCP 为低,
 * 而非只把 CC 写 0(后者在 EDGE_ALIGN_UP 下会让零事件与比较事件重合,可能挤出毛刺)。
 *
 * ⚠ STEP_MOTOR_STEP_TIMER_CLK_HZ 必须与 SysConfig 中 SMotor 的分频结果一致
 *   (clockDivider=8 × clockPrescale=64,32MHz → 62500Hz)。二者不一致不会报编译错,
 *   但转速会整体偏离;改动任一侧后须核对生成的 SMotor_INST_CLK_FREQ。
 */
#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 硬件映射(随 SysConfig 变化)===== */
#define STEP_MOTOR_BEAM_DIR_PORT    SMotor_IO_PORT
#define STEP_MOTOR_BEAM_DIR_PIN     SMotor_IO_DIR1_PIN
#define STEP_MOTOR_BEAM_EN_PORT     SMotor_IO_PORT
#define STEP_MOTOR_BEAM_EN_PIN      SMotor_IO_EN1_PIN
#define STEP_MOTOR_BEAM_PWM_TIMER   SMotor_INST
#define STEP_MOTOR_BEAM_PWM_CHANNEL DL_TIMER_CC_0_INDEX
#define STEP_MOTOR_QEI_TIMER        SMotor_QEI_INST

/* ===== 极性(三项均已上板实测,换驱动板或改接线后须重验)===== */
/* 使能脚高电平有效;驱动器为低有效则改 0。已实测:高有效,取 1。 */
#define STEP_MOTOR_BEAM_ENABLE_HIGH 1U
/* 正方向对应 DIR 脚高电平;转向相反则改 0。已实测:高电平即正向,取 1。 */
#define STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH 1U
/*
 * 编码器计数正方向与电机正方向相反时改 1。
 * 实测:est 为正(DIR 正向)时 cnt 为负,故取 1。本宏只翻转 GetEncoderCount /
 * GetMeasuredPosition / IsEncoderCountingUp,GetEncoderRaw 仍是硬件原值不翻。
 */
#define STEP_MOTOR_ENCODER_INVERT 1U

/* ===== 电机与驱动器参数 ===== */
/* 电机固有步距角,单位 deg。已核对铭牌:1.8deg,即 200 整步/转。 */
#define STEP_MOTOR_STEP_ANGLE_DEG 1.8f
/*
 * 驱动器细分数,须与驱动器拨码一致。
 * 实测值:Device Check 的 TURN 模式令开环角走满 360deg(发出 200x32=6400 个脉冲),
 * 电机轴实际转了 4 圈 → 每转 6400/4=1600 脉冲 → 1600/(360/1.8)=8 细分。
 * 已与驱动板拨码开关核对一致(拨码=8),见 STEP_MOTOR_STEP_TIMER_CLK_HZ 处的说明。
 * 换驱动板或拨拨码后必须重测,固件与硬件不一致不会报错,只会让转速整体差固定倍数。
 */
#define STEP_MOTOR_MICROSTEP 8.0f
/* 速度限幅,单位 deg/s。 */
#define STEP_MOTOR_MAX_SPEED_DEG_S 240.0f

/* ===== STEP 定时器 ===== */
/*
 * STEP 定时器输入时钟,由 SysConfig 中 SMotor 的 clockDivider(8) × clockPrescale(64)
 * 从 32MHz 总线时钟分出 = 62500 Hz。
 *
 * 这是提交 e621209 上实测可正常驱动步进电机的分频组合(同为 TIMG6 / PA29),
 * 沿用之。曾按 WHEELTEC 官方例程改成 1 MHz(其 CPU 为 80MHz PLL),换算上等价、
 * 分辨率更高,但在本板实测未能正常驱动,故回退。
 *
 * 已间接验证:TURN 模式量的是 MICROSTEP 与本值的**比值**——两者任一错了另一个都会
 * 补偿回来,角度刻度照样准,单靠转圈数分不出是谁错。但拟合出的 8 细分与驱动板拨码
 * 开关读数一致,说明本值没有被用来吸收误差,62500 Hz 正确。
 *
 * ⚠ 改动 SysConfig 分频后必须同步本值,并核对生成的 SMotor_INST_CLK_FREQ。
 */
#define STEP_MOTOR_STEP_TIMER_CLK_HZ 62500U

/*
 * 步进频率下限,由 16 位装载值上限决定(62500 / 65536 ≈ 0.95 Hz)。
 * 低于此频率无法产生,驱动会停止脉冲改为保持——即速度指令存在死区,
 * 死区宽度 = MIN_STEP_FREQ × STEP_ANGLE / MICROSTEP。
 */
#define STEP_MOTOR_MIN_STEP_FREQ_HZ 1U
/* 步进频率上限,取自官方例程;再高易失步。 */
#define STEP_MOTOR_MAX_STEP_FREQ_HZ 5000U
#define STEP_MOTOR_MAX_ARR 65535U

/*
 * 编码器每转的 QEI 计数。MSPM0 的 QEI 2-input 模式对 A/B 两路各取上升沿
 * (见 DL_Timer_configQEI 的 CC_TRIG_RISE),即 2 倍频,故本值 = 编码器线数 × 2。
 * 实测值:Device Check 的 HAND 模式断电手转一整圈,cnt 变化 2000,反推编码器为 1000 线。
 * 换编码器后必须重测,否则 StepMotor_GetMeasuredPosition() 的刻度是错的。
 */
#define STEP_MOTOR_ENCODER_COUNTS_PER_REV 2000.0f

/*
 * 开环估计位置的软限位默认值,单位 deg(电机轴角,非摆杆角)。
 *
 * ⚠ 当前取 ±100000 deg,等效于**不限位**——这是机械行程未标定前的临时设置,
 *   便于自由试转。限位逻辑本身仍在,只是阈值放到了实际到不了的位置。
 *   摆杆装机后必须按实际连杆减速比与行程改回真实值,或运行时用
 *   StepMotor_SetPositionLimit() 覆盖;它只能防止指令超程,不能替代物理限位开关。
 */
#define STEP_MOTOR_MIN_POSITION_DEG (-100000.0f)
#define STEP_MOTOR_MAX_POSITION_DEG 100000.0f

/**
 * @brief 步进电机通道。当前只有摆杆一路,保留枚举便于后续扩展。
 */
typedef enum {
    STEP_MOTOR_CHANNEL_BEAM = 0,
    STEP_MOTOR_CHANNEL_MAX
} STEP_MOTOR_CHANNEL;

/**
 * @brief 开环估计位置的软限位。
 */
typedef struct {
    /** 最小估计位置,单位 deg。 */
    float min_deg;
    /** 最大估计位置,单位 deg。 */
    float max_deg;
} STEP_MOTOR_POSITION_LIMIT;

/**
 * @brief 初始化步进电机输出、使能驱动器并启动 QEI 计数。
 * @note PWM/QEI/GPIO 外设本体由 SysConfig(SYSCFG_DL_init) 初始化;本函数额外补上
 *       生成代码缺失的 QEI startCounter,并把编码器计数清零。
 * @note 初始化后驱动器**持续使能**,电机始终具备保持力矩(摆杆有重力负载,
 *       失能会自重回落)。代价是静止时有保持电流的嗡嗡声与发热,且轴无法手动转动;
 *       标定编码器前需调 StepMotor_SetEnabled(ch, false) 主动断电。
 */
BSP_STATUS StepMotor_Init(void);

/**
 * @brief 使能/失能驱动器(控制 EN 脚)。
 * @note 失能后线圈断电,电机无保持力矩,负载会因自重回落——摆杆等有重力负载的场合
 *       只应在确认安全时失能。使能状态下静止会有保持电流的嗡嗡声,属正常现象。
 */
BSP_STATUS StepMotor_SetEnabled(STEP_MOTOR_CHANNEL channel, bool enable);

/**
 * @brief 查询驱动器当前是否使能。
 */
bool StepMotor_IsEnabled(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 设置指定通道开环速度。
 * @param channel 步进电机通道。
 * @param speed_deg_per_s 速度,单位 deg/s;正负表示方向,0 表示停止脉冲。
 * @note 速度会被限制到 ±STEP_MOTOR_MAX_SPEED_DEG_S,并在触及位置限位时归零。
 */
BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s);

/**
 * @brief 阻塞运行指定时间后停止。
 * @param duration_ms 运行时间,单位 ms;会被位置限位裁短。
 * @warning 阻塞式接口(内部 BSP_DelayMs),不可在控制拍内调用,仅供自检/标定使用。
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
 * @brief 推进开环位置积分并采样一次编码器。
 * @param now_ms 当前毫秒时间戳。
 */
BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms);

/**
 * @brief 更新全部通道。
 */
BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms);

/**
 * @brief 获取指定通道最近设置速度,单位 deg/s。
 */
float StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 获取最近一次实际应用的步进频率,单位 Hz;0 表示未出脉冲。
 * @note 这是限幅后的真实值,与指令速度换算出的理论值可能不同(死区/上限),
 *       诊断转速不对时应先看这个。
 */
uint32_t StepMotor_GetStepFrequencyHz(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 获取最近一次写入定时器的装载值。
 * @note 实际脉冲频率 = STEP_MOTOR_STEP_TIMER_CLK_HZ / (load + 1)。
 */
uint32_t StepMotor_GetTimerLoad(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 获取开环估计位置,单位 deg。
 * @note 由速度积分得到,丢步不会被察觉;与 StepMotor_GetMeasuredPosition() 对比可发现丢步。
 */
float StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 将开环估计位置清零。不执行物理归零,也不会停止电机。
 */
void StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel);

/**
 * @brief 采样 QEI 计数并累加到 32 位计数器。
 * @note 硬件计数器是 16 位(load 65535),本函数用差值累加消除环绕,故须以
 *       「两次调用之间计数变化不超过 32767」的频率调用,否则会丢圈。
 *       StepMotor_UpdateState() 已内含本调用。
 */
void StepMotor_UpdateEncoder(void);

/**
 * @brief 获取编码器累计计数(已消除 16 位环绕,带符号)。
 */
int32_t StepMotor_GetEncoderCount(void);

/**
 * @brief 获取 QEI 硬件原始 16 位计数(调试/标定用)。
 */
uint16_t StepMotor_GetEncoderRaw(void);

/**
 * @brief 获取编码器实测角度,单位 deg。
 * @note 刻度依赖 STEP_MOTOR_ENCODER_COUNTS_PER_REV,使用前须实测标定。
 */
float StepMotor_GetMeasuredPosition(void);

/**
 * @brief 编码器计数清零(同时清硬件计数器与累加器)。
 */
void StepMotor_ResetEncoder(void);

/**
 * @brief 读取 QEI 硬件方向标志:true 表示最近一次计数为递增。
 */
bool StepMotor_IsEncoderCountingUp(void);

/**
 * @brief 设置开环估计位置的软限位。
 */
BSP_STATUS StepMotor_SetPositionLimit(const STEP_MOTOR_POSITION_LIMIT *limit);

/**
 * @brief 获取开环估计位置的软限位。

 */
STEP_MOTOR_POSITION_LIMIT StepMotor_GetPositionLimit(void);

#ifdef __cplusplus
}
#endif

#endif /* STEP_MOTOR_H */

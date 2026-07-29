/**
 * @file  app_checks.c
 * @brief 外设自检任务实现：姿态、灰度、驱动与编码器诊断。
 */
#include "app_checks.h"
#include "app_fmt.h"

#include "ui.h"
#include "key.h"
#include "bsp_time.h"
#include "bsp_common.h"
#include "debug_uart.h"
#include "chassis.h"
#include "yahboom_track.h"
#include "hall_encoder.h"
#include "step_motor.h"
#include "wit_sdk.h"
#include "kinematics/kinematics.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 自检刷屏节流周期。 */
#define CHK_UI_PERIOD_MS 200U

/* 把字符串拷入 buf，返回长度（不终止），便于随后接 AppFmt_* 拼数字。 */
static uint8_t PutStr(char *buf, const char *s){
    uint8_t i = 0U;
    while (s[i] != '\0'){
        buf[i] = s[i];
        i++;
    }
    return i;
}

/* ============================ 灰度 I2C（Yahboom，I2C0） ============================ */
/* 与 JY61P 共 I2C0：进入时挂起 JY61P、退出时恢复。 */

static uint32_t gi_last_ui;
static uint32_t gi_ok_cnt;    /* 累计读取成功次数。 */
static uint32_t gi_err_cnt;   /* 累计读取失败次数。 */

static void ChkGrayI2c_Enter(void){
    JY61P_I2C_SetSuspended(true);    /* 让出 I2C0 */
    (void)YahboomTrack_Init();       /* 探测在线；同时清零驱动内部诊断计数 */
    gi_ok_cnt     = 0U;
    gi_err_cnt    = 0U;
    gi_last_ui    = 0U;
}

static APP_TASK_STATUS ChkGrayI2c_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - gi_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    gi_last_ui = now;

    uint8_t mask = 0U;
    BSP_STATUS st = YahboomTrack_ReadDetectedMask(&mask);
    if (st == BSP_STATUS_OK){ gi_ok_cnt++; } else { gi_err_cnt++; }

    char bits[YAHBOOM_TRACK_CHANNEL_COUNT + 1U];
    uint8_t active = 0U;
    for (uint8_t i = 0U; i < YAHBOOM_TRACK_CHANNEL_COUNT; i++){
        bool on = ((mask & (uint8_t)(1U << i)) != 0U);   /* bit0=X1 */
        bits[i] = on ? '1' : '0';
        if (on){
            active++;
        }
    }
    bits[YAHBOOM_TRACK_CHANNEL_COUNT] = '\0';

    char l2[16];
    char l3[16];
    char l4[16];
    uint8_t n;

    if (st == BSP_STATUS_OK){
        n = PutStr(l2, "act ");
        AppFmt_I32(&l2[n], (int32_t)active);
    } else {
        (void)PutStr(l2, "READ FAIL");
        l2[9] = '\0';
    }

    /* 累计成功/失败次数：便于判断是偶发瞬态（err 少）还是持续故障（err 涨得快）。 */
    n = PutStr(l3, "ok ");
    AppFmt_I32(&l3[n], (int32_t)gi_ok_cnt);
    while (l3[n] != '\0'){ n++; }
    n += PutStr(&l3[n], " er ");
    AppFmt_I32(&l3[n], (int32_t)gi_err_cnt);

    /* 诊断：W=写寄存器累计失败 R=读寄存器累计失败 s=最近失败码(超时=-4 / NACK=-1)。 */
    uint32_t wr_fail = 0U;
    uint32_t rd_fail = 0U;
    int32_t  last_status = 0;
    YahboomTrack_GetDiag(&rd_fail, &wr_fail, &last_status);   /* 注意形参序为 read, write */
    n = PutStr(l4, "W");
    AppFmt_I32(&l4[n], (int32_t)wr_fail);
    while (l4[n] != '\0'){ n++; }
    n += PutStr(&l4[n], " R");
    AppFmt_I32(&l4[n], (int32_t)rd_fail);
    while (l4[n] != '\0'){ n++; }
    n += PutStr(&l4[n], " s");
    AppFmt_I32(&l4[n], last_status);

    Ui_RenderLines("Chk Gray I2C", bits, l2, l3, l4, "BACK: exit", NULL);
    return APP_TASK_RUNNING;
}

static void ChkGrayI2c_Exit(void){
    JY61P_I2C_SetSuspended(false);   /* 归还 I2C0 给 JY61P */
}

/* ============================ 陀螺仪 JY61P ============================ */

static uint32_t gj_last_ui;

static void ChkGyroJy_Enter(void){
    JY61P_I2C_SetSuspended(false);   /* 确保 JY61P 占用 I2C0 */
    JY61P_I2C_Init();
    gj_last_ui = 0U;
}

static APP_TASK_STATUS ChkGyroJy_Tick(float dt){
    (void)dt;
    JY61P_I2C_Poll();                /* 每控制周期推进 I2C 状态机 */

    uint32_t now = BSP_Time_GetMs();
    if ((now - gj_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    gj_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    uint8_t n;

    WIT_IMU_DATA imu;
    if (WitGetData(&imu) == WIT_HAL_OK){
        n = PutStr(l1, "Gz ");
        AppFmt_Fixed(&l1[n], imu.gyro_deg_s.z, 1);
        n = PutStr(l2, "Yaw ");
        AppFmt_Fixed(&l2[n], imu.attitude_deg.yaw, 1);
    } else {
        (void)PutStr(l1, "Gz --"); l1[5] = '\0';
        (void)PutStr(l2, "Yaw --"); l2[6] = '\0';
    }
    n = PutStr(l3, "err ");
    AppFmt_I32(&l3[n], (int32_t)JY61P_I2C_GetErrorCount());

    Ui_RenderLines("Chk Gyro JY61P", l1, l2, l3, "BACK: exit", NULL, NULL);
    return APP_TASK_RUNNING;
}

/* ============================ TB6612（主动） ============================ */
/*
 * 安全策略：默认停机；每次短按发一个有界低占空比脉冲(20%/300ms 后自动 Brake)，
 * 验证各通道+方向。UP=左轮 DOWN=右轮 ENTER=双轮；BACK 由框架处理并退出。
 * 屏显编码器计数变化，顺带验证电机↔编码器接线。全程提示抬起车轮。
 */
#define TB_PULSE_DUTY 20.0f
#define TB_PULSE_MS   300U

static uint32_t tb_pulse_end;
static bool     tb_active;

static void ChkTb_Render(const char *action){
    char cntL[16];
    char cntR[16];
    uint8_t n = PutStr(cntL, "encL ");
    AppFmt_I32(&cntL[n], HallEncoder_GetCount(HALL_ENCODER_LEFT));
    n = PutStr(cntR, "encR ");
    AppFmt_I32(&cntR[n], HallEncoder_GetCount(HALL_ENCODER_RIGHT));
    Ui_RenderLines("Chk TB6612", "!! WHEELS UP !!", action, cntL,
                   cntR, "UP/DN/EN pulse", "BACK: exit");
}

static void ChkTb_Enter(void){
    (void)Chassis_Brake();
    tb_active = false;
    tb_pulse_end = 0U;
    ChkTb_Render("idle");
}

static APP_TASK_STATUS ChkTb_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 结束到期的脉冲。 */
    if (tb_active && ((int32_t)(now - tb_pulse_end) >= 0)){
        (void)Chassis_Brake();
        tb_active = false;
        ChkTb_Render("idle");
    }

    if (!tb_active){
        const char *action = NULL;
        if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
            (void)Chassis_SetDuty(TB_PULSE_DUTY, 0.0f);
            action = "L pulse";
        } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
            (void)Chassis_SetDuty(0.0f, TB_PULSE_DUTY);
            action = "R pulse";
        } else if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
            (void)Chassis_SetDuty(TB_PULSE_DUTY, TB_PULSE_DUTY);
            action = "L+R pulse";
        }

        if (action != NULL){
            tb_active = true;
            tb_pulse_end = now + TB_PULSE_MS;
            ChkTb_Render(action);
        }
    }

    return APP_TASK_RUNNING;
}

static void ChkTb_Exit(void){
    (void)Chassis_Brake();
}

/* ======================= 摆杆步进电机标定（主动） ======================= */
/*
 * 五个标定模式，ENTER 长按循环切换，BACK 退出整页。每个模式只覆盖
 * docs/step-motor-calibration.md 里的一到两项，并把该填回 step_motor.h 的那个数
 * 直接算好显示出来，免得在现场心算。
 *
 *  1/5 RUN    持续正/反转，ENTER 暂停（保留 est/cnt，可再按 UP/DOWN 接着累积）。
 *             看 est 与 qei 是否同向、slip(=est-qei) 是否发散。
 *             → DIR 极性 STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH（按 UP 看摆杆倒向）
 *             → 编码器方向 STEP_MOTOR_ENCODER_INVERT（est 与 cnt 是否同号）
 *  2/5 TURN   开环精确走 N 整圈后自动停。走完看轴上标记实际转过几圈：
 *             → 细分数 STEP_MOTOR_MICROSTEP = 当前值 / 实际圈数
 *             c/N 一栏同时给出编码器每转计数的交叉验证值。
 *  3/5 HAND   进入即断电，轴可手转；手转 N 圈后 cpr 栏就是实测每转计数。
 *             → STEP_MOTOR_ENCODER_COUNTS_PER_REV
 *             进本模式轴应能拧动、回其它模式应拧不动，据此定
 *             → STEP_MOTOR_BEAM_ENABLE_HIGH
 *  4/5 SWEEP  转速逐档上扫并记录 |slip| 峰值。峰值开始持续增大即已丢步，退一档：
 *             → STEP_MOTOR_MAX_STEP_FREQ_HZ 取 f 栏的值
 *             ⚠ 必须装上摆杆带真实负载测，空载能跑的频率挂上摆杆通常要打对折。
 *  5/5 SPAN   按住 UP/DOWN 点动到机械参考位，ENTER 交替打 A/B 点，d 栏给出两点
 *             间的电机轴角。配合量角器读到的摆杆实际角度 θ：
 *             减速比 k = d / θ；软限位 = (θ_max - 5) × k
 *
 * 标定有依赖顺序：MICROSTEP 错了 SWEEP 全错，COUNTS_PER_REV 错了 SPAN 全错。
 * 按 TURN → HAND → RUN → SWEEP → SPAN 的次序做，理由见标定手册。
 *
 * est 与编码器计数只在退出整页时清零（另有 TURN/HAND/SWEEP 起测时按各自需要清）。
 * 页内暂停一律保留读数——停下来正是为了读它。
 *
 * ⚠ 摆杆已装机时注意行程——当前软限位放在 ±100000° 等效于不限位，RUN/TURN/SWEEP
 *   都不会自动避让机械死点。随时按 ENTER 或 BACK 停止。
 */

/* SWEEP 的转速档，单位 deg/s。从远低于额定处起步，逐档确认再往上。 */
static const float sm_sweep_speed[] = {
    15.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f, 210.0f, 240.0f
};
#define SM_SWEEP_STEP_COUNT ((uint8_t)(sizeof(sm_sweep_speed) / sizeof(sm_sweep_speed[0])))

/* RUN 模式转速：够慢，肉眼能跟上轴上的标记。 */
#define SM_RUN_SPEED_DEG_S 30.0f
/* TURN 模式转速：慢一些，减小加减速带来的丢步，让圈数判读干净。 */
#define SM_TURN_SPEED_DEG_S 45.0f
/* SPAN 模式点动转速：慢，便于对准机械参考位。 */
#define SM_JOG_SPEED_DEG_S 20.0f

#define SM_TURN_MAX 10U
#define SM_HAND_MAX 20U

/* 遥测周期：20ms 以便看清起停瞬态与编码器跟随情况。 */
#define SM_TELEMETRY_PERIOD_MS 20U

typedef enum {
    SM_MODE_RUN = 0,
    SM_MODE_TURN,
    SM_MODE_HAND,
    SM_MODE_SWEEP,
    SM_MODE_SPAN,
    SM_MODE_MAX
} SM_MODE;

static const char *const sm_mode_tag[SM_MODE_MAX] = {
    "SM 1/5 RUN", "SM 2/5 TURN", "SM 3/5 HAND", "SM 4/5 SWEEP", "SM 5/5 SPAN"
};

static SM_MODE  sm_mode;
static uint32_t sm_last_ui;
static uint32_t sm_last_telemetry;
static bool     sm_running;
static uint8_t  sm_turn_target;      /* TURN：目标圈数。 */
static uint8_t  sm_hand_turns;       /* HAND：操作者已手转的圈数。 */
static uint8_t  sm_sweep_idx;        /* SWEEP：当前转速档索引。 */
static float    sm_slip_peak;        /* SWEEP：本档内 |est-qei| 的峰值。 */
static int32_t  sm_span_a;           /* SPAN：A 点编码器计数。 */
static int32_t  sm_span_b;           /* SPAN：B 点编码器计数。 */
static bool     sm_span_next_is_b;   /* SPAN：下次打点记 B 而非 A。 */
static bool     sm_span_has_a;
static bool     sm_span_has_b;

static float SmAbs(float v){
    return (v < 0.0f) ? -v : v;
}

/* 编码器计数换算成角度用的比例；COUNTS_PER_REV 未标定时这个刻度本身就是错的。 */
static float SmCountsToDeg(int32_t counts){
    if (STEP_MOTOR_ENCODER_COUNTS_PER_REV <= 0.0f){
        return 0.0f;
    }
    return ((float)counts * 360.0f) / STEP_MOTOR_ENCODER_COUNTS_PER_REV;
}

/* 开环角与实测角之差。持续发散即丢步，这是所有模式共用的健康指标。 */
static float SmSlip(void){
    return StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM) -
           StepMotor_GetMeasuredPosition();
}

/* 由转速换算步进频率，与驱动内部同式；SWEEP 直接把这个数报给操作者抄回头文件。 */
static uint32_t SmSpeedToFreq(float speed_deg_s){
    if (STEP_MOTOR_STEP_ANGLE_DEG <= 0.0f){
        return 0U;
    }
    return (uint32_t)((SmAbs(speed_deg_s) / STEP_MOTOR_STEP_ANGLE_DEG) *
                      STEP_MOTOR_MICROSTEP);
}

/* "前缀 + 定点数" 一行，省去每处重复的 PutStr/AppFmt 两步。 */
static void SmLineF(char *buf, const char *prefix, float value, uint8_t decimals){
    uint8_t n = PutStr(buf, prefix);
    AppFmt_Fixed(&buf[n], value, decimals);
}

static void SmLineI(char *buf, const char *prefix, int32_t value){
    uint8_t n = PutStr(buf, prefix);
    AppFmt_I32(&buf[n], value);
}

/* 纯文本行。PutStr 按约定不写结尾 '\0'，这里补上。 */
static void SmLineS(char *buf, const char *text){
    uint8_t n = PutStr(buf, text);
    buf[n] = '\0';
}

/*
 * [SM] 遥测。字段含义：
 *   m    当前标定模式序号
 *   t    ms 时间戳
 *   cmd  指令速度 deg/s
 *   f    驱动实际应用的步进频率 Hz（0=未出脉冲，落进死区或已停）
 *   load 定时器装载值，实际脉冲频率 = STEP_MOTOR_STEP_TIMER_CLK_HZ/(load+1)
 *   est  开环积分角 deg
 *   qei  编码器实测角 deg
 *   cnt  编码器累计计数
 *   raw  QEI 硬件 16 位原始计数
 *   en   驱动器使能
 *   dir  QEI 硬件方向标志
 */
static void ChkStepMotor_Telemetry(uint32_t now){
    DebugUart_Printf(
        "[SM] m=%u t=%lu cmd=%.2f f=%lu load=%lu est=%.3f qei=%.3f cnt=%ld raw=%u en=%u dir=%u\r\n",
        (unsigned)sm_mode,
        (unsigned long)now,
        (double)StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_BEAM),
        (unsigned long)StepMotor_GetStepFrequencyHz(STEP_MOTOR_CHANNEL_BEAM),
        (unsigned long)StepMotor_GetTimerLoad(STEP_MOTOR_CHANNEL_BEAM),
        (double)StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM),
        (double)StepMotor_GetMeasuredPosition(),
        (long)StepMotor_GetEncoderCount(),
        (unsigned)StepMotor_GetEncoderRaw(),
        (unsigned)(StepMotor_IsEnabled(STEP_MOTOR_CHANNEL_BEAM) ? 1U : 0U),
        (unsigned)(StepMotor_IsEncoderCountingUp() ? 1U : 0U));
}

static void ChkStepMotor_Render(const char *action){
    char l1[18];
    char l2[18];
    char l3[18];
    char l4[18];
    const char *hint = "";
    int32_t cnt = StepMotor_GetEncoderCount();

    switch (sm_mode){
    case SM_MODE_TURN:
        SmLineI(l1, "N ", (int32_t)sm_turn_target);
        SmLineF(l2, "est ", StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM), 0U);
        SmLineI(l3, "cnt ", cnt);
        /* 每转计数的交叉验证值：MICROSTEP 已经对了，这个数就该等于 CPR。 */
        SmLineI(l4, "c/N ", (sm_turn_target > 0U) ? (cnt / (int32_t)sm_turn_target) : 0);
        hint = "UP/DN N  ENT go";
        break;

    case SM_MODE_HAND:
        SmLineS(l1, StepMotor_IsEnabled(STEP_MOTOR_CHANNEL_BEAM)
                        ? "!! still ON !!" : "drv OFF turnable");
        SmLineI(l2, "cnt ", cnt);
        SmLineI(l3, "turns ", (int32_t)sm_hand_turns);
        /* 这一行就是 STEP_MOTOR_ENCODER_COUNTS_PER_REV 的实测值。 */
        SmLineI(l4, "cpr ", (sm_hand_turns > 0U)
                                ? ((cnt < 0 ? -cnt : cnt) / (int32_t)sm_hand_turns) : 0);
        hint = "UP/DN turns ENT 0";
        break;

    case SM_MODE_SWEEP:
        SmLineF(l1, "spd ", sm_sweep_speed[sm_sweep_idx], 0U);
        SmLineI(l2, "f ", (int32_t)SmSpeedToFreq(sm_sweep_speed[sm_sweep_idx]));
        SmLineF(l3, "slip ", SmSlip(), 1U);
        /* 峰值比瞬时值可靠：丢步是累积的，人眼盯瞬时数容易漏掉。 */
        SmLineF(l4, "peak ", sm_slip_peak, 1U);
        hint = "UP/DN spd ENT run";
        break;

    case SM_MODE_SPAN:
        if (sm_span_has_a){ SmLineF(l1, "A ", SmCountsToDeg(sm_span_a), 1U); }
        else              { SmLineS(l1, "A --"); }
        if (sm_span_has_b){ SmLineF(l2, "B ", SmCountsToDeg(sm_span_b), 1U); }
        else              { SmLineS(l2, "B --"); }
        if (sm_span_has_a && sm_span_has_b){
            SmLineF(l3, "d ", SmCountsToDeg(sm_span_b - sm_span_a), 1U);
        } else {
            SmLineS(l3, "d --");
        }
        SmLineF(l4, "qei ", StepMotor_GetMeasuredPosition(), 1U);
        hint = "hold UP/DN  ENT mark";
        break;

    case SM_MODE_RUN:
    default:
        SmLineF(l1, "est ", StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM), 1U);
        SmLineF(l2, "qei ", StepMotor_GetMeasuredPosition(), 1U);
        /* est 与 qei 同号且 slip 不发散，才说明方向与刻度都对。 */
        SmLineF(l3, "slip ", SmSlip(), 1U);
        SmLineI(l4, "cnt ", cnt);
        hint = "UP/DN run ENT pause";
        break;
    }

    Ui_RenderLines(sm_mode_tag[sm_mode], action, l1, l2, l3, l4, hint);
}

/* 切模式先停机；HAND 需要断电才能手转轴，离开时必须还回保持力矩。 */
static void ChkStepMotor_SetMode(SM_MODE mode){
    (void)StepMotor_Stop(STEP_MOTOR_CHANNEL_BEAM);
    sm_running = false;

    sm_mode = mode;
    (void)StepMotor_SetEnabled(STEP_MOTOR_CHANNEL_BEAM, mode != SM_MODE_HAND);

    if (mode == SM_MODE_HAND || mode == SM_MODE_TURN){
        StepMotor_ResetEncoder();
        StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM);
        /* 编码器零点变了，SPAN 之前打的点就不再可比，一并作废免得读到错的 d。 */
        sm_span_has_a     = false;
        sm_span_has_b     = false;
        sm_span_next_is_b = false;
    }
    if (mode == SM_MODE_SWEEP){
        sm_slip_peak = 0.0f;
    }
}

static void ChkStepMotor_Enter(void){
    (void)StepMotor_Init();
    sm_turn_target    = 1U;
    sm_hand_turns     = 1U;
    sm_sweep_idx      = 0U;
    sm_slip_peak      = 0.0f;
    sm_span_a         = 0;
    sm_span_b         = 0;
    sm_span_has_a     = false;
    sm_span_has_b     = false;
    sm_span_next_is_b = false;
    sm_last_ui        = 0U;
    sm_last_telemetry = 0U;

    /* 把固件里的换算常量原样打一行，便于与驱动器拨码、SysConfig 分频当场对账。 */
    DebugUart_Printf("[SM] --- enter, clk=%lu min_f=%u max_f=%u ustep=%d cpr=%d ---\r\n",
                     (unsigned long)STEP_MOTOR_STEP_TIMER_CLK_HZ,
                     (unsigned)STEP_MOTOR_MIN_STEP_FREQ_HZ,
                     (unsigned)STEP_MOTOR_MAX_STEP_FREQ_HZ,
                     (int)STEP_MOTOR_MICROSTEP,
                     (int)STEP_MOTOR_ENCODER_COUNTS_PER_REV);
    ChkStepMotor_SetMode(SM_MODE_RUN);
    ChkStepMotor_Render("idle");
}

/* 各模式的 UP/DOWN/ENTER 短按语义；返回要显示的动作名，NULL 表示本拍无操作。 */
static const char *ChkStepMotor_HandleKeys(KEY_EVENT ev_up,
                                           KEY_EVENT ev_dn,
                                           KEY_EVENT ev_en){
    switch (sm_mode){
    case SM_MODE_TURN:
        if (ev_up == KEY_EVENT_SHORT_PRESS && sm_turn_target < SM_TURN_MAX){
            sm_turn_target++;
            return "N+";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS && sm_turn_target > 1U){
            sm_turn_target--;
            return "N-";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            if (sm_running){
                (void)StepMotor_Stop(STEP_MOTOR_CHANNEL_BEAM);
                sm_running = false;
                return "ABORT";
            }
            StepMotor_ResetEncoder();
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM);
            (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, SM_TURN_SPEED_DEG_S);
            sm_running = true;
            return "GO";
        }
        break;

    case SM_MODE_HAND:
        /* 圈数由操作者自己数，软件只负责除法——手转不产生任何可信的圈数信号。 */
        if (ev_up == KEY_EVENT_SHORT_PRESS && sm_hand_turns < SM_HAND_MAX){
            sm_hand_turns++;
            return "turns+";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS && sm_hand_turns > 1U){
            sm_hand_turns--;
            return "turns-";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            StepMotor_ResetEncoder();
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM);
            sm_hand_turns = 1U;
            return "zero";
        }
        break;

    case SM_MODE_SWEEP:
        if (ev_up == KEY_EVENT_SHORT_PRESS && sm_sweep_idx + 1U < SM_SWEEP_STEP_COUNT){
            sm_sweep_idx++;
            sm_slip_peak = 0.0f;
            if (sm_running){
                (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, sm_sweep_speed[sm_sweep_idx]);
            }
            return "spd+";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS && sm_sweep_idx > 0U){
            sm_sweep_idx--;
            sm_slip_peak = 0.0f;
            if (sm_running){
                (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, sm_sweep_speed[sm_sweep_idx]);
            }
            return "spd-";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            if (sm_running){
                (void)StepMotor_Stop(STEP_MOTOR_CHANNEL_BEAM);
                sm_running = false;
                return "STOP";
            }
            /* 每档从零起测，否则上一档的丢步会记进本档峰值。 */
            StepMotor_ResetEncoder();
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM);
            sm_slip_peak = 0.0f;
            (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, sm_sweep_speed[sm_sweep_idx]);
            sm_running = true;
            return "RUN";
        }
        break;

    case SM_MODE_SPAN:
        /* UP/DOWN 在本模式是"按住点动"，由 Tick 直接读电平，这里不消费其短按。 */
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            int32_t cnt = StepMotor_GetEncoderCount();
            bool marked_b = sm_span_next_is_b;
            if (marked_b){
                sm_span_b = cnt;
                sm_span_has_b = true;
            } else {
                sm_span_a = cnt;
                sm_span_has_a = true;
            }
            sm_span_next_is_b = !marked_b;
            return marked_b ? "mark B" : "mark A";
        }
        break;

    case SM_MODE_RUN:
    default:
        if (ev_up == KEY_EVENT_SHORT_PRESS){
            (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, SM_RUN_SPEED_DEG_S);
            sm_running = true;
            return "RUN FWD";
        }
        if (ev_dn == KEY_EVENT_SHORT_PRESS){
            (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, -SM_RUN_SPEED_DEG_S);
            sm_running = true;
            return "RUN REV";
        }
        if (ev_en == KEY_EVENT_SHORT_PRESS){
            /*
             * 暂停：只停脉冲，保持通电，est/cnt 一律留着。停下来正是要比对这一段跑完的
             * est 与 cnt，当场清零等于把要看的数据抹掉；再按 UP/DOWN 可接着累积。
             * 清零挪到退出整页时做。
             */
            (void)StepMotor_Stop(STEP_MOTOR_CHANNEL_BEAM);
            sm_running = false;
            return "PAUSE";
        }
        break;
    }
    return NULL;
}

static APP_TASK_STATUS ChkStepMotor_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 持续推进开环积分并采样 QEI（后者必须高频调用以免 16 位计数环绕丢圈）。 */
    (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_BEAM, now);

    /* 三个键每拍都取，避免切模式后残留事件在新模式里意外触发。 */
    KEY_EVENT ev_up = Key_GetEvent(KEY_ID_UP);
    KEY_EVENT ev_dn = Key_GetEvent(KEY_ID_DOWN);
    KEY_EVENT ev_en = Key_GetEvent(KEY_ID_ENTER);

    const char *action;

    if (ev_en == KEY_EVENT_LONG_PRESS){
        ChkStepMotor_SetMode((SM_MODE)((sm_mode + 1U) % (uint8_t)SM_MODE_MAX));
        /* 用模式名当动作名：遥测里能看出切到了哪一模式，屏上也确认了这次长按被识别。 */
        action = sm_mode_tag[sm_mode];
    } else {
        action = ChkStepMotor_HandleKeys(ev_up, ev_dn, ev_en);
    }

    /* SPAN 点动：按住走、松开停。读稳定电平，与上面的事件消费互不干扰。 */
    if (sm_mode == SM_MODE_SPAN){
        float jog = 0.0f;
        if (Key_IsPressed(KEY_ID_UP))        { jog =  SM_JOG_SPEED_DEG_S; }
        else if (Key_IsPressed(KEY_ID_DOWN)) { jog = -SM_JOG_SPEED_DEG_S; }
        if (jog != StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_BEAM)){
            (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, jog);
            sm_running = (jog != 0.0f);
        }
    }

    /* TURN：开环走满 N 圈自动停，让"实际转了几圈"成为可判读的量。 */
    if ((sm_mode == SM_MODE_TURN) && sm_running){
        float target = 360.0f * (float)sm_turn_target;
        if (SmAbs(StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM)) >= target){
            (void)StepMotor_Stop(STEP_MOTOR_CHANNEL_BEAM);
            sm_running = false;
            action = "DONE";
        }
    }

    /* SWEEP：跟踪本档 |slip| 峰值，丢步是累积的，盯瞬时值容易漏。 */
    if ((sm_mode == SM_MODE_SWEEP) && sm_running){
        float slip = SmAbs(SmSlip());
        if (slip > sm_slip_peak){
            sm_slip_peak = slip;
        }
    }

    if (action != NULL){
        DebugUart_Printf("[SM] --- %s ---\r\n", action);
        ChkStepMotor_Render(action);
        sm_last_ui = now;
    }

    /* 停止时也节流刷新，便于 HAND 模式手动转轴时观察计数。 */
    if ((now - sm_last_ui) >= CHK_UI_PERIOD_MS){
        sm_last_ui = now;
        ChkStepMotor_Render(sm_running ? "running" : "idle");
    }

    /* 运动期间高频输出遥测；停止时低频输出，便于手动转轴时观察编码器。 */
    uint32_t telemetry_period = sm_running ? SM_TELEMETRY_PERIOD_MS : CHK_UI_PERIOD_MS;
    if ((now - sm_last_telemetry) >= telemetry_period){
        sm_last_telemetry = now;
        ChkStepMotor_Telemetry(now);
    }

    return APP_TASK_RUNNING;
}

static void ChkStepMotor_Exit(void){
    /* 只停脉冲并恢复使能——摆杆需要保持力矩，退出菜单不应让它落下去。 */
    (void)StepMotor_StopAll();
    (void)StepMotor_SetEnabled(STEP_MOTOR_CHANNEL_BEAM, true);

    /*
     * 位置与计数在这里统一清零，页内的暂停不清——这样一次进出就是一段干净的量程，
     * 下次进来不带上次的累计值。
     *
     * ⚠ 软限位标定完(见 docs/step-motor-calibration.md #10)之后要重新考虑这里:
     *   限位是对开环 est 判的,退出时清零等于让驱动忘记摆杆的真实所在,
     *   限位就失去意义,届时应改为只清编码器、或退出前先回机械零位。
     */
    StepMotor_ResetEncoder();
    StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM);
    sm_running = false;
}

/* ============================ 编码器 ============================ */

static uint32_t enc_last_ui;

static void ChkEnc_Enter(void){
    HallEncoder_Reset();
    enc_last_ui = 0U;
}

static APP_TASK_STATUS ChkEnc_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - enc_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    enc_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    n = PutStr(l1, "L cnt ");
    AppFmt_I32(&l1[n], HallEncoder_GetCount(HALL_ENCODER_LEFT));
    n = PutStr(l2, "L spd ");
    AppFmt_Fixed(&l2[n], HallEncoder_GetSpeed(HALL_ENCODER_LEFT), 2);
    n = PutStr(l3, "R cnt ");
    AppFmt_I32(&l3[n], HallEncoder_GetCount(HALL_ENCODER_RIGHT));
    n = PutStr(l4, "R spd ");
    AppFmt_Fixed(&l4[n], HallEncoder_GetSpeed(HALL_ENCODER_RIGHT), 2);

    n = PutStr(l5, "dir L");
    n += PutStr(&l5[n], (HallEncoder_GetDir(HALL_ENCODER_LEFT) == HALL_ENCODER_DIR_FORWARD)
                            ? "F" : "R");
    n += PutStr(&l5[n], " R");
    n += PutStr(&l5[n], (HallEncoder_GetDir(HALL_ENCODER_RIGHT) == HALL_ENCODER_DIR_FORWARD)
                            ? "F" : "R");
    l5[n] = '\0';

    Ui_RenderLines("Chk Encoder", l1, l2, l3, l4, l5, "BACK: exit");
    return APP_TASK_RUNNING;
}

/* ============================ 速度闭环 ============================ */

#define SPD_STEP 0.05f   /* 每次按键调整的目标速度步进, m/s */
#define SPD_MAX  3.0f   /* 目标速度上下限, m/s */

static float spd_target;
static uint32_t spd_last_ui;

static void ChkSpeedPid_Enter(void){
    HallEncoder_Reset();
    spd_target = 0.0f;
    Chassis_SetSpeed(0.0f);   /* 进入速度闭环, 目标 0。 */
    spd_last_ui = 0U;
}

static APP_TASK_STATUS ChkSpeedPid_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 按键调目标: UP +step, DOWN -step, ENTER 归零。 */
    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        spd_target += SPD_STEP;
        if (spd_target > SPD_MAX){ spd_target = SPD_MAX; }
    } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        spd_target -= SPD_STEP;
        if (spd_target < -SPD_MAX){ spd_target = -SPD_MAX; }
    } else if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        spd_target = 0.0f;
    }
    /* 每拍设目标; 速度环由 App_ControlTick 在本 tick 之后驱动出力。 */
    Chassis_SetSpeed(spd_target);

    /*
     * 遥测: 每控制拍(20ms/50Hz)输出一行, 供上位机
     * tools/visualizers/speed_pid_viz.py 绘图整定。
     * 字段: t 设备 ms; tl/tr 目标轮速; l/r 实测轮速(m/s); dl/dr 应用占空比(%,为上一拍值)。
     * 非阻塞(环形缓冲+TX 中断), 对控制环零阻塞。
     */
    CHASSIS_DUTY duty = Chassis_GetDuty();
    DebugUart_Printf("[SPD] t=%lu tl=%.3f tr=%.3f l=%.3f r=%.3f dl=%.1f dr=%.1f\r\n",
        (unsigned long)now,
        (double)Chassis_GetWheelSpeedTarget(HALL_ENCODER_LEFT),
        (double)Chassis_GetWheelSpeedTarget(HALL_ENCODER_RIGHT),
        (double)HallEncoder_GetSpeed(HALL_ENCODER_LEFT),
        (double)HallEncoder_GetSpeed(HALL_ENCODER_RIGHT),
        (double)duty.left_percent, (double)duty.right_percent);

    if ((now - spd_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    spd_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    uint8_t n;

    n = PutStr(l1, "tgt ");
    AppFmt_Fixed(&l1[n], spd_target, 2);
    n = PutStr(l2, "L spd ");
    AppFmt_Fixed(&l2[n], HallEncoder_GetSpeed(HALL_ENCODER_LEFT), 2);
    n = PutStr(l3, "R spd ");
    AppFmt_Fixed(&l3[n], HallEncoder_GetSpeed(HALL_ENCODER_RIGHT), 2);

    Ui_RenderLines("Chk Speed PID", "!! WHEELS UP !!", l1, l2, l3,
                   "UP/DN/EN adj", "BACK: exit");
    return APP_TASK_RUNNING;
}

/* ======================= 占空比-速度 扫描(开环诊断)======================= */

#define DSW_STEP 2.0f    /* 每次按键调整的占空比步进, % */
#define DSW_MAX  80.0f   /* 占空比上下限, %(安全起见不到满) */

static float dsw_duty;
static uint32_t dsw_last_ui;

static void ChkDutySweep_Enter(void){
    HallEncoder_Reset();
    dsw_duty = 0.0f;
    (void)Chassis_SetDuty(0.0f, 0.0f);   /* 开环, 双轮同占空比, 不走速度环。 */
    dsw_last_ui = 0U;
}

static APP_TASK_STATUS ChkDutySweep_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 按键调占空比: UP +step, DOWN -step, ENTER 归零。 */
    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        dsw_duty += DSW_STEP;
        if (dsw_duty > DSW_MAX){ dsw_duty = DSW_MAX; }
    } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        dsw_duty -= DSW_STEP;
        if (dsw_duty < -DSW_MAX){ dsw_duty = -DSW_MAX; }
    } else if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        dsw_duty = 0.0f;
    }
    (void)Chassis_SetDuty(dsw_duty, dsw_duty);   /* 开环固定占空比。 */

    /*
     * 遥测: 复用 [SPD] 行格式(tl/tr=0 无目标, l/r=实测轮速, dl/dr=应用占空比),
     * 直接用 tools/visualizers/speed_pid_viz.py 看"占空比阶梯 vs 实测速度",
     * 定位死区与噪声拐点。
     */
    DebugUart_Printf("[SPD] t=%lu tl=0.000 tr=0.000 l=%.3f r=%.3f dl=%.1f dr=%.1f\r\n",
        (unsigned long)now,
        (double)HallEncoder_GetSpeed(HALL_ENCODER_LEFT),
        (double)HallEncoder_GetSpeed(HALL_ENCODER_RIGHT),
        (double)dsw_duty, (double)dsw_duty);

    if ((now - dsw_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    dsw_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    uint8_t n;

    n = PutStr(l1, "duty ");
    AppFmt_Fixed(&l1[n], dsw_duty, 1);
    n = PutStr(l2, "L spd ");
    AppFmt_Fixed(&l2[n], HallEncoder_GetSpeed(HALL_ENCODER_LEFT), 2);
    n = PutStr(l3, "R spd ");
    AppFmt_Fixed(&l3[n], HallEncoder_GetSpeed(HALL_ENCODER_RIGHT), 2);

    Ui_RenderLines("Chk Duty Sweep", "!! WHEELS UP !!", l1, l2, l3,
                   "UP/DN/EN adj", "BACK: exit");
    return APP_TASK_RUNNING;
}

/* ============================ 描述符 ============================ */

const APP_TASK_DESC APP_CHK_GYRO_JY61P = {
    "Gyro JY61P", ChkGyroJy_Enter, ChkGyroJy_Tick, NULL
};
const APP_TASK_DESC APP_CHK_GRAY_I2C = {
    "Gray I2C", ChkGrayI2c_Enter, ChkGrayI2c_Tick, ChkGrayI2c_Exit
};
const APP_TASK_DESC APP_CHK_TB6612 = {
    "TB6612", ChkTb_Enter, ChkTb_Tick, ChkTb_Exit
};
const APP_TASK_DESC APP_CHK_STEP_MOTOR = {
    "Step Motor", ChkStepMotor_Enter, ChkStepMotor_Tick, ChkStepMotor_Exit
};
const APP_TASK_DESC APP_CHK_ENCODER = {
    "Encoder", ChkEnc_Enter, ChkEnc_Tick, NULL
};
const APP_TASK_DESC APP_CHK_SPEED_PID = {
    "Speed PID", ChkSpeedPid_Enter, ChkSpeedPid_Tick, NULL
};
const APP_TASK_DESC APP_CHK_DUTY_SWEEP = {
    "Duty Sweep", ChkDutySweep_Enter, ChkDutySweep_Tick, NULL
};

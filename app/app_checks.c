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
#include "ganv_gray.h"
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

/* ============================ 灰度 I2C（感为，I2C0） ============================ */
/* 与 JY61P/MPU6050 共 I2C0：进入时挂起 JY61P、退出时恢复。 */

static uint32_t gi_last_ui;
static uint32_t gi_ok_cnt;    /* 累计读取成功次数。 */
static uint32_t gi_err_cnt;   /* 累计读取失败次数。 */

static void ChkGrayI2c_Enter(void){
    JY61P_I2C_SetSuspended(true);    /* 让出 I2C0 */
    (void)GanvGray_Init();           /* 上电 ping 同步；同时清零驱动内部诊断计数 */
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
    BSP_STATUS st = GanvGray_ReadDigital(&mask);
    if (st == BSP_STATUS_OK){ gi_ok_cnt++; } else { gi_err_cnt++; }

    char bits[GANV_GRAY_CHANNEL_COUNT + 1U];
    uint8_t active = 0U;
    for (uint8_t i = 0U; i < GANV_GRAY_CHANNEL_COUNT; i++){
        bool on = ((mask & (uint8_t)(1U << i)) != 0U);   /* bit0=第1路 */
        bits[i] = on ? '1' : '0';
        if (on){
            active++;
        }
    }
    bits[GANV_GRAY_CHANNEL_COUNT] = '\0';

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

    /* 诊断：W=写命令阶段累计失败 R=读数据阶段累计失败 s=最近失败码(超时=-4 / NACK=-1)。 */
    uint32_t wr_fail = 0U;
    uint32_t rd_fail = 0U;
    int32_t  last_status = 0;
    GanvGray_GetDiag(&wr_fail, &rd_fail, &last_status);
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

/* ======================= 摆杆步进电机 + QEI（主动） ======================= */
/*
 * 持续旋转模式。驱动器上电即使能并**保持通电**，电机始终有保持力矩。
 *   UP        持续正转
 *   DOWN      持续反转
 *   ENTER 短按 停止脉冲 + 清零计数（仍通电，停在原地）
 *   ENTER 长按 切换驱动器使能——断电后轴可用手转动，用于标定编码器每转计数
 *   BACK      退出（Exit 只停脉冲，不断电）
 *
 * 用持续旋转而非短脉冲，是为了让「电机到底转不转」一眼可辨：转起来后轴上的标记
 * 会明显转动，编码器 cnt 也会单调累积；若只是原地振动，cnt 仍会在零附近来回。
 *
 * ⚠ 持续旋转不会自动停止，摆杆已装机时注意行程——当前软限位已放到 ±100000°
 *   等效于不限位。随时按 ENTER 或 BACK 停止。
 *
 * 标定用法：长按 ENTER 断电 → 短按 ENTER 清零 → 手动精确转一整圈 → 读 cnt 变化量，
 * 即 STEP_MOTOR_ENCODER_COUNTS_PER_REV 的真实值。
 */
#define SM_RUN_SPEED_DEG_S 30.0f

/* 遥测周期：20ms 以便看清起停瞬态与编码器跟随情况。 */
#define SM_TELEMETRY_PERIOD_MS 20U

static uint32_t sm_last_ui;
static uint32_t sm_last_telemetry;
static bool     sm_running;

/*
 * [SM] 遥测。字段含义：
 *   t    ms 时间戳
 *   cmd  指令速度 deg/s
 *   f    驱动实际应用的步进频率 Hz（0=未出脉冲，落进死区或已停）
 *   load 定时器装载值，实际脉冲频率 = 1000000/(load+1)
 *   est  开环积分角 deg
 *   qei  编码器实测角 deg
 *   cnt  编码器累计计数
 *   raw  QEI 硬件 16 位原始计数
 *   en   驱动器使能
 *   dir  QEI 硬件方向标志
 */
static void ChkStepMotor_Telemetry(uint32_t now){
    DebugUart_Printf(
        "[SM] t=%lu cmd=%.2f f=%lu load=%lu est=%.3f qei=%.3f cnt=%ld raw=%u en=%u dir=%u\r\n",
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
    char est[20];
    char meas[20];
    char raw[20];
    uint8_t n;

    n = PutStr(est, "est ");
    AppFmt_Fixed(&est[n], StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM), 1U);
    n = PutStr(meas, "qei ");
    AppFmt_Fixed(&meas[n], StepMotor_GetMeasuredPosition(), 1U);
    n = PutStr(raw, "cnt ");
    AppFmt_I32(&raw[n], StepMotor_GetEncoderCount());

    Ui_RenderLines("Chk StepMotor", action, est, meas, raw,
                   StepMotor_IsEnabled(STEP_MOTOR_CHANNEL_BEAM) ? "drv ON" : "drv OFF (turnable)",
                   "UP/DN run EN stop/hold");
}

static void ChkStepMotor_Enter(void){
    (void)StepMotor_Init();
    sm_running = false;
    sm_last_ui = 0U;
    sm_last_telemetry = 0U;
    DebugUart_Printf("[SM] --- enter, clk=%lu min_f=%u max_f=%u ustep=%d ---\r\n",
                     (unsigned long)STEP_MOTOR_STEP_TIMER_CLK_HZ,
                     (unsigned)STEP_MOTOR_MIN_STEP_FREQ_HZ,
                     (unsigned)STEP_MOTOR_MAX_STEP_FREQ_HZ,
                     (int)STEP_MOTOR_MICROSTEP);
    ChkStepMotor_Render("idle");
}

static APP_TASK_STATUS ChkStepMotor_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    /* 持续推进开环积分并采样 QEI（后者必须高频调用以免 16 位计数环绕丢圈）。 */
    (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_BEAM, now);

    const char *action = NULL;

    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, SM_RUN_SPEED_DEG_S);
        sm_running = true;
        action = "RUN FWD";
    } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        (void)StepMotor_SetSpeed(STEP_MOTOR_CHANNEL_BEAM, -SM_RUN_SPEED_DEG_S);
        sm_running = true;
        action = "RUN REV";
    } else {
        KEY_EVENT ev = Key_GetEvent(KEY_ID_ENTER);
        if (ev == KEY_EVENT_SHORT_PRESS){
            /* 停脉冲但保持通电，摆杆靠保持力矩停在原地。 */
            (void)StepMotor_Stop(STEP_MOTOR_CHANNEL_BEAM);
            StepMotor_ResetEncoder();
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_BEAM);
            sm_running = false;
            action = "STOP + zero";
        } else if (ev == KEY_EVENT_LONG_PRESS){
            /* 长按切换使能：断电后电机轴可手动转动，用于标定编码器每转计数。 */
            (void)StepMotor_Stop(STEP_MOTOR_CHANNEL_BEAM);
            sm_running = false;
            bool on = !StepMotor_IsEnabled(STEP_MOTOR_CHANNEL_BEAM);
            (void)StepMotor_SetEnabled(STEP_MOTOR_CHANNEL_BEAM, on);
            action = on ? "DRV ON" : "DRV OFF (turnable)";
        }
    }

    if (action != NULL){
        DebugUart_Printf("[SM] --- %s ---\r\n", action);
        ChkStepMotor_Render(action);
        sm_last_ui = now;
    }

    /* 停止时也节流刷新，便于手动转轴标定时观察计数。 */
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
    /* 只停脉冲，保持使能——摆杆需要保持力矩，退出菜单不应让它落下去。 */
    (void)StepMotor_StopAll();
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

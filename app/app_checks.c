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
#include "grayscale_sensor.h"
#include "ganv_gray.h"
#include "yahboom_track.h"
#include "hall_encoder.h"
#include "wit_sdk.h"
#include "mpu6050.h"
#include "straight_drive.h"
#include "kinematics/kinematics.h"
#include "yaw_estimator.h"

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

/* ============================ 灰度 ============================ */

static uint32_t gs_last_ui;

static void ChkGrayscale_Enter(void){
    gs_last_ui = 0U;
}

static APP_TASK_STATUS ChkGrayscale_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - gs_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    gs_last_ui = now;

    uint8_t mask = GrayscaleSensor_ReadMask();
    char bits[GRAYSCALE_SENSOR_CHANNEL_COUNT + 1U];
    uint8_t one_count = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++){
        bool on = ((mask & (uint8_t)(1U << i)) != 0U);
        bits[i] = on ? '1' : '0';
        if (on){
            one_count++;
        }
    }
    bits[GRAYSCALE_SENSOR_CHANNEL_COUNT] = '\0';

    char l2[16];
    uint8_t n = PutStr(l2, "ones ");
    AppFmt_I32(&l2[n], (int32_t)one_count);

    Ui_RenderLines("Chk Grayscale", bits, l2, "BACK: exit",
                   NULL, NULL, NULL);
    return APP_TASK_RUNNING;
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

/* ============================ Yahboom 循线 I2C（I2C0） ============================ */
/* 与 JY61P/MPU6050/感为灰度共 I2C0：进入时挂起 JY61P、退出时恢复。 */

static uint32_t yb_last_ui;
static uint32_t yb_ok_cnt;
static uint32_t yb_err_cnt;
static uint8_t yb_raw;
static uint8_t yb_mask;

static char HexDigit(uint8_t value){
    value &= 0x0FU;
    return (value < 10U) ? (char)('0' + value) : (char)('A' + value - 10U);
}

static void ChkYahboomI2c_Enter(void){
    JY61P_I2C_SetSuspended(true);    /* 让出 I2C0 */
    (void)YahboomTrack_Init();       /* 探测 0x12，同时清零驱动诊断 */
    yb_last_ui = 0U;
    yb_ok_cnt  = 0U;
    yb_err_cnt = 0U;
    yb_raw     = 0xFFU;
    yb_mask    = 0U;
}

static APP_TASK_STATUS ChkYahboomI2c_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - yb_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    yb_last_ui = now;

    uint8_t raw = 0xFFU;
    BSP_STATUS st = YahboomTrack_ReadRaw(&raw);
    if (st == BSP_STATUS_OK){
        yb_raw = raw;
        yb_mask = 0U;
        for (uint8_t i = 0U; i < YAHBOOM_TRACK_CHANNEL_COUNT; i++){
            if ((raw & (uint8_t)(1U << (7U - i))) == 0U){
                yb_mask |= (uint8_t)(1U << i);   /* bit0=X1，1=黑线 */
            }
        }
        yb_ok_cnt++;
    } else{
        yb_err_cnt++;
    }

    char bits[YAHBOOM_TRACK_CHANNEL_COUNT + 1U];
    uint8_t active = 0U;
    for (uint8_t i = 0U; i < YAHBOOM_TRACK_CHANNEL_COUNT; i++){
        bool detected = ((yb_mask & (uint8_t)(1U << i)) != 0U);
        bits[i] = detected ? '1' : '0';
        if (detected){ active++; }
    }
    bits[YAHBOOM_TRACK_CHANNEL_COUNT] = '\0';

    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    if (st == BSP_STATUS_OK){
        n = PutStr(l3, "raw 0x");
        l3[n++] = HexDigit((uint8_t)(yb_raw >> 4));
        l3[n++] = HexDigit(yb_raw);
        n += PutStr(&l3[n], " act ");
        AppFmt_I32(&l3[n], (int32_t)active);
    } else{
        n = PutStr(l3, "READ FAIL");
        l3[n] = '\0';
    }

    n = PutStr(l4, "ok ");
    AppFmt_I32(&l4[n], (int32_t)yb_ok_cnt);
    while (l4[n] != '\0'){ n++; }
    n += PutStr(&l4[n], " er ");
    AppFmt_I32(&l4[n], (int32_t)yb_err_cnt);

    uint32_t read_fail = 0U;
    uint32_t write_fail = 0U;
    int32_t last_status = 0;
    YahboomTrack_GetDiag(&read_fail, &write_fail, &last_status);
    n = PutStr(l5, "R");
    AppFmt_I32(&l5[n], (int32_t)read_fail);
    while (l5[n] != '\0'){ n++; }
    n += PutStr(&l5[n], " W");
    AppFmt_I32(&l5[n], (int32_t)write_fail);
    while (l5[n] != '\0'){ n++; }
    n += PutStr(&l5[n], " s");
    AppFmt_I32(&l5[n], last_status);

    Ui_RenderLines("Chk Yahboom I2C", bits, "X1->X8 1=BLACK", l3, l4, l5,
                   "BACK: exit");
    return APP_TASK_RUNNING;
}

static void ChkYahboomI2c_Exit(void){
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

/* ======================== 航向 A/B 对比（无电机） ======================== */

#define YAW_AB_IMU_MAX_AGE_MS 60U

static uint32_t yab_last_ui_ms;
static uint32_t yab_last_sample_ms;
static uint32_t yab_last_sample_count;
static float yab_fused_deg;
static float yab_gyro_z_deg_s;
static bool yab_valid;
static YAW_ESTIMATOR yab_estimator;

static void ChkYawAb_Enter(void){
    /* 本任务不调用任何 Chassis/Motor 接口，只观察共享 I2C0 上的 JY61P。 */
    JY61P_I2C_SetSuspended(false);
    JY61P_I2C_Init();
    yab_last_ui_ms = 0U;
    yab_last_sample_ms = 0U;
    yab_last_sample_count = 0U;
    YawEstimator_Reset(&yab_estimator);
    yab_fused_deg = 0.0f;
    yab_gyro_z_deg_s = 0.0f;
    yab_valid = false;
}

static APP_TASK_STATUS ChkYawAb_Tick(float dt){
    (void)dt;
    JY61P_I2C_Poll();

    uint32_t now_ms = BSP_Time_GetMs();
    JY61P_I2C_SAMPLE sample;

    if (JY61P_I2C_GetSnapshot(&sample) &&
        (sample.sample_count != yab_last_sample_count)){
        yab_last_sample_count = sample.sample_count;

        if (JY61P_I2C_IsDataFresh(YAW_AB_IMU_MAX_AGE_MS)){
            float fused_deg = Kinematics_NormalizeAngleDeg(
                STRAIGHT_DRIVE_HEADING_YAW_SIGN *
                sample.data.attitude_deg.yaw);
            float gyro_z_deg_s =
                STRAIGHT_DRIVE_RATE_GYRO_SIGN * sample.data.gyro_deg_s.z;

            if (!yab_valid){
                /* A0 = B0：首次完整样本只负责建立公共角度原点。 */
                YawEstimator_Start(&yab_estimator, fused_deg);
                yab_valid = true;
            } else{
                float sample_dt_s =
                    (float)(sample.timestamp_ms - yab_last_sample_ms) * 0.001f;
                YawEstimator_Integrate(&yab_estimator, gyro_z_deg_s,
                                       sample_dt_s);
            }

            yab_last_sample_ms = sample.timestamp_ms;
            yab_fused_deg = fused_deg;
            yab_gyro_z_deg_s = gyro_z_deg_s;

            float delta_ba_deg = YawEstimator_GetFusionOffset(
                &yab_estimator, yab_fused_deg);
            DebugUart_Printf(
                "[YAB] t=%lu n=%lu a=%.3f b=%.3f ba=%.3f gz=%.3f\r\n",
                (unsigned long)now_ms,
                (unsigned long)sample.sample_count,
                (double)YawEstimator_GetIntegrated(&yab_estimator),
                (double)yab_fused_deg,
                (double)delta_ba_deg,
                (double)yab_gyro_z_deg_s);
        } else{
            /* 样本链中断后无法补积分；下一帧重新令 A0=B0。 */
            yab_valid = false;
        }
    } else if (!JY61P_I2C_IsDataFresh(YAW_AB_IMU_MAX_AGE_MS)){
        yab_valid = false;
    }

    if ((now_ms - yab_last_ui_ms) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    yab_last_ui_ms = now_ms;

    if (!yab_valid){
        Ui_RenderLines("Yaw A/B No Motor", "IMU waiting", "motor untouched",
                       "BACK: exit", NULL, NULL, NULL);
        return APP_TASK_RUNNING;
    }

    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    uint8_t n;

    n = PutStr(l1, "A ");
    AppFmt_Fixed(&l1[n], YawEstimator_GetIntegrated(&yab_estimator), 2U);
    n = PutStr(l2, "B "); AppFmt_Fixed(&l2[n], yab_fused_deg, 2U);
    n = PutStr(l3, "B-A ");
    AppFmt_Fixed(&l3[n], YawEstimator_GetFusionOffset(
        &yab_estimator, yab_fused_deg), 2U);
    n = PutStr(l4, "gz "); AppFmt_Fixed(&l4[n], yab_gyro_z_deg_s, 2U);
    n = PutStr(l5, "sample ");
    AppFmt_I32(&l5[n], (int32_t)yab_last_sample_count);

    Ui_RenderLines("Yaw A/B No Motor", l1, l2, l3, l4, l5, "BACK: exit");
    return APP_TASK_RUNNING;
}

/* ============================ 陀螺仪 MPU6050 ============================ */
/* 基础模式物理量检查；与 JY61P 共 I2C0，进入时挂起、退出时恢复。 */

static uint32_t gm_last_ui;
static uint8_t gm_page;
static BSP_STATUS gm_init_status;

static void ChkGyroMpu_Enter(void){
    JY61P_I2C_SetSuspended(true);    /* 让出 I2C0 */
    gm_init_status = MPU6050_Init();
    gm_last_ui = 0U;
    gm_page = 0U;
}

static APP_TASK_STATUS ChkGyroMpu_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();

    if ((Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS) ||
        (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS)){
        gm_page ^= 1U;
        gm_last_ui = 0U;
    }

    if ((now - gm_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    gm_last_ui = now;

    char l1[20];
    char l2[20];
    char l3[20];
    char l4[20];
    char l5[20];
    char l6[20];
    uint8_t n;

    bool connected = (gm_init_status == BSP_STATUS_OK) &&
                     MPU6050_TestConnection();
    MPU6050_MEASUREMENT m;
    if (!connected || (MPU6050_GetMeasurement(&m) != BSP_STATUS_OK)){
        n = PutStr(l1, connected ? "read FAIL" : "conn FAIL");
        l1[n] = '\0';
        Ui_RenderLines("Chk MPU6050", l1, "check wiring/bus", "BACK: exit",
                       NULL, NULL, NULL);
        return APP_TASK_RUNNING;
    }

    if (gm_page == 0U){
        (void)PutStr(l1, "unit m/s2"); l1[10] = '\0';
        n = PutStr(l2, "Ax "); AppFmt_Fixed(&l2[n], m.accel_x_mps2, 2U);
        n = PutStr(l3, "Ay "); AppFmt_Fixed(&l3[n], m.accel_y_mps2, 2U);
        n = PutStr(l4, "Az "); AppFmt_Fixed(&l4[n], m.accel_z_mps2, 2U);
        n = PutStr(l5, "|a| "); AppFmt_Fixed(&l5[n], m.accel_magnitude_mps2, 2U);
        n = PutStr(l6, "Temp C "); AppFmt_Fixed(&l6[n], m.temperature_c, 1U);
        Ui_RenderLines("MPU Acc 1/2", l1, l2, l3, l4, l5, l6);
    } else{
        (void)PutStr(l1, "unit deg/s"); l1[10] = '\0';
        n = PutStr(l2, "Gx "); AppFmt_Fixed(&l2[n], m.gyro_x_deg_s, 1U);
        n = PutStr(l3, "Gy "); AppFmt_Fixed(&l3[n], m.gyro_y_deg_s, 1U);
        n = PutStr(l4, "Gz "); AppFmt_Fixed(&l4[n], m.gyro_z_deg_s, 1U);
        n = PutStr(l5, "Pitch "); AppFmt_Fixed(&l5[n], m.pitch_deg, 1U);
        n = PutStr(l6, "Roll "); AppFmt_Fixed(&l6[n], m.roll_deg, 1U);
        Ui_RenderLines("MPU Gyro 2/2", l1, l2, l3, l4, l5, l6);
    }
    return APP_TASK_RUNNING;
}

static void ChkGyroMpu_Exit(void){
    JY61P_I2C_SetSuspended(false);   /* 归还 I2C0 给 JY61P */
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
     * 遥测: 每控制拍(20ms/50Hz)输出一行, 供上位机 tools/speed_pid_viz.py 绘图整定。
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
     * 直接用 tools/speed_pid_viz.py 看"占空比阶梯 vs 实测速度", 定位死区与噪声拐点。
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
const APP_TASK_DESC APP_CHK_YAW_AB = {
    "Yaw A/B", ChkYawAb_Enter, ChkYawAb_Tick, NULL
};
const APP_TASK_DESC APP_CHK_GYRO_MPU6050 = {
    "Gyro MPU6050", ChkGyroMpu_Enter, ChkGyroMpu_Tick, ChkGyroMpu_Exit
};
const APP_TASK_DESC APP_CHK_GRAYSCALE = {
    "Grayscale", ChkGrayscale_Enter, ChkGrayscale_Tick, NULL
};
const APP_TASK_DESC APP_CHK_GRAY_I2C = {
    "Gray I2C", ChkGrayI2c_Enter, ChkGrayI2c_Tick, ChkGrayI2c_Exit
};
const APP_TASK_DESC APP_CHK_YAHBOOM_I2C = {
    "Yahboom I2C", ChkYahboomI2c_Enter, ChkYahboomI2c_Tick, ChkYahboomI2c_Exit
};
const APP_TASK_DESC APP_CHK_TB6612 = {
    "TB6612", ChkTb_Enter, ChkTb_Tick, ChkTb_Exit
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

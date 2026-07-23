/**
 * @file  app_checks.c
 * @brief 外设自检任务实现：JY61P / MPU6050 / 灰度 / TB6612 / 编码器。
 */
#include "app_checks.h"
#include "app_fmt.h"

#include "ui.h"
#include "key.h"
#include "bsp_time.h"
#include "bsp_common.h"
#include "chassis.h"
#include "grayscale_sensor.h"
#include "ganv_gray.h"
#include "hall_encoder.h"
#include "wit_sdk.h"
#include "mpu6050.h"

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
    uint8_t active = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++){
        bool on = ((mask & (uint8_t)(1U << i)) != 0U);
        bits[i] = on ? '1' : '0';
        if (on){
            active++;
        }
    }
    bits[GRAYSCALE_SENSOR_CHANNEL_COUNT] = '\0';

    char l2[16];
    uint8_t n = PutStr(l2, "active ");
    AppFmt_I32(&l2[n], (int32_t)active);

    Ui_RenderLines("Chk Grayscale", bits, l2, "BACK: exit",
                   NULL, NULL, NULL);
    return APP_TASK_RUNNING;
}

/* ============================ 灰度 I2C（感为，I2C0） ============================ */
/* 与 JY61P/MPU6050 共 I2C0：进入时挂起 JY61P、退出时恢复。 */

static uint32_t gi_last_ui;
static uint8_t  gi_version;
static bool     gi_version_ok;

static void ChkGrayI2c_Enter(void){
    JY61P_I2C_SetSuspended(true);    /* 让出 I2C0 */
    (void)GanvGray_Init();           /* 上电 ping 同步；失败也进入，Tick 实时反映在线状态 */
    uint8_t v = 0U;
    gi_version_ok = (GanvGray_ReadVersion(&v) == BSP_STATUS_OK);
    gi_version    = gi_version_ok ? v : 0U;
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
    uint8_t n;

    if (st == BSP_STATUS_OK){
        n = PutStr(l2, "online act ");
        AppFmt_I32(&l2[n], (int32_t)active);
    } else {
        (void)PutStr(l2, "OFFLINE");
        l2[7] = '\0';
    }

    if (gi_version_ok){
        /* 版本字节 = 高 4bit.低 4bit，如 0x3E → v3.14。 */
        n = PutStr(l3, "ver ");
        AppFmt_I32(&l3[n], (int32_t)(gi_version >> 4));
        while (l3[n] != '\0'){ n++; }
        n += PutStr(&l3[n], ".");
        AppFmt_I32(&l3[n], (int32_t)(gi_version & 0x0FU));
    } else {
        (void)PutStr(l3, "ver --");
        l3[6] = '\0';
    }

    Ui_RenderLines("Chk Gray I2C", bits, l2, l3, "BACK: exit", NULL, NULL);
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

/* ============================ 陀螺仪 MPU6050 ============================ */
/* 与 JY61P 共 I2C0：进入时挂起 JY61P、退出时恢复。 */

static uint32_t gm_last_ui;

static void ChkGyroMpu_Enter(void){
    JY61P_I2C_SetSuspended(true);    /* 让出 I2C0 */
    (void)MPU6050_Init();
    gm_last_ui = 0U;
}

static APP_TASK_STATUS ChkGyroMpu_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - gm_last_ui) < CHK_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    gm_last_ui = now;

    char l1[20];
    char l2[20];
    uint8_t n;

    bool ok = MPU6050_TestConnection();
    (void)PutStr(l1, ok ? "conn OK" : "conn FAIL");
    l1[ok ? 7U : 9U] = '\0';

    MPU6050_MOTION6 m;
    if (ok && (MPU6050_GetMotion6(&m) == BSP_STATUS_OK)){
        n = PutStr(l2, "gz ");
        AppFmt_I32(&l2[n], (int32_t)m.gz);
    } else {
        (void)PutStr(l2, "gz --"); l2[5] = '\0';
    }

    Ui_RenderLines("Chk MPU6050", l1, l2, "BACK: exit", NULL, NULL, NULL);
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
    char cnt[16];
    uint8_t n = PutStr(cnt, "enc ");
    AppFmt_I32(&cnt[n], HallEncoder_GetCount());
    Ui_RenderLines("Chk TB6612", "!! WHEELS UP !!", action, cnt,
                   "UP/DN/EN pulse", "BACK: exit", NULL);
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
    uint8_t n;

    n = PutStr(l1, "cnt ");
    AppFmt_I32(&l1[n], HallEncoder_GetCount());
    n = PutStr(l2, "spd ");
    AppFmt_Fixed(&l2[n], HallEncoder_GetSpeed(), 2);
    n = PutStr(l3, "dst ");
    AppFmt_Fixed(&l3[n], HallEncoder_GetDistance(), 2);

    const char *dir = (HallEncoder_GetDir() == HALL_ENCODER_DIR_FORWARD)
                          ? "dir FWD" : "dir REV";

    Ui_RenderLines("Chk Encoder", l1, l2, l3, dir, "BACK: exit", NULL);
    return APP_TASK_RUNNING;
}

/* ============================ 描述符 ============================ */

const APP_TASK_DESC APP_CHK_GYRO_JY61P = {
    "Gyro JY61P", ChkGyroJy_Enter, ChkGyroJy_Tick, NULL
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
const APP_TASK_DESC APP_CHK_TB6612 = {
    "TB6612", ChkTb_Enter, ChkTb_Tick, ChkTb_Exit
};
const APP_TASK_DESC APP_CHK_ENCODER = {
    "Encoder", ChkEnc_Enter, ChkEnc_Tick, NULL
};

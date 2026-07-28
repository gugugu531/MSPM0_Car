/**
 * @file  app_line_task.c
 * @brief 循迹测试任务实现。
 *
 * Yahboom 任务由 line_follow / line_guided_drive 完成闭环；K230 红线任务由
 * vision_line_drive 完成 UART 解析和连续视觉控制。Yahboom 与 JY61P 共用 I2C0，前两项任务
 * 只在 JY61P 异步事务空闲时切换总线所有权。
 */
#include "app_line_task.h"

#include "line_follow.h"
#include "line_guided_drive.h"
#include "middleware/vision_line_drive/vision_line_drive.h"
#include "chassis.h"
#include "wit_sdk.h"

#include "ui.h"
#include "app_fmt.h"
#include "bsp_time.h"
#include "bsp_common.h"
#include "yahboom_track.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* 刷屏节流周期。 */
#define LT_UI_PERIOD_MS 150U
/** Yahboom 最近一次成功读数的最大允许年龄。 */
#define LT_LINE_SENSOR_MAX_AGE_MS 60U

static uint32_t lt_last_ui;
static uint8_t lt_detected_mask;
static uint32_t lt_detected_mask_ms;
static bool lt_sensor_ready;
static BSP_STATUS lt_sensor_init_status;

/*
 * Yahboom 是阻塞驱动，JY61P 是中断状态机。app 层负责共享 I2C0 的时间片：
 * 只在 JY61P 完全空闲时挂起它、完成一次 Yahboom 读取，再恢复并启动下一帧 IMU。
 */
static void LineSensor_Enter(void){
    JY61P_I2C_SetSuspended(true);
    lt_sensor_init_status = YahboomTrack_Init();
    JY61P_I2C_Init();

    lt_detected_mask = 0U;
    lt_detected_mask_ms = 0U;
    lt_sensor_ready = false;
}

static bool LineSensor_Tick(uint8_t *detected_mask){
    uint32_t now = BSP_Time_GetMs();

    if ((lt_sensor_init_status == BSP_STATUS_OK) && JY61P_I2C_IsIdle()){
        uint8_t latest_mask;
        JY61P_I2C_SetSuspended(true);
        BSP_STATUS status = YahboomTrack_ReadDetectedMask(&latest_mask);
        JY61P_I2C_SetSuspended(false);
        if (status == BSP_STATUS_OK){
            lt_detected_mask = latest_mask;
            lt_detected_mask_ms = now;
            lt_sensor_ready = true;
        }
    }

    /* 读取 Yahboom 后才 kick JY61P；控制器本拍消费上一帧完整 IMU 快照。 */
    JY61P_I2C_Poll();
    *detected_mask = lt_detected_mask;
    return lt_sensor_ready &&
           ((uint32_t)(now - lt_detected_mask_ms) <= LT_LINE_SENSOR_MAX_AGE_MS);
}

/* 把字符串拷入 buf, 返回长度(不终止), 便于随后接 AppFmt_* 拼数字。 */
static uint8_t LtPutStr(char *buf, const char *s){
    uint8_t i = 0U;
    while (s[i] != '\0'){
        buf[i] = s[i];
        i++;
    }
    return i;
}

static void LineFollowTest_Enter(void){
    LineFollow_Init(NULL);            /* 默认配置（含陀螺增稳），并复位控制状态。 */
    LineSensor_Enter();
    lt_last_ui = 0U;
}

static APP_TASK_STATUS LineFollowTest_Tick(float dt){
    uint8_t detected_mask;
    bool sensor_ready = LineSensor_Tick(&detected_mask);

    /* Yahboom 丢线、未初始化或读数超时都刹停，测试任务不自行搜索。 */
    BSP_STATUS st = sensor_ready
                        ? LineFollow_UpdateDetectedMask(detected_mask, dt)
                        : BSP_STATUS_NOT_READY;
    if (st != BSP_STATUS_OK){
        (void)Chassis_Brake();        /* 丢线/异常: 刹停(测试期安全, 不自行搜索)。 */
    }

    uint32_t now = BSP_Time_GetMs();
    if ((now - lt_last_ui) < LT_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    lt_last_ui = now;

    LINE_FOLLOW_OUTPUT out = LineFollow_GetOutput();

    /* Yahboom 归一化语义：X1→X8 从左到右显示，1 表示检测到黑线。 */
    char bits[LINE_FOLLOW_SENSOR_COUNT + 1U];
    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        bits[i] = ((detected_mask & (1U << i)) != 0U) ? '1' : '0';
    }
    bits[LINE_FOLLOW_SENSOR_COUNT] = '\0';

    char l2[20];
    char l3[20];
    char l4[20];
    uint8_t n;

    n = LtPutStr(l2, "err ");
    AppFmt_Fixed(&l2[n], out.error, 1);
    n = LtPutStr(l3, "dL ");
    AppFmt_Fixed(&l3[n], out.left_duty, 0);
    n = LtPutStr(l4, "dR ");
    AppFmt_Fixed(&l4[n], out.right_duty, 0);

    const char *status = out.line_lost ? "LINE LOST" : "track";

    Ui_RenderLines("Line Follow", bits, l2, l3, l4, status, "BACK: exit");
    return APP_TASK_RUNNING;
}

const APP_TASK_DESC APP_LINE_FOLLOW_TEST = {
    "Line Follow", LineFollowTest_Enter, LineFollowTest_Tick, NULL
};

/* ====================== 80% 灰度外环 + 航向内环 ====================== */

static void LineGuidedTest_Enter(void){
    LineSensor_Enter();
    LineGuidedDrive_Init();
    lt_last_ui = 0U;
}

static const char *LineGuided_PhaseText(LINE_GUIDED_PHASE phase){
    switch (phase){
        case LINE_GUIDED_PHASE_WAIT_IMU:      return "WAIT IMU";
        case LINE_GUIDED_PHASE_HEADING_HOLD:  return "YAW HOLD";
        case LINE_GUIDED_PHASE_LINE_PID:      return "LINE PID";
        default:                              return "UNKNOWN";
    }
}

static APP_TASK_STATUS LineGuidedTest_Tick(float dt){
    uint8_t detected_mask;
    bool sensor_ready = LineSensor_Tick(&detected_mask);
    if (LineGuidedDrive_Update(detected_mask, sensor_ready, dt) !=
        BSP_STATUS_OK){
        return APP_TASK_FAULT;
    }

    uint32_t now = BSP_Time_GetMs();
    if ((now - lt_last_ui) < LT_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    lt_last_ui = now;

    LINE_GUIDED_OUTPUT out = LineGuidedDrive_GetOutput();
    char bits[LINE_FOLLOW_SENSOR_COUNT + 1U];
    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        bits[i] = ((out.black_mask & (1U << i)) != 0U) ? '1' : '0';
    }
    bits[LINE_FOLLOW_SENSOR_COUNT] = '\0';

    char l2[20];
    char l3[28];
    char l4[20];
    char l5[20];
    uint8_t n;
    n = LtPutStr(l2, "err ");
    AppFmt_Fixed(&l2[n], out.line_error, 1U);
    n = LtPutStr(l3, "yaw/ref ");
    AppFmt_Fixed(&l3[n], out.corrected_yaw_deg, 1U);
    while (l3[n] != '\0'){ n++; }
    l3[n++] = '/';
    AppFmt_Fixed(&l3[n], out.heading_reference_deg, 1U);
    n = LtPutStr(l4, "corr ");
    AppFmt_Fixed(&l4[n], out.correction_percent, 1U);
    n = LtPutStr(l5, "d ");
    AppFmt_Fixed(&l5[n], out.left_duty_percent, 0U);
    while (l5[n] != '\0'){ n++; }
    l5[n++] = '/';
    AppFmt_Fixed(&l5[n], out.right_duty_percent, 0U);

    Ui_RenderLines("Line Guided 80", bits, l2, l3, l4, l5,
                   LineGuided_PhaseText(out.phase));
    return APP_TASK_RUNNING;
}

const APP_TASK_DESC APP_LINE_GUIDED_TEST = {
    "Line Guided 80", LineGuidedTest_Enter, LineGuidedTest_Tick, NULL
};

/* ========================= K230 红线视觉循迹 ========================= */

static void VisionLineTest_Enter(void){
    JY61P_I2C_Init();
    VisionLineDrive_Init();
    lt_last_ui = 0U;
}

static const char *VisionLine_PhaseText(VISION_LINE_PHASE phase){
    switch (phase){
        case VISION_LINE_PHASE_WAIT_IMU:     return "WAIT IMU";
        case VISION_LINE_PHASE_STARTUP_RATE: return "START RATE";
        case VISION_LINE_PHASE_TRACK:        return "VISION TRACK";
        default:                             return "UNKNOWN";
    }
}

static APP_TASK_STATUS VisionLineTest_Tick(float dt){
    JY61P_I2C_Poll();
    if (VisionLineDrive_Update(dt) != BSP_STATUS_OK){
        return APP_TASK_FAULT;
    }

    uint32_t now = BSP_Time_GetMs();
    if ((now - lt_last_ui) < LT_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    lt_last_ui = now;

    VISION_LINE_OUTPUT out = VisionLineDrive_GetOutput();
    char l1[24];
    char l2[28];
    char l3[28];
    char l4[28];
    char l5[24];
    uint8_t n;

    n = LtPutStr(l1, out.track_valid && out.frame_fresh ? "VERT RED OK" : "VERT RED LOST");
    l1[n] = '\0';

    n = LtPutStr(l2, "p/h ");
    AppFmt_Fixed(&l2[n], out.position_error, 3U);
    while (l2[n] != '\0'){ n++; }
    l2[n++] = '/';
    AppFmt_Fixed(&l2[n], out.heading_error_deg, 1U);

    n = LtPutStr(l3, "om/gz ");
    AppFmt_Fixed(&l3[n], out.omega_reference_deg_s, 1U);
    while (l3[n] != '\0'){ n++; }
    l3[n++] = '/';
    AppFmt_Fixed(&l3[n], out.gyro_z_deg_s, 1U);

    n = LtPutStr(l4, "q/age ");
    AppFmt_I32(&l4[n], (int32_t)out.confidence);
    while (l4[n] != '\0'){ n++; }
    l4[n++] = '/';
    AppFmt_I32(&l4[n], out.frame_fresh ? (int32_t)out.frame_age_ms : -1);

    n = LtPutStr(l5, "d ");
    AppFmt_Fixed(&l5[n], out.left_duty_percent, 0U);
    while (l5[n] != '\0'){ n++; }
    l5[n++] = '/';
    AppFmt_Fixed(&l5[n], out.right_duty_percent, 0U);

    Ui_RenderLines("Vision Red", l1, l2, l3, l4, l5,
                   VisionLine_PhaseText(out.phase));
    return APP_TASK_RUNNING;
}

static void VisionLineTest_Exit(void){
    VisionLineDrive_Stop();
}

const APP_TASK_DESC APP_VISION_LINE_TEST = {
    "Vision Red", VisionLineTest_Enter, VisionLineTest_Tick, VisionLineTest_Exit
};

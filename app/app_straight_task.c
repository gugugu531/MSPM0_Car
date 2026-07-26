/**
 * @file  app_straight_task.c
 * @brief 直行控制器的菜单任务、显示与调试遥测适配。
 */
#include "app_straight_task.h"

#include "app_fmt.h"
#include "bsp_time.h"
#include "debug_uart.h"
#include "key.h"
#include "straight_drive.h"
#include "ui.h"
#include "wit_sdk.h"

#include <stddef.h>
#include <stdint.h>

#define STRAIGHT_UI_PERIOD_MS 150U

static uint32_t s_last_ui_ms;

static uint8_t Straight_PutStr(char *buf, const char *text)
{
    uint8_t i = 0U;
    while (text[i] != '\0'){
        buf[i] = text[i];
        i++;
    }
    return i;
}

static uint8_t Straight_AppendStr(char *buf, uint8_t offset, const char *text)
{
    return (uint8_t)(offset + Straight_PutStr(&buf[offset], text));
}

static bool Straight_UsesGyro(STRAIGHT_DRIVE_MODE mode)
{
    return (mode == STRAIGHT_DRIVE_MODE_GYRO_RATE) ||
           (mode == STRAIGHT_DRIVE_MODE_GYRO_HEADING) ||
           (mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING) ||
           (mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) ||
           (mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) ||
           (mode == STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING) ||
           (mode == STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING);
}

static float Straight_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 以左右轮绝对里程的平均值估算本次测试已行驶的车体距离。 */
static float Straight_GetTravelDistance(const STRAIGHT_DRIVE_OUTPUT *output)
{
    return 0.5f * (Straight_Abs(output->distance_left_m) +
                   Straight_Abs(output->distance_right_m));
}

static const char *Straight_Title(STRAIGHT_DRIVE_MODE mode)
{
    switch (mode){
        case STRAIGHT_DRIVE_MODE_DUTY_OPEN:    return "Straight Duty";
        case STRAIGHT_DRIVE_MODE_SPEED:        return "Speed Closed";
        case STRAIGHT_DRIVE_MODE_GYRO_RATE:    return "Duty+Gyro Rate";
        case STRAIGHT_DRIVE_MODE_GYRO_HEADING: return "Duty+Yaw Hold";
        case STRAIGHT_DRIVE_MODE_RAMP_HEADING:
            return "Ramp Yaw Hold";
        case STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING:
            return "80 Rate->Yaw";
        case STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING:
            return "80 Enc->Yaw";
        case STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING:
            return "80 Int->Yaw";
        case STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING:
            return "100 Int->Yaw";
        default:                               return "Straight";
    }
}

static void Straight_Enter(STRAIGHT_DRIVE_MODE mode)
{
    s_last_ui_ms = 0U;

    /* app 拥有共享 I2C0 外设的调度，控制器只消费 BSP 已发布的数据。 */
    JY61P_I2C_SetSuspended(false);
    JY61P_I2C_Init();
    StraightDrive_Init(mode);
}

static void StraightDuty_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_DUTY_OPEN);
}

static void StraightSpeed_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_SPEED);
}

static void StraightRate_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_GYRO_RATE);
}

static void StraightHeading_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_GYRO_HEADING);
}

static void StraightRampHeading_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_RAMP_HEADING);
}

static void StraightRateThenHeading_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING);
}

static void StraightEncoderThenHeading_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING);
}

static void StraightIntegratedThenHeading_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING);
}

static void StraightFullIntegratedThenHeading_Enter(void)
{
    Straight_Enter(STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING);
}

static void Straight_HandleKeys(void)
{
    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        StraightDrive_AdjustCommand(1);
    } else if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        StraightDrive_AdjustCommand(-1);
    } else if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        StraightDrive_ZeroCommand();
    }
}

static void Straight_Render(const STRAIGHT_DRIVE_OUTPUT *output)
{
    char l0[24];
    char l1[24];
    char l2[24];
    char l3[24];
    char l4[24];
    uint8_t n;

    n = Straight_PutStr(l0,
        (output->mode == STRAIGHT_DRIVE_MODE_SPEED) ? "tgt " : "duty ");
    AppFmt_Fixed(&l0[n], output->applied_command,
        (output->mode == STRAIGHT_DRIVE_MODE_SPEED) ? 2U : 1U);
    while (l0[n] != '\0'){ n++; }
    if (output->mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING){
        n = Straight_AppendStr(l0, n, ">");
        AppFmt_Fixed(&l0[n], output->command, 0U);
        while (l0[n] != '\0'){ n++; }
    }
    n = Straight_AppendStr(l0, n, " x");
    AppFmt_Fixed(&l0[n], Straight_GetTravelDistance(output), 2U);
    while (l0[n] != '\0'){ n++; }
    n = Straight_AppendStr(l0, n, "/");
    AppFmt_Fixed(&l0[n], STRAIGHT_TEST_TARGET_DISTANCE_M, 1U);

    n = Straight_PutStr(l1, "dL ");
    AppFmt_Fixed(&l1[n], output->duty_left_percent, 0U);
    while (l1[n] != '\0'){ n++; }
    n = Straight_AppendStr(l1, n, " dR ");
    AppFmt_Fixed(&l1[n], output->duty_right_percent, 0U);

    n = Straight_PutStr(l2, "vL ");
    AppFmt_Fixed(&l2[n], output->speed_left_mps, 2U);
    while (l2[n] != '\0'){ n++; }
    n = Straight_AppendStr(l2, n, " vR ");
    AppFmt_Fixed(&l2[n], output->speed_right_mps, 2U);

    if (Straight_UsesGyro(output->mode)){
        if (!output->imu_ready){
            n = Straight_PutStr(l3, "IMU waiting");
            l3[n] = '\0';
            n = Straight_PutStr(l4, "motor held zero");
            l4[n] = '\0';
        } else if ((output->mode == STRAIGHT_DRIVE_MODE_GYRO_RATE) ||
                   ((output->mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) &&
                    !output->startup_complete)){
            n = Straight_PutStr(l3,
                (output->mode == STRAIGHT_DRIVE_MODE_GYRO_RATE)
                    ? "gz "
                    : "RATE gz ");
            AppFmt_Fixed(&l3[n], output->gyro_z_deg_s, 1U);
            n = Straight_PutStr(l4, "corr ");
            AppFmt_Fixed(&l4[n], output->correction_percent, 1U);
        } else if ((output->mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) &&
                   !output->startup_complete){
            n = Straight_PutStr(l3, "ENC dx ");
            AppFmt_Fixed(&l3[n],
                output->distance_left_m - output->distance_right_m, 3U);
            n = Straight_PutStr(l4, "corr ");
            AppFmt_Fixed(&l4[n], output->correction_percent, 1U);
        } else if (((output->mode == STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING) ||
                    (output->mode == STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING)) &&
                   !output->startup_complete){
            n = Straight_PutStr(l3, "INT yaw ");
            AppFmt_Fixed(&l3[n], output->integrated_heading_deg, 1U);
            n = Straight_PutStr(l4, "corr ");
            AppFmt_Fixed(&l4[n], output->correction_percent, 1U);
        } else{
            n = Straight_PutStr(l3,
                (output->mode != STRAIGHT_DRIVE_MODE_RAMP_HEADING)
                    ? ((output->mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING)
                           ? "CRUISE yaw "
                           : ((output->mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING)
                                  ? "CRUISE yaw "
                                  : (((output->mode == STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING) ||
                                      (output->mode == STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING))
                                         ? "CRUISE yaw "
                                         : "yaw ")))
                    : (output->startup_complete ? "CRUISE yaw " : "RAMP yaw "));
            AppFmt_Fixed(&l3[n], output->yaw_deg, 1U);
            n = Straight_PutStr(l4, "ref ");
            if (output->heading_reference_valid){
                AppFmt_Fixed(&l4[n], output->heading_reference_deg, 1U);
            } else{
                n = Straight_AppendStr(l4, n, "--");
                l4[n] = '\0';
            }
        }
    } else{
        n = Straight_PutStr(l3, "EN: stop");
        l3[n] = '\0';
        n = Straight_PutStr(l4, "UP/DN: adjust");
        l4[n] = '\0';
    }

    Ui_RenderLines(Straight_Title(output->mode), l0, l1, l2, l3, l4,
                   "UP/DN EN0 BACK");
}

static void Straight_SendTelemetry(uint32_t now_ms,
                                   const STRAIGHT_DRIVE_OUTPUT *output)
{
#if STRAIGHT_TELEMETRY_ENABLED
    /* Speed Closed 的 duty 是上一控制拍已应用的速度 PID 输出。 */
    DebugUart_Printf(
        "[STR] t=%lu m=%u imu=%u cmd=%.3f act=%.3f phase=%u dl=%.1f dr=%.1f "
        "vl=%.3f vr=%.3f xl=%.4f xr=%.4f yaw=%.2f gz=%.2f iyaw=%.2f corr=%.2f\r\n",
        (unsigned long)now_ms,
        (unsigned int)output->mode,
        output->imu_ready ? 1U : 0U,
        (double)output->command,
        (double)output->applied_command,
        output->startup_complete ? 1U : 0U,
        (double)output->duty_left_percent,
        (double)output->duty_right_percent,
        (double)output->speed_left_mps,
        (double)output->speed_right_mps,
        (double)output->distance_left_m,
        (double)output->distance_right_m,
        (double)output->yaw_deg,
        (double)output->gyro_z_deg_s,
        (double)output->integrated_heading_deg,
        (double)output->correction_percent);
#else
    (void)now_ms;
    (void)output;
#endif
}

static APP_TASK_STATUS Straight_Tick(float dt_s)
{
    JY61P_I2C_Poll();
    Straight_HandleKeys();

    if (StraightDrive_Update(dt_s) != BSP_STATUS_OK){
        return APP_TASK_FAULT;
    }

    uint32_t now_ms = BSP_Time_GetMs();
    STRAIGHT_DRIVE_OUTPUT output = StraightDrive_GetOutput();
    Straight_SendTelemetry(now_ms, &output);

    if (Straight_GetTravelDistance(&output) >=
        STRAIGHT_TEST_TARGET_DISTANCE_M){
        /* 返回 DONE 后由 app_mode 在同一控制拍统一主动刹车并退回菜单。 */
        StraightDrive_ZeroCommand();
        return APP_TASK_DONE;
    }

    if ((now_ms - s_last_ui_ms) >= STRAIGHT_UI_PERIOD_MS){
        s_last_ui_ms = now_ms;
        Straight_Render(&output);
    }
    return APP_TASK_RUNNING;
}

const APP_TASK_DESC APP_STRAIGHT_DUTY_OPEN_TEST = {
    "Duty Open", StraightDuty_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_SPEED_TEST = {
    "Speed Closed", StraightSpeed_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_GYRO_RATE_TEST = {
    "Duty+Gyro Rate", StraightRate_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_GYRO_HEADING_TEST = {
    "Duty+Yaw Hold", StraightHeading_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_RAMP_HEADING_TEST = {
    "Ramp Yaw Hold", StraightRampHeading_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_RATE_THEN_HEADING_TEST = {
    "80 Rate->Yaw", StraightRateThenHeading_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_ENCODER_THEN_HEADING_TEST = {
    "80 Enc->Yaw", StraightEncoderThenHeading_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_INTEGRATED_THEN_HEADING_TEST = {
    "80 Int->Yaw", StraightIntegratedThenHeading_Enter, Straight_Tick, NULL
};

const APP_TASK_DESC APP_STRAIGHT_FULL_INTEGRATED_THEN_HEADING_TEST = {
    "100 Int->Yaw", StraightFullIntegratedThenHeading_Enter, Straight_Tick, NULL
};

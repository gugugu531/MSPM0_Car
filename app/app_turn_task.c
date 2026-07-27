#include "app_turn_task.h"

#include "app_fmt.h"
#include "bsp_time.h"
#include "debug_uart.h"
#include "middleware/turn_drive/turn_drive.h"
#include "ui.h"
#include "wit_sdk.h"

#include <stdint.h>

#define TURN_UI_PERIOD_MS 150U

static uint32_t s_last_ui_ms;

static uint8_t Turn_Append(char *buffer, uint8_t offset, const char *text)
{
    while (*text != '\0'){
        buffer[offset] = *text;
        offset++;
        text++;
    }
    buffer[offset] = '\0';
    return offset;
}

static uint8_t Turn_AppendFixed(char *buffer, uint8_t offset,
                                float value, uint8_t decimals)
{
    AppFmt_Fixed(&buffer[offset], value, decimals);
    while (buffer[offset] != '\0'){
        offset++;
    }
    return offset;
}

static const char *Turn_PhaseName(TURN_DRIVE_PHASE phase)
{
    switch (phase){
        case TURN_DRIVE_PHASE_WAIT_IMU: return "IMU wait";
        case TURN_DRIVE_PHASE_INTEGRATED_STRAIGHT: return "INT straight";
        case TURN_DRIVE_PHASE_STRAIGHT: return "Straight 2m";
        case TURN_DRIVE_PHASE_LEFT_DECELERATE: return "Left slow";
        case TURN_DRIVE_PHASE_LEFT_TURN_BRAKED: return "Left brake";
        case TURN_DRIVE_PHASE_POST_TURN_STRAIGHT: return "Post 1m";
        case TURN_DRIVE_PHASE_COMPLETE: return "Complete";
        default: return "Turn";
    }
}

static const char *Turn_Title(TURN_DRIVE_MODE mode)
{
    return (mode == TURN_DRIVE_MODE_FULL)
        ? "Full Fwd2m L90"
        : "Fwd2m L90 +1m";
}

static void Turn_Render(const TURN_DRIVE_OUTPUT *output)
{
    char line0[24];
    char line1[24];
    char line2[24];
    char line3[24];
    char line4[24];
    uint8_t length;

    (void)Turn_Append(line0, 0U, Turn_PhaseName(output->phase));
    length = Turn_Append(line1, 0U, "x ");
    length = Turn_AppendFixed(line1, length, output->distance_m, 2U);
    (void)Turn_Append(line1, length, "/2.0");
    length = Turn_Append(line2, 0U, "yaw ");
    length = Turn_AppendFixed(line2, length, output->yaw_deg, 1U);
    length = Turn_Append(line2, length, " tgt ");
    (void)Turn_AppendFixed(line2, length, output->turn_target_deg, 0U);
    length = Turn_Append(line3, 0U, "L ");
    length = Turn_AppendFixed(line3, length, output->duty_left_percent, 0U);
    length = Turn_Append(line3, length, " R ");
    (void)Turn_AppendFixed(line3, length, output->duty_right_percent, 0U);
    length = Turn_Append(line4, 0U, "post ");
    length = Turn_AppendFixed(line4, length, output->post_turn_distance_m, 2U);
    (void)Turn_Append(line4, length, "/1.0");

    Ui_RenderLines(Turn_Title(output->mode), line0, line1, line2, line3, line4,
                   "BACK: stop");
}

static void Turn_SendTelemetry(uint32_t now_ms,
                               const TURN_DRIVE_OUTPUT *output)
{
    DebugUart_Printf(
        "[TRN] t=%lu m=%u p=%u imu=%u x=%.3f post=%.3f yaw=%.2f ref=%.2f tgt=%.2f "
        "dl=%.1f dr=%.1f red=%.1f iyaw=%.2f gz=%.2f\r\n",
        (unsigned long)now_ms,
        (unsigned int)output->mode,
        (unsigned int)output->phase,
        output->imu_ready ? 1U : 0U,
        (double)output->distance_m,
        (double)output->post_turn_distance_m,
        (double)output->yaw_deg,
        (double)output->heading_reference_deg,
        (double)output->turn_target_deg,
        (double)output->duty_left_percent,
        (double)output->duty_right_percent,
        (double)output->turn_reduction_percent,
        (double)output->integrated_heading_deg,
        (double)output->gyro_z_deg_s);
}

static void Turn_EnterMode(TURN_DRIVE_MODE mode)
{
    s_last_ui_ms = 0U;
    JY61P_I2C_SetSuspended(false);
    JY61P_I2C_Init();
    TurnDrive_Init(mode);
}

static void Turn_Enter(void)
{
    Turn_EnterMode(TURN_DRIVE_MODE_STANDARD);
}

static void TurnFull_Enter(void)
{
    Turn_EnterMode(TURN_DRIVE_MODE_FULL);
}

static APP_TASK_STATUS Turn_Tick(float dt_s)
{
    uint32_t now_ms;
    TURN_DRIVE_OUTPUT output;

    JY61P_I2C_Poll();
    if (TurnDrive_Update(dt_s) != BSP_STATUS_OK){
        return APP_TASK_FAULT;
    }

    now_ms = BSP_Time_GetMs();
    output = TurnDrive_GetOutput();
    Turn_SendTelemetry(now_ms, &output);

    if (TurnDrive_IsComplete()){
        return APP_TASK_DONE;
    }

    if ((now_ms - s_last_ui_ms) >= TURN_UI_PERIOD_MS){
        s_last_ui_ms = now_ms;
        Turn_Render(&output);
    }
    return APP_TASK_RUNNING;
}

const APP_TASK_DESC APP_TURN_FWD2M_LEFT90_POST1M_TEST = {
    "Fwd2m L90 +1m", Turn_Enter, Turn_Tick, NULL
};

const APP_TASK_DESC APP_TURN_FULL_FWD2M_LEFT90_POST1M_TEST = {
    "Full Fwd2m L90", TurnFull_Enter, Turn_Tick, NULL
};

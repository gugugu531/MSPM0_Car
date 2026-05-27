#include "app_device_check.h"

#include "bsp_time.h"
#include "chassis.h"
#include "hall_encoder.h"
#include "key.h"
#include "line_follow.h"
#include "step_motor.h"
#include "ui.h"
#include "wit_sdk.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define APP_DEVICE_IMU_FRAME_SIZE 11U
#define APP_DEVICE_LOOP_DELAY_MS 20U
#define APP_DEVICE_RENDER_PERIOD_MS 100U
#define APP_DEVICE_MOTOR_DUTY_PERCENT 30.0f
#define APP_DEVICE_STEP_SPEED_DEG_S 120.0f
#define APP_DEVICE_STEP_RUN_MS 1000U

typedef enum {
    APP_DEVICE_MODULE_MOTOR = 0,
    APP_DEVICE_MODULE_YAW,
    APP_DEVICE_MODULE_PITCH,
    APP_DEVICE_MODULE_LINE,
    APP_DEVICE_MODULE_IMU,
    APP_DEVICE_MODULE_COUNT
} APP_DEVICE_MODULE;

typedef enum {
    APP_DEVICE_MOTOR_STOP = 0,
    APP_DEVICE_MOTOR_LEFT,
    APP_DEVICE_MOTOR_RIGHT,
    APP_DEVICE_MOTOR_BOTH,
    APP_DEVICE_MOTOR_COUNT
} APP_DEVICE_MOTOR_TEST;

typedef enum {
    APP_DEVICE_IMU_ATTITUDE = 0,
    APP_DEVICE_IMU_GYRO,
    APP_DEVICE_IMU_STATUS,
    APP_DEVICE_IMU_COUNT
} APP_DEVICE_IMU_TEST;

static uint8_t s_imu_rx_buffer[APP_DEVICE_IMU_FRAME_SIZE];
static uint8_t s_imu_rx_index;
static uint32_t s_imu_frame_count;

volatile uint32_t g_app_device_imu_rx_byte_count;
volatile uint32_t g_app_device_imu_valid_frame_count;
volatile uint32_t g_app_device_imu_invalid_frame_count;
volatile uint8_t g_app_device_imu_last_type;
volatile uint8_t g_app_device_imu_last_byte;

extern volatile uint32_t g_app_debug_uart_irq_count;
extern volatile uint32_t g_app_debug_uart_rx_irq_count;
extern volatile uint32_t g_app_debug_uart_drained_byte_count;
extern volatile uint32_t g_app_debug_uart_empty_rx_irq_count;
extern volatile uint32_t g_app_debug_uart_unhandled_irq_count;
extern volatile uint8_t g_app_debug_uart_last_iidx;

static APP_DEVICE_MOTOR_TEST s_motor_test;
static APP_DEVICE_IMU_TEST s_imu_test;

static bool AppDeviceCheck_IsImuFrameValid(const uint8_t *frame){
    uint16_t sum = 0U;

    if (frame[0] != 0x55U){
        return false;
    }

    for (uint8_t i = 0U; i < 10U; i++){
        sum += frame[i];
    }

    return frame[10] == (uint8_t)(sum & 0xFFU);
}

static bool AppDeviceCheck_IsImuFrameType(uint8_t type){
    return (type == 0x51U) || (type == 0x52U) || (type == 0x53U);
}

static void AppDeviceCheck_ResyncImuFrame(void){
    for (uint8_t i = 1U; i < APP_DEVICE_IMU_FRAME_SIZE; i++){
        if (s_imu_rx_buffer[i] == 0x55U){
            uint8_t remain = (uint8_t)(APP_DEVICE_IMU_FRAME_SIZE - i);
            memmove(s_imu_rx_buffer, &s_imu_rx_buffer[i], remain);
            s_imu_rx_index = remain;
            return;
        }
    }

    s_imu_rx_index = 0U;
}

void AppDeviceCheck_ProcessImuByte(uint8_t byte){
    g_app_device_imu_rx_byte_count++;
    g_app_device_imu_last_byte = byte;

    if ((s_imu_rx_index == 0U) && (byte != 0x55U)){
        return;
    }

    s_imu_rx_buffer[s_imu_rx_index++] = byte;

    if ((s_imu_rx_index == 2U) && !AppDeviceCheck_IsImuFrameType(s_imu_rx_buffer[1])){
        AppDeviceCheck_ResyncImuFrame();
        return;
    }

    if (s_imu_rx_index < APP_DEVICE_IMU_FRAME_SIZE){
        return;
    }

    if (AppDeviceCheck_IsImuFrameValid(s_imu_rx_buffer)){
        GYROSCOPE_DATA_Decoder(s_imu_rx_buffer);
        s_imu_frame_count++;
        g_app_device_imu_valid_frame_count++;
        g_app_device_imu_last_type = s_imu_rx_buffer[1];
        s_imu_rx_index = 0U;
        return;
    }

    g_app_device_imu_invalid_frame_count++;
    AppDeviceCheck_ResyncImuFrame();
}

static const char *AppDeviceCheck_ModuleTitle(APP_DEVICE_MODULE module){
    static const char *titles[APP_DEVICE_MODULE_COUNT] = {
        "Motor",
        "Yaw Step",
        "Pitch Step",
        "Line Sensor",
        "IMU",
    };

    return (module < APP_DEVICE_MODULE_COUNT) ? titles[module] : "Device";
}

static uint8_t AppDeviceCheck_TestCount(APP_DEVICE_MODULE module){
    switch (module){
        case APP_DEVICE_MODULE_MOTOR:
            return (uint8_t)APP_DEVICE_MOTOR_COUNT;
        case APP_DEVICE_MODULE_IMU:
            return (uint8_t)APP_DEVICE_IMU_COUNT;
        case APP_DEVICE_MODULE_YAW:
        case APP_DEVICE_MODULE_PITCH:
        case APP_DEVICE_MODULE_LINE:
        default:
            return 1U;
    }
}

static const char *AppDeviceCheck_MotorTestName(APP_DEVICE_MOTOR_TEST test){
    static const char *names[APP_DEVICE_MOTOR_COUNT] = {
        "Stop",
        "Left",
        "Right",
        "Both",
    };

    return (test < APP_DEVICE_MOTOR_COUNT) ? names[test] : "Stop";
}

static void AppDeviceCheck_FormatBinary8(uint8_t value, char *out, size_t out_size){
    if (out == NULL || out_size < 9U){
        return;
    }

    for (uint8_t i = 0U; i < 8U; i++){
        out[i] = ((value & (uint8_t)(1U << i)) != 0U) ? '1' : '0';
    }
    out[8] = '\0';
}

static void AppDeviceCheck_StopOutputs(void){
    (void)Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
    (void)StepMotor_StopAll();
}

static void AppDeviceCheck_ResetMotorTest(void){
    s_motor_test = APP_DEVICE_MOTOR_STOP;
    HallEncoder_Reset();
    (void)Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
}

static void AppDeviceCheck_ApplyMotorTest(void){
    HallEncoder_Reset();

    switch (s_motor_test){
        case APP_DEVICE_MOTOR_LEFT:
            (void)Chassis_SetDuty(APP_DEVICE_MOTOR_DUTY_PERCENT, 0.0f);
            break;
        case APP_DEVICE_MOTOR_RIGHT:
            (void)Chassis_SetDuty(0.0f, APP_DEVICE_MOTOR_DUTY_PERCENT);
            break;
        case APP_DEVICE_MOTOR_BOTH:
            (void)Chassis_SetDuty(APP_DEVICE_MOTOR_DUTY_PERCENT, APP_DEVICE_MOTOR_DUTY_PERCENT);
            break;
        case APP_DEVICE_MOTOR_STOP:
        default:
            (void)Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
            break;
    }
}

static void AppDeviceCheck_RunStepPulse(STEP_MOTOR_CHANNEL channel){
    (void)StepMotor_RunFor(channel, APP_DEVICE_STEP_SPEED_DEG_S, APP_DEVICE_STEP_RUN_MS);
    (void)StepMotor_RunFor(channel, -APP_DEVICE_STEP_SPEED_DEG_S, APP_DEVICE_STEP_RUN_MS);
}

static bool AppDeviceCheck_IsStepModule(APP_DEVICE_MODULE module){
    return (module == APP_DEVICE_MODULE_YAW) || (module == APP_DEVICE_MODULE_PITCH);
}

static void AppDeviceCheck_EnterModule(APP_DEVICE_MODULE module){
    AppDeviceCheck_StopOutputs();

    switch (module){
        case APP_DEVICE_MODULE_MOTOR:
            AppDeviceCheck_ResetMotorTest();
            break;
        case APP_DEVICE_MODULE_IMU:
            s_imu_test = APP_DEVICE_IMU_ATTITUDE;
            break;
        case APP_DEVICE_MODULE_YAW:
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW);
            break;
        case APP_DEVICE_MODULE_PITCH:
            StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH);
            break;
        case APP_DEVICE_MODULE_LINE:
        default:
            break;
    }
}

static void AppDeviceCheck_AdvanceTest(APP_DEVICE_MODULE module){
    switch (module){
        case APP_DEVICE_MODULE_MOTOR:
            s_motor_test = (APP_DEVICE_MOTOR_TEST)(((uint8_t)s_motor_test + 1U) %
                (uint8_t)APP_DEVICE_MOTOR_COUNT);
            AppDeviceCheck_ApplyMotorTest();
            break;
        case APP_DEVICE_MODULE_YAW:
            AppDeviceCheck_RunStepPulse(STEP_MOTOR_CHANNEL_YAW);
            break;
        case APP_DEVICE_MODULE_PITCH:
            AppDeviceCheck_RunStepPulse(STEP_MOTOR_CHANNEL_PITCH);
            break;
        case APP_DEVICE_MODULE_IMU:
            s_imu_test = (APP_DEVICE_IMU_TEST)(((uint8_t)s_imu_test + 1U) %
                (uint8_t)APP_DEVICE_IMU_COUNT);
            break;
        case APP_DEVICE_MODULE_LINE:
        default:
            break;
    }
}

static void AppDeviceCheck_Render(APP_DEVICE_MODULE module){
    char line0[24];
    char line1[24];
    char line2[24];
    char line3[24];

    switch (module){
        case APP_DEVICE_MODULE_MOTOR:{
            CHASSIS_DUTY duty = Chassis_GetDuty();
            snprintf(line0, sizeof(line0), "Mode:%s", AppDeviceCheck_MotorTestName(s_motor_test));
            snprintf(line1, sizeof(line1), "Duty:%0.0f,%0.0f", duty.left_percent, duty.right_percent);
            snprintf(line2, sizeof(line2), "Cnt:%ld", (long)HallEncoder_GetCount());
            snprintf(line3, sizeof(line3), "Spd:%0.2f Dis:%0.2f", Chassis_GetSpeed(), Chassis_GetDistance());
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "1:mode 2:mod", NULL);
            break;
        }
        case APP_DEVICE_MODULE_YAW:{
            (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_YAW, BSP_Time_GetMs());
            snprintf(line0, sizeof(line0), "Single:start");
            snprintf(line1, sizeof(line1), "Speed:%0.0f", StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_YAW));
            snprintf(line2, sizeof(line2), "Pos:%0.1f", StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_YAW));
            snprintf(line3, sizeof(line3), "+1s then -1s");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "1:start 2:mod", NULL);
            break;
        }
        case APP_DEVICE_MODULE_PITCH:{
            (void)StepMotor_UpdateState(STEP_MOTOR_CHANNEL_PITCH, BSP_Time_GetMs());
            snprintf(line0, sizeof(line0), "Single:start");
            snprintf(line1, sizeof(line1), "Speed:%0.0f", StepMotor_GetSpeed(STEP_MOTOR_CHANNEL_PITCH));
            snprintf(line2, sizeof(line2), "Pos:%0.1f", StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL_PITCH));
            snprintf(line3, sizeof(line3), "+1s then -1s");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "1:start 2:mod", NULL);
            break;
        }
        case APP_DEVICE_MODULE_LINE:{
            char binary[9];
            (void)LineFollow_UpdateSensor();
            AppDeviceCheck_FormatBinary8(LineFollow_GetSensorMask(), binary, sizeof(binary));
            snprintf(line0, sizeof(line0), "BIN:%s", binary);
            snprintf(line1, sizeof(line1), "HEX:0x%02X", LineFollow_GetSensorMask());
            snprintf(line2, sizeof(line2), "Active:%u", LineFollow_GetActiveCount());
            snprintf(line3, sizeof(line3), "Realtime");
            Ui_RenderLines(AppDeviceCheck_ModuleTitle(module), line0, line1, line2, line3, "2:module", NULL);
            break;
        }
        case APP_DEVICE_MODULE_IMU:
        default:{
            WIT_IMU_DATA data = {0};
            (void)WitGetData(&data);
            if (s_imu_test == APP_DEVICE_IMU_GYRO){
                snprintf(line0, sizeof(line0), "Gx:%0.1f", data.gyro_deg_s.x);
                snprintf(line1, sizeof(line1), "Gy:%0.1f", data.gyro_deg_s.y);
                snprintf(line2, sizeof(line2), "Gz:%0.1f", data.gyro_deg_s.z);
                snprintf(line3, sizeof(line3), "Frm:%lu", (unsigned long)s_imu_frame_count);
                Ui_RenderLines("IMU Gyro", line0, line1, line2, line3, "1:item 2:mod", NULL);
            } else if (s_imu_test == APP_DEVICE_IMU_STATUS){
                snprintf(line0, sizeof(line0), "Rx:%lu Ok:%lu",
                         (unsigned long)g_app_device_imu_rx_byte_count,
                         (unsigned long)g_app_device_imu_valid_frame_count);
                snprintf(line1, sizeof(line1), "Bad:%lu Type:%02X",
                         (unsigned long)g_app_device_imu_invalid_frame_count,
                         g_app_device_imu_last_type);
                snprintf(line2, sizeof(line2), "Irq:%lu Byte:%lu",
                         (unsigned long)g_app_debug_uart_irq_count,
                         (unsigned long)g_app_debug_uart_drained_byte_count);
                snprintf(line3, sizeof(line3), "IIDX:%u", g_app_debug_uart_last_iidx);
                Ui_RenderLines("IMU Status", line0, line1, line2, line3, "1:item 2:mod", NULL);
            } else{
                snprintf(line0, sizeof(line0), "Roll:%0.1f", data.attitude_deg.roll);
                snprintf(line1, sizeof(line1), "Pitch:%0.1f", data.attitude_deg.pitch);
                snprintf(line2, sizeof(line2), "Yaw:%0.1f", data.attitude_deg.yaw);
                snprintf(line3, sizeof(line3), "Frm:%lu", (unsigned long)s_imu_frame_count);
                Ui_RenderLines("IMU Att", line0, line1, line2, line3, "1:item 2:mod", NULL);
            }
            break;
        }
    }
}

void AppDeviceCheck_Run(void){
    APP_DEVICE_MODULE module = APP_DEVICE_MODULE_MOTOR;
    uint32_t last_render_ms = BSP_Time_GetMs();

    Key_ClearAllEvents();
    AppDeviceCheck_EnterModule(module);
    AppDeviceCheck_Render(module);

    while (1){
        if (Key_IsDoubleClick(KEY_ID_1)){
            module = (APP_DEVICE_MODULE)(((uint8_t)module + 1U) % (uint8_t)APP_DEVICE_MODULE_COUNT);
            AppDeviceCheck_EnterModule(module);
            Key_ClearAllEvents();
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        if (Key_IsShortPress(KEY_ID_1)){
            if (!AppDeviceCheck_IsStepModule(module) && AppDeviceCheck_TestCount(module) > 0U){
                AppDeviceCheck_AdvanceTest(module);
            }
            Key_ClearAllEvents();
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        if (AppDeviceCheck_IsStepModule(module) && Key_IsShortRelease(KEY_ID_1)){
            AppDeviceCheck_AdvanceTest(module);
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            AppDeviceCheck_StopOutputs();
            return;
        }

        if ((BSP_Time_GetMs() - last_render_ms) >= APP_DEVICE_RENDER_PERIOD_MS){
            AppDeviceCheck_Render(module);
            last_render_ms = BSP_Time_GetMs();
        }

        BSP_DelayMs(APP_DEVICE_LOOP_DELAY_MS);
    }
}

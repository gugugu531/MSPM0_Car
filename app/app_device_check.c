#include "app_device_check.h"

#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "key.h"
#include "line_follow.h"
#include "ui.h"
#include "wit_sdk.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define APP_DEVICE_IMU_FRAME_SIZE 11U
#define APP_DEVICE_LOOP_DELAY_MS 20U
#define APP_DEVICE_RENDER_PERIOD_MS 200U

typedef enum {
    APP_DEVICE_PAGE_CHASSIS = 0,
    APP_DEVICE_PAGE_GIMBAL,
    APP_DEVICE_PAGE_LINE,
    APP_DEVICE_PAGE_VISION,
    APP_DEVICE_PAGE_IMU_ANGLE,
    APP_DEVICE_PAGE_IMU_GYRO,
    APP_DEVICE_PAGE_IMU_ACC,
    APP_DEVICE_PAGE_IMU_STATUS,
    APP_DEVICE_PAGE_IMU_IRQ,
    APP_DEVICE_PAGE_COUNT
} APP_DEVICE_PAGE;

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

static const char *AppDeviceCheck_CanMvStatusText(CANMV_STATUS status){
    switch (status){
        case CANMV_STATUS_OK:
            return "OK";
        case CANMV_STATUS_NOT_FOUND:
            return "MISS";
        case CANMV_STATUS_LOST:
            return "LOST";
        case CANMV_STATUS_FRAME_DROP:
            return "DROP";
        case CANMV_STATUS_INIT:
        default:
            return "INIT";
    }
}

static void AppDeviceCheck_Render(APP_DEVICE_PAGE page){
    char line0[24];
    char line1[24];
    char line2[24];
    char line3[24];

    switch (page){
        case APP_DEVICE_PAGE_CHASSIS:{
            CHASSIS_DUTY duty = Chassis_GetDuty();
            snprintf(line0, sizeof(line0), "Duty:%0.0f,%0.0f", duty.left_percent, duty.right_percent);
            snprintf(line1, sizeof(line1), "Spd:%0.2f", Chassis_GetSpeed());
            snprintf(line2, sizeof(line2), "Dis:%0.2f", Chassis_GetDistance());
            Ui_RenderLines("Check Chassis", line0, line1, line2, "Short:next", "Long:back", NULL);
            break;
        }
        case APP_DEVICE_PAGE_GIMBAL:{
            GIMBAL_ANGLE angle = Gimbal_GetAngle();
            GIMBAL_SPEED speed = Gimbal_GetSpeed();
            snprintf(line0, sizeof(line0), "Yaw:%0.1f", angle.yaw_deg);
            snprintf(line1, sizeof(line1), "Pitch:%0.1f", angle.pitch_deg);
            snprintf(line2, sizeof(line2), "Spd:%0.0f,%0.0f", speed.yaw_deg_s, speed.pitch_deg_s);
            Ui_RenderLines("Check Gimbal", line0, line1, line2, "Short:next", "Long:back", NULL);
            break;
        }
        case APP_DEVICE_PAGE_LINE:
            (void)LineFollow_Update();
            snprintf(line0, sizeof(line0), "Mask:0x%02X", LineFollow_GetSensorMask());
            snprintf(line1, sizeof(line1), "Active:%u", LineFollow_GetActiveCount());
            snprintf(line2, sizeof(line2), "Half:%u Cross:%u",
                     LineFollow_IsHalfDetected() ? 1U : 0U,
                     LineFollow_IsCrossDetected() ? 1U : 0U);
            Ui_RenderLines("Check Line", line0, line1, line2, "Short:next", "Long:back", NULL);
            break;
        case APP_DEVICE_PAGE_VISION:{
            const CANMV_TARGET_DATA *laser = CanMvUart_GetTargetData(CANMV_TARGET_LASER);
            const CANMV_TARGET_DATA *rect = CanMvUart_GetTargetData(CANMV_TARGET_RECT);
            snprintf(line0, sizeof(line0), "Laser:%s", AppDeviceCheck_CanMvStatusText(CanMvUart_GetStatus(CANMV_TARGET_LASER)));
            snprintf(line1, sizeof(line1), "L:%u,%u", laser == NULL ? 0U : laser->value[0], laser == NULL ? 0U : laser->value[1]);
            snprintf(line2, sizeof(line2), "Rect:%s", AppDeviceCheck_CanMvStatusText(CanMvUart_GetStatus(CANMV_TARGET_RECT)));
            snprintf(line3, sizeof(line3), "R:%u,%u", rect == NULL ? 0U : rect->value[0], rect == NULL ? 0U : rect->value[1]);
            Ui_RenderLines("Check Vision", line0, line1, line2, line3, "Short:next", "Long:back");
            break;
        }
        case APP_DEVICE_PAGE_IMU_ANGLE:{
            WIT_IMU_DATA data = {0};
            (void)WitGetData(&data);
            snprintf(line0, sizeof(line0), "Yaw:%0.1f", data.attitude_deg.yaw);
            snprintf(line1, sizeof(line1), "Pitch:%0.1f", data.attitude_deg.pitch);
            snprintf(line2, sizeof(line2), "Roll:%0.1f", data.attitude_deg.roll);
            snprintf(line3, sizeof(line3), "Frm:%lu", (unsigned long)s_imu_frame_count);
            Ui_RenderLines("IMU Angle", line0, line1, line2, line3, "Short:next", "Long:back");
            break;
        }
        case APP_DEVICE_PAGE_IMU_GYRO:{
            WIT_IMU_DATA data = {0};
            (void)WitGetData(&data);
            snprintf(line0, sizeof(line0), "Gx:%0.1f", data.gyro_deg_s.x);
            snprintf(line1, sizeof(line1), "Gy:%0.1f", data.gyro_deg_s.y);
            snprintf(line2, sizeof(line2), "Gz:%0.1f", data.gyro_deg_s.z);
            snprintf(line3, sizeof(line3), "Frm:%lu", (unsigned long)s_imu_frame_count);
            Ui_RenderLines("IMU Gyro", line0, line1, line2, line3, "Short:next", "Long:back");
            break;
        }
        case APP_DEVICE_PAGE_IMU_ACC:{
            WIT_IMU_DATA data = {0};
            (void)WitGetData(&data);
            snprintf(line0, sizeof(line0), "Ax:%0.2f", data.acc_g.x);
            snprintf(line1, sizeof(line1), "Ay:%0.2f", data.acc_g.y);
            snprintf(line2, sizeof(line2), "Az:%0.2f", data.acc_g.z);
            snprintf(line3, sizeof(line3), "Temp:%0.1f", data.temperature_c);
            Ui_RenderLines("IMU Acc", line0, line1, line2, line3, "Short:next", "Long:back");
            break;
        }
        case APP_DEVICE_PAGE_IMU_STATUS:
            snprintf(line0, sizeof(line0), "Rx:%lu", (unsigned long)g_app_device_imu_rx_byte_count);
            snprintf(line1, sizeof(line1), "Ok:%lu", (unsigned long)g_app_device_imu_valid_frame_count);
            snprintf(line2, sizeof(line2), "Bad:%lu", (unsigned long)g_app_device_imu_invalid_frame_count);
            snprintf(line3, sizeof(line3), "Type:0x%02X", g_app_device_imu_last_type);
            Ui_RenderLines("IMU Status", line0, line1, line2, line3, "Short:next", "Long:back");
            break;
        case APP_DEVICE_PAGE_IMU_IRQ:
        default:
            snprintf(line0, sizeof(line0), "Irq:%lu", (unsigned long)g_app_debug_uart_irq_count);
            snprintf(line1, sizeof(line1), "RxIrq:%lu", (unsigned long)g_app_debug_uart_rx_irq_count);
            snprintf(line2, sizeof(line2), "Byte:%lu", (unsigned long)g_app_debug_uart_drained_byte_count);
            snprintf(line3, sizeof(line3), "IIDX:%u E:%lu", g_app_debug_uart_last_iidx,
                     (unsigned long)(g_app_debug_uart_empty_rx_irq_count + g_app_debug_uart_unhandled_irq_count));
            Ui_RenderLines("IMU IRQ", line0, line1, line2, line3, "Short:next", "Long:back");
            break;
        }
}

void AppDeviceCheck_Run(void){
    APP_DEVICE_PAGE page = APP_DEVICE_PAGE_CHASSIS;
    uint32_t last_render_ms = BSP_Time_GetMs();

    Key_ClearAllEvents();
    AppDeviceCheck_Render(page);

    while (1){
        if (Key_IsShortPress(KEY_ID_1)){
            page = (APP_DEVICE_PAGE)(((uint8_t)page + 1U) % (uint8_t)APP_DEVICE_PAGE_COUNT);
            Key_ClearAllEvents();
            AppDeviceCheck_Render(page);
            last_render_ms = BSP_Time_GetMs();
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            return;
        }

        if ((BSP_Time_GetMs() - last_render_ms) >= APP_DEVICE_RENDER_PERIOD_MS){
            AppDeviceCheck_Render(page);
            last_render_ms = BSP_Time_GetMs();
        }

        BSP_DelayMs(APP_DEVICE_LOOP_DELAY_MS);
    }
}

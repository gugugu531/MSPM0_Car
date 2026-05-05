/**
 * @file  AppLauncher.c
 * @brief 顶层应用启动框架，提供赛题入口和调试入口
 */
#include "AppLauncher.h"
#include "Delay.h"
#include "HallEncoder.h"
#include "InitStepMotor.h"
#include "Initialize.h"
#include "Key.h"
#include "Menu.h"
#include "OLED.h"
#include "SystemTime.h"
#include "TrackingRuntime.h"
#include "TrackingSensor.h"
#include "VisionState.h"
#include "WitSDK.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define APP_IMU_FRAME_SIZE 33
#define APP_MOTOR_TEST_RUN_DUTY 600
#define APP_MOTOR_TEST_TURN_DUTY 500

typedef enum {
    APP_ENTRY_MISSION = 0,
    APP_ENTRY_TEST,
    APP_ENTRY_COUNT
} AppEntry;

typedef enum {
    APP_TEST_MOTOR = 0,
    APP_TEST_STEPPER,
    APP_TEST_IMU,
    APP_TEST_VISION,
    APP_TEST_TRACKING,
    APP_TEST_COUNT
} AppTestItem;

typedef enum {
    MOTOR_TEST_STOP = 0,
    MOTOR_TEST_FORWARD,
    MOTOR_TEST_BACKWARD,
    MOTOR_TEST_TURN_LEFT,
    MOTOR_TEST_TURN_RIGHT,
    MOTOR_TEST_COUNT
} MotorTestMode;

typedef enum {
    STEPPER_TEST_STOP = 0,
    STEPPER_TEST_YAW_POS,
    STEPPER_TEST_YAW_NEG,
    STEPPER_TEST_PITCH_POS,
    STEPPER_TEST_PITCH_NEG,
    STEPPER_TEST_COUNT
} StepperTestMode;

typedef enum {
    IMU_PAGE_ACC = 0,
    IMU_PAGE_GYRO,
    IMU_PAGE_ANGLE,
    IMU_PAGE_COUNT
} ImuPage;

typedef enum {
    VISION_PAGE_LASER = 0,
    VISION_PAGE_RECT,
    VISION_PAGE_COUNT
} VisionPage;

static uint8_t s_imu_rx_buffer[APP_IMU_FRAME_SIZE];
static uint8_t s_imu_rx_index = 0;
static uint32_t s_imu_frame_count = 0;
static bool s_imu_uart_enabled = false;

static const char *const s_root_items[APP_ENTRY_COUNT] = {
    "Task flow",
    "Device check"
};

static const char *const s_test_items[APP_TEST_COUNT] = {
    "Motor speed",
    "Pan tilt",
    "Gyro angle",
    "Vision loc",
    "Track sensor"
};

static const char *const s_motor_modes[MOTOR_TEST_COUNT] = {
    "Stop",
    "Forward",
    "Reverse",
    "Spin Left",
    "Spin Right"
};

static const char *const s_stepper_modes[STEPPER_TEST_COUNT] = {
    "Stop",
    "Yaw +",
    "Yaw -",
    "Pitch +",
    "Pitch -"
};

static void App_ClearKeys(void){
    Key_ClearAllEvents();
}

static const char *App_CanMvErrorText(CanMV_Error error){
    switch (error){
        case CANMV_ERR_NONE:
            return "OK";
        case CANMV_ERR_NOT_FOUND:
            return "MISS";
        case CANMV_ERR_LOST:
            return "LOST";
        case CANMV_ERR_FRAME_DROP:
            return "DROP";
        case CANMV_ERR_INIT:
        default:
            return "INIT";
    }
}

static void App_ShowLines(const char *title,
                          const char *line1,
                          const char *line2,
                          const char *line3,
                          const char *line4,
                          const char *line5){
    OLED_Clear();
    OLED_ShowString(0, 0, title, 8);

    if (line1 != NULL){
        OLED_ShowString(0, 2, line1, 8);
    }
    if (line2 != NULL){
        OLED_ShowString(0, 3, line2, 8);
    }
    if (line3 != NULL){
        OLED_ShowString(0, 4, line3, 8);
    }
    if (line4 != NULL){
        OLED_ShowString(0, 6, line4, 8);
    }
    if (line5 != NULL){
        OLED_ShowString(0, 7, line5, 8);
    }
}

static void App_UpdateTestLines(const char *line1,
                                const char *line2,
                                const char *line3,
                                const char *line4){
    if (line1 != NULL){
        OLED_ShowStringClearLine(0, 2, line1, 8);
    }
    if (line2 != NULL){
        OLED_ShowStringClearLine(0, 3, line2, 8);
    }
    if (line3 != NULL){
        OLED_ShowStringClearLine(0, 4, line3, 8);
    }
    if (line4 != NULL){
        OLED_ShowStringClearLine(0, 6, line4, 8);
    }
}

static void App_EnableImuDebugUart(void){
    if (s_imu_uart_enabled){
        return;
    }

    DL_UART_Main_enableInterrupt(Debug_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_SetPriority(Debug_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(Debug_INST_INT_IRQN);
    s_imu_uart_enabled = true;
}

static bool App_IsImuFrameValid(const uint8_t *frame){
    uint16_t sum_acc = 0;
    uint16_t sum_gyro = 0;
    uint16_t sum_angle = 0;

    if (frame[0] != 0x55 || frame[11] != 0x55 || frame[22] != 0x55){
        return false;
    }

    for (uint8_t i = 0; i < 10; i++){
        sum_acc += frame[i];
        sum_gyro += frame[11 + i];
        sum_angle += frame[22 + i];
    }

    return frame[10] == (uint8_t)(sum_acc & 0xFF) &&
           frame[21] == (uint8_t)(sum_gyro & 0xFF) &&
           frame[32] == (uint8_t)(sum_angle & 0xFF);
}

static void App_ProcessImuByte(uint8_t data){
    if (s_imu_rx_index == 0 && data != 0x55){
        return;
    }

    s_imu_rx_buffer[s_imu_rx_index++] = data;

    if (s_imu_rx_index < APP_IMU_FRAME_SIZE){
        return;
    }

    if (App_IsImuFrameValid(s_imu_rx_buffer)){
        GYROSCOPE_DATA_Decoder(s_imu_rx_buffer);
        s_imu_frame_count++;
    }

    s_imu_rx_index = 0;
}

static void App_RunMotorTest(void){
    MotorTestMode mode = MOTOR_TEST_STOP;
    uint16_t duty = 0;
    char line1[24];
    char line2[24];
    char line3[24];
    uint32_t last_refresh = 0;

    Motor_Brake();
    App_ClearKeys();
    App_ShowLines("Drive check", "State:Stop", "Duty:0/1000", "Enc spd:0.00", "S run/state", "L back");

    while (1){
        key_event_t event = Key_GetEvent(KEY_ID_1);

        if (event == KEY_EVENT_SHORT_PRESS){
            mode = (MotorTestMode)((mode + 1) % MOTOR_TEST_COUNT);
        } else if (event == KEY_EVENT_LONG_PRESS){
            Motor_Brake();
            App_ClearKeys();
            return;
        }

        switch (mode){
            case MOTOR_TEST_STOP:
                duty = 0;
                Motor_Brake();
                break;
            case MOTOR_TEST_FORWARD:
                duty = APP_MOTOR_TEST_RUN_DUTY;
                Motor_SetLeft((int16_t)duty);
                Motor_SetRight((int16_t)duty);
                break;
            case MOTOR_TEST_BACKWARD:
                duty = APP_MOTOR_TEST_RUN_DUTY;
                Motor_SetLeft(-(int16_t)duty);
                Motor_SetRight(-(int16_t)duty);
                break;
            case MOTOR_TEST_TURN_LEFT:
                duty = APP_MOTOR_TEST_TURN_DUTY;
                Motor_SetLeft(-(int16_t)duty);
                Motor_SetRight((int16_t)duty);
                break;
            case MOTOR_TEST_TURN_RIGHT:
                duty = APP_MOTOR_TEST_TURN_DUTY;
                Motor_SetLeft((int16_t)duty);
                Motor_SetRight(-(int16_t)duty);
                break;
            default:
                duty = 0;
                Motor_Brake();
                break;
        }

        if (tick - last_refresh >= 120){
            last_refresh = tick;
            snprintf(line1, sizeof(line1), "State:%s", s_motor_modes[mode]);
            snprintf(line2, sizeof(line2), "Duty:%u/1000", duty);
            snprintf(line3, sizeof(line3), "Enc spd:%0.2f", Encoder_GetSpeed());
            App_UpdateTestLines(line1, line2, line3, NULL);
        }

        Delay_ms(20);
    }
}

static void App_RunStepperTest(void){
    StepperTestMode mode = STEPPER_TEST_STOP;
    char line1[24];
    char line2[24];
    char line3[24];
    uint32_t last_refresh = 0;

    YP_SMotor_Init();
    App_ClearKeys();
    App_ShowLines("Pan tilt", "State:Stop", "Yaw deg:0.0", "Pit deg:0.0", "S state", "L back");

    while (1){
        key_event_t event = Key_GetEvent(KEY_ID_1);

        if (event == KEY_EVENT_SHORT_PRESS){
            mode = (StepperTestMode)((mode + 1) % STEPPER_TEST_COUNT);
        } else if (event == KEY_EVENT_LONG_PRESS){
            YP_SMotor_SetSpeed(0.0f, 0.0f);
            DL_GPIO_clearPins(SMotor_IO_PORT, SMotor_IO_EN1_PIN);
            DL_GPIO_clearPins(SMotor_IO_PORT, SMotor_IO_EN2_PIN);
            App_ClearKeys();
            return;
        }

        switch (mode){
            case STEPPER_TEST_STOP:
                YP_SMotor_SetSpeed(0.0f, 0.0f);
                break;
            case STEPPER_TEST_YAW_POS:
                YP_SMotor_SetSpeed(120.0f, 0.0f);
                break;
            case STEPPER_TEST_YAW_NEG:
                YP_SMotor_SetSpeed(-120.0f, 0.0f);
                break;
            case STEPPER_TEST_PITCH_POS:
                YP_SMotor_SetSpeed(0.0f, 90.0f);
                break;
            case STEPPER_TEST_PITCH_NEG:
                YP_SMotor_SetSpeed(0.0f, -90.0f);
                break;
            default:
                YP_SMotor_SetSpeed(0.0f, 0.0f);
                break;
        }

        YP_SMotor_UpdateState();

        if (tick - last_refresh >= 120){
            last_refresh = tick;
            snprintf(line1, sizeof(line1), "State:%s", s_stepper_modes[mode]);
            snprintf(line2, sizeof(line2), "Yaw deg:%0.1f", GetYaw());
            snprintf(line3, sizeof(line3), "Pit deg:%0.1f", GetPitch());
            App_UpdateTestLines(line1, line2, line3, NULL);
        }

        Delay_ms(20);
    }
}

static void App_RunImuTest(void){
    ImuPage page = IMU_PAGE_ANGLE;
    char line1[24];
    char line2[24];
    char line3[24];
    char line4[24];
    uint32_t last_refresh = 0;

    App_EnableImuDebugUart();
    App_ClearKeys();
    App_ShowLines("Gyro check", "No IMU data", "Check UART0", "Frame wait...", "Frames:0", "S page L back");

    while (1){
        key_event_t event = Key_GetEvent(KEY_ID_1);

        if (event == KEY_EVENT_SHORT_PRESS){
            page = (ImuPage)((page + 1) % IMU_PAGE_COUNT);
        } else if (event == KEY_EVENT_LONG_PRESS){
            App_ClearKeys();
            return;
        }

        if (tick - last_refresh >= 150){
            last_refresh = tick;

            if (s_imu_frame_count == 0){
                snprintf(line1, sizeof(line1), "No IMU data");
                snprintf(line2, sizeof(line2), "Check UART0");
                snprintf(line3, sizeof(line3), "Frame wait...");
            } else{
                switch (page){
                    case IMU_PAGE_ACC:
                        snprintf(line1, sizeof(line1), "AX:%0.2f", GyroscopeChannelData[0]);
                        snprintf(line2, sizeof(line2), "AY:%0.2f", GyroscopeChannelData[1]);
                        snprintf(line3, sizeof(line3), "AZ:%0.2f", GyroscopeChannelData[2]);
                        break;
                    case IMU_PAGE_GYRO:
                        snprintf(line1, sizeof(line1), "GX:%0.1f", GyroscopeChannelData[3]);
                        snprintf(line2, sizeof(line2), "GY:%0.1f", GyroscopeChannelData[4]);
                        snprintf(line3, sizeof(line3), "GZ:%0.1f", GyroscopeChannelData[5]);
                        break;
                    case IMU_PAGE_ANGLE:
                    default:
                        snprintf(line1, sizeof(line1), "Roll:%0.1f", GyroscopeChannelData[6]);
                        snprintf(line2, sizeof(line2), "Pitch:%0.1f", GyroscopeChannelData[7]);
                        snprintf(line3, sizeof(line3), "Yaw:%0.1f", GyroscopeChannelData[8]);
                        break;
                }
            }

            snprintf(line4, sizeof(line4), "Frames:%lu", (unsigned long)s_imu_frame_count);
            App_UpdateTestLines(line1, line2, line3, line4);
        }

        Delay_ms(20);
    }
}

static void App_RunVisionTest(void){
    VisionPage page = VISION_PAGE_LASER;
    char line1[24];
    char line2[24];
    char line3[24];
    uint32_t last_refresh = 0;

    App_ClearKeys();
    App_ShowLines("Vision check", "Laser:INIT", "X:0 Y:0", "Dot2:0 0", "S page", "L back");

    while (1){
        key_event_t event = Key_GetEvent(KEY_ID_1);

        if (event == KEY_EVENT_SHORT_PRESS){
            page = (VisionPage)((page + 1) % VISION_PAGE_COUNT);
        } else if (event == KEY_EVENT_LONG_PRESS){
            App_ClearKeys();
            return;
        }

        if (tick - last_refresh >= 150){
            last_refresh = tick;

            if (page == VISION_PAGE_LASER){
                snprintf(line1, sizeof(line1), "Laser:%s", App_CanMvErrorText(Laser_error));
                snprintf(line2, sizeof(line2), "X:%u Y:%u", Laser_Loc[0], Laser_Loc[1]);
                snprintf(line3, sizeof(line3), "Dot2:%u %u", Laser_Loc[2], Laser_Loc[3]);
            } else{
                snprintf(line1, sizeof(line1), "Rect:%s", App_CanMvErrorText(Rect_error));
                snprintf(line2, sizeof(line2), "P0:%u,%u", Rect_Loc[0], Rect_Loc[1]);
                snprintf(line3, sizeof(line3), "P1:%u,%u", Rect_Loc[2], Rect_Loc[3]);
            }

            App_UpdateTestLines(line1, line2, line3, NULL);
        }

        Delay_ms(20);
    }
}

static void App_RunTrackingTest(void){
    char sensor_bits[16];
    char line1[24];
    char line2[24];
    uint32_t last_refresh = 0;

    App_ClearKeys();
    App_ShowLines("Track check", "Bits:00000000", "Car spd:0.00", "8 sensor input", NULL, "L back");

    while (1){
        key_event_t event = Key_GetEvent(KEY_ID_1);

        if (event == KEY_EVENT_LONG_PRESS){
            App_ClearKeys();
            return;
        }

        TrackingSensor_Read(Digital);

        if (tick - last_refresh >= 120){
            last_refresh = tick;

            for (uint8_t i = 0; i < 8; i++){
                sensor_bits[i] = (char)('0' + (Digital[i] ? 1 : 0));
            }
            sensor_bits[8] = '\0';

            snprintf(line1, sizeof(line1), "Bits:%s", sensor_bits);
            snprintf(line2, sizeof(line2), "Car spd:%0.2f", Encoder_GetSpeed());
            App_UpdateTestLines(line1, line2, NULL, NULL);
        }

        Delay_ms(20);
    }
}

static void App_RunSelectedTest(AppTestItem test_item){
    switch (test_item){
        case APP_TEST_MOTOR:
            App_RunMotorTest();
            break;
        case APP_TEST_STEPPER:
            App_RunStepperTest();
            break;
        case APP_TEST_IMU:
            App_RunImuTest();
            break;
        case APP_TEST_VISION:
            App_RunVisionTest();
            break;
        case APP_TEST_TRACKING:
            App_RunTrackingTest();
            break;
        default:
            break;
    }
}

static void App_RunTestMenu(void){
    AppTestItem selected = APP_TEST_MOTOR;
    char line1[24];
    bool need_refresh = true;

    App_ClearKeys();

    while (1){
        key_event_t event = Key_GetEvent(KEY_ID_1);

        if (event == KEY_EVENT_SHORT_PRESS){
            selected = (AppTestItem)((selected + 1) % APP_TEST_COUNT);
            need_refresh = true;
        } else if (event == KEY_EVENT_LONG_PRESS){
            App_ClearKeys();
            App_RunSelectedTest(selected);
            need_refresh = true;
        } else if (event == KEY_EVENT_DOUBLE_CLICK){
            App_ClearKeys();
            return;
        }

        if (need_refresh){
            snprintf(line1, sizeof(line1), "%u/%u %s",
                     (unsigned int)(selected + 1),
                     (unsigned int)APP_TEST_COUNT,
                     s_test_items[selected]);
            App_ShowLines("Device check", line1, "Run debug page", NULL, "S next L enter", "D back");
            need_refresh = false;
        }

        Delay_ms(20);
    }
}

void App_Launch(void){
    AppEntry selected = APP_ENTRY_MISSION;
    char line1[24];
    bool need_refresh = true;

    App_ClearKeys();

    while (1){
        key_event_t event = Key_GetEvent(KEY_ID_1);

        if (event == KEY_EVENT_SHORT_PRESS){
            selected = (AppEntry)((selected + 1) % APP_ENTRY_COUNT);
            need_refresh = true;
        } else if (event == KEY_EVENT_LONG_PRESS){
            App_ClearKeys();

            if (selected == APP_ENTRY_MISSION){
                menu_init();
                menu_begin();
            } else{
                App_RunTestMenu();
            }

            need_refresh = true;
        }

        if (need_refresh){
            snprintf(line1, sizeof(line1), "%u/%u %s",
                     (unsigned int)(selected + 1),
                     (unsigned int)APP_ENTRY_COUNT,
                     s_root_items[selected]);
            App_ShowLines("Mode select", line1, "Choose run mode", NULL, "S next", "L enter");
            need_refresh = false;
        }

        Delay_ms(20);
    }
}

void App_DebugUartHandler(void){
    if (DL_UART_getPendingInterrupt(Debug_INST) == DL_UART_IIDX_RX){
        App_ProcessImuByte((uint8_t)DL_UART_Main_receiveData(Debug_INST));
    }
}

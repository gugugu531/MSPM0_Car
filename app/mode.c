/**
 * @file  Mode.c
 * @brief 测试模式与比赛模式的流程控制实现
 */
#include "mode.h"
#include "system_error_state.h"
#include "error_handler.h"
#include "tracking_runtime.h"
#include "vision_state.h"
#include <stdint.h>
#include <stdio.h>
#include "bsp_time.h"
#include "motor_system.h"
#include "kinematics/kinematics.h"
#include "oled.h"
#include "geometry/geometry.h"
#include "step_motor_ctrl.h"
#include "line_tracking/line_tracking.h"
#include "grayscale_sensor.h"

extern char CircleNum;

static int turn_state = 0;

static bool Mode_IsRoadDetected(int num_min, int num_max){
    int active_count = 0;

    for (uint8_t i = 0; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++){
        if (Digital[i] == 0U){
            active_count++;
        }
    }

    return (active_count >= num_min) && (active_count <= num_max);
}

static bool Mode_IsHalfDetected(void){
    return Mode_IsRoadDetected(3, 6);
}

static bool Mode_IsEmptyDetected(void){
    return Mode_IsRoadDetected(0, 0);
}

void mode_test_distance(void){
    while (1){
        Motor_SetLeft(-200);
        Motor_SetRight(200);
    }
}

void mode_test_coordinate(void){
    YP_SMotor_Init();

    while (1){
        YP_SMotor_SetSpeed(180, 0);
        Motor_SetLeft(300);
        Motor_SetRight(300);
        BSP_DelayMs(500);

        Motor_SetLeft(-300);
        Motor_SetRight(-300);
        YP_SMotor_SetSpeed(-180, 0);
        BSP_DelayMs(500);
    }
}

void mode_test_circle(void){
    while (1){
        UpdateSInedge();
        GrayscaleSensor_Read(Digital);
        if (!mode_turn_step()){
            Motor_Brake();
            break;
        }
        BSP_DelayMs(10);
    }
}

void mode_test_connection(void){
    char message[50];
    while (1){
        CORE_POINT2F paper = {0.1f, 0.1f};
        uint16_t rect_data[8] = {
            Rect_Loc[0],
            Rect_Loc[1],
            Rect_Loc[2],
            Rect_Loc[3],
            Rect_Loc[4],
            Rect_Loc[5],
            Rect_Loc[6],
            Rect_Loc[7],
        };
        GEOMETRY_RECT2F rect;

        Geometry_RectFromArray(rect_data, &rect);
        CORE_POINT2F cor = Geometry_PaperToRectPoint(paper, 315.0f, 212.0f, &rect);
        sprintf(message, "Camera: (%.2f, %.2f)", cor.x, cor.y);
        OLED_ShowString(0, 0, message, 8);
    }
}

void mode_test_tracking(void){
    while (1){
        GrayscaleSensor_Read(Digital);
        (void)LineTracking_Update(0.01f);
        if (Mode_IsEmptyDetected()){
            Motor_Brake();
            return;
        }
    }
}

void mode_problem_b_1(void){
    int cn = mode_set_circle_num(CircleNum);
#ifdef MODE_DEBUG
    char debug_message[50];
    sprintf(debug_message, "CircleNum: %d", cn);
    OLED_ShowString(0, 0, debug_message, 8);
#endif

    while (1){
        GrayscaleSensor_Read(Digital);
        UpdateSInedge();

        if (Mode_IsHalfDetected() && (cn * 4 == edge - 1)){
            Motor_Brake();
            return;
        }

        if (!mode_turn_step()){
            (void)LineTracking_Update(0.01f);
        }

        BSP_DelayMs(10);
    }
}

void mode_problem_b_2_3(void){
#ifdef MODE_DEBUG
    OLED_ShowString(0, 0, "Task B Laser", 8);
#endif

    YP_SMotor_Init();
    while (1){
        SetLaserPosition();
        SetTargetCenter();
        if (mode_init_guard()){
            PID_SMotor_Cont();
        }
        BSP_DelayMs(10);
    }
}

void mode_problem_h_1(void){
#ifdef MODE_DEBUG
    OLED_ShowString(0, 0, "Task H Circle", 8);
#endif

    int cn = mode_set_circle_num(1);

    YP_SMotor_Init();
    while (1){
        GrayscaleSensor_Read(Digital);
        UpdateSInedge();

        if (Mode_IsHalfDetected() && (cn * 4 == edge)){
            Motor_Brake();
            return;
        }

        if (!mode_turn_step()){
            DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
            (void)LineTracking_Update(0.05f);
        } else{
            DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        }

        SetLaserPosition();
        SetTargetCenter();
        Compute_excur();
        PID_SMotor_Cont();
        BSP_DelayMs(50);
    }
}

void mode_problem_h_2(void){
#ifdef MODE_DEBUG
    OLED_ShowString(0, 0, "Task H Track", 8);
#endif

    YP_SMotor_Init();
    while (1){
        GrayscaleSensor_Read(Digital);
        UpdateSInedge();

        if (Mode_IsHalfDetected() && (4 == edge)){
            Motor_Brake();
            DL_GPIO_clearPins(SMotor_IO_PORT, SMotor_IO_EN1_PIN);
            DL_GPIO_clearPins(SMotor_IO_PORT, SMotor_IO_EN2_PIN);
            return;
        }

        if (!mode_turn_step()){
            (void)LineTracking_Update(0.01f);
        }

        SetLaserPosition();
        SetTargetCircle();
        Compute_excur();
        PID_SMotor_Cont();
        BSP_DelayMs(10);
    }
}

int mode_set_circle_num(char num){
    if (num >= '0' && num <= '9'){
        return num - '0';
    }

    sprintf(error_message, "Invalid CircleNum input: %c\n", num);
    error_handler();
    return -1;
}

bool mode_turn_step(void){
    static float now_s_inedge = 0.0f;

    if (Mode_IsHalfDetected() && turn_state == 0){
        turn_state = 1;
        now_s_inedge = sInedge;
    }

    if (turn_state == 1){
        float first_dis = DisSensorToWheel * 1e-3f + now_s_inedge - 0.06f;
        float second_dis = first_dis + DEG_TO_RAD(90) * WHEEL_DIS * 1e-3f * 0.8f;

        if (sInedge < first_dis){
            Motor_SetLeft(200);
            Motor_SetRight(200);
            return true;
        }

        if (sInedge >= first_dis && sInedge < second_dis){
            turning = true;
            Motor_SetLeft(-120);
            Motor_SetRight(120);
            return true;
        }

        turning = false;
        sInedge = 0.0f;
        turn_state = 0;
        edge++;
    }

    return false;
}

bool mode_init_guard(void){
    if (Laser_error == CANMV_STATUS_NOT_FOUND){
        YP_SMotor_SetSpeed(-90, 0);
        return false;
    }

    return true;
}

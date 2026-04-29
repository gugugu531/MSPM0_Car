/**
 * @file  Mode.c
 * @brief 测试模式与比赛模式的流程控制实现
 */
#include "Mode.h"
#include "AppState.h"
#include "ErrorHandler.h"
#include "TrackingRuntime.h"
#include "VisionState.h"
#include <stdio.h>
#include "Delay.h"
#include "Initialize.h"
#include "Kinematics.h"
#include "Oled.h"
#include "SensorProc.h"
#include "StepMotorCtrl.h"
#include "Tracking.h"
#include "TrackingSensor.h"

extern char CircleNum;

static int turn_state = 0;
bool turning = false;

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
        Delay_ms(500);

        Motor_SetLeft(-300);
        Motor_SetRight(-300);
        YP_SMotor_SetSpeed(-180, 0);
        Delay_ms(500);
    }
}

void mode_test_circle(void){
    while (1){
        UpdateSInedge();
        TrackingSensor_Read(Digital);
        if (!mode_turn_step()){
            Motor_Brake();
            break;
        }
        Delay_ms(10);
    }
}

void mode_test_connection(void){
    char message[50];
    while (1){
        Coordinate cor = {0.0f, 0.0f};
        Coordinate paper = {0.1f, 0.1f};

        cor = paper_to_camera(paper);
        sprintf(message, "Camera: (%.2f, %.2f)", cor.x, cor.y);
        OLED_ShowString(0, 0, message, 8);
    }
}

void mode_test_tracking(void){
    while (1){
        TrackingSensor_Read(Digital);
        lineWalking_low();
        if (empty_Detect()){
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
        TrackingSensor_Read(Digital);
        UpdateSInedge();

        if (half_Detect() && (cn * 4 == edge - 1)){
            Motor_Brake();
            return;
        }

        if (!mode_turn_step()){
            lineWalking_low();
        }

        Delay_ms(10);
    }
}

void mode_problem_b_2_3(void){
#ifdef MODE_DEBUG
    OLED_ShowString(0, 0, "ProB2/3", 8);
#endif

    YP_SMotor_Init();
    while (1){
        SetLaserPosition();
        SetTargetCenter();
        if (mode_init_guard()){
            PID_SMotor_Cont();
        }
        Delay_ms(10);
    }
}

void mode_problem_h_1(void){
#ifdef MODE_DEBUG
    OLED_ShowString(0, 0, "ProH1", 8);
#endif

    int cn = mode_set_circle_num(1);

    YP_SMotor_Init();
    while (1){
        TrackingSensor_Read(Digital);
        UpdateSInedge();

        if (half_Detect() && (cn * 4 == edge)){
            Motor_Brake();
            return;
        }

        if (!mode_turn_step()){
            DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
            lineWalking_low();
        } else{
            DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        }

        SetLaserPosition();
        SetTargetCenter();
        Compute_excur();
        PID_SMotor_Cont();
        Delay_ms(50);
    }
}

void mode_problem_h_2(void){
#ifdef MODE_DEBUG
    OLED_ShowString(0, 0, "ProH2", 8);
#endif

    YP_SMotor_Init();
    while (1){
        TrackingSensor_Read(Digital);
        UpdateSInedge();

        if (half_Detect() && (4 == edge)){
            Motor_Brake();
            DL_GPIO_clearPins(SMotor_IO_PORT, SMotor_IO_EN1_PIN);
            DL_GPIO_clearPins(SMotor_IO_PORT, SMotor_IO_EN2_PIN);
            return;
        }

        if (!mode_turn_step()){
            lineWalking_low();
        }

        SetLaserPosition();
        SetTargetCircle();
        Compute_excur();
        PID_SMotor_Cont();
        Delay_ms(10);
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

    if (half_Detect() && turn_state == 0){
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
    if (Laser_error == CANMV_ERR_NOT_FOUND){
        YP_SMotor_SetSpeed(-90, 0);
        return false;
    }

    return true;
}

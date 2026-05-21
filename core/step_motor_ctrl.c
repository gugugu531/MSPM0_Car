/**
 * @file  StepMotorCtrl.c
 * @brief 云台步进电机控制与视觉目标跟踪实现
 */
#include "step_motor_ctrl.h"
#include "bsp_time.h"
#include "tracking_runtime.h"
#include "vision_state.h"
#include <stdbool.h>
#include <math.h>
#include "hall_encoder.h"
#include "motor_system.h"
#include "geometry/geometry.h"

static CORE_POINT2F laser_position = {0.0f, 0.0f};
static CORE_POINT2F target_position = {0.0f, 0.0f};
static bool is_updated = false;
static bool is_new_mode = true;
static CORE_ATTITUDE2F cor = {0.0f, 0.0f};

#define STEP_MOTOR_CTRL_PAPER_WIDTH_MM 315.0f
#define STEP_MOTOR_CTRL_PAPER_HEIGHT_MM 212.0f
#define STEP_MOTOR_CTRL_CIRCLE_RADIUS_MM 60.0f

void PID_SMotor_Cont(void){
    static PID_CONTROLLER pid_x, pid_y;
    static uint32_t last_update_time = 0;
    static float last_output_wyaw = 0.0f;
    static float last_output_wpitch = 0.0f;
    uint32_t current_time = BSP_Time_GetMs();

    if (is_new_mode){
        const PID_CONFIG pid_config = {
            .kp = 0.2f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integral_limit = 1000.0f,
            .output_limit = 0.0f,
            .mode = PID_MODE_POSITION,
        };

        PID_Init(&pid_x, &pid_config);
        PID_Init(&pid_y, &pid_config);
        is_new_mode = false;
        last_update_time = current_time;
        return;
    }

    float output_wyaw = 0.0f;
    float output_wpitch = 0.0f;

    if (!is_updated && (Laser_error == CANMV_STATUS_OK)){
        output_wyaw = last_output_wyaw;
        output_wpitch = last_output_wpitch;
    } else if (is_updated){
        is_updated = false;
        float dt = (current_time - last_update_time) / 1000.0f;
        output_wyaw = -1.0f * PID_Update(&pid_x, target_position.x, laser_position.x, dt);
        output_wpitch = PID_Update(&pid_y, target_position.y, laser_position.y, dt);
        last_update_time = current_time;
    }

    output_wpitch += cor.pitch;
    output_wyaw += cor.yaw;

    last_output_wyaw = output_wyaw;
    last_output_wpitch = output_wpitch;

    YP_SMotor_SetSpeed(output_wyaw, output_wpitch);
    YP_SMotor_UpdateState();
}

void SetTargetCenter(void){
    if (Laser_error == CANMV_STATUS_OK && Laser_Loc[0] != 0 && Laser_Loc[1] != 0){
        target_position.x = Laser_Loc[0];
        target_position.y = 480 - Laser_Loc[1];
        is_updated = true;
    }
}

void SetLaserPosition(void){
    if (Laser_error == CANMV_STATUS_OK){
        laser_position.x = Laser_Loc[2];
        laser_position.y = 480 - Laser_Loc[3];
    }
}

void SetTargetCircle(void){
    if (Rect_error == CANMV_STATUS_OK){
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
        CORE_POINT2F paper_center = {
            .x = STEP_MOTOR_CTRL_PAPER_WIDTH_MM / 2.0f,
            .y = STEP_MOTOR_CTRL_PAPER_HEIGHT_MM / 2.0f,
        };
        float target_angle_deg = (float)(edge - 1) * 90.0f + sInedge;
        CORE_POINT2F paper_target = Geometry_CirclePointDeg(paper_center,
                                                            STEP_MOTOR_CTRL_CIRCLE_RADIUS_MM,
                                                            target_angle_deg);

        Geometry_RectFromArray(rect_data, &rect);
        target_position = Geometry_PaperToRectPoint(paper_target,
                                                    STEP_MOTOR_CTRL_PAPER_WIDTH_MM,
                                                    STEP_MOTOR_CTRL_PAPER_HEIGHT_MM,
                                                    &rect);
        is_updated = true;
    }
}

void Compute_excur(void){
    if (turning){
        cor.yaw = -120.0f;
        DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
        return;
    }

    DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
    float d = getDistance();

    switch (edge % 4){
        case 0:
            cor.yaw = -(Encoder_GetSpeed() * fabs(0.5f - sInedge) / d) / d;
            break;
        case 1:
            cor.yaw = (Encoder_GetSpeed() * 0.5f / d) / d;
            break;
        case 2:
            cor.yaw = (Encoder_GetSpeed() * fabs(0.5f - sInedge) / d) / d;
            break;
        case 3:
            cor.yaw = -(Encoder_GetSpeed() * 0.5f / d) / d;
            break;
        default:
            break;
    }
}

float getDistance(void){
    switch (edge % 4){
        case 0:
            return sqrt(pow(0.5f - sInedge, 2) + pow(0.5f, 2));
        case 1:
            return sqrt(pow(0.5f, 2) + pow(0.5f + sInedge, 2));
        case 2:
            return sqrt(pow(0.5f - sInedge, 2) + pow(1.5f, 2));
        case 3:
            return sqrt(pow(0.5f, 2) + pow(1.5f - sInedge, 2));
        default:
            return 0.0f;
    }
}

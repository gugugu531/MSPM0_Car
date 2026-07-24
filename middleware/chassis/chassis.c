/**
 * @file  chassis.c
 * @brief Middleware 层底盘组合服务实现。
 */
#include "chassis.h"
#include "kinematics/kinematics.h"
#include "pid/pid.h"
#include <stddef.h>

static CHASSIS_DUTY chassis_duty;

/* 速度闭环状态: 每轮一个位置式 PID + 目标轮速; 模式默认开环。 */
static CHASSIS_CONTROL_MODE chassis_mode;
static PID_CONTROLLER chassis_speed_pid[HALL_ENCODER_COUNT];
static float chassis_speed_target[HALL_ENCODER_COUNT];

static BSP_STATUS Chassis_CombineStatus(BSP_STATUS current, BSP_STATUS next){
    return (current == BSP_STATUS_OK) ? next : current;
}

static void Chassis_ClearDuty(void){
    chassis_duty.left_percent = 0.0f;
    chassis_duty.right_percent = 0.0f;
}

/* 原始出力: 直接下发占空比并刷新记录, 不改控制模式(供开环 SetDuty 与闭环 UpdateSpeedControl 共用)。 */
static BSP_STATUS Chassis_ApplyDuty(float left_percent, float right_percent){
    BSP_STATUS status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_LEFT, left_percent);
    BSP_STATUS right_status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_RIGHT, right_percent);

    status = Chassis_CombineStatus(status, right_status);

    if (status == BSP_STATUS_OK){
        chassis_duty.left_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_LEFT);
        chassis_duty.right_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_RIGHT);
    }

    return status;
}

/* 用默认增益初始化两轮速度环并复位为开环、目标清零。 */
static void Chassis_SpeedControlInit(void){
    PID_CONFIG cfg = {
        .kp = CHASSIS_SPEED_KP,
        .ki = CHASSIS_SPEED_KI,
        .kd = CHASSIS_SPEED_KD,
        .integral_limit = CHASSIS_SPEED_INTEGRAL_LIMIT,
        .output_limit = CHASSIS_SPEED_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };

    for (uint8_t id = 0U; id < (uint8_t)HALL_ENCODER_COUNT; id++){
        PID_Init(&chassis_speed_pid[id], &cfg);
        chassis_speed_target[id] = 0.0f;
    }
    chassis_mode = CHASSIS_CONTROL_DUTY;
}

/* 退出速度闭环: 切回开环并清目标(供 SetDuty/Stop 调用)。 */
static void Chassis_ExitSpeedControl(void){
    chassis_mode = CHASSIS_CONTROL_DUTY;
    chassis_speed_target[HALL_ENCODER_LEFT] = 0.0f;
    chassis_speed_target[HALL_ENCODER_RIGHT] = 0.0f;
}

BSP_STATUS Chassis_Init(void){
    Chassis_ClearDuty();
    Chassis_SpeedControlInit();

    BSP_STATUS status = TB6612FNG_Init();
    status = Chassis_CombineStatus(status, HallEncoder_Init());
    return status;
}

BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent){
    Chassis_ExitSpeedControl();
    return Chassis_ApplyDuty(left_percent, right_percent);
}

void Chassis_SetWheelSpeed(float left_mps, float right_mps){
    if (chassis_mode != CHASSIS_CONTROL_SPEED){
        /* 从开环切入闭环: 复位 PID, 避免旧积分/微分残留造成跳变。 */
        PID_Reset(&chassis_speed_pid[HALL_ENCODER_LEFT]);
        PID_Reset(&chassis_speed_pid[HALL_ENCODER_RIGHT]);
        chassis_mode = CHASSIS_CONTROL_SPEED;
    }
    chassis_speed_target[HALL_ENCODER_LEFT] = left_mps;
    chassis_speed_target[HALL_ENCODER_RIGHT] = right_mps;
}

void Chassis_SetSpeed(float body_mps){
    Chassis_SetWheelSpeed(body_mps, body_mps);
}

BSP_STATUS Chassis_UpdateSpeedControl(float dt_s){
    if (chassis_mode != CHASSIS_CONTROL_SPEED){
        return BSP_STATUS_OK;   /* 开环: 不干预出力。 */
    }

    float left_duty = PID_Update(&chassis_speed_pid[HALL_ENCODER_LEFT],
        chassis_speed_target[HALL_ENCODER_LEFT],
        HallEncoder_GetSpeed(HALL_ENCODER_LEFT), dt_s);
    float right_duty = PID_Update(&chassis_speed_pid[HALL_ENCODER_RIGHT],
        chassis_speed_target[HALL_ENCODER_RIGHT],
        HallEncoder_GetSpeed(HALL_ENCODER_RIGHT), dt_s);

    return Chassis_ApplyDuty(left_duty, right_duty);
}

CHASSIS_CONTROL_MODE Chassis_GetControlMode(void){
    return chassis_mode;
}

float Chassis_GetWheelSpeedTarget(HALL_ENCODER_ID wheel){
    return chassis_speed_target[wheel];
}

BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode){
    BSP_STATUS status = BSP_STATUS_OK;

    Chassis_ExitSpeedControl();

    switch (mode){
        case CHASSIS_STOP_MODE_BRAKE:
            status = TB6612FNG_BrakeAll();
            break;
        case CHASSIS_STOP_MODE_COAST:
            status = TB6612FNG_CoastAll();
            break;
        default:
            return BSP_STATUS_INVALID_ARG;
    }

    if (status == BSP_STATUS_OK){
        Chassis_ClearDuty();
    }

    return status;
}

BSP_STATUS Chassis_Brake(void){
    return Chassis_Stop(CHASSIS_STOP_MODE_BRAKE);
}

BSP_STATUS Chassis_Coast(void){
    return Chassis_Stop(CHASSIS_STOP_MODE_COAST);
}

CHASSIS_DUTY Chassis_GetDuty(void){
    return chassis_duty;
}

float Chassis_GetSpeed(void){
    /*
     * 车体线速度 = 左右轮线速度均值(Kinematics_WheelToBody 的 linear 分量)。
     * 角速度分量需 track_width(暂缺, 见 core/kinematics 约定), 故传 0 只取线速度。
     */
    KINEMATICS_VELOCITY body = Kinematics_WheelToBody(
        HallEncoder_GetSpeed(HALL_ENCODER_LEFT),
        HallEncoder_GetSpeed(HALL_ENCODER_RIGHT),
        0.0f);
    return body.linear_mps;
}

float Chassis_GetWheelSpeed(HALL_ENCODER_ID wheel){
    return HallEncoder_GetSpeed(wheel);
}

float Chassis_GetDistance(void){
    return (HallEncoder_GetDistance(HALL_ENCODER_LEFT) +
            HallEncoder_GetDistance(HALL_ENCODER_RIGHT)) * 0.5f;
}

float Chassis_GetWheelDistance(HALL_ENCODER_ID wheel){
    return HallEncoder_GetDistance(wheel);
}

void Chassis_ResetDistance(void){
    HallEncoder_ResetDistance();
}

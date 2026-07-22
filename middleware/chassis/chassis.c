/**
 * @file  chassis.c
 * @brief Middleware 层底盘组合服务实现。
 */
#include "chassis.h"
#include <stddef.h>

static CHASSIS_DUTY chassis_duty;

static BSP_STATUS Chassis_CombineStatus(BSP_STATUS current, BSP_STATUS next){
    return (current == BSP_STATUS_OK) ? next : current;
}

static void Chassis_ClearDuty(void){
    chassis_duty.left_percent = 0.0f;
    chassis_duty.right_percent = 0.0f;
}

BSP_STATUS Chassis_Init(void){
    Chassis_ClearDuty();

    BSP_STATUS status = TB6612FNG_Init();
    status = Chassis_CombineStatus(status, HallEncoder_Init());
    return status;
}

BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent){
    BSP_STATUS status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_LEFT, left_percent);
    BSP_STATUS right_status = TB6612FNG_SetDuty(TB6612FNG_CHANNEL_RIGHT, right_percent);

    status = Chassis_CombineStatus(status, right_status);

    if (status == BSP_STATUS_OK){
        chassis_duty.left_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_LEFT);
        chassis_duty.right_percent = TB6612FNG_GetDuty(TB6612FNG_CHANNEL_RIGHT);
    }

    return status;
}

BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode){
    BSP_STATUS status = BSP_STATUS_OK;

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
    return HallEncoder_GetSpeed();
}

float Chassis_GetDistance(void){
    return HallEncoder_GetDistance();
}

void Chassis_ResetDistance(void){
    HallEncoder_ResetDistance();
}

/**
 * @file  yaw_estimator.c
 * @brief 纯角速度积分航向及融合角偏移计算实现。
 */
#include "yaw_estimator.h"

#include "kinematics/kinematics.h"

#include <stddef.h>

void YawEstimator_Reset(YAW_ESTIMATOR *estimator)
{
    if (estimator == NULL){
        return;
    }
    *estimator = (YAW_ESTIMATOR){0};
}

void YawEstimator_Start(YAW_ESTIMATOR *estimator, float fused_heading_deg)
{
    if (estimator == NULL){
        return;
    }
    float heading = Kinematics_NormalizeAngleDeg(fused_heading_deg);
    estimator->initial_fused_deg = heading;
    estimator->integrated_deg = heading;
    estimator->initialized = true;
}

void YawEstimator_Integrate(YAW_ESTIMATOR *estimator,
                            float gyro_z_deg_s,
                            float dt_s)
{
    if ((estimator == NULL) || !estimator->initialized || (dt_s <= 0.0f)){
        return;
    }
    estimator->integrated_deg = Kinematics_NormalizeAngleDeg(
        estimator->integrated_deg + gyro_z_deg_s * dt_s);
}

float YawEstimator_GetIntegrated(const YAW_ESTIMATOR *estimator)
{
    return ((estimator != NULL) && estimator->initialized)
               ? estimator->integrated_deg
               : 0.0f;
}

float YawEstimator_GetInitialFused(const YAW_ESTIMATOR *estimator)
{
    return ((estimator != NULL) && estimator->initialized)
               ? estimator->initial_fused_deg
               : 0.0f;
}

float YawEstimator_GetFusionOffset(const YAW_ESTIMATOR *estimator,
                                   float fused_heading_deg)
{
    if ((estimator == NULL) || !estimator->initialized){
        return 0.0f;
    }
    return Kinematics_AngleDiffDeg(fused_heading_deg,
                                   estimator->integrated_deg);
}

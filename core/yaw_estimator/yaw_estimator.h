/**
 * @file  yaw_estimator.h
 * @brief 纯角速度积分航向及融合角偏移计算，不读取硬件、不包含任务阶段策略。
 */
#ifndef YAW_ESTIMATOR_H
#define YAW_ESTIMATOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float initial_fused_deg;
    float integrated_deg;
    float previous_gyro_z_deg_s;
    bool initialized;
} YAW_ESTIMATOR;

/** 清空估计器；清空后必须重新 Start。 */
void YawEstimator_Reset(YAW_ESTIMATOR *estimator);
/** 以融合角建立公共原点 A0=B0，并保存梯形积分的首个角速度样本。 */
void YawEstimator_Start(YAW_ESTIMATOR *estimator,
                        float fused_heading_deg,
                        float initial_gyro_z_deg_s);
/** 使用上一角速度、本次角速度和实际时间间隔，以梯形积分推进纯积分角 A。 */
void YawEstimator_Integrate(YAW_ESTIMATOR *estimator,
                            float gyro_z_deg_s,
                            float dt_s);
/** 返回当前纯积分航向 A；未初始化时返回 0。 */
float YawEstimator_GetIntegrated(const YAW_ESTIMATOR *estimator);
/** 返回启动融合角 B0；未初始化时返回 0。 */
float YawEstimator_GetInitialFused(const YAW_ESTIMATOR *estimator);
/** 返回当前融合角相对纯积分角的最短角偏移 B-A。 */
float YawEstimator_GetFusionOffset(const YAW_ESTIMATOR *estimator,
                                   float fused_heading_deg);

#ifdef __cplusplus
}
#endif

#endif /* YAW_ESTIMATOR_H */

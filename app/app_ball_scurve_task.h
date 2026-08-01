/**
 * @file  app_ball_scurve_task.h
 * @brief 要求 3 正式业务入口与滚球控制调试入口。
 */
#ifndef APP_BALL_SCURVE_TASK_H
#define APP_BALL_SCURVE_TASK_H

#include "app_task.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 第三题正式入口：先守住 O；ENTER 后 +50 剖面结束即折返，最终在 −50 mm 停稳。 */
extern const APP_TASK_DESC APP_H3_CHALLENGE;

/**
 * 单点保持 0mm。无剖面，控制律持续以 x_ref=0 运行。
 * 用于单独验证抖动破静摩擦、扰动恢复能力，遥测格式与 SCurve 相同。
 * 配套上位机：tools/visualizers/ball_hold_monitor.html
 */
extern const APP_TASK_DESC APP_H3_BALL_HOLD;
/**
 * 单点保持 −50mm(−5cm)。与 APP_H3_BALL_HOLD 共用同一控制律，仅目标不同。
 * 供设备检查页直接观测末点保持行为。
 */
extern const APP_TASK_DESC APP_H3_BALL_HOLD_M5;

/**
 * 将编码器当前位置按 H3 连杆查表换算为动力学水平偏置，并保存到本次上电的 RAM 配置。
 * 不清零编码器、不改 180 cnt 几何查表；超出软限位或 ±2° 标定范围时拒绝。
 */
bool AppBallLevel_SetFromEncoderCount(int32_t encoder_count);
/** 查询当前 H3 动力学水平偏置，单位 deg。 */
float AppBallLevel_GetBiasDeg(void);
/** 本次上电后是否已由设备检查页捕获过水平点。 */
bool AppBallLevel_IsRuntimeCalibrated(void);

/**
 * H4/H5/H6 组合任务使用的固定位置保持生命周期。
 * 与 APP_H3_BALL_HOLD 共用同一控制器和参数，但不会刹停底盘或刷新 OLED。
 */
void AppBallHold_Enter(void);
/** 设置组合任务的固定保持目标；调用前须先 AppBallHold_Enter()。 */
bool AppBallHold_SetTargetMm(float target_mm);
/** 设置车辆沿钢球正坐标方向的规划加速度；静止/匀速传 0。 */
void AppBallHold_SetVehicleAcceleration(float acceleration_mps2);
/**
 * 组合任务中启用/关闭 IMU 加速度前馈。
 * 仅应在车体加速或减速时启用（巡航、制动等匀速段关闭），避免 IMU 噪声扰动滚球。
 * 独立 H3 Hold 忽略此设置（始终启用）。
 */
void AppBallHold_SetImuFeedforward(bool enable);
APP_TASK_STATUS AppBallHold_Tick(float dt);
void AppBallHold_Exit(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_SCURVE_TASK_H */

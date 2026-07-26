/**
 * @file  app_straight_task.h
 * @brief 直行测试任务描述符与遥测配置。
 */
#ifndef APP_STRAIGHT_TASK_H
#define APP_STRAIGHT_TASK_H

#include "app_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 是否每个 20ms 控制拍通过 Debug_Ex/UART1 输出 [STR] 遥测。 */
#define STRAIGHT_TELEMETRY_ENABLED                 1U
/** 首页 Straight Test 各模式的单次目标行驶距离，单位 m。 */
#define STRAIGHT_TEST_TARGET_DISTANCE_M            3.0f

extern const APP_TASK_DESC APP_STRAIGHT_DUTY_OPEN_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_SPEED_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_GYRO_RATE_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_GYRO_HEADING_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_RAMP_HEADING_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_RATE_THEN_HEADING_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_ENCODER_THEN_HEADING_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_INTEGRATED_THEN_HEADING_TEST;
extern const APP_TASK_DESC APP_STRAIGHT_FULL_INTEGRATED_THEN_HEADING_TEST;

#ifdef __cplusplus
}
#endif

#endif /* APP_STRAIGHT_TASK_H */

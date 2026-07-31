/**
 * @file  app_line_task.h
 * @brief 循迹测试任务描述符（供主菜单引用）。
 */
#ifndef APP_LINE_TASK_H
#define APP_LINE_TASK_H

#include "app_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 车辆纵向安装尺寸；循迹任务的测量点现统一为前置灰度阵列。 */
#define APP_TRACK_REAR_TO_AXLE_M      0.070f
#define APP_TRACK_REAR_TO_SENSOR_M    0.195f
#define APP_TRACK_SENSOR_TO_AXLE_M \
    (APP_TRACK_REAR_TO_SENSOR_M - APP_TRACK_REAR_TO_AXLE_M)

/** 赛题要求 2：空载高速整圈。 */
extern const APP_TASK_DESC APP_H2_EMPTY_LAP;
/** 赛题要求 3：静态滚球；水管控制未接入时只提供安全占位入口。 */
extern const APP_TASK_DESC APP_H3_BALL_STATIC;
/** 赛题要求 4：载球 A→B 直线。 */
extern const APP_TASK_DESC APP_H4_LOADED_STRAIGHT;
/** 赛题要求 5：载球、目标 O 点整圈；当前只运行底盘部分。 */
extern const APP_TASK_DESC APP_H5_LOADED_LAP_CENTER;
/** 赛题要求 6：载球、任意目标整圈；当前只运行底盘部分。 */
extern const APP_TASK_DESC APP_H6_LOADED_LAP_TARGET;

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_TASK_H */

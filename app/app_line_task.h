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

/*
 * 赛题指定测量点与传感器的纵向安装尺寸（沿车辆前进方向）。
 * 测量点暂定车尾；轮轴中心线在其前方约 7 cm，灰度阵列在其前方约 19.5 cm。
 */
#define APP_TRACK_MEASURE_TO_AXLE_M  0.070f
#define APP_TRACK_MEASURE_TO_SENSOR_M 0.195f

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

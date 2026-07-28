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

/** 循迹测试：使用 Yahboom 8 路模块观测，跑 line_follow 完整闭环。 */
extern const APP_TASK_DESC APP_LINE_FOLLOW_TEST;
/** 80% 直接起步：角速度启动后运行 Yahboom 循线外环 + 航向内环。 */
extern const APP_TASK_DESC APP_LINE_GUIDED_TEST;
/** K230 红线视觉循迹：角速度起步后持续融合位置/方向偏差。 */
extern const APP_TASK_DESC APP_VISION_LINE_TEST;

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_TASK_H */

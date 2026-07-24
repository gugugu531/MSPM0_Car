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

/** 循迹测试：用 middleware/line_tracking 完整闭环跑巡线，上板验证循迹行为。 */
extern const APP_TASK_DESC APP_LINE_TRACK_TEST;

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_TASK_H */

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

/** 循迹：使用 Yahboom 8 路循线观测，跑 line_follow 完整闭环。 */
extern const APP_TASK_DESC APP_LINE_FOLLOW_TEST;

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_TASK_H */

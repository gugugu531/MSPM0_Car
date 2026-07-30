/**
 * @file  app_ball_scurve_task.h
 * @brief 纯 S 曲线滚球点到点任务（要求 3 的 O→+5cm→−5cm 序列）。
 */
#ifndef APP_BALL_SCURVE_TASK_H
#define APP_BALL_SCURVE_TASK_H

#include "app_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 纯 S 曲线前馈 + 剖面跟踪反馈，无低速捕获逻辑。 */
extern const APP_TASK_DESC APP_H3_BALL_SCURVE;

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_SCURVE_TASK_H */


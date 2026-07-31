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

/** 完整 S 曲线序列：0→+50→−50 mm，前馈 + 剖面跟踪 PD + 抖动。 */
extern const APP_TASK_DESC APP_H3_BALL_SCURVE;

/**
 * 单点保持 0mm。无剖面，控制律持续以 x_ref=0 运行。
 * 用于单独验证抖动破静摩擦、扰动恢复能力，遥测格式与 SCurve 相同。
 * 配套上位机：tools/visualizers/ball_hold_monitor.html
 */
extern const APP_TASK_DESC APP_H3_BALL_HOLD;

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_SCURVE_TASK_H */

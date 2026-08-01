/**
 * @file  app_ball_scurve_task.h
 * @brief 要求 3 正式业务入口与滚球控制调试入口。
 */
#ifndef APP_BALL_SCURVE_TASK_H
#define APP_BALL_SCURVE_TASK_H

#include "app_task.h"
#include <stdbool.h>

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
 * H4/H5 组合任务使用的 0mm 保持生命周期。
 * 与 APP_H3_BALL_HOLD 共用同一控制器和参数，但不会刹停底盘或刷新 OLED。
 */
void AppBallHold_Enter(void);
/** 设置组合任务的固定保持目标；调用前须先 AppBallHold_Enter()。 */
bool AppBallHold_SetTargetMm(float target_mm);
APP_TASK_STATUS AppBallHold_Tick(float dt);
void AppBallHold_Exit(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_SCURVE_TASK_H */

/**
 * @file  app_e_task.h
 * @brief App 层 2025 年电赛 E 题基本任务入口。
 */
#ifndef APP_E_TASK_H
#define APP_E_TASK_H

#include <stdint.h>

/**
 * @brief 执行指定圈数的巡线任务。
 * @param lap_count 目标圈数。
 */
void AppE_RunLineFollow(uint8_t lap_count);

/**
 * @brief 执行拐角识别刹车测试。
 *
 * 进入后先循线运行，检测并确认到空线拐角后先主动刹车，
 * 再以左轮负值、右轮正值的原地差速固定左转，用于单独调试直角弯动作参数。
 */
void AppE_RunCornerBrakeTest(void);

/**
 * @brief 执行 2 秒靶心瞄准任务。
 */
void AppE_RunAimCenter2s(void);

/**
 * @brief 执行 E3 Yaw+ 扫描和瞄准任务。
 */
void AppE_RunRectScanYawPositive(void);

/**
 * @brief 执行 E3 Yaw- 扫描和瞄准任务。
 */
void AppE_RunRectScanYawNegative(void);

#endif /* APP_E_TASK_H */

/**
 * @file  app_checks.h
 * @brief 外设自检任务描述符（挂在菜单树的 Device Check 子菜单下）。
 *
 * 均遵守 APP_TASK_DESC 契约，只靠 BACK 短按退出（不返回 DONE）。
 * 两个陀螺仪自检共用 I2C0，靠各自 on_enter/on_exit 挂起/恢复对方，分时占用总线。
 */
#ifndef APP_CHECKS_H
#define APP_CHECKS_H

#include "app_task.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const APP_TASK_DESC APP_CHK_GYRO_JY61P;    /**< JY61P 陀螺/姿态（I2C0）。 */
extern const APP_TASK_DESC APP_CHK_GYRO_MPU6050;  /**< MPU6050 物理六轴/静态倾角（I2C0）。 */
extern const APP_TASK_DESC APP_CHK_GRAYSCALE;     /**< 8 路灰度数字量（GPIO）。 */
extern const APP_TASK_DESC APP_CHK_GRAY_I2C;      /**< 感为 8 路灰度 I2C（I2C0，0x4F）。 */
extern const APP_TASK_DESC APP_CHK_YAHBOOM_I2C;   /**< Yahboom 8 路循线 I2C（I2C0，0x12）。 */
extern const APP_TASK_DESC APP_CHK_TB6612;        /**< TB6612 电机通道脉冲（主动）。 */
extern const APP_TASK_DESC APP_CHK_ENCODER;       /**< 霍尔编码器计数/速度/里程。 */
extern const APP_TASK_DESC APP_CHK_SPEED_PID;     /**< 双轮速度闭环 PID（主动，整定用）。 */
extern const APP_TASK_DESC APP_CHK_DUTY_SWEEP;    /**< 开环占空比扫描, 查各占空比下编码器读速。 */

#ifdef __cplusplus
}
#endif

#endif /* APP_CHECKS_H */

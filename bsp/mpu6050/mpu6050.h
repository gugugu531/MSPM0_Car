/**
 * @file  mpu6050.h
 * @brief BSP MPU6050 六轴 IMU 驱动 (I2C0, 地址 0x68)。
 *
 * 移植自 Arduino i2cdevlib (Jeff Rowberg) MPU6050 库。分步实现:
 *   Step1 (本文件): 阻塞 I2C 底层 + 基础配置 + 原始 6 轴读取 + WHO_AM_I 自检。
 *   后续: DMP 固件加载 + FIFO 四元数 → yaw/pitch/roll 融合姿态。
 *
 * ⚠ 与 JY61P 同在 I2C0 总线 (地址不同: MPU6050=0x68 / JY61P=0x50)。本驱动用阻塞 I2C,
 *   而 JY61P 现为异步中断驱动且独占 I2C0_IRQHandler; 二者同时主动发起事务会在总线上冲突。
 *   集成时须二选一 (停用 JY61P 驱动, 或加总线仲裁)。本驱动的读取仅可在线程上下文调用,
 *   不可在 ISR 内调用 (阻塞)。
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "bsp_common.h"

#include <stdbool.h>
#include <stdint.h>

/* 7-bit I2C 地址 (AD0=低, Arduino 例程默认)。 */
#define MPU6050_I2C_ADDR_7BIT 0x68U

/**
 * @brief 六轴原始读数 (加速度计 LSB / 陀螺 LSB, 量程见 Init)。
 */
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6050_MOTION6;

/**
 * @brief DMP 融合姿态 (yaw/pitch/roll 单位 deg) + 陀螺 (deg/s)。
 */
typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float gyro_x_deg_s;
    float gyro_y_deg_s;
    float gyro_z_deg_s;
} MPU6050_ATTITUDE;

/**
 * @brief 基础初始化: 时钟源=X 陀螺 PLL, 陀螺 ±250°/s, 加速度 ±2g, 退出睡眠。
 * @note 阻塞, 开机调用一次。
 */
BSP_STATUS MPU6050_Init(void);

/**
 * @brief 连通性自检: 读 WHO_AM_I 设备 ID, 期望 0x34。
 * @return true = 检测到 MPU6050。
 */
bool MPU6050_TestConnection(void);

/**
 * @brief 读取一帧原始六轴数据 (ACCEL_XOUT_H 起 14 字节)。
 * @note 阻塞; 仅可在线程上下文调用。
 */
BSP_STATUS MPU6050_GetMotion6(MPU6050_MOTION6 *out);

/**
 * @brief 加载并初始化 DMP (MotionApps v2.0): 复位→写固件/配置→采样率/量程→
 *        FIFO/DMP 使能序列。完成后 DMP 处于关闭态, 由 MPU6050_SetDMPEnabled(true) 启动。
 * @return 0 成功; 1 固件写校验失败; 2 配置写校验失败; 3 等待 FIFO 超时。
 * @note 阻塞 (~秒级), 仅开机调用一次; 与 JY61P 共用 I2C0, 集成时须二选一。
 */
uint8_t MPU6050_DmpInitialize(void);

/**
 * @brief 使能/关闭 DMP (USER_CTRL.DMP_EN)。DMP 初始化后调 true 开始出 FIFO 姿态包。
 */
void MPU6050_SetDMPEnabled(bool enable);

/**
 * @brief 从 DMP FIFO 取最新一帧姿态 (四元数解算 yaw/pitch/roll + 陀螺 deg/s)。
 * @return BSP_STATUS_OK 取到新帧; BSP_STATUS_NOT_READY 无满包或 FIFO 溢出已复位;
 *         BSP_STATUS_NULL/ERROR 参数/总线错误。
 * @note 阻塞(~1ms), 仅可在线程上下文调用; 无新包时保持上一次姿态由调用方处理。
 *       会自动丢弃陈旧包只保留最新, 并处理 FIFO 溢出(复位)。
 */
BSP_STATUS MPU6050_DmpGetAttitude(MPU6050_ATTITUDE *out);

#endif /* MPU6050_H */

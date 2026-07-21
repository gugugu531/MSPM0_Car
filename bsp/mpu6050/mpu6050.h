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

#endif /* MPU6050_H */

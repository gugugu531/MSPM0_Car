/**
 * @file  mpu6050.h
 * @brief BSP MPU6050 六轴 IMU 驱动 (I2C0, 地址 0x68)。
 *
 * 移植自 Arduino i2cdevlib (Jeff Rowberg) MPU6050 库，当前同时提供：
 *   - 基础模式：±2g / ±250°/s 配置、原始六轴、温度、物理量和静态倾角；
 *   - DMP 模式：MotionApps v2.0 固件加载、FIFO 四元数和 yaw/pitch/roll 姿态。
 *
 * ⚠ 与 JY61P(0x50)、感为灰度(0x4F)和 Yahboom 循线(0x12)共用 I2C0。本驱动使用
 *   阻塞 I2C，而 JY61P 是中断驱动；app 调用本驱动前必须挂起 JY61P，结束后再恢复。
 *   本驱动只能在线程上下文调用，不可在 ISR 内调用。
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
 * @brief 基础采样换算后的物理量（量程由 MPU6050_Init 固定为 ±2g/±250°/s）。
 *
 * pitch/roll 仅由当前加速度方向估算；存在明显线性加速度时不代表真实姿态。
 */
typedef struct {
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float accel_magnitude_mps2;
    float gyro_x_deg_s;
    float gyro_y_deg_s;
    float gyro_z_deg_s;
    float temperature_c;
    float pitch_deg;
    float roll_deg;
} MPU6050_MEASUREMENT;

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
 * @brief 一次读取加速度、温度和角速度，并换算基础物理量与静态倾角。
 * @note 仅适用于 MPU6050_Init() 建立的 ±2g / ±250°/s 基础模式。
 *       MPU6050_DmpInitialize() 会把陀螺量程切到 ±2000°/s；其后若要再次调用本接口，
 *       必须先重新执行 MPU6050_Init() 恢复基础量程。
 */
BSP_STATUS MPU6050_GetMeasurement(MPU6050_MEASUREMENT *out);

/**
 * @brief 加载并初始化 DMP (MotionApps v2.0): 复位→写固件/配置→采样率/量程→
 *        FIFO/DMP 使能序列。完成后 DMP 处于关闭态, 由 MPU6050_SetDMPEnabled(true) 启动。
 * @return 0 成功; 1 固件写校验失败; 2 配置写校验失败; 3 等待 FIFO 超时。
 * @note 阻塞（秒级），仅适合初始化/bring-up；调用期间必须独占共享 I2C0。
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

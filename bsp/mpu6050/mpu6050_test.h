/**
 * @file  mpu6050_test.h
 * @brief MPU6050 DMP 自检/测试入口 (bring-up 用)。
 */
#ifndef MPU6050_TEST_H
#define MPU6050_TEST_H

/**
 * @brief MPU6050 DMP 阻塞自检: WHO_AM_I → dmpInitialize → 使能 → 循环经 debug 串口
 *        打印 yaw/pitch/roll 与 gz。
 *
 * ⚠ 本函数【阻塞不返回】(循环打印, 复位退出), 仅供 bring-up 手动调用。
 * ⚠ MPU6050 与 JY61P 共用 I2C0: 调用前须停用 JY61P 轮询 (注释掉 SysTick 里的
 *   JY61P_I2C_Poll 或不初始化 JY61P), 否则两驱动会在总线上冲突。
 * 用法: 在 main 初始化后 (SYSCFG/BSP_Time/串口就绪、__enable_irq 之后) 调用一次。
 */
void MPU6050_RunDmpTest(void);

#endif /* MPU6050_TEST_H */

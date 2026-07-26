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
 * ⚠ WHO_AM_I 或 DMP 初始化失败时返回；初始化成功后进入循环打印，只能复位退出。
 * ⚠ MPU6050 与 JY61P 共用 I2C0：调用前须执行 JY61P_I2C_SetSuspended(true)，
 *   或保证 JY61P 尚未初始化；本 bring-up 函数不会替调用者管理总线所有权。
 * 用法: 在 main 初始化后 (SYSCFG/BSP_Time/串口就绪、__enable_irq 之后) 调用一次。
 */
void MPU6050_RunDmpTest(void);

#endif /* MPU6050_TEST_H */

/**
 * @file  mpu6050_dmp_fw.h
 * @brief MPU6050 DMP 固件数据 (MotionApps v2.0) 声明。
 *
 * 数据体见 mpu6050_dmp_fw.c, 从 Arduino i2cdevlib 原样抽取。
 */
#ifndef MPU6050_DMP_FW_H
#define MPU6050_DMP_FW_H

#include <stdint.h>

#define MPU6050_DMP_CODE_SIZE    1929U   /* dmpMemory: DMP 固件代码 */
#define MPU6050_DMP_CONFIG_SIZE  192U    /* dmpConfig: 配置集 */
#define MPU6050_DMP_UPDATES_SIZE 47U     /* dmpUpdates: 运行期更新集 */

extern const uint8_t mpu6050_dmp_memory[MPU6050_DMP_CODE_SIZE];
extern const uint8_t mpu6050_dmp_config[MPU6050_DMP_CONFIG_SIZE];
extern const uint8_t mpu6050_dmp_updates[MPU6050_DMP_UPDATES_SIZE];

#endif /* MPU6050_DMP_FW_H */

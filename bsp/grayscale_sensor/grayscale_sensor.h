/**
 * @file  grayscale_sensor.h
 * @brief BSP 8 路光敏灰度传感器数字量接口。
 */
#ifndef GRAYSCALE_SENSOR_H
#define GRAYSCALE_SENSOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRAYSCALE_SENSOR_CHANNEL_COUNT 8U

/* 引脚映射到 SysConfig 生成的板级名 (逻辑通道 0..7 对应物理 Tracking_8..1, 见 .c)。 */
#define GRAYSCALE_SENSOR_1_PORT     Tracking_Tracking_1_PORT
#define GRAYSCALE_SENSOR_1_PIN      Tracking_Tracking_1_PIN
#define GRAYSCALE_SENSOR_2_PORT     Tracking_Tracking_2_PORT
#define GRAYSCALE_SENSOR_2_PIN      Tracking_Tracking_2_PIN
#define GRAYSCALE_SENSOR_3_PORT     Tracking_Tracking_3_PORT
#define GRAYSCALE_SENSOR_3_PIN      Tracking_Tracking_3_PIN
#define GRAYSCALE_SENSOR_4_PORT     Tracking_Tracking_4_PORT
#define GRAYSCALE_SENSOR_4_PIN      Tracking_Tracking_4_PIN
#define GRAYSCALE_SENSOR_5_PORT     Tracking_Tracking_5_PORT
#define GRAYSCALE_SENSOR_5_PIN      Tracking_Tracking_5_PIN
#define GRAYSCALE_SENSOR_6_PORT     Tracking_Tracking_6_PORT
#define GRAYSCALE_SENSOR_6_PIN      Tracking_Tracking_6_PIN
#define GRAYSCALE_SENSOR_7_PORT     Tracking_Tracking_7_PORT
#define GRAYSCALE_SENSOR_7_PIN      Tracking_Tracking_7_PIN
#define GRAYSCALE_SENSOR_8_PORT     Tracking_Tracking_8_PORT
#define GRAYSCALE_SENSOR_8_PIN      Tracking_Tracking_8_PIN

/* 有效电平: 1 = 低电平有效 (多数数字灰度模块), BSP 内翻转为"有效为 1"。 */
#define GRAYSCALE_SENSOR_ACTIVE_LOW 1U

/**
 * @brief 灰度传感器通道编号。
 */
typedef enum {
    GRAYSCALE_SENSOR_CHANNEL_0 = 0,
    GRAYSCALE_SENSOR_CHANNEL_1,
    GRAYSCALE_SENSOR_CHANNEL_2,
    GRAYSCALE_SENSOR_CHANNEL_3,
    GRAYSCALE_SENSOR_CHANNEL_4,
    GRAYSCALE_SENSOR_CHANNEL_5,
    GRAYSCALE_SENSOR_CHANNEL_6,
    GRAYSCALE_SENSOR_CHANNEL_7,
    GRAYSCALE_SENSOR_CHANNEL_MAX
} GRAYSCALE_SENSOR_CHANNEL;

/**
 * @brief 读取全部 8 路灰度传感器数字状态。
 * @param digital_array 输出数组，长度必须不小于 GRAYSCALE_SENSOR_CHANNEL_COUNT。
 * @note 输出值已经根据 GRAYSCALE_SENSOR_ACTIVE_LOW 转换为“有效为 1”。
 */
void GrayscaleSensor_Read(uint8_t digital_array[GRAYSCALE_SENSOR_CHANNEL_COUNT]);

/**
 * @brief 读取 8 路灰度传感器位掩码。
 * @return bit0 对应通道 0，置 1 表示该通道有效。
 */
uint8_t GrayscaleSensor_ReadMask(void);

/**
 * @brief 读取单个灰度传感器通道。
 * @return 1 表示有效，0 表示无效或通道非法。
 */
uint8_t GrayscaleSensor_ReadSingle(GRAYSCALE_SENSOR_CHANNEL channel);

#ifdef __cplusplus
}
#endif

#endif /* GRAYSCALE_SENSOR_H */

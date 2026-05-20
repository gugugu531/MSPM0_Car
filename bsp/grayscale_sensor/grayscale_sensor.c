/**
 * @file  grayscale_sensor.c
 * @brief BSP 8 路光敏灰度传感器数字量实现。
 */
#include "grayscale_sensor.h"
#include <stddef.h>

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} GRAYSCALE_SENSOR_HW_CONFIG;

static const GRAYSCALE_SENSOR_HW_CONFIG s_grayscale_sensor_hw[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {
    [GRAYSCALE_SENSOR_CHANNEL_0] = {
        .port = GRAYSCALE_SENSOR_8_PORT,
        .pin = GRAYSCALE_SENSOR_8_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_1] = {
        .port = GRAYSCALE_SENSOR_7_PORT,
        .pin = GRAYSCALE_SENSOR_7_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_2] = {
        .port = GRAYSCALE_SENSOR_6_PORT,
        .pin = GRAYSCALE_SENSOR_6_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_3] = {
        .port = GRAYSCALE_SENSOR_5_PORT,
        .pin = GRAYSCALE_SENSOR_5_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_4] = {
        .port = GRAYSCALE_SENSOR_4_PORT,
        .pin = GRAYSCALE_SENSOR_4_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_5] = {
        .port = GRAYSCALE_SENSOR_3_PORT,
        .pin = GRAYSCALE_SENSOR_3_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_6] = {
        .port = GRAYSCALE_SENSOR_2_PORT,
        .pin = GRAYSCALE_SENSOR_2_PIN,
    },
    [GRAYSCALE_SENSOR_CHANNEL_7] = {
        .port = GRAYSCALE_SENSOR_1_PORT,
        .pin = GRAYSCALE_SENSOR_1_PIN,
    },
};

uint8_t GrayscaleSensor_ReadSingle(GRAYSCALE_SENSOR_CHANNEL channel){
    if (channel >= GRAYSCALE_SENSOR_CHANNEL_MAX){
        return 0U;
    }

    const GRAYSCALE_SENSOR_HW_CONFIG *hw = &s_grayscale_sensor_hw[channel];
    uint8_t high_level = (DL_GPIO_readPins(hw->port, hw->pin) != 0U) ? 1U : 0U;

    if (GRAYSCALE_SENSOR_ACTIVE_LOW != 0U){
        return (uint8_t)(!high_level);
    }

    return high_level;
}

void GrayscaleSensor_Read(uint8_t digital_array[GRAYSCALE_SENSOR_CHANNEL_COUNT]){
    if (digital_array == NULL){
        return;
    }

    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++){
        digital_array[i] = GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i);
    }
}

uint8_t GrayscaleSensor_ReadMask(void){
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++){
        if (GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i) != 0U){
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

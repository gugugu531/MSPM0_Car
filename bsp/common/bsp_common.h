/**
 * @file  bsp_common.h
 * @brief BSP 公共类型定义
 */
#ifndef BSP_COMMON_H
#define BSP_COMMON_H

#include <stdint.h>

typedef enum {
    BSP_STATUS_OK = 0,
    BSP_STATUS_ERROR = -1,
    BSP_STATUS_NULL = -2,
    BSP_STATUS_INVALID_ARG = -3,
    BSP_STATUS_TIMEOUT = -4,
    BSP_STATUS_BUSY = -5,
    BSP_STATUS_NOT_READY = -6,
} BSP_STATUS;

typedef struct {
    float x;
    float y;
} BSP_POINT2F;

typedef struct {
    float yaw;
    float pitch;
} BSP_ATTITUDE2F;

#endif /* BSP_COMMON_H */

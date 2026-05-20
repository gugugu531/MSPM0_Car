/**
 * @file  canmv_uart.h
 * @brief BSP CanMV UART 协议解析接口。
 */
#ifndef CANMV_UART_H
#define CANMV_UART_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CANMV_UART_INST
#define CANMV_UART_INST UART2
#endif

#ifndef CANMV_UART_IRQN
#define CANMV_UART_IRQN UART2_INT_IRQn
#endif

#ifndef CANMV_RX_BUFFER_LEN
#define CANMV_RX_BUFFER_LEN 100U
#endif

#ifndef CANMV_TARGET_VALUE_CAPACITY
#define CANMV_TARGET_VALUE_CAPACITY 10U
#endif

#define CANMV_FRAME_START 0x12U
#define CANMV_FRAME_END   0x5BU

#define CANMV_LASER_BEGIN      2U
#define CANMV_LASER_BYTE_COUNT 8U

#define CANMV_RECT_BEGIN      10U
#define CANMV_RECT_BYTE_COUNT 16U

#define CANMV_MIN_FRAME_LEN (CANMV_RECT_BEGIN + CANMV_RECT_BYTE_COUNT + 1U)

typedef enum {
    CANMV_TARGET_LASER = 0,
    CANMV_TARGET_RECT,
    CANMV_TARGET_MAX
} CANMV_TARGET;

typedef enum {
    CANMV_STATUS_INIT = -1,
    CANMV_STATUS_OK = 0,
    CANMV_STATUS_NOT_FOUND = 1,
    CANMV_STATUS_LOST = 2,
    CANMV_STATUS_FRAME_DROP = 3,
} CANMV_STATUS;

typedef struct {
    uint16_t value[CANMV_TARGET_VALUE_CAPACITY];
    uint8_t count;
    CANMV_STATUS status;
} CANMV_TARGET_DATA;

BSP_STATUS CanMvUart_Init(void);

void CanMvUart_ProcessByte(uint8_t byte);
void CanMvUart_ProcessRx(void);

BSP_STATUS CanMvUart_SendByte(uint8_t byte);
BSP_STATUS CanMvUart_SendString(const char *str);

CANMV_STATUS CanMvUart_GetStatus(CANMV_TARGET target);
uint8_t CanMvUart_GetData(CANMV_TARGET target, uint16_t *out, uint8_t max_count);
const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target);

#ifdef __cplusplus
}
#endif

#endif /* CANMV_UART_H */

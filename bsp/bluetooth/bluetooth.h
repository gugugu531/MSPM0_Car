/**
 * @file  bluetooth.h
 * @brief BSP 蓝牙串口 (BlueTooth/UART0, 9600 8N1) 收发。
 *
 * 用于蓝牙串口通信测试(收发 ASCII 字符串)。
 *   接收: RX 中断把字节存入环形缓冲, 线程侧非阻塞取。
 *   发送: 直接写 TX FIFO(与 TI uart_echo_interrupts 例程一致), FIFO 满时有界自旋等空位。
 * 波特率由 SysConfig 配为 9600。UART0 中断入口 BlueTooth_INST_IRQHandler 只处理 RX。
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化蓝牙接收: 清缓冲, 设 RX FIFO 阈值并使能 RX 中断。
 * @note  须在 SYSCFG_DL_init() 之后调用(UART 外设由其初始化)。
 */
void BlueTooth_Init(void);

/**
 * @brief 关闭蓝牙 RX 中断(测试退出时收尾)。
 */
void BlueTooth_Deinit(void);

/**
 * @brief 非阻塞取出已收字节到 buf(最多 max 个), 返回实际取出数。
 */
uint16_t BlueTooth_Read(uint8_t *buf, uint16_t max);

/**
 * @brief 非阻塞发送若干字节: 写入发送环形缓冲(满则丢弃剩余并计数), 由 TX 中断慢慢发出。
 */
void BlueTooth_Write(const uint8_t *data, uint16_t len);

/**
 * @brief 非阻塞发送以 '\0' 结尾的字符串。
 */
void BlueTooth_Puts(const char *s);

/**
 * @brief 自 Init 起累计收到的字节数。
 */
uint32_t BlueTooth_GetRxCount(void);

/**
 * @brief 环形缓冲满而丢弃的累计字节数(接收过载诊断)。
 */
uint32_t BlueTooth_GetDroppedBytes(void);

/**
 * @brief UART RX 错误(溢出/break/帧/校验)累计次数。上电期线路噪声会使其非零, 属正常。
 */
uint32_t BlueTooth_GetRxErrors(void);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_H */

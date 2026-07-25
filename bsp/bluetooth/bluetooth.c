/**
 * @file  bluetooth.c
 * @brief BSP 蓝牙串口收发实现 (BlueTooth/UART0)。
 *
 * 接收: RX 中断把字节存入环形缓冲, 线程侧非阻塞取。
 * 发送: 直接写 TX FIFO(与 TI uart_echo_interrupts 例程一致)——transmitData 即写 FIFO、
 *       硬件自动发出, TX 中断只是流控通知、基础发送不需要它。FIFO 满时有界自旋等空位,
 *       绝不无限阻塞。发送量小(测试用), 短暂占用可接受。
 */
#include "bluetooth.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

/* 蓝牙串口实例 (BlueTooth/UART0, 见 board/sys_config)。 */
#define BT_INST BlueTooth_INST

/* RX 环形缓冲长度, 必须是 2 的幂(用掩码回绕)。 */
#define BT_RX_BUF_LEN 256U
#define BT_RX_MASK (BT_RX_BUF_LEN - 1U)

/* 发送时等 TX FIFO 空位的自旋上限(有界, 防 FIFO 永满时死等)。 */
#define BT_TX_SPIN_MAX 100000U

static uint8_t rx_buf[BT_RX_BUF_LEN];
static volatile uint16_t rx_head;    /* 生产者(RX ISR) 写入位置 */
static volatile uint16_t rx_tail;    /* 消费者(线程) 读取位置 */
static volatile uint32_t rx_total;   /* 自 Init 起累计收到字节数 */
static volatile uint32_t rx_dropped; /* 接收缓冲满丢弃的字节数 */

void BlueTooth_Init(void){
    rx_head = 0U;
    rx_tail = 0U;
    rx_total = 0U;
    rx_dropped = 0U;
    /* RX FIFO 到 1 个字节即触发中断; 使能 RX 中断并放开 NVIC 线(发送不用中断)。 */
    DL_UART_Main_setRXFIFOThreshold(BT_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(BT_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(BlueTooth_INST_INT_IRQN);
    NVIC_EnableIRQ(BlueTooth_INST_INT_IRQN);
}

void BlueTooth_Deinit(void){
    DL_UART_Main_disableInterrupt(BT_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_DisableIRQ(BlueTooth_INST_INT_IRQN);
}

uint16_t BlueTooth_Read(uint8_t *buf, uint16_t max){
    uint16_t n = 0U;

    if (buf == NULL){
        return 0U;
    }
    while ((n < max) && (rx_tail != rx_head)){
        buf[n] = rx_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & BT_RX_MASK);
        n++;
    }
    return n;
}

void BlueTooth_Write(const uint8_t *data, uint16_t len){
    if (data == NULL){
        return;
    }
    for (uint16_t i = 0U; i < len; i++){
        /* 等 TX FIFO 有空位(有界自旋), 再直接写入——硬件自动发出。 */
        uint32_t spin = BT_TX_SPIN_MAX;
        while (DL_UART_Main_isTXFIFOFull(BT_INST) && (spin > 0U)){
            spin--;
        }
        DL_UART_Main_transmitData(BT_INST, data[i]);
    }
}

void BlueTooth_Puts(const char *s){
    uint16_t n = 0U;

    if (s == NULL){
        return;
    }
    while (s[n] != '\0'){
        n++;
    }
    BlueTooth_Write((const uint8_t *)s, n);
}

uint32_t BlueTooth_GetRxCount(void){
    return rx_total;
}

uint32_t BlueTooth_GetDroppedBytes(void){
    return rx_dropped;
}

/*
 * 蓝牙 (BlueTooth/UART0) 中断入口。只处理 RX: 把 RX FIFO 字节尽数搬入接收环形缓冲
 * (满则丢弃并计数, 绝不阻塞)。发送为直接写 FIFO, 不走中断。
 */
void BlueTooth_INST_IRQHandler(void){
    switch (DL_UART_Main_getPendingInterrupt(BT_INST)){
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(BT_INST)){
                uint8_t byte = DL_UART_Main_receiveData(BT_INST);
                uint16_t next = (uint16_t)((rx_head + 1U) & BT_RX_MASK);
                if (next == rx_tail){
                    rx_dropped++;   /* 缓冲满: 丢弃该字节。 */
                } else {
                    rx_buf[rx_head] = byte;
                    rx_head = next;
                    rx_total++;
                }
            }
            break;
        default:
            break;
    }
}

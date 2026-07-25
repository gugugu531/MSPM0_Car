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

/* RX 侧错误中断集合: 置位后须清除并排空, 否则 FIFO 满会永久丢弃后续数据(见 Init 注释)。 */
#define BT_RX_ERROR_INTERRUPTS                     \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |        \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR   |        \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |        \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR)

static uint8_t rx_buf[BT_RX_BUF_LEN];
static volatile uint16_t rx_head;    /* 生产者(RX ISR) 写入位置 */
static volatile uint16_t rx_tail;    /* 消费者(线程) 读取位置 */
static volatile uint32_t rx_total;   /* 自 Init 起累计收到字节数 */
static volatile uint32_t rx_dropped; /* 接收缓冲满丢弃的字节数 */
static volatile uint32_t rx_errors;  /* UART RX 错误(溢出/break/帧/校验)次数 */

/* 排空 RX FIFO 并丢弃其中数据(读 RXDATA 会一并带走该条目的错误标志)。 */
static void BlueTooth_FlushRx(void){
    while (!DL_UART_Main_isRXFIFOEmpty(BT_INST)){
        (void)DL_UART_Main_receiveData(BT_INST);
    }
}

void BlueTooth_Init(void){
    rx_head = 0U;
    rx_tail = 0U;
    rx_total = 0U;
    rx_dropped = 0U;
    rx_errors = 0U;

    /*
     * 冷上电健壮性(关键): UART0 在 SYSCFG_DL_init() 就已使能并开始接收, 而本函数要等
     * 进入任务才被调用。这期间蓝牙模块正在自身上电——其 TX 处于高阻/输出垃圾, 会把 RX FIFO
     * 灌满并置位 OVRERR。TI 文档: FIFO 满后新数据被直接丢弃, 必须 CPU 读走才能恢复接收。
     * 故这里主动排空 FIFO + 清错误/中断标志, 不依赖"使能中断后靠中断自举冲走积压"。
     * (按复位键时模块已稳定、TX 空闲为高, RX 线干净, 所以复位后一切正常——正是此差异。)
     */
    DL_UART_Main_setRXFIFOThreshold(BT_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    BlueTooth_FlushRx();
    DL_UART_Main_clearInterruptStatus(BT_INST,
        DL_UART_MAIN_INTERRUPT_RX | BT_RX_ERROR_INTERRUPTS);

    /* 先清 NVIC 挂起再使能, 避免把刚使能后立刻置位的 pending 误清掉。 */
    NVIC_ClearPendingIRQ(BlueTooth_INST_INT_IRQN);
    NVIC_EnableIRQ(BlueTooth_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(BT_INST,
        DL_UART_MAIN_INTERRUPT_RX | BT_RX_ERROR_INTERRUPTS);
}

void BlueTooth_Deinit(void){
    DL_UART_Main_disableInterrupt(BT_INST,
        DL_UART_MAIN_INTERRUPT_RX | BT_RX_ERROR_INTERRUPTS);
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

uint32_t BlueTooth_GetRxErrors(void){
    return rx_errors;
}

/*
 * 蓝牙 (BlueTooth/UART0) 中断入口, 处理 RX 与 RX 错误(发送为直接写 FIFO, 不走中断):
 *   RX   : 把 RX FIFO 字节尽数搬入接收环形缓冲(缓冲满则丢弃并计数, 绝不阻塞)。
 *   错误 : 溢出/break/帧/校验错误——排空 FIFO 让接收恢复(FIFO 满则后续数据被硬件丢弃),
 *          读 IIDX 已清该标志, 只累加计数供诊断。
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
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            BlueTooth_FlushRx();   /* 排空以恢复接收 */
            rx_errors++;
            break;
        default:
            break;
    }
}

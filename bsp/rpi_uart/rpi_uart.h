/**
 * @file  rpi_uart.h
 * @brief 树莓派滚球视觉链路，使用 Rpi_UART/UART2（115200 8N1）。
 *
 * UART 中断只把字节和到达时刻写入 RX 环形缓冲；11 字节协议解析、CRC 校验和状态更新
 * 均在 RpiUart_Poll() 的主循环上下文执行，不在 ISR 内做控制逻辑。
 */
#ifndef RPI_UART_H
#define RPI_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RPI_UART_PROTOCOL_VERSION 1U
#define RPI_UART_FRAME_SIZE       11U
#define RPI_UART_DEGRADED_AGE_MS  60U
#define RPI_UART_TIMEOUT_MS       200U
#define RPI_UART_XFER_MS          0.955f

#define RPI_UART_FLAG_VALID       (1U << 0)
#define RPI_UART_FLAG_MOVING      (1U << 1)
#define RPI_UART_FLAG_V_VALID     (1U << 2)
#define RPI_UART_FLAG_EDGE        (1U << 3)
#define RPI_UART_FLAG_QUALITY_MASK (3U << 4)

typedef struct {
    uint8_t  seq;
    uint8_t  age_ms;
    int16_t  x_01mm;
    int16_t  velocity_mm_s;
    uint8_t  flags;
    uint32_t rx_end_ms;
    bool     velocity_trusted;
} RPI_UART_MEASUREMENT;

typedef struct {
    uint32_t rx_frames;
    uint32_t crc_fail;
    uint32_t ver_fail;
    uint32_t seq_gap;
    uint32_t invalid;
    uint32_t uart_rx_bytes;
    uint32_t uart_rx_dropped;
    uint32_t uart_errors;
    uint16_t max_age_ms;
    uint16_t frame_rate_x10;
} RPI_UART_STATS;

typedef struct {
    float x_mm;
    float velocity_mm_s;
    float age_ms;
    bool  degraded;
    bool  hold_output;
    bool  moving;
    bool  edge;
    bool  velocity_trusted;
    uint8_t quality;
} RPI_UART_PREDICTION;

/** 初始化 UART2 RX 中断，并复位解析器和诊断状态。 */
void RpiUart_Init(void);

/** 取出并解析当前排队的全部字节；须由主循环周期调用。 */
void RpiUart_Poll(void);

/** 清零计数器和帧率窗口，但不丢弃上一次有效测量。 */
void RpiUart_ResetStats(void);

/** 读取上一次 VALID 测量；无效帧绝不会覆盖它。 */
bool RpiUart_GetLatest(RPI_UART_MEASUREMENT *measurement);

/** 读取协议层与 UART 硬件层诊断统计。 */
void RpiUart_GetStats(RPI_UART_STATS *stats);

/**
 * 按 a=(5/7)g*sin(theta) 把球状态前推到当前时刻。
 * 超过 200 ms 没有 VALID 帧时返回 false；调用方必须关闭球环并让摆杆回水平角。
 * 最近一帧 VALID=0 时 hold_output=true，调用方必须保持上一次控制输出。
 * 速度不可信时，前推会去掉测量速度项，只保留已知倾角产生的模型项。
 */
bool RpiUart_Predict(float theta_rad, RPI_UART_PREDICTION *prediction);

/** CRC-8：多项式 0x07、初值 0xFF、不反转、无最终异或；公开以便跑固定测试向量。 */
uint8_t RpiUart_Crc8(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* RPI_UART_H */

/**
 * @file  cy_z.h
 * @brief 创源 CY-Z 串口陀螺仪协议驱动（CY_Z/UART3，115200 8N1）。
 */
#ifndef CY_Z_H
#define CY_Z_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t sequence;
    float angle_deg;
    float gyro_deg_s;
    uint32_t timestamp_ms;
    uint32_t frame_count;
} CY_Z_SAMPLE;

typedef struct {
    uint8_t command;
    uint8_t result;
    uint8_t sequence;
    uint32_t ack_count;
} CY_Z_ACK;

typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t ack_frames;
    uint32_t crc_errors;
    uint32_t discarded_bytes;
    uint32_t rx_dropped_bytes;
    uint32_t uart_errors;
} CY_Z_DIAG;

enum {
    CY_Z_RESULT_OK = 0x00U,
    CY_Z_RESULT_NOT_STATIC = 0x01U,
    CY_Z_RESULT_BAD_COMMAND = 0x02U,
    CY_Z_RESULT_FAILED = 0x03U,
};

/** 清空解析状态并初始化 UART3 RX 中断。须在 SYSCFG_DL_init() 后调用。 */
void CyZ_Init(void);
/** 关闭 UART3 RX 中断；退出 CY-Z 测试页时调用。 */
void CyZ_Deinit(void);
/** 从 UART3 RX 环形缓冲取数并推进解析器。 */
void CyZ_Poll(void);
/** 获取最近的有效遥测快照。 */
bool CyZ_GetSnapshot(CY_Z_SAMPLE *out);
/** 获取最近 ACK；只有收到过 ACK 时返回 true。 */
bool CyZ_GetAck(CY_Z_ACK *out);
/** 最近遥测是否存在且未超过 max_age_ms。 */
bool CyZ_IsFresh(uint32_t max_age_ms);
/** 获取协议解析诊断计数。 */
void CyZ_GetDiag(CY_Z_DIAG *out);

/** 角度清零；发送时模块必须静止。 */
void CyZ_SendZeroAngle(uint8_t sequence);
/** 重新估计零偏并清零；约 2 秒内模块必须保持静止。 */
void CyZ_SendRecalibrateBias(uint8_t sequence);
/** 查询模式下主动请求一帧遥测。 */
void CyZ_RequestTelemetry(uint8_t sequence);

#ifdef __cplusplus
}
#endif

#endif /* CY_Z_H */

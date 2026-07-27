/**
 * @file  cy_z.c
 * @brief 创源 CY-Z 定长串口帧解析与控制命令实现。
 *
 * 遥测：AA 55 seq:u16 angle:f32 gyro:f32 crc:u16 55 AA（小端）。
 * ACK ：A5 5B cmd result seq crc:u16 5B。
 * 两种 CRC 均为 CRC-16/MODBUS，且只覆盖帧中间的数据字段。
 */
#include "cy_z.h"

#include "bsp_time.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <string.h>

#define CY_Z_TELEMETRY_SIZE 16U
#define CY_Z_ACK_SIZE        8U
#define CY_Z_COMMAND_SIZE    8U
#define CY_Z_RX_BUFFER_SIZE  128U
#define CY_Z_RX_BUFFER_MASK  (CY_Z_RX_BUFFER_SIZE - 1U)
#define CY_Z_TX_TIMEOUT      100000U

#define CY_Z_CMD_RESET             0x01U
#define CY_Z_CMD_QUERY_TELEMETRY   0x04U

static uint8_t s_window[CY_Z_TELEMETRY_SIZE];
static uint8_t s_window_count;
static uint8_t s_rx_buffer[CY_Z_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static CY_Z_SAMPLE s_sample;
static CY_Z_ACK s_ack;
static CY_Z_DIAG s_diag;

static uint16_t CyZ_Crc16Modbus(const uint8_t *data, uint8_t length){
    uint16_t crc = 0xFFFFU;

    for (uint8_t i = 0U; i < length; i++){
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++){
            if ((crc & 1U) != 0U){
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else{
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static uint16_t CyZ_ReadU16Le(const uint8_t *data){
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static float CyZ_ReadFloatLe(const uint8_t *data){
    uint32_t raw = (uint32_t)data[0] |
                   ((uint32_t)data[1] << 8U) |
                   ((uint32_t)data[2] << 16U) |
                   ((uint32_t)data[3] << 24U);
    float value = 0.0f;

    if (sizeof(value) == sizeof(raw)){
        memcpy(&value, &raw, sizeof(value));
    }
    return value;
}

static void CyZ_DiscardFirstByte(void){
    for (uint8_t i = 0U; i + 1U < s_window_count; i++){
        s_window[i] = s_window[i + 1U];
    }
    if (s_window_count > 0U){
        s_window_count--;
        s_diag.discarded_bytes++;
    }
}

static bool CyZ_TryParseAck(void){
    uint8_t start;
    uint16_t expected_crc;

    if (s_window_count < CY_Z_ACK_SIZE){
        return false;
    }
    start = (uint8_t)(s_window_count - CY_Z_ACK_SIZE);
    if ((s_window[start] != 0xA5U) ||
        (s_window[start + 1U] != 0x5BU) ||
        (s_window[start + 7U] != 0x5BU)){
        return false;
    }

    expected_crc = CyZ_Crc16Modbus(&s_window[start + 2U], 3U);
    if (CyZ_ReadU16Le(&s_window[start + 5U]) != expected_crc){
        s_diag.crc_errors++;
        return false;
    }

    s_ack.command = s_window[start + 2U];
    s_ack.result = s_window[start + 3U];
    s_ack.sequence = s_window[start + 4U];
    s_ack.ack_count++;
    s_diag.ack_frames++;
    s_window_count = 0U;
    return true;
}

static bool CyZ_TryParseTelemetry(void){
    uint16_t expected_crc;

    if (s_window_count < CY_Z_TELEMETRY_SIZE){
        return false;
    }
    if ((s_window[0] != 0xAAU) || (s_window[1] != 0x55U) ||
        (s_window[14] != 0x55U) || (s_window[15] != 0xAAU)){
        return false;
    }

    expected_crc = CyZ_Crc16Modbus(&s_window[2], 10U);
    if (CyZ_ReadU16Le(&s_window[12]) != expected_crc){
        s_diag.crc_errors++;
        return false;
    }

    s_sample.sequence = CyZ_ReadU16Le(&s_window[2]);
    s_sample.angle_deg = CyZ_ReadFloatLe(&s_window[4]);
    s_sample.gyro_deg_s = CyZ_ReadFloatLe(&s_window[8]);
    s_sample.timestamp_ms = BSP_Time_GetMs();
    s_sample.frame_count++;
    s_diag.valid_frames++;
    s_window_count = 0U;
    return true;
}

static void CyZ_PushByte(uint8_t byte){
    if (s_window_count >= CY_Z_TELEMETRY_SIZE){
        CyZ_DiscardFirstByte();
    }
    s_window[s_window_count++] = byte;

    if (CyZ_TryParseAck()){
        return;
    }
    if (s_window_count >= CY_Z_TELEMETRY_SIZE){
        if (!CyZ_TryParseTelemetry()){
            CyZ_DiscardFirstByte();
        }
    }
}

static void CyZ_DrainHardwareRx(void){
    while (!DL_UART_Main_isRXFIFOEmpty(CY_Z_INST)){
        (void)DL_UART_Main_receiveData(CY_Z_INST);
    }
}

static uint16_t CyZ_ReadRx(uint8_t *data, uint16_t max_length){
    uint16_t count = 0U;

    while ((count < max_length) && (s_rx_tail != s_rx_head)){
        data[count++] = s_rx_buffer[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) & CY_Z_RX_BUFFER_MASK);
    }
    return count;
}

static void CyZ_Write(const uint8_t *data, uint8_t length){
    for (uint8_t i = 0U; i < length; i++){
        uint32_t timeout = CY_Z_TX_TIMEOUT;
        while (DL_UART_Main_isTXFIFOFull(CY_Z_INST)){
            if (timeout-- == 0U){
                return;
            }
        }
        DL_UART_Main_transmitData(CY_Z_INST, data[i]);
    }
}

static void CyZ_SendCommand(uint8_t command, uint8_t parameter, uint8_t sequence){
    uint8_t frame[CY_Z_COMMAND_SIZE];
    uint16_t crc;

    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = command;
    frame[3] = parameter;
    frame[4] = sequence;
    crc = CyZ_Crc16Modbus(&frame[2], 3U);
    frame[5] = (uint8_t)(crc & 0xFFU);
    frame[6] = (uint8_t)(crc >> 8U);
    frame[7] = 0x5AU;
    CyZ_Write(frame, CY_Z_COMMAND_SIZE);
}

void CyZ_Init(void){
    NVIC_DisableIRQ(CY_Z_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(CY_Z_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR);

    memset(s_window, 0, sizeof(s_window));
    s_window_count = 0U;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_sample = (CY_Z_SAMPLE){0};
    s_ack = (CY_Z_ACK){0};
    s_diag = (CY_Z_DIAG){0};

    DL_GPIO_setDigitalInternalResistor(
        GPIO_CY_Z_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);
    DL_UART_Main_setRXFIFOThreshold(CY_Z_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    CyZ_DrainHardwareRx();
    DL_UART_Main_clearInterruptStatus(CY_Z_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
    DL_UART_Main_enableInterrupt(CY_Z_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
    NVIC_SetPriority(CY_Z_INST_INT_IRQN, 1U);
    NVIC_ClearPendingIRQ(CY_Z_INST_INT_IRQN);
    NVIC_EnableIRQ(CY_Z_INST_INT_IRQN);
}

void CyZ_Deinit(void){
    NVIC_DisableIRQ(CY_Z_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(CY_Z_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
    CyZ_DrainHardwareRx();
    NVIC_ClearPendingIRQ(CY_Z_INST_INT_IRQN);
}

void CyZ_Poll(void){
    uint8_t bytes[32];
    uint16_t count;

    do {
        count = CyZ_ReadRx(bytes, (uint16_t)sizeof(bytes));
        for (uint16_t i = 0U; i < count; i++){
            CyZ_PushByte(bytes[i]);
        }
    } while (count == sizeof(bytes));
}

bool CyZ_GetSnapshot(CY_Z_SAMPLE *out){
    if (out == NULL){
        return false;
    }
    *out = s_sample;
    return s_sample.frame_count != 0U;
}

bool CyZ_GetAck(CY_Z_ACK *out){
    if (out == NULL){
        return false;
    }
    *out = s_ack;
    return s_ack.ack_count != 0U;
}

bool CyZ_IsFresh(uint32_t max_age_ms){
    return (s_sample.frame_count != 0U) &&
           ((uint32_t)(BSP_Time_GetMs() - s_sample.timestamp_ms) <= max_age_ms);
}

void CyZ_GetDiag(CY_Z_DIAG *out){
    if (out != NULL){
        *out = s_diag;
    }
}

void CyZ_SendZeroAngle(uint8_t sequence){
    CyZ_SendCommand(CY_Z_CMD_RESET, 0x01U, sequence);
}

void CyZ_SendRecalibrateBias(uint8_t sequence){
    CyZ_SendCommand(CY_Z_CMD_RESET, 0x02U, sequence);
}

void CyZ_RequestTelemetry(uint8_t sequence){
    CyZ_SendCommand(CY_Z_CMD_QUERY_TELEMETRY, 0x00U, sequence);
}

void CY_Z_INST_IRQHandler(void){
    switch (DL_UART_Main_getPendingInterrupt(CY_Z_INST)){
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(CY_Z_INST)){
                uint8_t byte = DL_UART_Main_receiveData(CY_Z_INST);
                uint16_t next = (uint16_t)((s_rx_head + 1U) & CY_Z_RX_BUFFER_MASK);
                s_diag.rx_bytes++;
                if (next == s_rx_tail){
                    s_diag.rx_dropped_bytes++;
                } else{
                    s_rx_buffer[s_rx_head] = byte;
                    s_rx_head = next;
                }
            }
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            CyZ_DrainHardwareRx();
            s_diag.uart_errors++;
            break;

        default:
            break;
    }
}

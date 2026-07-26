/**
 * @file  yahboom_track.h
 * @brief BSP Yahboom 8 路循线模块 I2C 驱动（I2C0，地址 0x12）。
 *
 * 协议原始状态字节为 bit7=X1 ... bit0=X8，且 0=黑线/指示灯亮、1=白底/指示灯灭。
 * YahboomTrack_ReadDetectedMask() 会将其转换为应用友好的检测掩码：bit0=X1 ... bit7=X8，
 * 1=检测到黑线。注意 GPIO GrayscaleSensor_ReadMask() 的置 1 语义相反，两者并非统一 ABI。
 *
 * 与 JY61P、MPU6050、感为灰度共用 I2C0。本驱动为阻塞式，仅可在线程上下文调用；
 * 调用期间必须由上层挂起 JY61P 的异步 I2C 状态机。
 */
#ifndef YAHBOOM_TRACK_H
#define YAHBOOM_TRACK_H

#include "bsp_common.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define YAHBOOM_TRACK_CHANNEL_COUNT 8U

#ifndef YAHBOOM_TRACK_I2C_ADDR_7BIT
#define YAHBOOM_TRACK_I2C_ADDR_7BIT 0x12U
#endif

/**
 * @brief 初始化并尝试读取状态寄存器，确认模块能应答。
 * @return BSP_STATUS_OK 表示模块在线；否则返回总线错误或超时。
 * @note 不会执行官方建议的 20 秒探头预热等待，也不会自动进入校准模式。
 */
BSP_STATUS YahboomTrack_Init(void);

/**
 * @brief 读取协议原始状态字节（寄存器 0x30）。
 * @param raw 输出：bit7=X1 ... bit0=X8；0=黑线/灯亮，1=白底/灯灭。
 */
BSP_STATUS YahboomTrack_ReadRaw(uint8_t *raw);

/**
 * @brief 读取归一化黑线检测掩码。
 * @param mask 输出：bit0=X1 ... bit7=X8；1=检测到黑线，0=白底。
 */
BSP_STATUS YahboomTrack_ReadDetectedMask(uint8_t *mask);

/**
 * @brief 写校准控制寄存器 0x01。
 * @param enabled true 写 1 进入校准，false 写 0 退出校准。
 * @note 进入后仍须按 Yahboom 官方流程，用板载按键分别记录黑线与白底。
 */
BSP_STATUS YahboomTrack_SetCalibration(bool enabled);

/**
 * @brief 获取驱动诊断计数。
 * @param read_fail   输出：读寄存器失败累计次数（可为 NULL）。
 * @param write_fail  输出：写寄存器失败累计次数（可为 NULL）。
 * @param last_status 输出：最近一次失败状态（可为 NULL）。
 * @note 计数在 YahboomTrack_Init() 时清零。
 */
void YahboomTrack_GetDiag(uint32_t *read_fail, uint32_t *write_fail,
                          int32_t *last_status);

#ifdef __cplusplus
}
#endif

#endif /* YAHBOOM_TRACK_H */

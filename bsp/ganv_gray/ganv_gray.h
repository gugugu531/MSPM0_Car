/**
 * @file  ganv_gray.h
 * @brief BSP 感为(GANV)8 路灰度传感器 I2C 驱动 (I2C0, 默认地址 0x4F)。
 *
 * 感为八路灰度经 I2C 输出:主用途是一次读回 8 路数字量(每 bit 一路),辅以 ping/版本/
 * 错误/重启等诊断。协议里"命令符"即寄存器:写 1 字节命令 + REPEATED START 读 N 字节;
 * 开机默认命令为 0xDD(数字量),故也可直接读。校准为设备端按键操作(白/黑场),无 I2C 命令。
 *
 * ⚠ 与 MPU6050(0x68)/JY61P(0x50) 同在 I2C0 总线(SysConfig 实例 MPU6050_JY61P_Tracking)。
 *   本驱动为阻塞 I2C,仅可在线程上下文调用,不可在 ISR 内调用;与 JY61P 异步中断驱动同时
 *   主动发起事务会在总线上冲突,须由上层分时(与 MPU6050 自检同法:进挂起/出恢复对方)。
 */
#ifndef GANV_GRAY_H
#define GANV_GRAY_H

#include "bsp_common.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GANV_GRAY_CHANNEL_COUNT 8U

/*
 * 7 位从机地址:高 5bit 软件地址位出厂为 0b10011,低 2bit 由板上 AD1/AD0 跳线帽决定。
 *   AD1=AD0=1(双跳线帽) → 0x4F;都不装 → 0x4C。改跳线帽后覆盖此宏即可。
 *   实测无需查表:Init 会 ping,返回 0x66 即地址正确。
 */
#ifndef GANV_GRAY_I2C_ADDR_7BIT
#define GANV_GRAY_I2C_ADDR_7BIT 0x4FU
#endif

/**
 * @brief 初始化并与传感器同步(有限次 ping 等待其上电就绪)。
 * @return BSP_STATUS_OK 收到在线应答(0x66);未就绪返回 BSP_STATUS_TIMEOUT,总线故障返回 ERROR。
 * @note I2C0 外设本体由 SysConfig(SYSCFG_DL_init)初始化,本函数只做上电同步,不重配外设。
 *       灰度为可选外设,建议调用方将失败作为可恢复处理,不进入致命停机。
 */
BSP_STATUS GanvGray_Init(void);

/**
 * @brief ping 探测传感器是否在线(命令 0xAA,期望返回 0x66)。
 * @return BSP_STATUS_OK 在线;应答值不符返回 BSP_STATUS_NOT_READY;总线故障返回对应错误码。
 */
BSP_STATUS GanvGray_Ping(void);

/**
 * @brief 读取 8 路数字量位掩码(命令 0xDD)。
 * @param mask 输出:bit0=第 1 路 … bit7=第 8 路,置 1 表示该路检测到(设备端 LED 亮)。
 * @return 事务状态;mask 为 NULL 时返回 BSP_STATUS_NULL。
 */
BSP_STATUS GanvGray_ReadDigital(uint8_t *mask);

/**
 * @brief 读取固件版本号(命令 0xC1)。高 4bit.低 4bit,如 0x3E 表示 V3.14。
 */
BSP_STATUS GanvGray_ReadVersion(uint8_t *version);

/**
 * @brief 读取错误寄存器(命令 0xDE)。bit1=按键长期短路,bit0=对管过曝;读后设备自动清零。
 */
BSP_STATUS GanvGray_ReadError(uint8_t *err);

/**
 * @brief 触发设备软件重启(命令 0xC0,纯写无应答)。重启后设备需重新就绪,调用方应随后 Init/Ping。
 */
BSP_STATUS GanvGray_Reboot(void);

#ifdef __cplusplus
}
#endif

#endif /* GANV_GRAY_H */

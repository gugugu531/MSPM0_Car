/**
 * @file  ganv_gray.c
 * @brief BSP 感为(GANV)8 路灰度传感器 I2C 驱动实现 (I2C0, 阻塞)。
 *
 * 底层 I2C 事务复用 MSPM0 DriverLib 阻塞 FIFO 模式,与同总线的 mpu6050 阻塞驱动同构:
 *   - 读:写 1 字节命令(抑制 STOP)→ REPEATED START → 读 N 字节;
 *   - 写:仅发 1 字节命令 + STOP(用于软件重启 0xC0)。
 */
#include "ganv_gray.h"

#include "ti_msp_dl_config.h"
#include "bsp_time.h"

#include <stddef.h>
#include <stdbool.h>

/* I2C 外设实例(I2C0,与 MPU6050/JY61P 共用)与自旋超时。 */
#define GANV_GRAY_I2C_INST     MPU6050_JY61P_Tracking_INST
#define GANV_GRAY_I2C_TIMEOUT  100000U
/* 上电同步:每次 ping 间隔 1ms,最多重试 100 次(约 100ms)后判超时。 */
#define GANV_GRAY_PING_RETRY   100U

/* ===== 命令符(见手册 7.17)===== */
#define GANV_GRAY_CMD_DIGITAL  0xDDU   /* 读 8 路数字量,1 字节。 */
#define GANV_GRAY_CMD_PING     0xAAU   /* ping,期望应答 0x66。 */
#define GANV_GRAY_PING_REPLY   0x66U
#define GANV_GRAY_CMD_VERSION  0xC1U   /* 固件版本号。 */
#define GANV_GRAY_CMD_ERROR    0xDEU   /* 错误寄存器(读后自清零)。 */
#define GANV_GRAY_CMD_REBOOT   0xC0U   /* 软件重启(纯写,无应答)。 */

/* ═══════════════════ 阻塞 I2C 底层(勿在 ISR 调用)═══════════════════════════ */

/* 等控制器回到空闲,超时则复位控制器发 STOP。 */
static bool GanvGray_WaitIdle(void)
{
    uint32_t timeout = GANV_GRAY_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(GANV_GRAY_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(GANV_GRAY_I2C_INST);
            return false;
        }
    }
    return true;
}

/* 读: TX CMD(STOP_DISABLE) → REPEATED START → RX DATA[0..len-1]。 */
static BSP_STATUS GanvGray_ReadCmd(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    if ((data == NULL) || (len == 0U)){ return BSP_STATUS_INVALID_ARG; }

    DL_I2C_flushControllerRXFIFO(GANV_GRAY_I2C_INST);
    DL_I2C_flushControllerTXFIFO(GANV_GRAY_I2C_INST);
    if (!GanvGray_WaitIdle()){ return BSP_STATUS_TIMEOUT; }

    DL_I2C_transmitControllerData(GANV_GRAY_I2C_INST, cmd);
    DL_I2C_startControllerTransferAdvanced(GANV_GRAY_I2C_INST, GANV_GRAY_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,   /* 抑制 STOP -> RX 阶段自动 REPEATED START */
        DL_I2C_CONTROLLER_ACK_DISABLE);

    timeout = GANV_GRAY_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(GANV_GRAY_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ return BSP_STATUS_TIMEOUT; }
    }
    if ((DL_I2C_getControllerStatus(GANV_GRAY_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(GANV_GRAY_I2C_INST);
        return BSP_STATUS_ERROR;
    }

    DL_I2C_startControllerTransfer(GANV_GRAY_I2C_INST, GANV_GRAY_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    for (i = 0U; i < len; i++){
        timeout = GANV_GRAY_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(GANV_GRAY_I2C_INST)){
            if (timeout-- == 0U){ return BSP_STATUS_TIMEOUT; }
        }
        data[i] = DL_I2C_receiveControllerData(GANV_GRAY_I2C_INST);
    }
    return GanvGray_WaitIdle() ? BSP_STATUS_OK : BSP_STATUS_TIMEOUT;
}

/* 写: TX CMD + STOP(用于无数据命令,如软件重启)。 */
static BSP_STATUS GanvGray_WriteCmd(uint8_t cmd)
{
    uint32_t timeout;

    if (!GanvGray_WaitIdle()){ return BSP_STATUS_TIMEOUT; }

    DL_I2C_flushControllerTXFIFO(GANV_GRAY_I2C_INST);
    DL_I2C_transmitControllerData(GANV_GRAY_I2C_INST, cmd);
    DL_I2C_startControllerTransfer(GANV_GRAY_I2C_INST, GANV_GRAY_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    timeout = GANV_GRAY_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(GANV_GRAY_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ return BSP_STATUS_TIMEOUT; }
    }
    if ((DL_I2C_getControllerStatus(GANV_GRAY_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(GANV_GRAY_I2C_INST);
        return BSP_STATUS_ERROR;
    }
    return GanvGray_WaitIdle() ? BSP_STATUS_OK : BSP_STATUS_TIMEOUT;
}

/* ═══════════════════════════ 公开接口 ═══════════════════════════════════════ */

BSP_STATUS GanvGray_Ping(void)
{
    uint8_t reply = 0U;
    BSP_STATUS st = GanvGray_ReadCmd(GANV_GRAY_CMD_PING, &reply, 1U);
    if (st != BSP_STATUS_OK){ return st; }
    return (reply == GANV_GRAY_PING_REPLY) ? BSP_STATUS_OK : BSP_STATUS_NOT_READY;
}

BSP_STATUS GanvGray_Init(void)
{
    /* 主控可能比传感器先就绪:反复 ping 直到收到 0x66,或到达重试上限判超时。 */
    for (uint8_t i = 0U; i < GANV_GRAY_PING_RETRY; i++){
        if (GanvGray_Ping() == BSP_STATUS_OK){
            return BSP_STATUS_OK;
        }
        BSP_DelayMs(1U);
    }
    return BSP_STATUS_TIMEOUT;
}

BSP_STATUS GanvGray_ReadDigital(uint8_t *mask)
{
    if (mask == NULL){ return BSP_STATUS_NULL; }
    return GanvGray_ReadCmd(GANV_GRAY_CMD_DIGITAL, mask, 1U);
}

BSP_STATUS GanvGray_ReadVersion(uint8_t *version)
{
    if (version == NULL){ return BSP_STATUS_NULL; }
    return GanvGray_ReadCmd(GANV_GRAY_CMD_VERSION, version, 1U);
}

BSP_STATUS GanvGray_ReadError(uint8_t *err)
{
    if (err == NULL){ return BSP_STATUS_NULL; }
    return GanvGray_ReadCmd(GANV_GRAY_CMD_ERROR, err, 1U);
}

BSP_STATUS GanvGray_Reboot(void)
{
    return GanvGray_WriteCmd(GANV_GRAY_CMD_REBOOT);
}

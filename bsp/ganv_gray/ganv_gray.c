/**
 * @file  ganv_gray.c
 * @brief BSP 感为(GANV)8 路灰度传感器 I2C 驱动实现 (I2C0, 阻塞)。
 *
 * 底层 I2C 事务用 MSPM0 DriverLib 阻塞 FIFO 模式,采用手册方法 2/3 的「命令与读取分离」:
 *   - 读:写 1 字节命令(独立事务,带 STOP)+ 独立读 N 字节(START+RX+STOP);
 *   - 写:仅发 1 字节命令 + STOP(用于软件重启 0xC0)。
 *
 * 注:早期用 repeated-start(TX 抑制 STOP 后靠 BUSY 捕捉中间态再发 RX)会踩到确定性时序
 * 竞争,使读取隔次交替失败(RETRY=1 约 50%,RETRY=2 才 err=0)。改为两个各带 STOP 的完整
 * 独立事务后,每次收尾干净,不再交替;重试仅作真·瞬态兜底。详见 git 历史。
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
/* 命令切换(写命令帧)的重试次数。高频读命中命令缓存不写命令,仅切换命令时才写,
 * 故给足重试以兜住写阶段偶发 NACK;读阶段实测可靠,不重试。 */
#define GANV_GRAY_RETRY        3U

/* ===== 命令符(见手册 7.17)===== */
#define GANV_GRAY_CMD_DIGITAL  0xDDU   /* 读 8 路数字量,1 字节。 */
#define GANV_GRAY_CMD_PING     0xAAU   /* ping,期望应答 0x66。 */
#define GANV_GRAY_PING_REPLY   0x66U
#define GANV_GRAY_CMD_VERSION  0xC1U   /* 固件版本号。 */
#define GANV_GRAY_CMD_ERROR    0xDEU   /* 错误寄存器(读后自清零)。 */
#define GANV_GRAY_CMD_REBOOT   0xC0U   /* 软件重启(纯写,无应答)。 */

/* ===== 诊断计数(健康监控,GanvGray_Init 时清零)===== */
static uint32_t diag_wr_fail;   /* 写命令(命令切换)阶段累计失败次数。 */
static uint32_t diag_rd_fail;   /* 读数据阶段累计失败次数。 */
static BSP_STATUS diag_last;    /* 最近一次失败的状态码。 */

/* 传感器"当前命令指针"缓存:手册方法 2/3——写一次命令后可反复读。仅命令变化时写,
 * 连读同命令直接读、跳过写,避免高频写命令触发写阶段隔次 NACK(见诊断: W 涨 / R=0 / NACK)。 */
static uint8_t  current_cmd;
static bool     current_cmd_valid;

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

/* 写: TX CMD + STOP(完整独立事务,用于命令帧或无数据命令如软件重启)。 */
static BSP_STATUS GanvGray_WriteCmd(uint8_t cmd)
{
    uint32_t timeout;

    DL_I2C_flushControllerTXFIFO(GANV_GRAY_I2C_INST);
    if (!GanvGray_WaitIdle()){ return BSP_STATUS_TIMEOUT; }

    DL_I2C_transmitControllerData(GANV_GRAY_I2C_INST, cmd);
    DL_I2C_startControllerTransfer(GANV_GRAY_I2C_INST, GANV_GRAY_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    timeout = GANV_GRAY_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(GANV_GRAY_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(GANV_GRAY_I2C_INST);
            return BSP_STATUS_TIMEOUT;
        }
    }
    if ((DL_I2C_getControllerStatus(GANV_GRAY_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(GANV_GRAY_I2C_INST);
        return BSP_STATUS_ERROR;
    }
    return GanvGray_WaitIdle() ? BSP_STATUS_OK : BSP_STATUS_TIMEOUT;
}

/* 读: START + ADDR(R) + RX DATA[0..len-1] + STOP(完整独立事务)。
 * 数据帧靠 RXFIFO 非空判定,不依赖 BUSY 捕捉中间态,故无隔次交替问题。 */
static BSP_STATUS GanvGray_ReadBytes(uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    DL_I2C_flushControllerRXFIFO(GANV_GRAY_I2C_INST);
    if (!GanvGray_WaitIdle()){ return BSP_STATUS_TIMEOUT; }

    DL_I2C_startControllerTransfer(GANV_GRAY_I2C_INST, GANV_GRAY_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    for (i = 0U; i < len; i++){
        timeout = GANV_GRAY_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(GANV_GRAY_I2C_INST)){
            if (timeout-- == 0U){
                DL_I2C_resetControllerTransfer(GANV_GRAY_I2C_INST);
                return BSP_STATUS_TIMEOUT;
            }
        }
        data[i] = DL_I2C_receiveControllerData(GANV_GRAY_I2C_INST);
    }
    return GanvGray_WaitIdle() ? BSP_STATUS_OK : BSP_STATUS_TIMEOUT;
}

/* 确保传感器当前命令指针 = cmd:命中缓存则不写(手册方法 2/3);否则写命令帧(带重试)。
 * 高频读同一命令时几乎不写命令,从根上避开写阶段隔次 NACK。 */
static BSP_STATUS GanvGray_SelectCmd(uint8_t cmd)
{
    BSP_STATUS st = BSP_STATUS_ERROR;

    if (current_cmd_valid && (current_cmd == cmd)){
        return BSP_STATUS_OK;
    }

    for (uint8_t i = 0U; i < GANV_GRAY_RETRY; i++){
        st = GanvGray_WriteCmd(cmd);
        if (st == BSP_STATUS_OK){
            current_cmd       = cmd;
            current_cmd_valid = true;
            return BSP_STATUS_OK;
        }
    }
    current_cmd_valid = false;   /* 写失败,命令指针未知,下次重选 */
    diag_wr_fail++;
    diag_last = st;
    return st;
}

/* 读命令 = 选择命令(必要时写) + 读数据。读阶段实测可靠,不重试。 */
static BSP_STATUS GanvGray_ReadCmd(uint8_t cmd, uint8_t *data, uint8_t len)
{
    BSP_STATUS st;

    if ((data == NULL) || (len == 0U)){ return BSP_STATUS_INVALID_ARG; }

    st = GanvGray_SelectCmd(cmd);
    if (st != BSP_STATUS_OK){ return st; }

    st = GanvGray_ReadBytes(data, len);
    if (st != BSP_STATUS_OK){
        diag_rd_fail++;
        diag_last = st;
    }
    return st;
}

/* ═══════════════════════════ 公开接口 ═══════════════════════════════════════ */

BSP_STATUS GanvGray_Ping(void)
{
    uint8_t reply = 0U;
    BSP_STATUS st;

    /* ping 有硬件回滚特性(不改持久命令指针):每次显式写 0xAA 并读,绕过命令缓存,
     * 也不更新 current_cmd —— 回滚后指针不变,缓存仍有效。 */
    st = GanvGray_WriteCmd(GANV_GRAY_CMD_PING);
    if (st != BSP_STATUS_OK){ return st; }
    st = GanvGray_ReadBytes(&reply, 1U);
    if (st != BSP_STATUS_OK){ return st; }
    return (reply == GANV_GRAY_PING_REPLY) ? BSP_STATUS_OK : BSP_STATUS_NOT_READY;
}

BSP_STATUS GanvGray_Init(void)
{
    diag_wr_fail = 0U;
    diag_rd_fail = 0U;
    diag_last    = BSP_STATUS_OK;
    current_cmd_valid = false;   /* 命令缓存失效,首次读会显式选命令 */

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
    current_cmd_valid = false;   /* 重启后传感器命令指针复位为默认,缓存失效 */
    return GanvGray_WriteCmd(GANV_GRAY_CMD_REBOOT);
}

void GanvGray_GetDiag(uint32_t *wr_fail, uint32_t *rd_fail, int32_t *last_status)
{
    if (wr_fail != NULL){ *wr_fail = diag_wr_fail; }
    if (rd_fail != NULL){ *rd_fail = diag_rd_fail; }
    if (last_status != NULL){ *last_status = (int32_t)diag_last; }
}

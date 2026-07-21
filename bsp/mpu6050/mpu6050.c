/**
 * @file  mpu6050.c
 * @brief BSP MPU6050 六轴 IMU 驱动实现 (I2C0, 阻塞) — Step1: 基础读取与自检。
 *
 * 移植自 Arduino i2cdevlib (Jeff Rowberg) MPU6050 / I2Cdev:
 *   - 位域读写 (ReadBits/WriteBits) 保持与 I2Cdev 相同的 bitStart=字段高位 约定;
 *   - 底层 I2C 事务用 MSPM0 DriverLib 阻塞 FIFO 模式 (与旧 JY61P 阻塞驱动同构)。
 */
#include "mpu6050.h"
#include "mpu6050_dmp_fw.h"
#include "ti_msp_dl_config.h"
#include "bsp_time.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* I2C 外设实例 (I2C0, 与 JY61P 共用) 与自旋超时。 */
#define MPU6050_I2C_INST     MPU6050_JY61P_Tracking_INST
#define MPU6050_I2C_TIMEOUT  100000U

/* ===== 寄存器地址 ===== */
#define MPU6050_RA_SMPLRT_DIV    0x19U
#define MPU6050_RA_CONFIG        0x1AU
#define MPU6050_RA_GYRO_CONFIG   0x1BU
#define MPU6050_RA_ACCEL_CONFIG  0x1CU
#define MPU6050_RA_ACCEL_XOUT_H  0x3BU
#define MPU6050_RA_PWR_MGMT_1    0x6BU
#define MPU6050_RA_WHO_AM_I      0x75U

/* ===== 位域定义 (bitStart = 字段最高位, 与 I2Cdev 约定一致) ===== */
#define MPU6050_PWR1_SLEEP_BIT       6U
#define MPU6050_PWR1_CLKSEL_BIT      2U
#define MPU6050_PWR1_CLKSEL_LENGTH   3U
#define MPU6050_CLOCK_PLL_XGYRO      0x01U
#define MPU6050_GCONFIG_FS_SEL_BIT   4U
#define MPU6050_GCONFIG_FS_SEL_LEN   2U
#define MPU6050_ACONFIG_AFS_SEL_BIT  4U
#define MPU6050_ACONFIG_AFS_SEL_LEN  2U
#define MPU6050_GYRO_FS_250          0x00U
#define MPU6050_ACCEL_FS_2           0x00U
#define MPU6050_WHO_AM_I_BIT         6U
#define MPU6050_WHO_AM_I_LENGTH      6U
#define MPU6050_DEVICE_ID            0x34U

/* ═══════════════════ 阻塞 I2C 底层 (勿在 ISR 调用) ═══════════════════════════ */

/* 等控制器回到空闲, 超时则复位控制器发 STOP。 */
static bool MPU6050_WaitIdle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
            return false;
        }
    }
    return true;
}

/* 写: START + ADDR(W) + REG + DATA[0..len-1] + STOP。
 * 关键: 先把 REG + 尽量多的数据填入 TX FIFO(8深), 再启动传输, 之后边发边补;
 * 若"先全填再启动", len>7 时 FIFO 会满且不排空(传输未启动)导致死等超时。 */
static bool MPU6050_WriteRegs(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint32_t timeout;
    uint8_t sent = 0U;

    if (!MPU6050_WaitIdle()){ return false; }

    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg);
    while ((sent < len) && !DL_I2C_isControllerTXFIFOFull(MPU6050_I2C_INST)){
        DL_I2C_transmitControllerData(MPU6050_I2C_INST, data[sent++]);
    }

    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, MPU6050_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)(len + 1U));

    while (sent < len){
        timeout = MPU6050_I2C_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(MPU6050_I2C_INST)){
            if (timeout-- == 0U){ return false; }
        }
        DL_I2C_transmitControllerData(MPU6050_I2C_INST, data[sent++]);
    }

    timeout = MPU6050_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ return false; }
    }
    if ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
        return false;
    }
    return MPU6050_WaitIdle();
}

/* 读 (REPEATED START): TX REG (STOP_DISABLE) → RX DATA[0..len-1]。 */
static bool MPU6050_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    if ((data == NULL) || (len == 0U)){ return false; }

    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    if (!MPU6050_WaitIdle()){ return false; }

    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg);
    DL_I2C_startControllerTransferAdvanced(MPU6050_I2C_INST, MPU6050_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,   /* 抑制 STOP -> RX 阶段自动 REPEATED START */
        DL_I2C_CONTROLLER_ACK_DISABLE);

    timeout = MPU6050_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){ return false; }
    }
    if ((DL_I2C_getControllerStatus(MPU6050_I2C_INST)
         & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
        return false;
    }

    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, MPU6050_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    for (i = 0U; i < len; i++){
        timeout = MPU6050_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST)){
            if (timeout-- == 0U){ return false; }
        }
        data[i] = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }
    return MPU6050_WaitIdle();
}

/* ===== I2Cdev 风格便捷读写 (供配置与后续 DMP 复用) ===== */

static bool MPU6050_WriteByte(uint8_t reg, uint8_t val)
{
    return MPU6050_WriteRegs(reg, &val, 1U);
}

static bool MPU6050_ReadByte(uint8_t reg, uint8_t *val)
{
    return MPU6050_ReadRegs(reg, val, 1U);
}

/* 写位域: bitStart=字段高位, length=位宽, data 右对齐 (读-改-写)。 */
static bool MPU6050_WriteBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t b;
    uint8_t mask;

    if (!MPU6050_ReadByte(reg, &b)){ return false; }
    mask = (uint8_t)(((1U << length) - 1U) << (bitStart - length + 1U));
    data = (uint8_t)(data << (bitStart - length + 1U));
    data = (uint8_t)(data & mask);
    b = (uint8_t)(b & (uint8_t)~mask);
    b = (uint8_t)(b | data);
    return MPU6050_WriteByte(reg, b);
}

/* 写单个位。 */
static bool MPU6050_WriteBit(uint8_t reg, uint8_t bitNum, bool value)
{
    uint8_t b;

    if (!MPU6050_ReadByte(reg, &b)){ return false; }
    b = value ? (uint8_t)(b | (1U << bitNum)) : (uint8_t)(b & (uint8_t)~(1U << bitNum));
    return MPU6050_WriteByte(reg, b);
}

/* 读位域: 返回右对齐字段值。 */
static bool MPU6050_ReadBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    uint8_t b;
    uint8_t mask;

    if (!MPU6050_ReadByte(reg, &b)){ return false; }
    mask = (uint8_t)(((1U << length) - 1U) << (bitStart - length + 1U));
    b = (uint8_t)(b & mask);
    b = (uint8_t)(b >> (bitStart - length + 1U));
    *data = b;
    return true;
}

/* ═══════════════════ 对外接口 ═══════════════════════════════════════════════ */

BSP_STATUS MPU6050_Init(void)
{
    bool ok = true;

    /* 与 Arduino MPU6050::initialize() 等价: 时钟源=X陀螺PLL, 陀螺±250, 加速度±2g, 退睡眠。 */
    ok = ok && MPU6050_WriteBits(MPU6050_RA_PWR_MGMT_1,
                                 MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH,
                                 MPU6050_CLOCK_PLL_XGYRO);
    ok = ok && MPU6050_WriteBits(MPU6050_RA_GYRO_CONFIG,
                                 MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LEN,
                                 MPU6050_GYRO_FS_250);
    ok = ok && MPU6050_WriteBits(MPU6050_RA_ACCEL_CONFIG,
                                 MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LEN,
                                 MPU6050_ACCEL_FS_2);
    ok = ok && MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false);

    return ok ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

bool MPU6050_TestConnection(void)
{
    uint8_t id = 0U;

    if (!MPU6050_ReadBits(MPU6050_RA_WHO_AM_I,
                          MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &id)){
        return false;
    }
    return id == MPU6050_DEVICE_ID;
}

BSP_STATUS MPU6050_GetMotion6(MPU6050_MOTION6 *out)
{
    uint8_t buf[14];

    if (out == NULL){ return BSP_STATUS_NULL; }
    if (!MPU6050_ReadRegs(MPU6050_RA_ACCEL_XOUT_H, buf, 14U)){ return BSP_STATUS_ERROR; }

    out->ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    out->ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    out->az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    /* buf[6],buf[7] = 温度, 跳过 */
    out->gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    out->gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    out->gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
    return BSP_STATUS_OK;
}

/* ═══════════════════ DMP (MotionApps v2.0) 初始化 ═══════════════════════════
 * 移植自 Arduino i2cdevlib MPU6050_6Axis_MotionApps20.h 的 dmpInitialize(),
 * 用上面的位域/块读写 helper 内联各寄存器操作 (不逐一造 setter)。仅开机调用。
 * ══════════════════════════════════════════════════════════════════════════ */

/* DMP 相关寄存器 */
#define MPU6050_RA_XG_OFFS_TC     0x00U   /* [7]PWR_MODE [6:1]XG_OFFS_TC [0]OTP_BNK_VLD */
#define MPU6050_RA_YG_OFFS_TC     0x01U
#define MPU6050_RA_ZG_OFFS_TC     0x02U
#define MPU6050_RA_MOT_THR        0x1FU
#define MPU6050_RA_MOT_DUR        0x20U
#define MPU6050_RA_ZRMOT_THR      0x21U
#define MPU6050_RA_ZRMOT_DUR      0x22U
#define MPU6050_RA_I2C_SLV0_ADDR  0x25U
#define MPU6050_RA_INT_ENABLE     0x38U
#define MPU6050_RA_INT_STATUS     0x3AU
#define MPU6050_RA_USER_CTRL      0x6AU
#define MPU6050_RA_BANK_SEL       0x6DU
#define MPU6050_RA_MEM_START_ADDR 0x6EU
#define MPU6050_RA_MEM_R_W        0x6FU
#define MPU6050_RA_DMP_CFG_1      0x70U
#define MPU6050_RA_DMP_CFG_2      0x71U
#define MPU6050_RA_FIFO_COUNTH    0x72U
#define MPU6050_RA_FIFO_R_W       0x74U

/* 位定义 */
#define MPU6050_PWR1_DEVICE_RESET_BIT 7U
#define MPU6050_TC_GYRO_OFFSET_BIT    6U
#define MPU6050_TC_GYRO_OFFSET_LEN    6U
#define MPU6050_TC_OTP_BNK_VLD_BIT    0U
#define MPU6050_CFG_EXT_SYNC_BIT      5U
#define MPU6050_CFG_EXT_SYNC_LEN      3U
#define MPU6050_CFG_DLPF_BIT          2U
#define MPU6050_CFG_DLPF_LEN          3U
#define MPU6050_EXT_SYNC_TEMP_OUT_L   0x01U
#define MPU6050_DLPF_BW_42            0x03U
#define MPU6050_GYRO_FS_2000          0x03U
#define MPU6050_CLOCK_PLL_ZGYRO       0x03U
#define MPU6050_UC_DMP_EN_BIT         7U
#define MPU6050_UC_FIFO_EN_BIT        6U
#define MPU6050_UC_I2C_MST_EN_BIT     5U
#define MPU6050_UC_DMP_RESET_BIT      3U
#define MPU6050_UC_FIFO_RESET_BIT     2U
#define MPU6050_UC_I2C_MST_RESET_BIT  1U
#define MPU6050_DMP_CHUNK             16U
#define MPU6050_DMP_FIFO_WAIT_MS      100U   /* 等 FIFO 出数的超时(防 init 失败时死等) */

/* ── 内存 bank 原语 ── */
static void MPU6050_SetMemoryBank(uint8_t bank, bool prefetch, bool userBank)
{
    bank = (uint8_t)(bank & 0x1FU);
    if (userBank){ bank = (uint8_t)(bank | 0x20U); }
    if (prefetch){ bank = (uint8_t)(bank | 0x40U); }
    (void)MPU6050_WriteByte(MPU6050_RA_BANK_SEL, bank);
}

static void MPU6050_SetMemoryStartAddr(uint8_t addr)
{
    (void)MPU6050_WriteByte(MPU6050_RA_MEM_START_ADDR, addr);
}

static uint8_t MPU6050_ReadMemoryByte(void)
{
    uint8_t v = 0U;
    (void)MPU6050_ReadByte(MPU6050_RA_MEM_R_W, &v);
    return v;
}

/* 写内存块 (16B chunk, 跨 256B bank 边界处理, verify 回读校验)。data 位于 flash。 */
static bool MPU6050_WriteMemoryBlock(const uint8_t *data, uint16_t size,
                                     uint8_t bank, uint8_t addr, bool verify)
{
    uint8_t vbuf[MPU6050_DMP_CHUNK];
    uint16_t i = 0U;

    MPU6050_SetMemoryBank(bank, false, false);
    MPU6050_SetMemoryStartAddr(addr);
    while (i < size){
        uint16_t chunk = MPU6050_DMP_CHUNK;
        if ((uint16_t)(i + chunk) > size){ chunk = (uint16_t)(size - i); }
        if (chunk > (uint16_t)(256U - addr)){ chunk = (uint16_t)(256U - addr); }
        if (!MPU6050_WriteRegs(MPU6050_RA_MEM_R_W, data + i, (uint8_t)chunk)){ return false; }
        if (verify){
            MPU6050_SetMemoryBank(bank, false, false);
            MPU6050_SetMemoryStartAddr(addr);
            if (!MPU6050_ReadRegs(MPU6050_RA_MEM_R_W, vbuf, (uint8_t)chunk)){ return false; }
            if (memcmp(data + i, vbuf, chunk) != 0){ return false; }
        }
        i = (uint16_t)(i + chunk);
        addr = (uint8_t)(addr + chunk);        /* uint8_t 在 256 处回绕到 0 */
        if (i < size){
            if (addr == 0U){ bank++; }
            MPU6050_SetMemoryBank(bank, false, false);
            MPU6050_SetMemoryStartAddr(addr);
        }
    }
    return true;
}

/* 读内存块 (dmpUpdates 第 6 段用)。 */
static bool MPU6050_ReadMemoryBlock(uint8_t *data, uint16_t size, uint8_t bank, uint8_t addr)
{
    uint16_t i = 0U;

    MPU6050_SetMemoryBank(bank, false, false);
    MPU6050_SetMemoryStartAddr(addr);
    while (i < size){
        uint16_t chunk = MPU6050_DMP_CHUNK;
        if ((uint16_t)(i + chunk) > size){ chunk = (uint16_t)(size - i); }
        if (chunk > (uint16_t)(256U - addr)){ chunk = (uint16_t)(256U - addr); }
        if (!MPU6050_ReadRegs(MPU6050_RA_MEM_R_W, data + i, (uint8_t)chunk)){ return false; }
        i = (uint16_t)(i + chunk);
        addr = (uint8_t)(addr + chunk);
        if (i < size){
            if (addr == 0U){ bank++; }
            MPU6050_SetMemoryBank(bank, false, false);
            MPU6050_SetMemoryStartAddr(addr);
        }
    }
    return true;
}

/* 写 DMP 配置集: [bank][offset][length][data...] 序列; length==0 为特殊指令。 */
static bool MPU6050_WriteDMPConfig(const uint8_t *data, uint16_t size)
{
    uint16_t i = 0U;

    while (i < size){
        uint8_t bank = data[i++];
        uint8_t offset = data[i++];
        uint8_t length = data[i++];

        if (length > 0U){
            if (!MPU6050_WriteMemoryBlock(data + i, length, bank, offset, true)){ return false; }
            i = (uint16_t)(i + length);
        } else {
            uint8_t special = data[i++];
            if (special == 0x01U){
                if (!MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, 0x32U)){ return false; }
            } else {
                return false;
            }
        }
    }
    return true;
}

/* ── FIFO / 状态 helper ── */
static void MPU6050_ResetFIFO(void)
{
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_FIFO_RESET_BIT, true);
}

static uint16_t MPU6050_GetFIFOCount(void)
{
    uint8_t b[2] = {0U, 0U};
    (void)MPU6050_ReadRegs(MPU6050_RA_FIFO_COUNTH, b, 2U);
    return (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
}

static bool MPU6050_GetFIFOBytes(uint8_t *data, uint16_t len)
{
    if (len == 0U){ return true; }
    return MPU6050_ReadRegs(MPU6050_RA_FIFO_R_W, data, (uint8_t)len);
}

static uint8_t MPU6050_GetIntStatus(void)
{
    uint8_t v = 0U;
    (void)MPU6050_ReadByte(MPU6050_RA_INT_STATUS, &v);
    return v;
}

/* 等 FIFO 达到 min 字节, 带超时 (参考代码为无超时死等, 此处防 init 失败挂死)。 */
static bool MPU6050_WaitFIFO(uint16_t min_bytes)
{
    uint32_t start = BSP_Time_GetMs();
    while (MPU6050_GetFIFOCount() < min_bytes){
        if ((uint32_t)(BSP_Time_GetMs() - start) >= MPU6050_DMP_FIFO_WAIT_MS){
            return false;
        }
    }
    return true;
}

/* 读一段 dmpUpdates: 3 字节头(bank/offset/length) + length 字节数据 到 upd[]。 */
static uint16_t MPU6050_LoadDmpUpdate(uint8_t *upd, uint16_t pos)
{
    uint8_t j;
    for (j = 0U; (j < 4U) || (j < (uint8_t)(upd[2] + 3U)); j++, pos++){
        upd[j] = mpu6050_dmp_updates[pos];
    }
    return pos;
}

uint8_t MPU6050_DmpInitialize(void)
{
    uint8_t fifo[128];
    uint8_t upd[16];
    uint8_t tc = 0U;
    int8_t xgTC, ygTC, zgTC;
    uint16_t pos;
    uint16_t count;
    uint8_t u;

    /* 复位 → 退睡眠 */
    (void)MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
    BSP_DelayMs(30U);
    (void)MPU6050_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false);

    /* 读硬件版本 (仅按参考流程走内存 bank 选择, 结果不使用) */
    MPU6050_SetMemoryBank(0x10U, true, true);
    MPU6050_SetMemoryStartAddr(0x06U);
    (void)MPU6050_ReadMemoryByte();
    MPU6050_SetMemoryBank(0U, false, false);

    /* 保存 X/Y/Z gyro offset TC (后面写回) */
    (void)MPU6050_ReadBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_GYRO_OFFSET_BIT, MPU6050_TC_GYRO_OFFSET_LEN, &tc); xgTC = (int8_t)tc;
    (void)MPU6050_ReadBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_GYRO_OFFSET_BIT, MPU6050_TC_GYRO_OFFSET_LEN, &tc); ygTC = (int8_t)tc;
    (void)MPU6050_ReadBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_GYRO_OFFSET_BIT, MPU6050_TC_GYRO_OFFSET_LEN, &tc); zgTC = (int8_t)tc;

    /* slave / I2C master 复位序列 */
    (void)MPU6050_WriteByte(MPU6050_RA_I2C_SLV0_ADDR, 0x7FU);
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_I2C_MST_EN_BIT, false);
    (void)MPU6050_WriteByte(MPU6050_RA_I2C_SLV0_ADDR, 0x68U);
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_I2C_MST_RESET_BIT, true);
    BSP_DelayMs(20U);

    /* 写 DMP 固件与配置 */
    if (!MPU6050_WriteMemoryBlock(mpu6050_dmp_memory, MPU6050_DMP_CODE_SIZE, 0U, 0U, true)){
        return 1U;
    }
    if (!MPU6050_WriteDMPConfig(mpu6050_dmp_config, MPU6050_DMP_CONFIG_SIZE)){
        return 2U;
    }

    /* 时钟=Z陀螺 / 中断使能 / 200Hz / 外同步 / DLPF42 / 陀螺±2000 / DMP cfg */
    (void)MPU6050_WriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_ZGYRO);
    (void)MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, 0x12U);
    (void)MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, 4U);
    (void)MPU6050_WriteBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_BIT, MPU6050_CFG_EXT_SYNC_LEN, MPU6050_EXT_SYNC_TEMP_OUT_L);
    (void)MPU6050_WriteBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_BIT, MPU6050_CFG_DLPF_LEN, MPU6050_DLPF_BW_42);
    (void)MPU6050_WriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LEN, MPU6050_GYRO_FS_2000);
    (void)MPU6050_WriteByte(MPU6050_RA_DMP_CFG_1, 0x03U);
    (void)MPU6050_WriteByte(MPU6050_RA_DMP_CFG_2, 0x00U);

    /* 清 OTP bank valid, 写回 gyro offset TC */
    (void)MPU6050_WriteBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, false);
    (void)MPU6050_WriteBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_GYRO_OFFSET_BIT, MPU6050_TC_GYRO_OFFSET_LEN, (uint8_t)xgTC);
    (void)MPU6050_WriteBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_GYRO_OFFSET_BIT, MPU6050_TC_GYRO_OFFSET_LEN, (uint8_t)ygTC);
    (void)MPU6050_WriteBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_GYRO_OFFSET_BIT, MPU6050_TC_GYRO_OFFSET_LEN, (uint8_t)zgTC);

    /* dmpUpdates: 依次 7 段 */
    pos = 0U;
    pos = MPU6050_LoadDmpUpdate(upd, pos);   /* 1 */
    (void)MPU6050_WriteMemoryBlock(upd + 3, upd[2], upd[0], upd[1], true);
    pos = MPU6050_LoadDmpUpdate(upd, pos);   /* 2 */
    (void)MPU6050_WriteMemoryBlock(upd + 3, upd[2], upd[0], upd[1], true);

    MPU6050_ResetFIFO();
    count = MPU6050_GetFIFOCount();
    if (count > sizeof(fifo)){ count = sizeof(fifo); }
    (void)MPU6050_GetFIFOBytes(fifo, count);

    (void)MPU6050_WriteByte(MPU6050_RA_MOT_THR, 2U);
    (void)MPU6050_WriteByte(MPU6050_RA_ZRMOT_THR, 156U);
    (void)MPU6050_WriteByte(MPU6050_RA_MOT_DUR, 80U);
    (void)MPU6050_WriteByte(MPU6050_RA_ZRMOT_DUR, 0U);

    MPU6050_ResetFIFO();
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_FIFO_EN_BIT, true);
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_DMP_EN_BIT, true);
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_DMP_RESET_BIT, true);

    for (u = 0U; u < 3U; u++){   /* 3,4,5 */
        pos = MPU6050_LoadDmpUpdate(upd, pos);
        (void)MPU6050_WriteMemoryBlock(upd + 3, upd[2], upd[0], upd[1], true);
    }

    if (!MPU6050_WaitFIFO(3U)){ return 3U; }
    count = MPU6050_GetFIFOCount();
    if (count > sizeof(fifo)){ count = sizeof(fifo); }
    (void)MPU6050_GetFIFOBytes(fifo, count);
    (void)MPU6050_GetIntStatus();

    pos = MPU6050_LoadDmpUpdate(upd, pos);   /* 6 (读) */
    (void)MPU6050_ReadMemoryBlock(upd + 3, upd[2], upd[0], upd[1]);

    if (!MPU6050_WaitFIFO(3U)){ return 3U; }
    count = MPU6050_GetFIFOCount();
    if (count > sizeof(fifo)){ count = sizeof(fifo); }
    (void)MPU6050_GetFIFOBytes(fifo, count);
    (void)MPU6050_GetIntStatus();

    pos = MPU6050_LoadDmpUpdate(upd, pos);   /* 7 (写) */
    (void)MPU6050_WriteMemoryBlock(upd + 3, upd[2], upd[0], upd[1], true);

    /* 关 DMP (运行时再开), 清 FIFO/中断 */
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_DMP_EN_BIT, false);
    MPU6050_ResetFIFO();
    (void)MPU6050_GetIntStatus();
    return 0U;
}

void MPU6050_SetDMPEnabled(bool enable)
{
    (void)MPU6050_WriteBit(MPU6050_RA_USER_CTRL, MPU6050_UC_DMP_EN_BIT, enable);
}

/* ═══════════════════ DMP 运行期姿态 (四元数 → yaw/pitch/roll) ═══════════════ */

#define MPU6050_DMP_PACKET_SIZE 42U
#define MPU6050_RAD_TO_DEG      57.29577951f
#define MPU6050_GYRO_LSB_2000   16.4f   /* ±2000°/s 量程下 LSB → deg/s */

/* 从 42B DMP 包的偏移解出带符号 int16 (大端)。 */
static int16_t MPU6050_PacketI16(const uint8_t *pkt, uint8_t hi)
{
    return (int16_t)(((uint16_t)pkt[hi] << 8) | pkt[hi + 1U]);
}

BSP_STATUS MPU6050_DmpGetAttitude(MPU6050_ATTITUDE *out)
{
    uint8_t pkt[MPU6050_DMP_PACKET_SIZE];
    uint8_t int_status;
    uint16_t count;
    float qw, qx, qy, qz;
    float grav_x, grav_y, grav_z;

    if (out == NULL){ return BSP_STATUS_NULL; }

    int_status = MPU6050_GetIntStatus();
    count = MPU6050_GetFIFOCount();

    /* FIFO 溢出: 复位, 本次无有效帧。 */
    if (((int_status & 0x10U) != 0U) || (count == 1024U)){
        MPU6050_ResetFIFO();
        return BSP_STATUS_NOT_READY;
    }
    if (count < MPU6050_DMP_PACKET_SIZE){
        return BSP_STATUS_NOT_READY;   /* 尚无完整包 */
    }

    /* 追新: 丢弃陈旧包, 只保留最新一包。 */
    while (count >= (uint16_t)(2U * MPU6050_DMP_PACKET_SIZE)){
        if (!MPU6050_GetFIFOBytes(pkt, MPU6050_DMP_PACKET_SIZE)){ return BSP_STATUS_ERROR; }
        count = (uint16_t)(count - MPU6050_DMP_PACKET_SIZE);
    }
    if (!MPU6050_GetFIFOBytes(pkt, MPU6050_DMP_PACKET_SIZE)){ return BSP_STATUS_ERROR; }

    /* 四元数 (int16/16384)。 */
    qw = (float)MPU6050_PacketI16(pkt, 0U)  / 16384.0f;
    qx = (float)MPU6050_PacketI16(pkt, 4U)  / 16384.0f;
    qy = (float)MPU6050_PacketI16(pkt, 8U)  / 16384.0f;
    qz = (float)MPU6050_PacketI16(pkt, 12U) / 16384.0f;

    /* 重力向量。 */
    grav_x = 2.0f * (qx * qz - qw * qy);
    grav_y = 2.0f * (qw * qx + qy * qz);
    grav_z = qw * qw - qx * qx - qy * qy + qz * qz;

    /* yaw/pitch/roll (rad → deg)。 */
    out->yaw_deg = atan2f(2.0f * qx * qy - 2.0f * qw * qz,
                          2.0f * qw * qw + 2.0f * qx * qx - 1.0f) * MPU6050_RAD_TO_DEG;
    out->pitch_deg = atanf(grav_x / sqrtf(grav_y * grav_y + grav_z * grav_z)) * MPU6050_RAD_TO_DEG;
    out->roll_deg = atanf(grav_y / sqrtf(grav_x * grav_x + grav_z * grav_z)) * MPU6050_RAD_TO_DEG;

    /* 陀螺原始 (int16, ±2000°/s) → deg/s。 */
    out->gyro_x_deg_s = (float)MPU6050_PacketI16(pkt, 16U) / MPU6050_GYRO_LSB_2000;
    out->gyro_y_deg_s = (float)MPU6050_PacketI16(pkt, 20U) / MPU6050_GYRO_LSB_2000;
    out->gyro_z_deg_s = (float)MPU6050_PacketI16(pkt, 24U) / MPU6050_GYRO_LSB_2000;

    return BSP_STATUS_OK;
}

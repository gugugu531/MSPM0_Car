/**
 * @file  mpu6050.c
 * @brief BSP MPU6050 六轴 IMU 驱动实现 (I2C0, 阻塞) — Step1: 基础读取与自检。
 *
 * 移植自 Arduino i2cdevlib (Jeff Rowberg) MPU6050 / I2Cdev:
 *   - 位域读写 (ReadBits/WriteBits) 保持与 I2Cdev 相同的 bitStart=字段高位 约定;
 *   - 底层 I2C 事务用 MSPM0 DriverLib 阻塞 FIFO 模式 (与旧 JY61P 阻塞驱动同构)。
 */
#include "mpu6050.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

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

/* 写: START + ADDR(W) + REG + DATA[0..len-1] + STOP。 */
static bool MPU6050_WriteRegs(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint32_t timeout;

    if (!MPU6050_WaitIdle()){ return false; }

    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_transmitControllerData(MPU6050_I2C_INST, reg);
    for (i = 0U; i < len; i++){
        uint32_t tx_timeout = MPU6050_I2C_TIMEOUT;
        while (DL_I2C_isControllerTXFIFOFull(MPU6050_I2C_INST)){
            if (tx_timeout-- == 0U){ return false; }
        }
        DL_I2C_transmitControllerData(MPU6050_I2C_INST, data[i]);
    }

    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, MPU6050_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)(len + 1U));

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

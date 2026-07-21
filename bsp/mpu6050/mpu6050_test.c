/**
 * @file  mpu6050_test.c
 * @brief MPU6050 DMP 自检/测试实现 (bring-up)。
 */
#include "mpu6050_test.h"
#include "mpu6050.h"
#include "debug_uart.h"
#include "bsp_time.h"

#define MPU6050_TEST_PERIOD_MS 50U

void MPU6050_RunDmpTest(void)
{
    MPU6050_ATTITUDE att;
    uint8_t rc;

    DebugUart_Puts("\r\n[MPU6050] DMP self-test start\r\n");
    DebugUart_Puts("[MPU6050] note: JY61P shares I2C0, disable its poll first!\r\n");

    if (!MPU6050_TestConnection()){
        DebugUart_Puts("[MPU6050] WHO_AM_I FAIL (check wiring/addr/bus conflict)\r\n");
        return;
    }
    DebugUart_Puts("[MPU6050] connected. dmpInitialize (~1s)...\r\n");

    rc = MPU6050_DmpInitialize();
    DebugUart_Printf("[MPU6050] dmpInitialize rc=%u (0=OK,1=fw,2=cfg,3=fifo timeout)\r\n",
                     (unsigned)rc);
    if (rc != 0U){
        DebugUart_Puts("[MPU6050] DMP init failed, abort\r\n");
        return;
    }

    MPU6050_SetDMPEnabled(true);
    DebugUart_Puts("[MPU6050] DMP enabled, streaming ypr (reset board to stop)...\r\n");

    for (;;){
        if (MPU6050_DmpGetAttitude(&att) == BSP_STATUS_OK){
            DebugUart_Printf("[MPU6050] yaw=%7.2f pitch=%7.2f roll=%7.2f  gz=%7.1f\r\n",
                             att.yaw_deg, att.pitch_deg, att.roll_deg, att.gyro_z_deg_s);
        }
        BSP_DelayMs(MPU6050_TEST_PERIOD_MS);
    }
}

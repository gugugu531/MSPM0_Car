/**
 * @file vision_line_drive.h
 * @brief K230 红线位置/方向融合循迹，含 1 s 角速度起步与 UART 帧解析。
 */
#ifndef VISION_LINE_DRIVE_H
#define VISION_LINE_DRIVE_H

#include "bsp_common.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_LINE_FRAME_SIZE                 10U
#define VISION_LINE_FRAME_MAX_AGE_MS           120U
#define VISION_LINE_IMU_MAX_AGE_MS             60U
#define VISION_LINE_MIN_CONFIDENCE              60U
#define VISION_LINE_BASE_DUTY_PERCENT          80.0f
#define VISION_LINE_STARTUP_DURATION_MS        1000U

/** K230 误差融合为目标角速度：位置已归一化到 [-1,1]，方向单位 deg。 */
#define VISION_LINE_POSITION_RATE_KP           55.0f
#define VISION_LINE_HEADING_RATE_KP             1.4f
#define VISION_LINE_OMEGA_LIMIT_DEG_S         100.0f
#define VISION_LINE_STEERING_SIGN               1.0f
#define VISION_LINE_GYRO_SIGN                   1.0f

#define VISION_LINE_RATE_PID_KP                 0.20f
#define VISION_LINE_RATE_PID_KI                 0.0f
#define VISION_LINE_RATE_PID_KD                 0.0f
#define VISION_LINE_RATE_PID_INTEGRAL_LIMIT   100.0f
#define VISION_LINE_RATE_PID_OUTPUT_LIMIT      24.0f

typedef enum {
    VISION_LINE_PHASE_WAIT_IMU = 0,
    VISION_LINE_PHASE_STARTUP_RATE,
    VISION_LINE_PHASE_TRACK
} VISION_LINE_PHASE;

typedef struct {
    VISION_LINE_PHASE phase;
    bool imu_ready;
    bool frame_fresh;
    bool track_valid;
    uint8_t sequence;
    uint8_t confidence;
    uint32_t frame_age_ms;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t sequence_drops;
    float position_error;
    float heading_error_deg;
    float gyro_z_deg_s;
    float omega_reference_deg_s;
    float correction_percent;
    float left_duty_percent;
    float right_duty_percent;
} VISION_LINE_OUTPUT;

void VisionLineDrive_Init(void);
/** 从 DebugUart RX 环形缓冲取字节、推进控制并至多写一次底盘。 */
BSP_STATUS VisionLineDrive_Update(float dt_s);
void VisionLineDrive_Stop(void);
VISION_LINE_OUTPUT VisionLineDrive_GetOutput(void);

/** 公开单字节入口便于协议离线测试；正常运行由 Update 自动调用。 */
void VisionLineDrive_IngestByte(uint8_t byte, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* VISION_LINE_DRIVE_H */

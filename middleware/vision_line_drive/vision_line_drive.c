/**
 * @file vision_line_drive.c
 * @brief K230 红线循迹控制与 A5 5A 定长帧解析实现。
 */
#include "vision_line_drive.h"

#include "bsp_time.h"
#include "chassis.h"
#include "debug_uart.h"
#include "kinematics/kinematics.h"
#include "pid/pid.h"
#include "wit_sdk.h"

#include <string.h>

#define VISION_FRAME_START0 0xA5U
#define VISION_FRAME_START1 0x5AU
#define VISION_FLAG_VALID   0x01U

static VISION_LINE_OUTPUT vision_output;
static PID_CONTROLLER rate_pid;
static uint8_t frame_buffer[VISION_LINE_FRAME_SIZE];
static uint8_t frame_index;
static uint32_t startup_begin_ms;
static uint32_t last_frame_ms;
static bool have_frame;
static bool have_sequence;
static uint8_t last_sequence;

static float VisionLine_Clamp(float value, float low, float high){
    if (value < low){ return low; }
    if (value > high){ return high; }
    return value;
}

static int16_t VisionLine_ReadI16Be(const uint8_t *data){
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void VisionLine_ResetControl(void){
    PID_Reset(&rate_pid);
    startup_begin_ms = 0U;
    vision_output.phase = VISION_LINE_PHASE_WAIT_IMU;
    vision_output.imu_ready = false;
    vision_output.omega_reference_deg_s = 0.0f;
    vision_output.correction_percent = 0.0f;
    vision_output.left_duty_percent = 0.0f;
    vision_output.right_duty_percent = 0.0f;
}

static bool VisionLine_FrameChecksumOk(const uint8_t *frame){
    uint8_t checksum = 0U;
    for (uint8_t i = 0U; i < VISION_LINE_FRAME_SIZE - 1U; i++){
        checksum = (uint8_t)(checksum + frame[i]);
    }
    return checksum == frame[VISION_LINE_FRAME_SIZE - 1U];
}

static void VisionLine_AcceptFrame(uint32_t now_ms){
    uint8_t sequence = frame_buffer[2];
    uint8_t flags = frame_buffer[3];

    if (have_sequence){
        uint8_t expected = (uint8_t)(last_sequence + 1U);
        vision_output.sequence_drops += (uint8_t)(sequence - expected);
    }
    have_sequence = true;
    last_sequence = sequence;
    vision_output.sequence = sequence;
    vision_output.track_valid = (flags & VISION_FLAG_VALID) != 0U;
    vision_output.position_error =
        (float)VisionLine_ReadI16Be(&frame_buffer[4]) * 0.001f;
    vision_output.heading_error_deg =
        (float)VisionLine_ReadI16Be(&frame_buffer[6]) * 0.01f;
    vision_output.confidence = frame_buffer[8];
    vision_output.valid_frames++;
    last_frame_ms = now_ms;
    have_frame = true;
}

void VisionLineDrive_IngestByte(uint8_t byte, uint32_t now_ms){
    if (frame_index == 0U){
        if (byte == VISION_FRAME_START0){
            frame_buffer[frame_index++] = byte;
        }
        return;
    }
    if (frame_index == 1U){
        if (byte == VISION_FRAME_START1){
            frame_buffer[frame_index++] = byte;
        } else if (byte == VISION_FRAME_START0){
            frame_buffer[0] = byte;
        } else{
            frame_index = 0U;
        }
        return;
    }

    frame_buffer[frame_index++] = byte;
    if (frame_index < VISION_LINE_FRAME_SIZE){
        return;
    }
    if (VisionLine_FrameChecksumOk(frame_buffer)){
        VisionLine_AcceptFrame(now_ms);
    } else{
        vision_output.checksum_errors++;
    }
    frame_index = (byte == VISION_FRAME_START0) ? 1U : 0U;
    if (frame_index == 1U){
        frame_buffer[0] = byte;
    }
}

static void VisionLine_PollUart(uint32_t now_ms){
    uint8_t bytes[32];
    uint16_t count;
    do {
        count = DebugUart_Read(bytes, (uint16_t)sizeof(bytes));
        for (uint16_t i = 0U; i < count; i++){
            VisionLineDrive_IngestByte(bytes[i], now_ms);
        }
    } while (count == (uint16_t)sizeof(bytes));
}

static bool VisionLine_UpdateImu(void){
    JY61P_I2C_SAMPLE sample;
    if (!JY61P_I2C_IsDataFresh(VISION_LINE_IMU_MAX_AGE_MS) ||
        !JY61P_I2C_GetSnapshot(&sample)){
        vision_output.imu_ready = false;
        return false;
    }
    vision_output.gyro_z_deg_s =
        VISION_LINE_GYRO_SIGN * sample.data.gyro_deg_s.z;
    vision_output.imu_ready = true;
    return true;
}

static BSP_STATUS VisionLine_Apply(float omega_reference, float dt_s){
    float correction = PID_Update(&rate_pid, omega_reference,
                                  vision_output.gyro_z_deg_s, dt_s);
    KINEMATICS_DIFFERENTIAL_OUTPUT duty = Kinematics_DifferentialMix(
        VISION_LINE_BASE_DUTY_PERCENT,
        VISION_LINE_STEERING_SIGN * correction, 100.0f);
    vision_output.omega_reference_deg_s = omega_reference;
    vision_output.correction_percent = correction;
    vision_output.left_duty_percent = duty.left;
    vision_output.right_duty_percent = duty.right;
    return Chassis_SetDuty(duty.left, duty.right);
}

void VisionLineDrive_Init(void){
    PID_CONFIG config = {
        .kp = VISION_LINE_RATE_PID_KP,
        .ki = VISION_LINE_RATE_PID_KI,
        .kd = VISION_LINE_RATE_PID_KD,
        .integral_limit = VISION_LINE_RATE_PID_INTEGRAL_LIMIT,
        .output_limit = VISION_LINE_RATE_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };
    memset(&vision_output, 0, sizeof(vision_output));
    PID_Init(&rate_pid, &config);
    frame_index = 0U;
    last_frame_ms = 0U;
    have_frame = false;
    have_sequence = false;
    last_sequence = 0U;
    VisionLine_ResetControl();
}

BSP_STATUS VisionLineDrive_Update(float dt_s){
    uint32_t now_ms = BSP_Time_GetMs();
    VisionLine_PollUart(now_ms);
    vision_output.frame_age_ms = have_frame ? (now_ms - last_frame_ms) : UINT32_MAX;
    vision_output.frame_fresh =
        have_frame && vision_output.frame_age_ms <= VISION_LINE_FRAME_MAX_AGE_MS;

    if (!VisionLine_UpdateImu()){
        VisionLine_ResetControl();
        return Chassis_SetDuty(0.0f, 0.0f);
    }

    if (vision_output.phase == VISION_LINE_PHASE_WAIT_IMU){
        vision_output.phase = VISION_LINE_PHASE_STARTUP_RATE;
        startup_begin_ms = now_ms;
        PID_Reset(&rate_pid);
    }

    if (vision_output.phase == VISION_LINE_PHASE_STARTUP_RATE){
        if ((now_ms - startup_begin_ms) < VISION_LINE_STARTUP_DURATION_MS){
            return VisionLine_Apply(0.0f, dt_s);
        }
        vision_output.phase = VISION_LINE_PHASE_TRACK;
        PID_Reset(&rate_pid);
    }

    if (!vision_output.frame_fresh || !vision_output.track_valid ||
        vision_output.confidence < VISION_LINE_MIN_CONFIDENCE){
        PID_Reset(&rate_pid);
        vision_output.omega_reference_deg_s = 0.0f;
        vision_output.correction_percent = 0.0f;
        vision_output.left_duty_percent = 0.0f;
        vision_output.right_duty_percent = 0.0f;
        return Chassis_SetDuty(0.0f, 0.0f);
    }

    float omega_reference =
        VISION_LINE_POSITION_RATE_KP * vision_output.position_error +
        VISION_LINE_HEADING_RATE_KP * vision_output.heading_error_deg;
    omega_reference = VisionLine_Clamp(omega_reference,
        -VISION_LINE_OMEGA_LIMIT_DEG_S, VISION_LINE_OMEGA_LIMIT_DEG_S);
    return VisionLine_Apply(omega_reference, dt_s);
}

void VisionLineDrive_Stop(void){
    VisionLine_ResetControl();
    (void)Chassis_SetDuty(0.0f, 0.0f);
}

VISION_LINE_OUTPUT VisionLineDrive_GetOutput(void){
    return vision_output;
}

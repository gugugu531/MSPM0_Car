/**
 * @file  line_guided_drive.c
 * @brief 80% 直接起步的 Yahboom 灰度 PID / 航向 PID 切换控制器实现。
 */
#include "line_guided_drive.h"

#include "bsp_time.h"
#include "chassis.h"
#include "filter/filter.h"
#include "kinematics/kinematics.h"
#include "line_follow.h"
#include "pid/pid.h"
#include "wit_sdk.h"
#include "yaw_estimator.h"

static LINE_GUIDED_OUTPUT s_output;
static PID_CONTROLLER s_line_pid;
static PID_CONTROLLER s_heading_pid;
static YAW_ESTIMATOR s_yaw_estimator;
static uint32_t s_startup_begin_ms;
static uint32_t s_last_imu_sample_ms;
static uint32_t s_last_imu_sample_count;
static float s_filtered_line_error;
static bool s_yaw_correction_started;
static bool s_yaw_correction_frozen;

static void LineGuidedDrive_ResetControl(void)
{
    PID_Reset(&s_line_pid);
    PID_Reset(&s_heading_pid);
    YawEstimator_Reset(&s_yaw_estimator);
    s_startup_begin_ms = 0U;
    s_last_imu_sample_ms = 0U;
    s_last_imu_sample_count = 0U;
    s_filtered_line_error = 0.0f;
    s_yaw_correction_started = false;
    s_yaw_correction_frozen = false;
    s_output.phase = LINE_GUIDED_PHASE_WAIT_IMU;
    s_output.integrated_heading_deg = 0.0f;
    s_output.yaw_correction_deg = 0.0f;
    s_output.corrected_yaw_deg = s_output.yaw_deg;
    s_output.heading_reference_deg = 0.0f;
    s_output.correction_percent = 0.0f;
}

static void LineGuidedDrive_EnterHeadingHold(float entry_yaw_deg)
{
    s_output.phase = LINE_GUIDED_PHASE_HEADING_HOLD;
    s_output.heading_reference_deg = entry_yaw_deg;
    PID_Reset(&s_heading_pid);
}

static BSP_STATUS LineGuidedDrive_UpdateObservation(uint8_t detected_mask)
{
    LINE_FOLLOW_OBSERVATION observation;
    BSP_STATUS status = LineFollow_ObserveDetectedMask(
        detected_mask, LINE_FOLLOW_DEFAULT_POSITION_SCALE, &observation);
    if (status != BSP_STATUS_OK){
        return status;
    }

    s_output.level_mask = observation.level_mask;
    s_output.black_mask = observation.black_mask;
    s_output.black_count = observation.black_count;
    s_output.line_lost = observation.line_lost;
    s_output.line_error = observation.error;
    return BSP_STATUS_OK;
}

static bool LineGuidedDrive_UpdateImu(JY61P_I2C_SAMPLE *sample)
{
    if (!JY61P_I2C_IsDataFresh(LINE_GUIDED_IMU_MAX_AGE_MS) ||
        !JY61P_I2C_GetSnapshot(sample)){
        s_output.imu_ready = false;
        return false;
    }

    s_output.gyro_z_deg_s =
        LINE_GUIDED_INTEGRATION_GYRO_SIGN * sample->data.gyro_deg_s.z;
    s_output.yaw_deg = Kinematics_NormalizeAngleDeg(
        LINE_GUIDED_HEADING_YAW_SIGN * sample->data.attitude_deg.yaw);
    s_output.corrected_yaw_deg = Kinematics_NormalizeAngleDeg(
        s_output.yaw_deg + s_output.yaw_correction_deg);
    s_output.imu_ready = true;
    return true;
}

static void LineGuidedDrive_UpdateYawCorrection(const JY61P_I2C_SAMPLE *sample)
{
    if (!s_yaw_correction_started){
        YawEstimator_Start(&s_yaw_estimator, s_output.yaw_deg);
        s_output.integrated_heading_deg =
            YawEstimator_GetIntegrated(&s_yaw_estimator);
        s_output.heading_reference_deg = s_output.yaw_deg;
        s_output.yaw_correction_deg = 0.0f;
        s_output.corrected_yaw_deg = s_output.yaw_deg;
        s_startup_begin_ms = BSP_Time_GetMs();
        s_last_imu_sample_ms = sample->timestamp_ms;
        s_last_imu_sample_count = sample->sample_count;
        s_yaw_correction_started = true;
    }

    if (!s_yaw_correction_frozen &&
        (sample->sample_count != s_last_imu_sample_count)){
        float sample_dt_s =
            (float)(sample->timestamp_ms - s_last_imu_sample_ms) * 0.001f;
        YawEstimator_Integrate(&s_yaw_estimator,
                               s_output.gyro_z_deg_s,
                               sample_dt_s);
        s_output.integrated_heading_deg =
            YawEstimator_GetIntegrated(&s_yaw_estimator);
        s_last_imu_sample_ms = sample->timestamp_ms;
        s_last_imu_sample_count = sample->sample_count;
    }

    if (!s_yaw_correction_frozen){
        s_output.yaw_correction_deg = Kinematics_AngleDiffDeg(
            s_output.integrated_heading_deg, s_output.yaw_deg);
        s_output.corrected_yaw_deg = Kinematics_NormalizeAngleDeg(
            s_output.yaw_deg + s_output.yaw_correction_deg);
    }

    if (!s_yaw_correction_frozen &&
        ((uint32_t)(BSP_Time_GetMs() - s_startup_begin_ms) >=
         LINE_GUIDED_YAW_CORRECTION_DURATION_MS)){
        s_yaw_correction_frozen = true;
    }
}

static float LineGuidedDrive_UpdateControlMode(float dt_s)
{
    bool outer_line_detected =
        (s_output.black_mask & LINE_GUIDED_OUTER_SENSOR_MASK) != 0U;

    if (outer_line_detected){
        if (s_output.phase != LINE_GUIDED_PHASE_LINE_PID){
            s_output.phase = LINE_GUIDED_PHASE_LINE_PID;
            s_filtered_line_error = s_output.line_error;
            PID_Reset(&s_line_pid);
        } else{
            s_filtered_line_error = Filter_LowpassEma(
                s_filtered_line_error, s_output.line_error,
                LINE_GUIDED_OUTER_ERROR_LPF_ALPHA);
        }

        return PID_Update(&s_line_pid, s_filtered_line_error, 0.0f, dt_s);
    }

    if (s_output.phase != LINE_GUIDED_PHASE_HEADING_HOLD){
        LineGuidedDrive_EnterHeadingHold(s_output.corrected_yaw_deg);
    }

    float heading_error = Kinematics_AngleDiffDeg(
        s_output.heading_reference_deg, s_output.corrected_yaw_deg);
    return PID_Update(&s_heading_pid, heading_error, 0.0f, dt_s);
}

void LineGuidedDrive_Init(void)
{
    const PID_CONFIG line_config = {
        .kp = LINE_GUIDED_LINE_PID_KP,
        .ki = LINE_GUIDED_LINE_PID_KI,
        .kd = LINE_GUIDED_LINE_PID_KD,
        .integral_limit = LINE_GUIDED_LINE_PID_INTEGRAL_LIMIT,
        .output_limit = LINE_GUIDED_LINE_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };
    const PID_CONFIG heading_config = {
        .kp = LINE_GUIDED_HEADING_PID_KP,
        .ki = LINE_GUIDED_HEADING_PID_KI,
        .kd = LINE_GUIDED_HEADING_PID_KD,
        .integral_limit = LINE_GUIDED_HEADING_PID_INTEGRAL_LIMIT,
        .output_limit = LINE_GUIDED_HEADING_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };

    s_output = (LINE_GUIDED_OUTPUT){0};
    PID_Init(&s_line_pid, &line_config);
    PID_Init(&s_heading_pid, &heading_config);
    LineGuidedDrive_ResetControl();
    Chassis_ResetDistance();
}

BSP_STATUS LineGuidedDrive_Update(uint8_t detected_mask,
                                  bool sensor_ready,
                                  float dt_s)
{
    s_output.line_sensor_ready = sensor_ready;
    if (!sensor_ready){
        /* 未开始运动或采样中断时重新计时，避免停住的时间被算入启动阶段。 */
        LineGuidedDrive_ResetControl();
        s_output.line_sensor_ready = false;
        s_output.left_duty_percent = 0.0f;
        s_output.right_duty_percent = 0.0f;
        return Chassis_SetDuty(0.0f, 0.0f);
    }

    BSP_STATUS observe_status = LineGuidedDrive_UpdateObservation(detected_mask);
    if (observe_status != BSP_STATUS_OK){
        return observe_status;
    }

    JY61P_I2C_SAMPLE sample;
    if (!LineGuidedDrive_UpdateImu(&sample)){
        LineGuidedDrive_ResetControl();
        s_output.left_duty_percent = 0.0f;
        s_output.right_duty_percent = 0.0f;
        return Chassis_SetDuty(0.0f, 0.0f);
    }

    LineGuidedDrive_UpdateYawCorrection(&sample);
    s_output.correction_percent = LineGuidedDrive_UpdateControlMode(dt_s);

    KINEMATICS_DIFFERENTIAL_OUTPUT duty = Kinematics_DifferentialMix(
        LINE_GUIDED_BASE_DUTY_PERCENT,
        s_output.correction_percent,
        100.0f);
    s_output.left_duty_percent = duty.left;
    s_output.right_duty_percent = duty.right;
    return Chassis_SetDuty(duty.left, duty.right);
}

void LineGuidedDrive_Stop(void)
{
    LineGuidedDrive_ResetControl();
    s_output.left_duty_percent = 0.0f;
    s_output.right_duty_percent = 0.0f;
    (void)Chassis_SetDuty(0.0f, 0.0f);
}

LINE_GUIDED_OUTPUT LineGuidedDrive_GetOutput(void)
{
    return s_output;
}

#include "turn_drive.h"

#include "bsp_time.h"
#include "chassis.h"
#include "kinematics/kinematics.h"
#include "pid/pid.h"
#include "wit_sdk.h"
#include "yaw_estimator.h"

typedef struct {
    float base_duty_percent;
    float left_deceleration_end_duty_percent;
    uint32_t left_deceleration_duration_ms;
    bool integrate_on_new_samples;
} TURN_DRIVE_CONFIG;

static const TURN_DRIVE_CONFIG s_standard_config = {
    TURN_DRIVE_BASE_DUTY_PERCENT,
    TURN_DRIVE_LEFT_DECELERATION_END_DUTY_PERCENT,
    TURN_DRIVE_LEFT_DECELERATION_DURATION_MS,
    true,
};

static const TURN_DRIVE_CONFIG s_full_config = {
    TURN_DRIVE_FULL_BASE_DUTY_PERCENT,
    TURN_DRIVE_FULL_LEFT_DECELERATION_END_DUTY_PERCENT,
    TURN_DRIVE_FULL_LEFT_DECELERATION_DURATION_MS,
    true,
};

static TURN_DRIVE_OUTPUT s_output;
static const TURN_DRIVE_CONFIG *s_config;
static PID_CONTROLLER s_heading_pid;
static YAW_ESTIMATOR s_yaw_estimator;
static uint32_t s_integrated_phase_start_ms;
static uint32_t s_integrated_last_sample_ms;
static uint32_t s_integrated_last_sample_count;
static uint32_t s_imu_sample_ms;
static uint32_t s_imu_sample_count;
static uint32_t s_left_deceleration_start_ms;
static uint32_t s_turn_start_ms;
static float s_post_turn_distance_start_m;

static float TurnDrive_Clamp(float value, float lower, float upper)
{
    return Kinematics_Clamp(value, lower, upper);
}

static void TurnDrive_InitPid(void)
{
    PID_CONFIG heading_config = {
        .kp = TURN_DRIVE_HEADING_KP,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 100.0f,
        .output_limit = TURN_DRIVE_HEADING_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };

    if (s_output.mode == TURN_DRIVE_MODE_FULL){
        heading_config.output_limit = TURN_DRIVE_FULL_HEADING_OUTPUT_LIMIT;
    }
    PID_Init(&s_heading_pid, &heading_config);
}

static bool TurnDrive_UpdateImu(void)
{
    JY61P_I2C_SAMPLE sample;

    if (!JY61P_I2C_IsDataFresh(TURN_DRIVE_IMU_MAX_AGE_MS) ||
        !JY61P_I2C_GetSnapshot(&sample)){
        s_output.imu_ready = false;
        return false;
    }

    s_output.yaw_deg = Kinematics_NormalizeAngleDeg(
        TURN_DRIVE_YAW_SIGN * sample.data.attitude_deg.yaw);
    s_output.corrected_yaw_deg = Kinematics_NormalizeAngleDeg(
        s_output.yaw_deg + s_output.yaw_correction_deg);
    s_output.gyro_z_deg_s =
        TURN_DRIVE_INTEGRATION_GYRO_SIGN * sample.data.gyro_deg_s.z;
    s_imu_sample_ms = sample.timestamp_ms;
    s_imu_sample_count = sample.sample_count;
    s_output.imu_ready = true;
    return true;
}

static void TurnDrive_EnterIntegratedStraight(void)
{
    s_integrated_phase_start_ms = BSP_Time_GetMs();
    s_integrated_last_sample_ms = s_imu_sample_ms;
    s_integrated_last_sample_count = s_imu_sample_count;
    YawEstimator_Start(&s_yaw_estimator, s_output.yaw_deg);
    s_output.integrated_heading_deg =
        YawEstimator_GetIntegrated(&s_yaw_estimator);
    s_output.heading_reference_deg =
        YawEstimator_GetInitialFused(&s_yaw_estimator);
    s_output.yaw_correction_deg = 0.0f;
    s_output.corrected_yaw_deg = s_output.yaw_deg;
    s_output.phase = TURN_DRIVE_PHASE_INTEGRATED_STRAIGHT;
    PID_Reset(&s_heading_pid);
}

static void TurnDrive_EnterLeftTurn(void)
{
    s_output.turn_target_deg = Kinematics_NormalizeAngleDeg(
        s_output.heading_reference_deg + TURN_DRIVE_LEFT_YAW_DELTA_DEG);
    s_output.phase = TURN_DRIVE_PHASE_LEFT_DECELERATE;
    s_output.turn_reduction_percent = 0.0f;
    s_left_deceleration_start_ms = BSP_Time_GetMs();
    s_turn_start_ms = BSP_Time_GetMs();
}

static void TurnDrive_EnterPostTurnStraight(void)
{
    s_post_turn_distance_start_m = Chassis_GetDistance();
    s_output.post_turn_distance_m = 0.0f;
    s_output.phase = TURN_DRIVE_PHASE_POST_TURN_STRAIGHT;
    PID_Reset(&s_heading_pid);
}

static void TurnDrive_UpdateIntegratedHeading(float dt_s)
{
    if (s_config->integrate_on_new_samples){
        if (s_imu_sample_count != s_integrated_last_sample_count){
            float sample_dt_s = (float)(s_imu_sample_ms -
                s_integrated_last_sample_ms) * 0.001f;
            YawEstimator_Integrate(&s_yaw_estimator,
                                   s_output.gyro_z_deg_s, sample_dt_s);
            s_integrated_last_sample_ms = s_imu_sample_ms;
            s_integrated_last_sample_count = s_imu_sample_count;
        }
    } else{
        YawEstimator_Integrate(&s_yaw_estimator,
                               s_output.gyro_z_deg_s, dt_s);
    }
    s_output.integrated_heading_deg =
        YawEstimator_GetIntegrated(&s_yaw_estimator);
    s_output.yaw_correction_deg = Kinematics_AngleDiffDeg(
        s_output.integrated_heading_deg, s_output.yaw_deg);
    s_output.corrected_yaw_deg = Kinematics_NormalizeAngleDeg(
        s_output.yaw_deg + s_output.yaw_correction_deg);
}

static BSP_STATUS TurnDrive_UpdatePhase(float dt_s)
{
    float distance_m = Chassis_GetDistance();

    s_output.distance_m = distance_m;

    switch (s_output.phase){
        case TURN_DRIVE_PHASE_WAIT_IMU:
            TurnDrive_EnterIntegratedStraight();
            return BSP_STATUS_OK;
        case TURN_DRIVE_PHASE_INTEGRATED_STRAIGHT:
            TurnDrive_UpdateIntegratedHeading(dt_s);
            if ((BSP_Time_GetMs() - s_integrated_phase_start_ms) >=
                TURN_DRIVE_INTEGRATED_PHASE_DURATION_MS){
                s_output.phase = TURN_DRIVE_PHASE_STRAIGHT;
            }
            return BSP_STATUS_OK;
        case TURN_DRIVE_PHASE_STRAIGHT:
            if (distance_m >= TURN_DRIVE_PRE_TURN_DISTANCE_M){
                TurnDrive_EnterLeftTurn();
            }
            return BSP_STATUS_OK;
        case TURN_DRIVE_PHASE_LEFT_DECELERATE:
            if ((BSP_Time_GetMs() - s_left_deceleration_start_ms) >=
                s_config->left_deceleration_duration_ms){
                s_output.phase = TURN_DRIVE_PHASE_LEFT_TURN_BRAKED;
            }
            return BSP_STATUS_OK;
        case TURN_DRIVE_PHASE_LEFT_TURN_BRAKED:
            if ((BSP_Time_GetMs() - s_turn_start_ms) >=
                TURN_DRIVE_TURN_TIMEOUT_MS){
                return BSP_STATUS_TIMEOUT;
            }
            if (Kinematics_AngleDiffDeg(s_output.turn_target_deg,
                                        s_output.corrected_yaw_deg) <=
                TURN_DRIVE_TURN_COMPLETE_TOLERANCE_DEG){
                TurnDrive_EnterPostTurnStraight();
            }
            return BSP_STATUS_OK;
        case TURN_DRIVE_PHASE_POST_TURN_STRAIGHT:
            s_output.post_turn_distance_m =
                distance_m - s_post_turn_distance_start_m;
            if (s_output.post_turn_distance_m >=
                TURN_DRIVE_POST_TURN_DISTANCE_M){
                s_output.phase = TURN_DRIVE_PHASE_COMPLETE;
            }
            return BSP_STATUS_OK;
        case TURN_DRIVE_PHASE_COMPLETE:
        default:
            return BSP_STATUS_OK;
    }
}

static BSP_STATUS TurnDrive_ApplyCorrection(float correction_percent)
{
    if (s_output.mode == TURN_DRIVE_MODE_FULL){
        float left_duty_percent = s_config->base_duty_percent +
            correction_percent;
        float right_duty_percent = s_config->base_duty_percent -
            correction_percent;
        float max_duty_percent = (left_duty_percent > right_duty_percent)
            ? left_duty_percent : right_duty_percent;

        if (max_duty_percent > 100.0f){
            float overflow_percent = max_duty_percent - 100.0f;
            left_duty_percent -= overflow_percent;
            right_duty_percent -= overflow_percent;
        }
        return Chassis_SetDuty(TurnDrive_Clamp(left_duty_percent,
                                               -100.0f, 100.0f),
                               TurnDrive_Clamp(right_duty_percent,
                                               -100.0f, 100.0f));
    }
    return Chassis_SetDuty(
        TurnDrive_Clamp(s_config->base_duty_percent + correction_percent,
                        -100.0f, 100.0f),
        TurnDrive_Clamp(s_config->base_duty_percent - correction_percent,
                         -100.0f, 100.0f));
}

static BSP_STATUS TurnDrive_ApplyHeading(float heading_reference_deg,
                                         float dt_s)
{
    float heading_error_deg = Kinematics_AngleDiffDeg(
        heading_reference_deg, s_output.corrected_yaw_deg);
    float correction_percent = PID_Update(&s_heading_pid,
                                          heading_error_deg, 0.0f, dt_s);
    return TurnDrive_ApplyCorrection(correction_percent);
}

static BSP_STATUS TurnDrive_Apply(float dt_s)
{
    float elapsed_ratio;
    float left_duty_percent;
    BSP_STATUS status;

    s_output.turn_reduction_percent = 0.0f;

    switch (s_output.phase){
        case TURN_DRIVE_PHASE_WAIT_IMU:
        case TURN_DRIVE_PHASE_COMPLETE:
            return Chassis_SetDuty(0.0f, 0.0f);
        case TURN_DRIVE_PHASE_INTEGRATED_STRAIGHT:
            return TurnDrive_ApplyHeading(s_output.heading_reference_deg, dt_s);
        case TURN_DRIVE_PHASE_STRAIGHT:
            return TurnDrive_ApplyHeading(s_output.heading_reference_deg, dt_s);
        case TURN_DRIVE_PHASE_LEFT_DECELERATE:
            elapsed_ratio = (float)(BSP_Time_GetMs() -
                s_left_deceleration_start_ms) /
                (float)s_config->left_deceleration_duration_ms;
            left_duty_percent = s_config->base_duty_percent +
                (s_config->left_deceleration_end_duty_percent -
                 s_config->base_duty_percent) *
                TurnDrive_Clamp(elapsed_ratio, 0.0f, 1.0f);
            s_output.turn_reduction_percent =
                s_config->base_duty_percent - left_duty_percent;
            return Chassis_SetDuty(left_duty_percent,
                                   s_config->base_duty_percent);
        case TURN_DRIVE_PHASE_LEFT_TURN_BRAKED:
            status = Chassis_SetDuty(0.0f, s_config->base_duty_percent);
            if (status != BSP_STATUS_OK){
                return status;
            }
            s_output.turn_reduction_percent = s_config->base_duty_percent;
            return TB6612FNG_Brake(TB6612FNG_CHANNEL_LEFT);
        case TURN_DRIVE_PHASE_POST_TURN_STRAIGHT:
            return TurnDrive_ApplyHeading(s_output.turn_target_deg, dt_s);
        default:
            return BSP_STATUS_ERROR;
    }
}

static void TurnDrive_UpdateOutput(void)
{
    CHASSIS_DUTY duty = Chassis_GetDuty();

    s_output.duty_left_percent = duty.left_percent;
    s_output.duty_right_percent = duty.right_percent;
    s_output.distance_m = Chassis_GetDistance();
    if (s_output.phase == TURN_DRIVE_PHASE_POST_TURN_STRAIGHT){
        s_output.post_turn_distance_m =
            s_output.distance_m - s_post_turn_distance_start_m;
    }
}

void TurnDrive_Init(TURN_DRIVE_MODE mode)
{
    s_output = (TURN_DRIVE_OUTPUT){0};
    s_output.mode = mode;
    s_output.phase = TURN_DRIVE_PHASE_WAIT_IMU;
    s_config = (mode == TURN_DRIVE_MODE_FULL)
        ? &s_full_config : &s_standard_config;
    s_integrated_phase_start_ms = 0U;
    s_integrated_last_sample_ms = 0U;
    s_integrated_last_sample_count = 0U;
    s_imu_sample_ms = 0U;
    s_imu_sample_count = 0U;
    s_left_deceleration_start_ms = 0U;
    s_turn_start_ms = 0U;
    s_post_turn_distance_start_m = 0.0f;
    YawEstimator_Reset(&s_yaw_estimator);
    TurnDrive_InitPid();
    Chassis_ResetDistance();
    TurnDrive_UpdateOutput();
}

BSP_STATUS TurnDrive_Update(float dt_s)
{
    BSP_STATUS phase_status;
    BSP_STATUS status;

    if (!TurnDrive_UpdateImu()){
        (void)Chassis_SetDuty(0.0f, 0.0f);
        TurnDrive_UpdateOutput();
        return (s_output.phase == TURN_DRIVE_PHASE_WAIT_IMU)
                   ? BSP_STATUS_OK : BSP_STATUS_NOT_READY;
    }

    phase_status = TurnDrive_UpdatePhase(dt_s);
    if (phase_status != BSP_STATUS_OK){
        (void)Chassis_SetDuty(0.0f, 0.0f);
        TurnDrive_UpdateOutput();
        return phase_status;
    }
    status = TurnDrive_Apply(dt_s);
    TurnDrive_UpdateOutput();
    return status;
}

TURN_DRIVE_OUTPUT TurnDrive_GetOutput(void)
{
    return s_output;
}

bool TurnDrive_IsComplete(void)
{
    return s_output.phase == TURN_DRIVE_PHASE_COMPLETE;
}

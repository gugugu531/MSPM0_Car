/**
 * @file  straight_drive.c
 * @brief 直行测试方式的指令、姿态反馈和底盘输出控制。
 */
#include "straight_drive.h"

#include "bsp_time.h"
#include "chassis.h"
#include "kinematics/kinematics.h"
#include "pid/pid.h"
#include "wit_sdk.h"
#include "yaw_estimator.h"

static STRAIGHT_DRIVE_OUTPUT s_output;
static PID_CONTROLLER s_rate_pid;
static PID_CONTROLLER s_encoder_pid;
static PID_CONTROLLER s_integrated_heading_pid;
static PID_CONTROLLER s_heading_pid;
static float s_startup_elapsed_s;
static uint32_t s_integrated_phase_start_ms;
static bool s_integrated_phase_started;
static uint32_t s_integrated_last_sample_ms;
static uint32_t s_integrated_last_sample_count;
static uint32_t s_imu_sample_ms;
static uint32_t s_imu_sample_count;
static YAW_ESTIMATOR s_yaw_estimator;

/** 两个 A/B 切换实验共用阶段管理；80% 版保留旧积分法作为实车对照。 */
static bool StraightDrive_IsIntegratedHeadingMode(void)
{
    return (s_output.mode == STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING) ||
           (s_output.mode == STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING);
}

static bool StraightDrive_UsesGyro(void)
{
    return (s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_RATE) ||
           (s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_HEADING) ||
           (s_output.mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING) ||
           (s_output.mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) ||
           (s_output.mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) ||
           StraightDrive_IsIntegratedHeadingMode();
}

static float StraightDrive_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 在 middleware 内完成测试专用线性斜坡，不向 core 引入通用接口。 */
static float StraightDrive_RampToward(float current, float target,
                                      float max_step)
{
    float delta = target - current;
    if (StraightDrive_Abs(delta) <= max_step){
        return target;
    }
    return current + ((delta > 0.0f) ? max_step : -max_step);
}

static float StraightDrive_ClampCommand(float command)
{
    float limit;
    if (s_output.mode == STRAIGHT_DRIVE_MODE_SPEED){
        limit = STRAIGHT_DRIVE_SPEED_LIMIT;
    } else if (s_output.mode ==
               STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING){
        limit = STRAIGHT_DRIVE_OUTPUT_LIMIT;
    } else{
        limit = STRAIGHT_DRIVE_DUTY_LIMIT;
    }
    return Kinematics_Clamp(command, -limit, limit);
}

static void StraightDrive_ResetPid(void)
{
    PID_Reset(&s_rate_pid);
    PID_Reset(&s_encoder_pid);
    PID_Reset(&s_integrated_heading_pid);
    PID_Reset(&s_heading_pid);
}

static void StraightDrive_InitPid(void)
{
    const PID_CONFIG rate_config = {
        .kp = STRAIGHT_DRIVE_RATE_PID_KP,
        .ki = STRAIGHT_DRIVE_RATE_PID_KI,
        .kd = STRAIGHT_DRIVE_RATE_PID_KD,
        .integral_limit = STRAIGHT_DRIVE_RATE_PID_INTEGRAL_LIMIT,
        .output_limit = STRAIGHT_DRIVE_RATE_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };
    PID_CONFIG heading_config = {
        .kp = STRAIGHT_DRIVE_HEADING_PID_KP,
        .ki = STRAIGHT_DRIVE_HEADING_PID_KI,
        .kd = STRAIGHT_DRIVE_HEADING_PID_KD,
        .integral_limit = STRAIGHT_DRIVE_HEADING_PID_INTEGRAL_LIMIT,
        .output_limit = STRAIGHT_DRIVE_HEADING_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };
    if (s_output.mode == STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING){
        heading_config.output_limit =
            STRAIGHT_DRIVE_FULL_HEADING_OUTPUT_LIMIT;
    }
    const PID_CONFIG encoder_config = {
        .kp = STRAIGHT_DRIVE_ENCODER_PID_KP,
        .ki = STRAIGHT_DRIVE_ENCODER_PID_KI,
        .kd = STRAIGHT_DRIVE_ENCODER_PID_KD,
        .integral_limit = STRAIGHT_DRIVE_ENCODER_PID_INTEGRAL_LIMIT,
        .output_limit = STRAIGHT_DRIVE_ENCODER_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };

    PID_Init(&s_rate_pid, &rate_config);
    PID_Init(&s_encoder_pid, &encoder_config);
    PID_Init(&s_integrated_heading_pid, &heading_config);
    PID_Init(&s_heading_pid, &heading_config);
}

static void StraightDrive_UpdateImu(void)
{
    JY61P_I2C_SAMPLE sample;

    if (!JY61P_I2C_IsDataFresh(STRAIGHT_DRIVE_IMU_MAX_AGE_MS) ||
        !JY61P_I2C_GetSnapshot(&sample)){
        s_output.imu_ready = false;
        s_output.correction_percent = 0.0f;
        if ((s_output.mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING) ||
            (s_output.mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) ||
            (s_output.mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) ||
            StraightDrive_IsIntegratedHeadingMode()){
            s_output.startup_complete = false;
            s_startup_elapsed_s = 0.0f;
            s_integrated_phase_started = false;
            YawEstimator_Reset(&s_yaw_estimator);
            s_integrated_last_sample_ms = 0U;
            s_integrated_last_sample_count = 0U;
            s_output.integrated_heading_deg = 0.0f;
            s_output.heading_reference_valid = false;
            if (s_output.mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING){
                /* 恢复通信后从零重新软启动，避免占空比阶跃。 */
                s_output.applied_command = 0.0f;
            }
        }
        StraightDrive_ResetPid();
        return;
    }

    s_output.gyro_z_deg_s =
        STRAIGHT_DRIVE_RATE_GYRO_SIGN * sample.data.gyro_deg_s.z;
    s_output.yaw_deg = Kinematics_NormalizeAngleDeg(
        STRAIGHT_DRIVE_HEADING_YAW_SIGN * sample.data.attitude_deg.yaw);
    s_imu_sample_ms = sample.timestamp_ms;
    s_imu_sample_count = sample.sample_count;
    s_output.imu_ready = true;
}

static void StraightDrive_UpdateStartup(float dt_s)
{
    if (StraightDrive_IsIntegratedHeadingMode()){
        s_output.applied_command = s_output.command;
        if (!s_output.imu_ready || (s_output.command == 0.0f)){
            s_output.startup_complete = false;
            s_integrated_phase_started = false;
            YawEstimator_Reset(&s_yaw_estimator);
            s_integrated_last_sample_ms = 0U;
            s_integrated_last_sample_count = 0U;
            s_output.integrated_heading_deg = 0.0f;
            return;
        }

        if (!s_integrated_phase_started){
            uint32_t now_ms = BSP_Time_GetMs();
            s_integrated_phase_start_ms = now_ms;
            s_integrated_phase_started = true;
            /* A0 = B0：纯积分航向与启动时 JY61P 航向使用同一坐标原点。 */
            YawEstimator_Start(&s_yaw_estimator, s_output.yaw_deg);
            s_output.integrated_heading_deg =
                YawEstimator_GetIntegrated(&s_yaw_estimator);
            s_integrated_last_sample_ms = s_imu_sample_ms;
            s_integrated_last_sample_count = s_imu_sample_count;
        }

        if (!s_output.startup_complete){
            uint32_t now_ms = BSP_Time_GetMs();
            uint32_t elapsed_ms = now_ms - s_integrated_phase_start_ms;

            if (s_output.mode ==
                STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING){
                uint32_t sample_count = s_imu_sample_count;
                if (sample_count != s_integrated_last_sample_count){
                    float sample_dt_s =
                        (float)(s_imu_sample_ms - s_integrated_last_sample_ms) *
                        0.001f;
                    YawEstimator_Integrate(&s_yaw_estimator,
                                           s_output.gyro_z_deg_s,
                                           sample_dt_s);
                    s_output.integrated_heading_deg =
                        YawEstimator_GetIntegrated(&s_yaw_estimator);
                    s_integrated_last_sample_ms = s_imu_sample_ms;
                    s_integrated_last_sample_count = sample_count;
                }
            }

            if (elapsed_ms >= STRAIGHT_DRIVE_INTEGRATED_PHASE_DURATION_MS){
                s_output.startup_complete = true;
            } else if (s_output.mode ==
                      STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING){
                /* 保留原 80% 对照版本：每控制拍按固定 dt 积分。 */
                YawEstimator_Integrate(&s_yaw_estimator,
                                       s_output.gyro_z_deg_s, dt_s);
                s_output.integrated_heading_deg =
                    YawEstimator_GetIntegrated(&s_yaw_estimator);
            }
        }
        return;
    }

    if ((s_output.mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) ||
        (s_output.mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING)){
        float phase_duration_s =
            (s_output.mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING)
                ? STRAIGHT_DRIVE_RATE_PHASE_DURATION_S
                : STRAIGHT_DRIVE_ENCODER_PHASE_DURATION_S;
        s_output.applied_command = s_output.command;
        if (!s_output.imu_ready || (s_output.command == 0.0f)){
            s_output.startup_complete = false;
            s_startup_elapsed_s = 0.0f;
            return;
        }

        if (!s_output.startup_complete){
            if (s_startup_elapsed_s >= phase_duration_s){
                s_output.startup_complete = true;
            } else{
                s_startup_elapsed_s += dt_s;
            }
        }
        return;
    }

    if (s_output.mode != STRAIGHT_DRIVE_MODE_RAMP_HEADING){
        s_output.applied_command = s_output.command;
        s_output.startup_complete = true;
        return;
    }

    if (!s_output.imu_ready || (s_output.command == 0.0f)){
        s_output.applied_command = 0.0f;
        s_output.startup_complete = false;
        return;
    }

    s_output.applied_command = StraightDrive_RampToward(
        s_output.applied_command, s_output.command,
        STRAIGHT_DRIVE_RAMP_RATE_PERCENT_S * dt_s);
    s_output.startup_complete =
        (s_output.applied_command == s_output.command);
}

static BSP_STATUS StraightDrive_Apply(float dt_s)
{
    if (s_output.mode == STRAIGHT_DRIVE_MODE_SPEED){
        Chassis_SetSpeed(s_output.applied_command);
        return BSP_STATUS_OK;
    }

    if (!StraightDrive_UsesGyro()){
        s_output.correction_percent = 0.0f;
        return Chassis_SetDuty(s_output.applied_command,
                               s_output.applied_command);
    }

    if (!s_output.imu_ready || (s_output.applied_command == 0.0f)){
        s_output.correction_percent = 0.0f;
        return Chassis_SetDuty(0.0f, 0.0f);
    }

    if (((s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_HEADING) ||
         (s_output.mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING) ||
         ((s_output.mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) &&
          s_output.startup_complete) ||
         ((s_output.mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) &&
          s_output.startup_complete) ||
         (StraightDrive_IsIntegratedHeadingMode() &&
          s_output.startup_complete)) &&
        !s_output.heading_reference_valid){
        if (StraightDrive_IsIntegratedHeadingMode()){
            /*
             * A0=B0。切换时以 reference=B0+(B1-A1) 抵消 B 在猛烈加速期间
             * 相对纯积分 A 产生的偏移；B1-A1 按最短角处理跨越 ±180°。
             */
            float integration_correction = YawEstimator_GetFusionOffset(
                &s_yaw_estimator, s_output.yaw_deg);
            s_output.heading_reference_deg = Kinematics_NormalizeAngleDeg(
                YawEstimator_GetInitialFused(&s_yaw_estimator) +
                integration_correction);
        } else{
            /* 其余定时切换模式在第二阶段首拍捕获当前融合航向。 */
            s_output.heading_reference_deg = s_output.yaw_deg;
        }
        s_output.heading_reference_valid = true;
        PID_Reset(&s_heading_pid);
    }

    if ((s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_RATE) ||
        ((s_output.mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) &&
         !s_output.startup_complete)){
        s_output.correction_percent = PID_Update(
            &s_rate_pid, 0.0f, s_output.gyro_z_deg_s, dt_s);
    } else if ((s_output.mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) &&
               !s_output.startup_complete){
        /* 左轮累计路程更大时反馈为正，PID 给出负修正：左轮降、右轮升。 */
        float wheel_distance_difference =
            Chassis_GetWheelDistance(HALL_ENCODER_LEFT) -
            Chassis_GetWheelDistance(HALL_ENCODER_RIGHT);
        s_output.correction_percent = PID_Update(
            &s_encoder_pid, 0.0f, wheel_distance_difference, dt_s);
    } else if (StraightDrive_IsIntegratedHeadingMode() &&
               !s_output.startup_complete){
        float heading_error = Kinematics_AngleDiffDeg(
            YawEstimator_GetInitialFused(&s_yaw_estimator),
            s_output.integrated_heading_deg);
        s_output.correction_percent = PID_Update(
            &s_integrated_heading_pid, heading_error, 0.0f, dt_s);
    } else{
        float heading_error = Kinematics_AngleDiffDeg(
            s_output.heading_reference_deg, s_output.yaw_deg);
        s_output.correction_percent = PID_Update(
            &s_heading_pid, heading_error, 0.0f, dt_s);
    }

    if (s_output.mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING){
        /* 低占空比斜坡阶段不允许差速修正令任一车轮反转。 */
        float correction_limit = StraightDrive_Abs(s_output.applied_command);
        s_output.correction_percent = Kinematics_Clamp(
            s_output.correction_percent, -correction_limit,
            correction_limit);
    }

    KINEMATICS_DIFFERENTIAL_OUTPUT duty;
    if (s_output.mode == STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING){
        /*
         * 100% 基础占空比没有向上调节余量。先保留 2*corr 轮间差，再把两轮
         * 同减溢出量，使快侧保持 100%，靠降低慢侧完成转向修正。
         */
        duty.left = s_output.applied_command + s_output.correction_percent;
        duty.right = s_output.applied_command - s_output.correction_percent;
        float max_duty = (duty.left > duty.right) ? duty.left : duty.right;
        if (max_duty > STRAIGHT_DRIVE_OUTPUT_LIMIT){
            float overflow = max_duty - STRAIGHT_DRIVE_OUTPUT_LIMIT;
            duty.left -= overflow;
            duty.right -= overflow;
        }
        duty.left = Kinematics_Clamp(duty.left,
            -STRAIGHT_DRIVE_OUTPUT_LIMIT, STRAIGHT_DRIVE_OUTPUT_LIMIT);
        duty.right = Kinematics_Clamp(duty.right,
            -STRAIGHT_DRIVE_OUTPUT_LIMIT, STRAIGHT_DRIVE_OUTPUT_LIMIT);
    } else{
        duty = Kinematics_DifferentialMix(
            s_output.applied_command, s_output.correction_percent,
            STRAIGHT_DRIVE_OUTPUT_LIMIT);
    }
    return Chassis_SetDuty(duty.left, duty.right);
}

static void StraightDrive_UpdateWheelState(void)
{
    CHASSIS_DUTY duty = Chassis_GetDuty();
    s_output.duty_left_percent = duty.left_percent;
    s_output.duty_right_percent = duty.right_percent;
    s_output.speed_left_mps = Chassis_GetWheelSpeed(HALL_ENCODER_LEFT);
    s_output.speed_right_mps = Chassis_GetWheelSpeed(HALL_ENCODER_RIGHT);
    s_output.distance_left_m = Chassis_GetWheelDistance(HALL_ENCODER_LEFT);
    s_output.distance_right_m = Chassis_GetWheelDistance(HALL_ENCODER_RIGHT);
}

void StraightDrive_Init(STRAIGHT_DRIVE_MODE mode)
{
    s_output = (STRAIGHT_DRIVE_OUTPUT){0};
    s_output.mode = mode;
    if (mode == STRAIGHT_DRIVE_MODE_SPEED){
        s_output.command = STRAIGHT_DRIVE_DEFAULT_SPEED;
    } else if ((mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) ||
               (mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) ||
               (mode == STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING)){
        s_output.command = STRAIGHT_DRIVE_SWITCH_HEADING_DUTY_PERCENT;
    } else if (mode == STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING){
        s_output.command = STRAIGHT_DRIVE_FULL_HEADING_DUTY_PERCENT;
    } else{
        s_output.command = STRAIGHT_DRIVE_DEFAULT_DUTY;
    }
    s_output.applied_command =
        (mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING)
            ? 0.0f
            : s_output.command;
    s_output.startup_complete =
        (mode != STRAIGHT_DRIVE_MODE_RAMP_HEADING) &&
        (mode != STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) &&
        (mode != STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) &&
        (mode != STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING) &&
        (mode != STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING);
    s_startup_elapsed_s = 0.0f;
    s_integrated_phase_start_ms = 0U;
    s_integrated_phase_started = false;
    s_imu_sample_ms = 0U;
    s_imu_sample_count = 0U;
    YawEstimator_Reset(&s_yaw_estimator);
    s_integrated_last_sample_ms = 0U;
    s_integrated_last_sample_count = 0U;
    StraightDrive_InitPid();
    Chassis_ResetDistance();
    StraightDrive_UpdateWheelState();
}

void StraightDrive_AdjustCommand(int8_t steps)
{
    float step = (s_output.mode == STRAIGHT_DRIVE_MODE_SPEED)
                     ? STRAIGHT_DRIVE_SPEED_STEP
                     : STRAIGHT_DRIVE_DUTY_STEP;
    bool was_stopped = (s_output.command == 0.0f);
    s_output.command = StraightDrive_ClampCommand(
        s_output.command + ((float)steps * step));

    if (s_output.command == 0.0f){
        StraightDrive_ZeroCommand();
    } else if (was_stopped &&
               ((s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_HEADING) ||
                (s_output.mode == STRAIGHT_DRIVE_MODE_RAMP_HEADING) ||
                (s_output.mode == STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING) ||
                (s_output.mode == STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING) ||
                StraightDrive_IsIntegratedHeadingMode())){
        s_output.startup_complete = false;
        s_startup_elapsed_s = 0.0f;
        s_integrated_phase_started = false;
        YawEstimator_Reset(&s_yaw_estimator);
        s_integrated_last_sample_ms = 0U;
        s_integrated_last_sample_count = 0U;
        s_output.integrated_heading_deg = 0.0f;
        s_output.heading_reference_valid = false;
        PID_Reset(&s_heading_pid);
    }
}

void StraightDrive_ZeroCommand(void)
{
    s_output.command = 0.0f;
    s_output.applied_command = 0.0f;
    s_output.correction_percent = 0.0f;
    s_output.startup_complete = false;
    s_startup_elapsed_s = 0.0f;
    s_integrated_phase_started = false;
    YawEstimator_Reset(&s_yaw_estimator);
    s_integrated_last_sample_ms = 0U;
    s_integrated_last_sample_count = 0U;
    s_output.integrated_heading_deg = 0.0f;
    s_output.heading_reference_valid = false;
    StraightDrive_ResetPid();
}

BSP_STATUS StraightDrive_Update(float dt_s)
{
    StraightDrive_UpdateImu();
    StraightDrive_UpdateStartup(dt_s);
    BSP_STATUS status = StraightDrive_Apply(dt_s);
    StraightDrive_UpdateWheelState();
    return status;
}

STRAIGHT_DRIVE_OUTPUT StraightDrive_GetOutput(void)
{
    return s_output;
}

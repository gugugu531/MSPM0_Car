/**
 * @file  straight_drive.c
 * @brief 四种直行方式的指令、姿态反馈和底盘输出控制。
 */
#include "straight_drive.h"

#include "chassis.h"
#include "kinematics/kinematics.h"
#include "pid/pid.h"
#include "wit_sdk.h"

static STRAIGHT_DRIVE_OUTPUT s_output;
static PID_CONTROLLER s_rate_pid;
static PID_CONTROLLER s_heading_pid;

static bool StraightDrive_UsesGyro(void)
{
    return (s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_RATE) ||
           (s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_HEADING);
}

static float StraightDrive_ClampCommand(float command)
{
    float limit = (s_output.mode == STRAIGHT_DRIVE_MODE_SPEED)
                      ? STRAIGHT_DRIVE_SPEED_LIMIT
                      : STRAIGHT_DRIVE_DUTY_LIMIT;
    return Kinematics_Clamp(command, -limit, limit);
}

static void StraightDrive_ResetPid(void)
{
    PID_Reset(&s_rate_pid);
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
    const PID_CONFIG heading_config = {
        .kp = STRAIGHT_DRIVE_HEADING_PID_KP,
        .ki = STRAIGHT_DRIVE_HEADING_PID_KI,
        .kd = STRAIGHT_DRIVE_HEADING_PID_KD,
        .integral_limit = STRAIGHT_DRIVE_HEADING_PID_INTEGRAL_LIMIT,
        .output_limit = STRAIGHT_DRIVE_HEADING_PID_OUTPUT_LIMIT,
        .mode = PID_MODE_POSITION,
    };

    PID_Init(&s_rate_pid, &rate_config);
    PID_Init(&s_heading_pid, &heading_config);
}

static void StraightDrive_UpdateImu(void)
{
    WIT_IMU_DATA imu;

    if (!JY61P_I2C_IsDataFresh(STRAIGHT_DRIVE_IMU_MAX_AGE_MS) ||
        (WitGetData(&imu) != WIT_HAL_OK)){
        s_output.imu_ready = false;
        s_output.correction_percent = 0.0f;
        StraightDrive_ResetPid();
        return;
    }

    s_output.gyro_z_deg_s =
        STRAIGHT_DRIVE_RATE_GYRO_SIGN * imu.gyro_deg_s.z;
    s_output.yaw_deg = Kinematics_NormalizeAngleDeg(
        STRAIGHT_DRIVE_HEADING_YAW_SIGN * imu.attitude_deg.yaw);
    s_output.imu_ready = true;
}

static BSP_STATUS StraightDrive_Apply(float dt_s)
{
    if (s_output.mode == STRAIGHT_DRIVE_MODE_SPEED){
        Chassis_SetSpeed(s_output.command);
        return BSP_STATUS_OK;
    }

    if (!StraightDrive_UsesGyro()){
        s_output.correction_percent = 0.0f;
        return Chassis_SetDuty(s_output.command, s_output.command);
    }

    if (!s_output.imu_ready || (s_output.command == 0.0f)){
        s_output.correction_percent = 0.0f;
        return Chassis_SetDuty(0.0f, 0.0f);
    }

    if (s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_RATE){
        s_output.correction_percent = PID_Update(
            &s_rate_pid, 0.0f, s_output.gyro_z_deg_s, dt_s);
    } else{
        if (!s_output.heading_reference_valid){
            s_output.heading_reference_deg = s_output.yaw_deg;
            s_output.heading_reference_valid = true;
            PID_Reset(&s_heading_pid);
        }
        float heading_error = Kinematics_AngleDiffDeg(
            s_output.heading_reference_deg, s_output.yaw_deg);
        s_output.correction_percent = PID_Update(
            &s_heading_pid, heading_error, 0.0f, dt_s);
    }

    KINEMATICS_DIFFERENTIAL_OUTPUT duty = Kinematics_DifferentialMix(
        s_output.command, s_output.correction_percent,
        STRAIGHT_DRIVE_OUTPUT_LIMIT);
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
    s_output.command = (mode == STRAIGHT_DRIVE_MODE_SPEED)
                           ? STRAIGHT_DRIVE_DEFAULT_SPEED
                           : STRAIGHT_DRIVE_DEFAULT_DUTY;
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
               (s_output.mode == STRAIGHT_DRIVE_MODE_GYRO_HEADING)){
        s_output.heading_reference_valid = false;
        PID_Reset(&s_heading_pid);
    }
}

void StraightDrive_ZeroCommand(void)
{
    s_output.command = 0.0f;
    s_output.correction_percent = 0.0f;
    s_output.heading_reference_valid = false;
    StraightDrive_ResetPid();
}

BSP_STATUS StraightDrive_Update(float dt_s)
{
    StraightDrive_UpdateImu();
    BSP_STATUS status = StraightDrive_Apply(dt_s);
    StraightDrive_UpdateWheelState();
    return status;
}

STRAIGHT_DRIVE_OUTPUT StraightDrive_GetOutput(void)
{
    return s_output;
}

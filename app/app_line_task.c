/**
 * @file  app_line_task.c
 * @brief 循迹任务实现。
 *
 * 循迹闭环由 line_follow 完成。Yahboom 8 路循线与 JY61P 共用 I2C0，本任务负责总线
 * 时间片：只在 JY61P 异步事务空闲时切换总线所有权。
 */
#include "app_line_task.h"

#include "line_follow.h"
#include "chassis.h"
#include "kinematics/kinematics.h"
#include "wit_sdk.h"

#include "ui.h"
#include "app_fmt.h"
#include "bsp_time.h"
#include "bsp_common.h"
#include "debug_uart.h"
#include "yahboom_track.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* 刷屏节流周期。 */
#define LT_UI_PERIOD_MS 150U
/** Debug_Ex 运动遥测周期；50 ms 一行，给控制环和串口吞吐留出余量。 */
#define LT_TELEMETRY_PERIOD_MS 50U
/** 灰度最近一次成功读数的最大允许年龄。 */
#define LT_LINE_SENSOR_MAX_AGE_MS 60U
/** 弯道灰度瞬时全白时，允许纯曲率/陀螺保持的最长时间。 */
#define LT_CURVE_GYRO_ONLY_GRACE_MS 200U
/** 标称环线中心线周长：2*1.5 + 2*pi*0.5。 */
#define LT_NOMINAL_LAP_DISTANCE_M 6.1416f
/** 终点识别和减速只在标称 A 点前 0.4 m 内开放，避免起点横线误触发。 */
#define LT_FINISH_ARM_MARGIN_M 0.400f
/** 实测平行通过 A 点横线可稳定命中至少 3 路；末段首帧命中即制动。 */
#define LT_FINISH_ENTER_MIN_COUNT 3U
/** A 点横线漏检时允许的最大编码器超程。 */
#define LT_FINISH_ODOM_OVERRUN_M 0.200f
/** line_follow 默认 correction 的最终半幅（DIFFERENTIAL_LIMIT/2）。 */
#define LT_LINE_CORRECTION_LIMIT \
    (LINE_FOLLOW_DEFAULT_DIFFERENTIAL_LIMIT * 0.5f)
/** 进入终点停车态的剩余距离门限。 */
#define LT_STOP_DISTANCE_TOLERANCE_M 0.005f
/** 要求 4 的 B 点标称里程，以及通过 B 后的安全停车点。 */
#define LT_STRAIGHT_B_DISTANCE_M 1.500f
#define LT_STRAIGHT_STOP_DISTANCE_M 1.600f
#define LT_STRAIGHT_TERMINAL_BLIND_START_M 1.550f
/** H4 启用启动航向外环与陀螺角速度内环，灰度仍不参与转向控制。 */
#define LT_H4_LINE_FOLLOW_ENABLED 0U
#define LT_H4_CRUISE_SPEED_MPS 0.280f
#define LT_H4_ACCEL_LIMIT_MPS2 0.090f
#define LT_H4_JERK_LIMIT_MPS3 0.180f
#define LT_H4_GYRO_MAX_AGE_MS 60U
#define LT_H4_HEADING_YAW_SIGN (-1.0f)
#define LT_H4_HEADING_KP 2.0f
#define LT_H4_HEADING_OMEGA_LIMIT_DEGPS 12.0f
#define LT_H4_STARTUP_SPEED_MPS 0.060f
#define LT_H4_STARTUP_STABLE_TICKS 5U
#define LT_H4_STARTUP_TIMEOUT_MS 1200U
#define LT_H4_STARTUP_BLEND_MS 300U
#define LT_H4_TERMINAL_BLEND_MS 300U
#define LT_H4_SLIP_STEER_MAX_MPS 0.040f
#define LT_H4_SLIP_STEER_SPEED_RATIO 0.45f
#define LT_H4_STOP_COMMAND_MPS 0.001f
#define LT_H4_STOP_MEASURED_MPS 0.020f
/** 对称限 jerk 减速所需距离：0.5*v*(v/a + a/j)。 */
#define LT_H4_DECEL_DISTANCE_M \
    (0.5f * LT_H4_CRUISE_SPEED_MPS * \
     ((LT_H4_CRUISE_SPEED_MPS / LT_H4_ACCEL_LIMIT_MPS2) + \
      (LT_H4_ACCEL_LIMIT_MPS2 / LT_H4_JERK_LIMIT_MPS3)))
#define LT_H4_DECEL_START_M \
    (LT_STRAIGHT_STOP_DISTANCE_M - LT_H4_DECEL_DISTANCE_M)
#define LT_FINISH_CAPTURE_SPEED_EMPTY_MPS 0.150f
#define LT_FINISH_CAPTURE_SPEED_LOADED_MPS 0.120f
/** 制动包络只使用纵向加速度权限的一半，为 jerk 建立和执行延迟留距离。 */
#define LT_TERMINAL_BRAKE_ENVELOPE_RATIO 0.50f
/** 四段状态机的赛道标称几何，以及两轮接地点中心的实测轮距。 */
#define LT_TRACK_STRAIGHT_LENGTH_M 1.500f
#define LT_TRACK_CURVE_RADIUS_M 0.500f
#define LT_EFFECTIVE_TRACK_WIDTH_M 0.206f
/** 首轮实测弯道累计差速仅达理论值 87.5%，先用温和比例补偿滚阻/侧滑。 */
#define LT_CURVE_FEEDFORWARD_SCALE_S2 1.10f
#define LT_CURVE_FEEDFORWARD_SCALE_S4 1.00f
#define LT_PI 3.1415926f
#define LT_RAD_TO_DEG (180.0f / LT_PI)
/** H 题实测专用灰度角速度外环：降低增益并限制其残差权限。 */
#define LT_GYRO_LINE_KP 1.25f
#define LT_GYRO_LINE_LIMIT_EMPTY_DEGPS 20.0f
#define LT_GYRO_LINE_LIMIT_LOADED_DEGPS 15.0f
#define LT_GYRO_REF_LIMIT_EMPTY_DEGPS 75.0f
#define LT_GYRO_REF_LIMIT_LOADED_DEGPS 50.0f
/*
 * 编码器里程在灰度阵列位于 A 时清零，车辆相对原车尾基准向后摆放 19.5 cm。
 * 四段控制边界直接表示灰度阵列到达 B/C/D/A；轮轴位于阵列后方 12.5 cm。
 * S4 弯道控制保持到阵列重新识别 A 点横线并立即制动。
 */
#define LT_S1_OFFSET_M             0.000f   /* S1 入弯位置偏置 */
#define LT_S2_OFFSET_M             0.000f   /* S2 出弯位置偏置 */
#define LT_S3_OFFSET_M             0.000f   /* S3 入弯位置偏置 */

#define LT_SEGMENT_S1_END_M LT_TRACK_STRAIGHT_LENGTH_M
#define LT_SEGMENT_S2_END_M \
    (LT_SEGMENT_S1_END_M + LT_PI * LT_TRACK_CURVE_RADIUS_M)
#define LT_SEGMENT_S3_END_M \
    (LT_SEGMENT_S2_END_M + LT_TRACK_STRAIGHT_LENGTH_M)
#define LT_SEGMENT_S4_CURVE_END_M \
    (LT_SEGMENT_S3_END_M + LT_PI * LT_TRACK_CURVE_RADIUS_M)

/*
 * ===== 手动推车编码器标定点（沿前进方向里程，单位 m）=====
 * 当前默认使用以灰度阵列为测量点的标称几何值，后续可用实测编码器数据覆盖：
 *   LT_SEGMENT_S2_END_M       : S2(第一弯)->S3(第二直道) 切换里程
 *   LT_SEGMENT_S3_END_M       : S3->S4(第二弯) 切换里程
 *   LT_SEGMENT_S4_CURVE_END_M : 灰度阵列回到 A 点的标称里程
 *   LT_LAP_STOP_DISTANCE_M    : A 点横线的标称里程（仅用于减速参考）
 */
/**
 * 停车参考点微调偏移量（单位: m）。
 *  > 0 (如 +0.050f)：将减速/停车参考点整体后移 5cm
 *  < 0 (如 -0.050f)：将减速/停车参考点整体提前 5cm
 */
#define LT_LAP_STOP_OFFSET_M     0.100f  /* 终点位置偏置 */

#define LT_LAP_STOP_DISTANCE_M \
    (LT_NOMINAL_LAP_DISTANCE_M + LT_LAP_STOP_OFFSET_M)
#define LT_LAP_ODOM_FALLBACK_DISTANCE_M \
    (LT_LAP_STOP_DISTANCE_M + LT_FINISH_ODOM_OVERRUN_M)

/*
 * S2->S3 进入第二直道后，用陀螺仪(期望航向角速度=0)回正一段距离。
 * S4 后不再设置回正窗口，因为灰度阵列到达 A 点即为任务停车位置。
 */
#define LT_S3_GYRO_RECOVER_M     0.30f

typedef enum {
    LT_ROUTE_LAP = 0,
    LT_ROUTE_STRAIGHT,
} LT_ROUTE;

typedef enum {
    LT_SEGMENT_S1 = 0,
    LT_SEGMENT_S2,
    LT_SEGMENT_S3,
    LT_SEGMENT_S4,
} LT_SEGMENT;

typedef struct {
    uint8_t requirement;
    const char *title;
    LT_ROUTE route;
    float cruise_speed_mps;
    float accel_limit_mps2;
    float jerk_limit_mps3;
    float feedback_limit_mps;
    float curve_feedback_limit_mps;
    float steer_limit_mps;
    float steer_slew_mps2;
} LT_PROFILE;

/* H2 无球，可独立使用高速参数；H4/5/6 全部使用给滚球闭环留裕度的载球参数。 */
static const LT_PROFILE LT_PROFILE_H2 = {
    2U, "H2 Empty Lap", LT_ROUTE_LAP,
    0.450f, 0.300f, 0.600f, 0.078f, 0.035f, 0.120f, 0.180f,
};
static const LT_PROFILE LT_PROFILE_H4 = {
    4U, "H4 Loaded A-B", LT_ROUTE_STRAIGHT,
    LT_H4_CRUISE_SPEED_MPS, LT_H4_ACCEL_LIMIT_MPS2,
    LT_H4_JERK_LIMIT_MPS3, 0.045f, 0.025f, 0.070f, 0.100f,
};
static const LT_PROFILE LT_PROFILE_H5 = {
    5U, "H5 Loaded Lap O", LT_ROUTE_LAP,
    0.260f, 0.120f, 0.240f, 0.045f, 0.025f, 0.070f, 0.100f,
};
static const LT_PROFILE LT_PROFILE_H6 = {
    6U, "H6 Loaded Any", LT_ROUTE_LAP,
    0.260f, 0.120f, 0.240f, 0.045f, 0.025f, 0.070f, 0.100f,
};

typedef enum {
    LT_STATE_FOLLOW = 0,
    LT_STATE_FINISH_OFFSET,
    LT_STATE_STOPPED,
    LT_STATE_LINE_LOST,
} LT_STATE;

typedef enum {
    LT_FINISH_SOURCE_NONE = 0,
    LT_FINISH_SOURCE_LINE,
    LT_FINISH_SOURCE_ODOM,
    LT_FINISH_SOURCE_LINE_END,
} LT_FINISH_SOURCE;

typedef enum {
    LT_H4_PHASE_ACCEL = 0,
    LT_H4_PHASE_CRUISE,
    LT_H4_PHASE_DECEL,
    LT_H4_PHASE_PASS_B,
    LT_H4_PHASE_STOP,
} LT_H4_PHASE;

static uint32_t lt_last_ui;
static uint32_t lt_last_telemetry;
static uint32_t lt_start_ms;
static uint32_t lt_stop_ms;
static uint32_t lt_run_id;
static uint8_t lt_detected_mask;
static uint32_t lt_detected_mask_ms;
static bool lt_sensor_ready;
static bool lt_line_acquired;
static BSP_STATUS lt_sensor_init_status;
static LT_STATE lt_state;
static LT_FINISH_SOURCE lt_finish_source;
static float lt_finish_target_distance_m;
static float lt_speed_command_mps;
static float lt_terminal_speed_ceiling_mps;
static float lt_accel_command_mps2;
static float lt_steer_command_mps;
static float lt_curve_feedforward_mps;
static uint32_t lt_last_line_seen_ms;
static bool lt_curve_gyro_only;
static const LT_PROFILE *lt_profile;
static bool lt_straight_b_passed;
static uint32_t lt_straight_b_ms;
static LT_H4_PHASE lt_h4_phase;
static bool lt_h4_heading_reference_valid;
static float lt_h4_heading_reference_deg;
static float lt_h4_heading_error_deg;
static float lt_h4_heading_omega_ref_deg_s;
static uint32_t lt_h4_motion_start_ms;
static uint32_t lt_h4_startup_blend_ms;
static uint32_t lt_h4_terminal_blend_ms;
static uint8_t lt_h4_startup_stable_ticks;
static bool lt_h4_startup_complete;
static LT_SEGMENT lt_segment;
/* 入直道后的陀螺仪回正窗口：active 期间忽略灰度、按 gz 走直线，直到里程达 end。 */
static bool lt_gyro_recover_active;
static float lt_gyro_recover_end_m;

/*
 * Yahboom 循线是阻塞驱动，JY61P 是中断状态机。app 层负责共享 I2C0 的时间片：
 * 只在 JY61P 完全空闲时挂起它、完成一次灰度读取，再恢复并启动下一帧 IMU。
 */
static void LineSensor_Enter(void){
    JY61P_I2C_SetSuspended(true);
    lt_sensor_init_status = YahboomTrack_Init();
    JY61P_I2C_Init();

    lt_detected_mask = 0U;
    lt_detected_mask_ms = 0U;
    lt_sensor_ready = false;
    lt_line_acquired = false;
}

static bool LineSensor_Tick(uint8_t *detected_mask){
    uint32_t now = BSP_Time_GetMs();

    if ((lt_sensor_init_status == BSP_STATUS_OK) && JY61P_I2C_IsIdle()){
        uint8_t latest_mask;
        JY61P_I2C_SetSuspended(true);
        /* ReadDetectedMask 已归一化为 line_follow 约定: bit0=X1 … bit7=X8, 1=检测到黑线。 */
        BSP_STATUS status = YahboomTrack_ReadDetectedMask(&latest_mask);
        JY61P_I2C_SetSuspended(false);
        if (status == BSP_STATUS_OK){
            lt_detected_mask = latest_mask;
            lt_detected_mask_ms = now;
            lt_sensor_ready = true;
        }
    }

    /* 读取灰度后才 kick JY61P；控制器本拍消费上一帧完整 IMU 快照。 */
    JY61P_I2C_Poll();
    *detected_mask = lt_detected_mask;
    return lt_sensor_ready &&
           ((uint32_t)(now - lt_detected_mask_ms) <= LT_LINE_SENSOR_MAX_AGE_MS);
}

/* 把字符串拷入 buf, 返回长度(不终止), 便于随后接 AppFmt_* 拼数字。 */
static uint8_t LtPutStr(char *buf, const char *s){
    uint8_t i = 0U;
    while (s[i] != '\0'){
        buf[i] = s[i];
        i++;
    }
    return i;
}

static const char *LineFollowTest_StateName(void){
    switch (lt_state){
        case LT_STATE_FOLLOW:
            return "FOLLOW";
        case LT_STATE_FINISH_OFFSET:
            return "OFFSET";
        case LT_STATE_STOPPED:
            return "STOP";
        case LT_STATE_LINE_LOST:
            return "LOST";
        default:
            return "?";
    }
}

static const char *LineFollowTest_FinishSourceName(void){
    switch (lt_finish_source){
        case LT_FINISH_SOURCE_LINE:
            return "LINE";
        case LT_FINISH_SOURCE_ODOM:
            return "ODOM";
        case LT_FINISH_SOURCE_LINE_END:
            return "END";
        default:
            return "NONE";
    }
}

static const char *LineFollowTest_SegmentName(void){
    if ((lt_profile != NULL) && (lt_profile->requirement == 4U)){
        switch (lt_h4_phase){
            case LT_H4_PHASE_ACCEL:
                return "ACCEL";
            case LT_H4_PHASE_CRUISE:
                return "CRUISE";
            case LT_H4_PHASE_DECEL:
                return "DECEL";
            case LT_H4_PHASE_PASS_B:
                return "PASS_B";
            case LT_H4_PHASE_STOP:
                return "STOP";
            default:
                return "?";
        }
    }

    switch (lt_segment){
        case LT_SEGMENT_S1:
            return "S1";
        case LT_SEGMENT_S2:
            return "S2";
        case LT_SEGMENT_S3:
            return "S3";
        case LT_SEGMENT_S4:
            return "S4";
        default:
            return "?";
    }
}

/* 末段只按里程武装：在此之前忽略 A 起点横线和赛道中的宽线干扰。 */
static bool LineFollowTest_InLapFinishApproach(float distance_m){
    return (lt_profile != NULL) && (lt_profile->route == LT_ROUTE_LAP) &&
        (lt_segment == LT_SEGMENT_S4) &&
        (distance_m >= (LT_LAP_STOP_DISTANCE_M - LT_FINISH_ARM_MARGIN_M));
}

/* 开启一段陀螺仪回正窗口，直到里程达到 end_distance_m。 */
static void LineFollowTest_ArmGyroRecover(float end_distance_m){
    lt_gyro_recover_active = true;
    lt_gyro_recover_end_m = end_distance_m;
}

static void LineFollowTest_UpdateSegment(uint32_t now, float distance_m){
    if ((lt_profile == NULL) || (lt_profile->route != LT_ROUTE_LAP)){
        return;
    }

    LT_SEGMENT previous = lt_segment;
    if ((lt_segment == LT_SEGMENT_S1) &&
        (distance_m >= LT_SEGMENT_S1_END_M)){
        lt_segment = LT_SEGMENT_S2;
    }
    if ((lt_segment == LT_SEGMENT_S2) &&
        (distance_m >= LT_SEGMENT_S2_END_M)){
        lt_segment = LT_SEGMENT_S3;
        /* 改动1：S2 出弯进 S3 用陀螺仪回正一段，再交还灰度循迹。 */
        LineFollowTest_ArmGyroRecover(LT_SEGMENT_S2_END_M + LT_S3_GYRO_RECOVER_M);
    }
    if ((lt_segment == LT_SEGMENT_S3) &&
        (distance_m >= LT_SEGMENT_S3_END_M)){
        lt_segment = LT_SEGMENT_S4;
    }

    if (lt_segment != previous){
        DebugUart_Printf("[TRK] segment req=%u seg=%s t=%lu s=%.3f\r\n",
            (unsigned int)lt_profile->requirement,
            LineFollowTest_SegmentName(),
            (unsigned long)(now - lt_start_ms), distance_m);
    }
}

static float LineFollowTest_FinishRemaining(float distance_m){
    if ((lt_profile != NULL) && (lt_profile->route == LT_ROUTE_STRAIGHT)){
        float remaining_m = LT_STRAIGHT_STOP_DISTANCE_M - distance_m;
        return (remaining_m > 0.0f) ? remaining_m : 0.0f;
    }

    /* 直道循迹终点补偿（H4 用 FINISH_OFFSET）。 */
    if (lt_state == LT_STATE_FINISH_OFFSET){
        float remaining_m = lt_finish_target_distance_m - distance_m;
        return (remaining_m > 0.0f) ? remaining_m : 0.0f;
    }

    /* LAP：末段编码器只负责提供到 A 横线的减速参考。 */
    if (LineFollowTest_InLapFinishApproach(distance_m)){
        float remaining_m = LT_LAP_STOP_DISTANCE_M - distance_m;
        return (remaining_m > 0.0f) ? remaining_m : 0.0f;
    }
    return 0.0f;
}

static bool LineFollowTest_IsFinishLine(float distance_m, bool sensor_ready,
                                        uint8_t detected_mask){
    if ((lt_state != LT_STATE_FOLLOW) || !sensor_ready ||
        !LineFollowTest_InLapFinishApproach(distance_m)){
        return false;
    }

    LINE_FOLLOW_OBSERVATION observation;
    if (LineFollow_ObserveDetectedMask(detected_mask, 1.0f, &observation) !=
        BSP_STATUS_OK){
        return false;
    }
    return observation.black_count >= LT_FINISH_ENTER_MIN_COUNT;
}

static float LineFollowTest_MoveTowards(float value, float target,
                                        float max_step){
    if (value < target){
        value += max_step;
        return (value < target) ? value : target;
    }
    if (value > target){
        value -= max_step;
        return (value > target) ? value : target;
    }
    return value;
}

static void LineFollowTest_ApplySpeedLimit(float speed_limit_mps, float dt_s){
    float previous_speed_mps = lt_speed_command_mps;
    float accel_target_mps2 = 0.0f;
    if (lt_speed_command_mps < speed_limit_mps){
        float speed_margin_mps = speed_limit_mps - lt_speed_command_mps;
        float ramp_down_margin_mps =
            (lt_accel_command_mps2 > 0.0f)
                ? (lt_accel_command_mps2 * lt_accel_command_mps2) /
                      (2.0f * lt_profile->jerk_limit_mps3)
                : 0.0f;
        accel_target_mps2 = (speed_margin_mps <= ramp_down_margin_mps)
                                ? 0.0f
                                : lt_profile->accel_limit_mps2;
    } else if (lt_speed_command_mps > speed_limit_mps){
        float speed_margin_mps = lt_speed_command_mps - speed_limit_mps;
        float ramp_down_margin_mps =
            (lt_accel_command_mps2 < 0.0f)
                ? (lt_accel_command_mps2 * lt_accel_command_mps2) /
                      (2.0f * lt_profile->jerk_limit_mps3)
                : 0.0f;
        accel_target_mps2 = (speed_margin_mps <= ramp_down_margin_mps)
                                ? 0.0f
                                : -lt_profile->accel_limit_mps2;
    }

    lt_accel_command_mps2 = LineFollowTest_MoveTowards(
        lt_accel_command_mps2, accel_target_mps2,
        lt_profile->jerk_limit_mps3 * dt_s);
    lt_speed_command_mps += lt_accel_command_mps2 * dt_s;
    if (lt_speed_command_mps < 0.0f){
        lt_speed_command_mps = 0.0f;
    } else if ((previous_speed_mps <= speed_limit_mps) &&
               (lt_speed_command_mps > speed_limit_mps)){
        lt_speed_command_mps = speed_limit_mps;
    } else if ((previous_speed_mps > speed_limit_mps) &&
               (lt_speed_command_mps < speed_limit_mps)){
        lt_speed_command_mps = speed_limit_mps;
    }
}

static bool H4_UpdateHeadingReference(uint32_t now){
    JY61P_I2C_SAMPLE sample;
    if (!JY61P_I2C_IsDataFresh(LT_H4_GYRO_MAX_AGE_MS) ||
        !JY61P_I2C_GetSnapshot(&sample)){
        return false;
    }

    float yaw_deg = Kinematics_NormalizeAngleDeg(
        LT_H4_HEADING_YAW_SIGN * sample.data.attitude_deg.yaw);
    if (!lt_h4_heading_reference_valid){
        lt_h4_heading_reference_valid = true;
        lt_h4_heading_reference_deg = yaw_deg;
        lt_h4_motion_start_ms = now;
        DebugUart_Printf(
            "[TRK] H4 heading lock run=%lu t=%lu href=%.1f\r\n",
            (unsigned long)lt_run_id, (unsigned long)(now - lt_start_ms),
            lt_h4_heading_reference_deg);
    }

    lt_h4_heading_error_deg = Kinematics_AngleDiffDeg(
        lt_h4_heading_reference_deg, yaw_deg);
    lt_h4_heading_omega_ref_deg_s = Kinematics_Clamp(
        LT_H4_HEADING_KP * lt_h4_heading_error_deg,
        -LT_H4_HEADING_OMEGA_LIMIT_DEGPS,
        LT_H4_HEADING_OMEGA_LIMIT_DEGPS);
    return true;
}

static void H4_UpdateStartupState(uint32_t now){
    if (lt_h4_startup_complete || !lt_h4_heading_reference_valid){
        return;
    }

    float measured_speed_mps = 0.5f *
        (fabsf(Chassis_GetWheelSpeed(HALL_ENCODER_LEFT)) +
         fabsf(Chassis_GetWheelSpeed(HALL_ENCODER_RIGHT)));
    if (measured_speed_mps >= LT_H4_STARTUP_SPEED_MPS){
        if (lt_h4_startup_stable_ticks < LT_H4_STARTUP_STABLE_TICKS){
            lt_h4_startup_stable_ticks++;
        }
    } else{
        lt_h4_startup_stable_ticks = 0U;
    }

    uint32_t startup_ms = now - lt_h4_motion_start_ms;
    if ((lt_h4_startup_stable_ticks >= LT_H4_STARTUP_STABLE_TICKS) ||
        (startup_ms >= LT_H4_STARTUP_TIMEOUT_MS)){
        lt_h4_startup_complete = true;
        lt_h4_startup_blend_ms = now;
        DebugUart_Printf(
            "[TRK] H4 startup done run=%lu t=%lu v=%.3f reason=%s\r\n",
            (unsigned long)lt_run_id, (unsigned long)(now - lt_start_ms),
            measured_speed_mps,
            (lt_h4_startup_stable_ticks >= LT_H4_STARTUP_STABLE_TICKS)
                ? "SPEED" : "TIME");
    }
}

static void H4_UpdateHeadingSteering(const LINE_FOLLOW_OUTPUT *out,
                                     uint32_t now, float dt_s){
    float speed_ratio = lt_speed_command_mps / LT_H4_CRUISE_SPEED_MPS;
    float normal_authority_mps = lt_profile->feedback_limit_mps * speed_ratio;
    float slip_authority_mps =
        LT_H4_SLIP_STEER_SPEED_RATIO * lt_speed_command_mps;
    if (slip_authority_mps > LT_H4_SLIP_STEER_MAX_MPS){
        slip_authority_mps = LT_H4_SLIP_STEER_MAX_MPS;
    }

    float authority_mps;
    if (lt_h4_phase >= LT_H4_PHASE_DECEL){
        /* 减速停车与起步使用相同的防打滑权限，并从巡航权限平滑放宽。 */
        float terminal_blend = (float)(now - lt_h4_terminal_blend_ms) /
                               (float)LT_H4_TERMINAL_BLEND_MS;
        if (terminal_blend > 1.0f){
            terminal_blend = 1.0f;
        }
        authority_mps = normal_authority_mps +
            (slip_authority_mps - normal_authority_mps) * terminal_blend;
    } else{
        float normal_blend = 0.0f;
        if (lt_h4_startup_complete){
            normal_blend = (float)(now - lt_h4_startup_blend_ms) /
                           (float)LT_H4_STARTUP_BLEND_MS;
            if (normal_blend > 1.0f){
                normal_blend = 1.0f;
            }
        }
        authority_mps = slip_authority_mps +
            (normal_authority_mps - slip_authority_mps) * normal_blend;
    }
    float steer_target_mps =
        (out->correction / LT_LINE_CORRECTION_LIMIT) * authority_mps;
    lt_steer_command_mps = LineFollowTest_MoveTowards(
        lt_steer_command_mps, steer_target_mps,
        lt_profile->steer_slew_mps2 * dt_s);
}

static void H4_UpdateLongitudinal(uint32_t now, float distance_m, float dt_s){
    if ((distance_m >= LT_H4_DECEL_START_M) &&
        (lt_h4_phase < LT_H4_PHASE_DECEL)){
        lt_h4_phase = LT_H4_PHASE_DECEL;
        lt_h4_terminal_blend_ms = now;
        DebugUart_Printf(
            "[TRK] H4 decel run=%lu t=%lu s=%.3f start=%.3f\r\n",
            (unsigned long)lt_run_id, (unsigned long)(now - lt_start_ms),
            distance_m, LT_H4_DECEL_START_M);
    } else if ((lt_h4_phase == LT_H4_PHASE_ACCEL) &&
               (lt_speed_command_mps >=
                (LT_H4_CRUISE_SPEED_MPS - 0.001f))){
        lt_h4_phase = LT_H4_PHASE_CRUISE;
    }

    float speed_limit_mps =
        (lt_h4_phase >= LT_H4_PHASE_DECEL) ? 0.0f
                                           : LT_H4_CRUISE_SPEED_MPS;
    LineFollowTest_ApplySpeedLimit(speed_limit_mps, dt_s);
}

static bool H4_IsMotionStopped(void){
    return (lt_h4_phase >= LT_H4_PHASE_DECEL) &&
        (lt_speed_command_mps <= LT_H4_STOP_COMMAND_MPS) &&
        (fabsf(Chassis_GetWheelSpeed(HALL_ENCODER_LEFT)) <=
         LT_H4_STOP_MEASURED_MPS) &&
        (fabsf(Chassis_GetWheelSpeed(HALL_ENCODER_RIGHT)) <=
         LT_H4_STOP_MEASURED_MPS);
}

/*
 * 纵向 S 曲线：先限制加速度变化率，再积分得到速度。接近终点时用
 * v<=sqrt(2*a*s) 的制动包络提前降速；A 横线命中后再立即电子制动。
 */
static void LineFollowTest_UpdateLongitudinal(float distance_m, float dt_s){
    float speed_limit_mps = lt_profile->cruise_speed_mps;
    float remaining_m = LineFollowTest_FinishRemaining(distance_m);
    /*
     * LAP 在 A 点前 0.4 m 进入减速区，编码器只提供减速参考；正常停车由横线触发。
     * 直道题(H4)沿用原终点补偿。
     */
    bool terminal_approach = (lt_profile->route == LT_ROUTE_STRAIGHT) ||
        (lt_state == LT_STATE_FINISH_OFFSET) ||
        LineFollowTest_InLapFinishApproach(distance_m);
    if (terminal_approach){
        float terminal_limit_mps = lt_profile->cruise_speed_mps;
        if (LineFollowTest_InLapFinishApproach(distance_m)){
            terminal_limit_mps = (lt_profile->requirement == 2U)
                                     ? LT_FINISH_CAPTURE_SPEED_EMPTY_MPS
                                     : LT_FINISH_CAPTURE_SPEED_LOADED_MPS;
        }
        if (remaining_m > 0.0f){
            float brake_envelope_mps2 = lt_profile->accel_limit_mps2 *
                                        LT_TERMINAL_BRAKE_ENVELOPE_RATIO;
            float brake_limit_mps = sqrtf(2.0f * brake_envelope_mps2 *
                                          remaining_m);
            if (brake_limit_mps < terminal_limit_mps){
                terminal_limit_mps = brake_limit_mps;
            }
        } else{
            /* 越过标称 A 点后继续把速度目标平滑压向零，等待横线或超程兜底。 */
            terminal_limit_mps = 0.0f;
        }
        /* 终点阶段速度上限只能下降，禁止越过标称 A 点后再次加速。 */
        if (terminal_limit_mps < lt_terminal_speed_ceiling_mps){
            lt_terminal_speed_ceiling_mps = terminal_limit_mps;
        }
        speed_limit_mps = lt_terminal_speed_ceiling_mps;
    } else{
        lt_terminal_speed_ceiling_mps = lt_profile->cruise_speed_mps;
    }

    LineFollowTest_ApplySpeedLimit(speed_limit_mps, dt_s);
}

static bool LineFollowTest_IsRightCurve(float distance_m){
    (void)distance_m;
    return (lt_profile->route == LT_ROUTE_LAP) &&
        ((lt_segment == LT_SEGMENT_S2) ||
         (lt_segment == LT_SEGMENT_S4));
}

static void LineFollowTest_UpdateCurveFeedforward(float distance_m, float dt_s){
    float target_mps = 0.0f;
    if (LineFollowTest_IsRightCurve(distance_m)){
        float feedforward_scale = (lt_segment == LT_SEGMENT_S4)
                                      ? LT_CURVE_FEEDFORWARD_SCALE_S4
                                      : LT_CURVE_FEEDFORWARD_SCALE_S2;
        target_mps = lt_speed_command_mps *
            (LT_EFFECTIVE_TRACK_WIDTH_M / (2.0f * LT_TRACK_CURVE_RADIUS_M)) *
            feedforward_scale;
    }
    lt_curve_feedforward_mps = LineFollowTest_MoveTowards(
        lt_curve_feedforward_mps, target_mps,
        lt_profile->steer_slew_mps2 * dt_s);
}

static float LineFollowTest_CurveOmegaFeedforwardDegS(float distance_m){
    (void)distance_m;
    /* 与已建立的轮速半差速同步，避免分段切换时角速度参考阶跃。 */
    return (2.0f * lt_curve_feedforward_mps /
            LT_EFFECTIVE_TRACK_WIDTH_M) * LT_RAD_TO_DEG;
}

static void LineFollowTest_UpdateSteering(const LINE_FOLLOW_OUTPUT *out,
                                          float distance_m, float dt_s){
    /* 低速时同比收窄反馈与总半差速，确保停车阶段两轮都不反转成原地旋转。 */
    float speed_ratio = lt_speed_command_mps /
                        lt_profile->cruise_speed_mps;
    bool right_curve = LineFollowTest_IsRightCurve(distance_m);
    bool curve_control_active = right_curve ||
        (fabsf(lt_curve_feedforward_mps) > 0.0001f);
    float feedback_limit_mps =
        (curve_control_active ? lt_profile->curve_feedback_limit_mps
                              : lt_profile->feedback_limit_mps) * speed_ratio;
    float steer_limit_mps = lt_profile->steer_limit_mps *
                            speed_ratio;

    float feedback_mps = (out->correction / LT_LINE_CORRECTION_LIMIT) *
                         feedback_limit_mps;
    float steer_target_mps = lt_curve_feedforward_mps + feedback_mps;
    if (steer_target_mps > steer_limit_mps){
        steer_target_mps = steer_limit_mps;
    } else if (steer_target_mps < -steer_limit_mps){
        steer_target_mps = -steer_limit_mps;
    }
    lt_steer_command_mps = LineFollowTest_MoveTowards(
        lt_steer_command_mps, steer_target_mps,
        lt_profile->steer_slew_mps2 * dt_s);
}

static void LineFollowTest_Telemetry(uint32_t now, bool sensor_ready){
    LINE_FOLLOW_OUTPUT out = LineFollow_GetOutput();
    JY61P_I2C_SAMPLE imu_sample;
    float yaw_deg = JY61P_I2C_GetSnapshot(&imu_sample)
                        ? imu_sample.data.attitude_deg.yaw
                        : 0.0f;
    if (lt_profile->requirement == 4U){
        yaw_deg = Kinematics_NormalizeAngleDeg(
            LT_H4_HEADING_YAW_SIGN * yaw_deg);
    }
    LINE_FOLLOW_OBSERVATION observation;
    BSP_STATUS observation_status = LineFollow_ObserveDetectedMask(
        lt_detected_mask, 1.0f, &observation);
    uint8_t black_count = (observation_status == BSP_STATUS_OK)
                              ? observation.black_count
                              : out.black_count;
    float line_error = (observation_status == BSP_STATUS_OK)
                           ? observation.error
                           : out.error;
    CHASSIS_DUTY duty = Chassis_GetDuty();
    float distance_m = Chassis_GetDistance();

    DebugUart_Printf(
        "[TRK] t=%lu run=%lu req=%u seg=%s st=%s fs=%s gm=%u sen=%u mask=%02X n=%u err=%.1f cor=%.2f vc=%.3f ac=%.3f "
        "vs=%.3f wref=%.1f wz=%.1f yaw=%.1f href=%.1f herr=%.1f hs=%u vl=%.3f vr=%.3f dl=%.1f dr=%.1f sl=%.3f sr=%.3f s=%.3f "
        "rem=%.3f drop=%lu\r\n",
        (unsigned long)(now - lt_start_ms),
        (unsigned long)lt_run_id, (unsigned int)lt_profile->requirement,
        LineFollowTest_SegmentName(), LineFollowTest_StateName(),
        LineFollowTest_FinishSourceName(),
        lt_curve_gyro_only ? 1U : 0U,
        sensor_ready ? 1U : 0U, (unsigned int)lt_detected_mask,
        (unsigned int)black_count, line_error, out.correction,
        lt_speed_command_mps, lt_accel_command_mps2,
        lt_steer_command_mps,
        out.omega_ref_deg_s, out.omega_measured_deg_s, yaw_deg,
        lt_h4_heading_reference_deg, lt_h4_heading_error_deg,
        lt_h4_startup_complete ? 1U : 0U,
        Chassis_GetWheelSpeed(HALL_ENCODER_LEFT),
        Chassis_GetWheelSpeed(HALL_ENCODER_RIGHT),
        duty.left_percent, duty.right_percent,
        Chassis_GetWheelDistance(HALL_ENCODER_LEFT),
        Chassis_GetWheelDistance(HALL_ENCODER_RIGHT), distance_m,
        LineFollowTest_FinishRemaining(distance_m),
        (unsigned long)DebugUart_GetDroppedBytes());
}

static void LineFollowTask_EnterCommon(const LT_PROFILE *profile){
    lt_profile = profile;
    lt_start_ms = BSP_Time_GetMs();
    lt_run_id++;
    if (lt_run_id == 0U){
        lt_run_id = 1U;
    }
    LINE_FOLLOW_CONFIG line_config = LineFollow_GetDefaultConfig();
    line_config.gyro_line_kp = LT_GYRO_LINE_KP;
    if (profile->requirement == 2U){
        line_config.omega_line_limit = LT_GYRO_LINE_LIMIT_EMPTY_DEGPS;
        line_config.omega_ref_limit = LT_GYRO_REF_LIMIT_EMPTY_DEGPS;
    } else{
        line_config.omega_line_limit = LT_GYRO_LINE_LIMIT_LOADED_DEGPS;
        line_config.omega_ref_limit = LT_GYRO_REF_LIMIT_LOADED_DEGPS;
    }
    LineFollow_Init(&line_config);
    LineSensor_Enter();
    lt_last_ui = 0U;
    lt_last_telemetry = 0U;
    lt_stop_ms = 0U;
    lt_state = LT_STATE_FOLLOW;
    lt_finish_source = LT_FINISH_SOURCE_NONE;
    lt_finish_target_distance_m =
        (profile->route == LT_ROUTE_STRAIGHT)
            ? LT_STRAIGHT_STOP_DISTANCE_M
            : LT_NOMINAL_LAP_DISTANCE_M;
    lt_speed_command_mps = 0.0f;
    lt_terminal_speed_ceiling_mps = profile->cruise_speed_mps;
    lt_accel_command_mps2 = 0.0f;
    lt_steer_command_mps = 0.0f;
    lt_curve_feedforward_mps = 0.0f;
    lt_last_line_seen_ms = lt_start_ms;
    lt_curve_gyro_only = false;
    lt_straight_b_passed = false;
    lt_straight_b_ms = 0U;
    lt_h4_phase = LT_H4_PHASE_ACCEL;
    lt_h4_heading_reference_valid = false;
    lt_h4_heading_reference_deg = 0.0f;
    lt_h4_heading_error_deg = 0.0f;
    lt_h4_heading_omega_ref_deg_s = 0.0f;
    lt_h4_motion_start_ms = 0U;
    lt_h4_startup_blend_ms = 0U;
    lt_h4_terminal_blend_ms = 0U;
    lt_h4_startup_stable_ticks = 0U;
    lt_h4_startup_complete = false;
    lt_segment = LT_SEGMENT_S1;
    lt_gyro_recover_active = false;
    lt_gyro_recover_end_m = 0.0f;

    DebugUart_Printf(
        "[TRK] --- enter run=%lu req=%u measure=sensor rear->axle=%.3fm rear->sensor=%.3fm "
        "sensor->axle=%.3fm track=%.3fm radius=%.3fm v=%.3f a=%.3f "
        "gkp=%.2f glim=%.1f ff2=%.2f ff4=%.2f pipe=%s ---\r\n",
        (unsigned long)lt_run_id, (unsigned int)profile->requirement,
        APP_TRACK_REAR_TO_AXLE_M, APP_TRACK_REAR_TO_SENSOR_M,
        APP_TRACK_SENSOR_TO_AXLE_M,
        LT_EFFECTIVE_TRACK_WIDTH_M, LT_TRACK_CURVE_RADIUS_M,
        profile->cruise_speed_mps, profile->accel_limit_mps2,
        line_config.gyro_line_kp, line_config.omega_line_limit,
        LT_CURVE_FEEDFORWARD_SCALE_S2, LT_CURVE_FEEDFORWARD_SCALE_S4,
        (profile->requirement >= 4U) ? "off" : "n/a");
}

static void H2_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H2); }
static void H4_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H4); }
static void H5_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H5); }
static void H6_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H6); }

static void LineFollowTest_Stop(uint32_t now, float distance_m){
    (void)Chassis_Brake();
    lt_speed_command_mps = 0.0f;
    lt_accel_command_mps2 = 0.0f;
    lt_steer_command_mps = 0.0f;
    lt_curve_feedforward_mps = 0.0f;
    lt_stop_ms = now;
    lt_state = LT_STATE_STOPPED;
    if (lt_profile->requirement == 4U){
        lt_h4_phase = LT_H4_PHASE_STOP;
    }
    DebugUart_Printf(
        "[TRK] stopped run=%lu fs=%s t=%lu s=%.3f target=%.3f\r\n",
        (unsigned long)lt_run_id, LineFollowTest_FinishSourceName(),
        (unsigned long)(lt_stop_ms - lt_start_ms), distance_m,
        lt_finish_target_distance_m);
}

static APP_TASK_STATUS LineFollowTest_Tick(float dt){
    uint8_t detected_mask;
    bool sensor_ready = LineSensor_Tick(&detected_mask);
    uint32_t now = BSP_Time_GetMs();
    float distance_m = Chassis_GetDistance();

    lt_curve_gyro_only = false;
    if (sensor_ready && (detected_mask != 0U)){
        lt_last_line_seen_ms = now;
    }

    LineFollowTest_UpdateSegment(now, distance_m);

    /* S2->S3 回正窗口走完即交还灰度循迹。 */
    if (lt_gyro_recover_active && (distance_m >= lt_gyro_recover_end_m)){
        lt_gyro_recover_active = false;
        DebugUart_Printf(
            "[TRK] gyro recover done run=%lu t=%lu seg=%s s=%.3f\r\n",
            (unsigned long)lt_run_id, (unsigned long)(now - lt_start_ms),
            LineFollowTest_SegmentName(), distance_m);
    }

    if ((lt_profile->route == LT_ROUTE_STRAIGHT) &&
        !lt_straight_b_passed &&
        (distance_m >= LT_STRAIGHT_B_DISTANCE_M)){
        lt_straight_b_passed = true;
        lt_straight_b_ms = now;
        if (lt_profile->requirement == 4U){
            lt_h4_phase = LT_H4_PHASE_PASS_B;
        }
        DebugUart_Printf("[TRK] pass B req=4 t=%lu s=%.3f\r\n",
            (unsigned long)(now - lt_start_ms), distance_m);
    }

    bool finish_line_detected = LineFollowTest_IsFinishLine(
        distance_m, sensor_ready, detected_mask);

    if ((LT_H4_LINE_FOLLOW_ENABLED != 0U) &&
        (lt_profile->route == LT_ROUTE_STRAIGHT) &&
        (lt_state == LT_STATE_FOLLOW) && lt_straight_b_passed &&
        (distance_m >= LT_STRAIGHT_TERMINAL_BLIND_START_M) &&
        sensor_ready && (detected_mask == 0U)){
        /* B 后末端纵线消失属于正常终点区，不应锁存成循迹丢线。 */
        lt_finish_source = LT_FINISH_SOURCE_LINE_END;
        lt_finish_target_distance_m = LT_STRAIGHT_STOP_DISTANCE_M;
        lt_state = LT_STATE_FINISH_OFFSET;
        DebugUart_Printf(
            "[TRK] straight line end run=%lu t=%lu s=%.3f target=%.3f\r\n",
            (unsigned long)lt_run_id,
            (unsigned long)(now - lt_start_ms), distance_m,
            lt_finish_target_distance_m);
    }

    /* LAP 正常由 A 横线停车；编码器只在横线漏检并超程 0.2 m 时兜底。 */
    bool lap_odom_fallback =
        (lt_state == LT_STATE_FOLLOW) &&
        (lt_profile->route == LT_ROUTE_LAP) &&
        (lt_segment == LT_SEGMENT_S4) &&
        (distance_m >= LT_LAP_ODOM_FALLBACK_DISTANCE_M);
    bool finish_position_reached =
        (lt_state != LT_STATE_STOPPED) &&
        (lt_state != LT_STATE_LINE_LOST) &&
        (((lt_profile->route == LT_ROUTE_STRAIGHT) &&
          (distance_m >= (LT_STRAIGHT_STOP_DISTANCE_M -
                           LT_STOP_DISTANCE_TOLERANCE_M))) ||
         finish_line_detected || lap_odom_fallback);
    if (finish_position_reached){
        if (finish_line_detected){
            lt_finish_source = LT_FINISH_SOURCE_LINE;
        } else if (lt_finish_source == LT_FINISH_SOURCE_NONE){
            lt_finish_source = LT_FINISH_SOURCE_ODOM;
        }
        LineFollowTest_Stop(now, distance_m);
    }

    if ((lt_state != LT_STATE_STOPPED) &&
        (lt_state != LT_STATE_LINE_LOST)){
        if (lt_profile->requirement == 4U){
            /* H4：启动航向外环给出 omega_ref，gz 内环生成反对称轮速差。 */
            LINE_FOLLOW_OUTPUT gyro_out;
            BSP_STATUS gyro_status = BSP_STATUS_NOT_READY;
            bool heading_sample_ready = H4_UpdateHeadingReference(now);
            if (!lt_h4_heading_reference_valid){
                /* 必须先锁存静止时的航向，避免运动后才把偏航位置当作目标。 */
                (void)Chassis_Brake();
                lt_speed_command_mps = 0.0f;
                lt_accel_command_mps2 = 0.0f;
                lt_steer_command_mps = 0.0f;
                lt_curve_feedforward_mps = 0.0f;
            } else{
                H4_UpdateLongitudinal(now, distance_m, dt);
                H4_UpdateStartupState(now);
                lt_curve_feedforward_mps = 0.0f;
            }
            if (heading_sample_ready){
                gyro_status = LineFollow_EvaluateOmegaFeedforwardOnly(
                    lt_h4_heading_omega_ref_deg_s, &gyro_out);
            }
            lt_curve_gyro_only = (gyro_status == BSP_STATUS_OK);
            if (lt_curve_gyro_only){
                H4_UpdateHeadingSteering(&gyro_out, now, dt);
            } else if (lt_h4_heading_reference_valid){
                /* IMU 暂时不可用时保留纵向运动，并平滑退化为双轮等速。 */
                lt_steer_command_mps = LineFollowTest_MoveTowards(
                    lt_steer_command_mps, 0.0f,
                    lt_profile->steer_slew_mps2 * dt);
            }
            if (H4_IsMotionStopped()){
                if (lt_finish_source == LT_FINISH_SOURCE_NONE){
                    lt_finish_source = LT_FINISH_SOURCE_ODOM;
                }
                LineFollowTest_Stop(now, distance_m);
            } else if (lt_h4_heading_reference_valid){
                Chassis_SetWheelSpeed(
                    lt_speed_command_mps + lt_steer_command_mps,
                    lt_speed_command_mps - lt_steer_command_mps);
            }
        } else if (lt_state == LT_STATE_FINISH_OFFSET){
            /* 横线中心锁存后只走几何补偿距离，直行可避免宽线退出后的错误转向。 */
            LineFollowTest_UpdateLongitudinal(distance_m, dt);
            lt_curve_feedforward_mps = LineFollowTest_MoveTowards(
                lt_curve_feedforward_mps, 0.0f,
                lt_profile->steer_slew_mps2 * dt);
            lt_steer_command_mps = LineFollowTest_MoveTowards(
                lt_steer_command_mps, 0.0f,
                lt_profile->steer_slew_mps2 * dt);
            Chassis_SetWheelSpeed(
                lt_speed_command_mps + lt_steer_command_mps,
                lt_speed_command_mps - lt_steer_command_mps);
        } else{
            LINE_FOLLOW_OUTPUT control_out;
            LineFollowTest_UpdateCurveFeedforward(distance_m, dt);
            float curve_omega_ff_deg_s =
                LineFollowTest_CurveOmegaFeedforwardDegS(distance_m);
            BSP_STATUS st;
            if (lt_gyro_recover_active){
                /*
                 * 改动1/2：入直道回正窗口内忽略灰度，仅按 gz 把航向角速度收敛到 0
                 * 走直线；陀螺快照不可用时退回灰度循迹，避免误刹。
                 */
                st = LineFollow_EvaluateOmegaFeedforwardOnly(0.0f, &control_out);
                lt_curve_gyro_only = (st == BSP_STATUS_OK);
                if ((st != BSP_STATUS_OK) && sensor_ready){
                    st = LineFollow_EvaluateDetectedMaskWithOmegaFeedforward(
                        detected_mask, dt, curve_omega_ff_deg_s, &control_out);
                }
            } else{
                st = sensor_ready
                         ? LineFollow_EvaluateDetectedMaskWithOmegaFeedforward(
                               detected_mask, dt, curve_omega_ff_deg_s,
                               &control_out)
                         : BSP_STATUS_NOT_READY;
                bool gyro_only_allowed =
                    (st != BSP_STATUS_OK) && sensor_ready &&
                    (detected_mask == 0U) && lt_line_acquired &&
                    LineFollowTest_IsRightCurve(distance_m) &&
                    ((uint32_t)(now - lt_last_line_seen_ms) <=
                     LT_CURVE_GYRO_ONLY_GRACE_MS);
                if (gyro_only_allowed){
                    st = LineFollow_EvaluateOmegaFeedforwardOnly(
                        curve_omega_ff_deg_s, &control_out);
                    lt_curve_gyro_only = (st == BSP_STATUS_OK);
                }
            }
            if (st != BSP_STATUS_OK){
                (void)Chassis_Brake();
                lt_speed_command_mps = 0.0f;
                lt_accel_command_mps2 = 0.0f;
                lt_steer_command_mps = 0.0f;
                lt_curve_feedforward_mps = 0.0f;
                if (lt_line_acquired){
                    lt_state = LT_STATE_LINE_LOST;
                    DebugUart_Printf(
                        "[TRK] line lost latched run=%lu t=%lu s=%.3f sensor=%u mask=%02X ---\r\n",
                        (unsigned long)lt_run_id,
                        (unsigned long)(now - lt_start_ms), distance_m,
                        sensor_ready ? 1U : 0U, (unsigned int)detected_mask);
                }
            } else{
                lt_line_acquired = true;
                LineFollowTest_UpdateLongitudinal(distance_m, dt);
                LineFollowTest_UpdateSteering(&control_out, distance_m, dt);
                /* 左右差速反对称，平均轮速始终等于纵向 S 曲线速度。 */
                Chassis_SetWheelSpeed(
                    lt_speed_command_mps + lt_steer_command_mps,
                    lt_speed_command_mps - lt_steer_command_mps);
            }
        }
    }

    if ((now - lt_last_telemetry) >= LT_TELEMETRY_PERIOD_MS){
        lt_last_telemetry = now;
        LineFollowTest_Telemetry(now, sensor_ready);
    }

    if ((now - lt_last_ui) < LT_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    lt_last_ui = now;

    LINE_FOLLOW_OUTPUT out = LineFollow_GetOutput();

    /* 归一化语义：X1→X8 从左到右显示，1 表示检测到黑线。 */
    char bits[LINE_FOLLOW_SENSOR_COUNT + 1U];
    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        bits[i] = ((detected_mask & (1U << i)) != 0U) ? '1' : '0';
    }
    bits[LINE_FOLLOW_SENSOR_COUNT] = '\0';

    char l2[20];
    char l3[20];
    char l4[20];
    uint8_t n;

    float display_error = out.error;
    if (lt_profile->requirement == 4U){
        LINE_FOLLOW_OBSERVATION observation;
        if (LineFollow_ObserveDetectedMask(
                detected_mask, 1.0f, &observation) == BSP_STATUS_OK){
            display_error = observation.error;
        }
    }
    n = LtPutStr(l2, "err ");
    AppFmt_Fixed(&l2[n], display_error, 1);
    n = LtPutStr(l3, "v ");
    AppFmt_Fixed(&l3[n], lt_speed_command_mps, 3);
    n = LtPutStr(l4, "s ");
    AppFmt_Fixed(&l4[n], Chassis_GetDistance(), 2);

    const char *status;
    if (lt_state == LT_STATE_STOPPED){
        status = "DONE / BACK";
    } else if (lt_state == LT_STATE_LINE_LOST){
        status = "LOST / BACK";
    } else if ((lt_profile->requirement == 4U) &&
               !lt_h4_heading_reference_valid){
        status = "IMU WAIT";
    } else if (lt_profile->requirement == 4U){
        status = LineFollowTest_SegmentName();
    } else if (!sensor_ready){
        status = "SENSOR WAIT";
    } else if (lt_curve_gyro_only){
        status = "GYRO HOLD";
    } else if (out.line_lost){
        status = "LINE LOST";
    } else if (lt_state == LT_STATE_FINISH_OFFSET){
        status = "FINAL OFFSET";
    } else if (lt_profile->requirement >= 4U){
        status = "PIPE OFF";
    } else{
        status = LineFollowTest_SegmentName();
    }

    if (lt_state == LT_STATE_STOPPED){
        n = LtPutStr(l4, "time ");
        uint32_t result_ms = (lt_profile->route == LT_ROUTE_STRAIGHT) &&
                             lt_straight_b_passed
                                 ? (lt_straight_b_ms - lt_start_ms)
                                 : (lt_stop_ms - lt_start_ms);
        AppFmt_Fixed(&l4[n], (float)result_ms * 0.001f, 2);
    }

    Ui_RenderLines(lt_profile->title, bits, l2, l3, l4, status, "BACK: exit");
    return APP_TASK_RUNNING;
}

static void H3_Enter(void){
    lt_start_ms = BSP_Time_GetMs();
    lt_last_ui = 0U;
    DebugUart_Printf("[TRK] --- enter req=3 pipe-control=not-connected ---\r\n");
}

static APP_TASK_STATUS H3_Tick(float dt){
    (void)dt;
    uint32_t now = BSP_Time_GetMs();
    if ((now - lt_last_ui) >= LT_UI_PERIOD_MS){
        lt_last_ui = now;
        Ui_RenderLines("H3 Ball Static", "NOT READY", "Pipe disabled",
                       "No actuator", "No motion", "SAFE / BACK",
                       "BACK: exit");
    }
    return APP_TASK_RUNNING;
}

const APP_TASK_DESC APP_H2_EMPTY_LAP = {
    "H2 Empty Lap", H2_Enter, LineFollowTest_Tick, NULL
};
const APP_TASK_DESC APP_H3_BALL_STATIC = {
    "H3 Ball Static", H3_Enter, H3_Tick, NULL
};
const APP_TASK_DESC APP_H4_LOADED_STRAIGHT = {
    "H4 Loaded A-B", H4_Enter, LineFollowTest_Tick, NULL
};
const APP_TASK_DESC APP_H5_LOADED_LAP_CENTER = {
    "H5 Loaded Lap O", H5_Enter, LineFollowTest_Tick, NULL
};
const APP_TASK_DESC APP_H6_LOADED_LAP_TARGET = {
    "H6 Loaded Any", H6_Enter, LineFollowTest_Tick, NULL
};

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
/** 标称环线中心线周长：2*1.5 + 2*pi*0.5。 */
#define LT_NOMINAL_LAP_DISTANCE_M 6.1416f
/** 终点识别只在标称触发位置前 0.4 m 内开放，避免普通宽线/干扰误触发。 */
#define LT_FINISH_ARM_MARGIN_M 0.400f
/** A 点横向启停线预期使 8 路同时检测到黑线。 */
#define LT_FINISH_LINE_MASK 0xFFU
/** line_follow 默认 correction 的最终半幅（DIFFERENTIAL_LIMIT/2）。 */
#define LT_LINE_CORRECTION_LIMIT \
    (LINE_FOLLOW_DEFAULT_DIFFERENTIAL_LIMIT * 0.5f)
/** 进入终点停车态的剩余距离与实测速度门限。 */
#define LT_STOP_DISTANCE_TOLERANCE_M 0.005f
#define LT_STOP_SPEED_TOLERANCE_MPS 0.030f
/** 要求 4 的 B 点标称里程，以及通过 B 后的安全停车点。 */
#define LT_STRAIGHT_B_DISTANCE_M 1.500f
#define LT_STRAIGHT_STOP_DISTANCE_M 1.600f
/** 四段状态机的赛道/底盘标称几何；有效轮距后续须用 Track Teach 覆盖。 */
#define LT_TRACK_STRAIGHT_LENGTH_M 1.500f
#define LT_TRACK_CURVE_RADIUS_M 0.500f
#define LT_EFFECTIVE_TRACK_WIDTH_M 0.150f
#define LT_PI 3.1415926f
/*
 * 编码器里程参考轮轴中心，而起点指定车尾位于 A；轮轴起步时已沿 AB 前置 7 cm。
 * S4 保持到任务结束，但其弯道前馈在轮轴到 A（S4_CURVE_END）后归零。
 */
#define LT_SEGMENT_S1_END_M \
    (LT_TRACK_STRAIGHT_LENGTH_M - APP_TRACK_MEASURE_TO_AXLE_M)
#define LT_SEGMENT_S2_END_M \
    (LT_SEGMENT_S1_END_M + LT_PI * LT_TRACK_CURVE_RADIUS_M)
#define LT_SEGMENT_S3_END_M \
    (LT_SEGMENT_S2_END_M + LT_TRACK_STRAIGHT_LENGTH_M)
#define LT_SEGMENT_S4_CURVE_END_M \
    (LT_SEGMENT_S3_END_M + LT_PI * LT_TRACK_CURVE_RADIUS_M)

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
    float steer_limit_mps;
    float steer_slew_mps2;
} LT_PROFILE;

/* H2 无球，可独立使用高速参数；H4/5/6 全部使用给滚球闭环留裕度的载球参数。 */
static const LT_PROFILE LT_PROFILE_H2 = {
    2U, "H2 Empty Lap", LT_ROUTE_LAP,
    0.450f, 0.300f, 0.600f, 0.078f, 0.180f,
};
static const LT_PROFILE LT_PROFILE_H4 = {
    4U, "H4 Loaded A-B", LT_ROUTE_STRAIGHT,
    0.260f, 0.120f, 0.240f, 0.045f, 0.100f,
};
static const LT_PROFILE LT_PROFILE_H5 = {
    5U, "H5 Loaded Lap O", LT_ROUTE_LAP,
    0.260f, 0.120f, 0.240f, 0.045f, 0.100f,
};
static const LT_PROFILE LT_PROFILE_H6 = {
    6U, "H6 Loaded Any", LT_ROUTE_LAP,
    0.260f, 0.120f, 0.240f, 0.045f, 0.100f,
};

typedef enum {
    LT_STATE_FOLLOW = 0,
    LT_STATE_FINISH_OFFSET,
    LT_STATE_STOPPED,
} LT_STATE;

static uint32_t lt_last_ui;
static uint32_t lt_last_telemetry;
static uint32_t lt_start_ms;
static uint32_t lt_stop_ms;
static uint8_t lt_detected_mask;
static uint32_t lt_detected_mask_ms;
static bool lt_sensor_ready;
static BSP_STATUS lt_sensor_init_status;
static LT_STATE lt_state;
static float lt_finish_detect_distance_m;
static float lt_speed_command_mps;
static float lt_accel_command_mps2;
static float lt_steer_command_mps;
static const LT_PROFILE *lt_profile;
static bool lt_straight_b_passed;
static uint32_t lt_straight_b_ms;
static LT_SEGMENT lt_segment;

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
        default:
            return "?";
    }
}

static const char *LineFollowTest_SegmentName(void){
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

static bool LineFollowTest_FinishArmed(float distance_m){
    if ((lt_profile == NULL) || (lt_profile->route != LT_ROUTE_LAP)){
        return false;
    }
    const float trigger_m = LT_NOMINAL_LAP_DISTANCE_M -
                            APP_TRACK_MEASURE_TO_SENSOR_M;
    return distance_m >= (trigger_m - LT_FINISH_ARM_MARGIN_M);
}

static float LineFollowTest_FinishRemaining(float distance_m){
    if ((lt_profile != NULL) && (lt_profile->route == LT_ROUTE_STRAIGHT)){
        float remaining_m = LT_STRAIGHT_STOP_DISTANCE_M - distance_m;
        return (remaining_m > 0.0f) ? remaining_m : 0.0f;
    }

    if (lt_state == LT_STATE_FINISH_OFFSET){
        float travelled_m = distance_m - lt_finish_detect_distance_m;
        float remaining_m = APP_TRACK_MEASURE_TO_SENSOR_M - travelled_m;
        return (remaining_m > 0.0f) ? remaining_m : 0.0f;
    }

    if (lt_state == LT_STATE_FOLLOW && LineFollowTest_FinishArmed(distance_m)){
        float remaining_m = LT_NOMINAL_LAP_DISTANCE_M - distance_m;
        return (remaining_m > 0.0f) ? remaining_m : 0.0f;
    }
    return 0.0f;
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

/*
 * 纵向 S 曲线：先限制加速度变化率，再积分得到速度。接近终点时用
 * v<=sqrt(2*a*s) 的制动包络提前降速，避免到 19.5 cm 补偿终点才硬刹。
 */
static void LineFollowTest_UpdateLongitudinal(float distance_m, float dt_s){
    float speed_limit_mps = lt_profile->cruise_speed_mps;
    float remaining_m = LineFollowTest_FinishRemaining(distance_m);
    bool terminal_approach = (lt_profile->route == LT_ROUTE_STRAIGHT) ||
        (lt_state == LT_STATE_FINISH_OFFSET) ||
        ((lt_state == LT_STATE_FOLLOW) &&
         LineFollowTest_FinishArmed(distance_m));
    if (terminal_approach){
        if (remaining_m > 0.0f){
            float brake_limit_mps = sqrtf(2.0f * lt_profile->accel_limit_mps2 *
                                          remaining_m);
            if (brake_limit_mps < speed_limit_mps){
                speed_limit_mps = brake_limit_mps;
            }
        } else{
            /* A 线漏检时也在标称一圈处柔和停住，绝不重新加速冲出赛道。 */
            speed_limit_mps = 0.0f;
        }
    }

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
    } else if (lt_speed_command_mps > speed_limit_mps){
        lt_speed_command_mps = speed_limit_mps;
    }
}

static void LineFollowTest_UpdateSteering(const LINE_FOLLOW_OUTPUT *out,
                                          float distance_m, float dt_s){
    /* 低速时同比收窄差速半幅，确保停车阶段两轮都不反转成原地旋转。 */
    float steer_limit_mps = lt_profile->steer_limit_mps *
                            (lt_speed_command_mps /
                             lt_profile->cruise_speed_mps);
    float feedforward_mps = 0.0f;
    bool right_curve = (lt_profile->route == LT_ROUTE_LAP) &&
        ((lt_segment == LT_SEGMENT_S2) ||
         ((lt_segment == LT_SEGMENT_S4) &&
          (distance_m < LT_SEGMENT_S4_CURVE_END_M)));
    if (right_curve){
        feedforward_mps = lt_speed_command_mps *
            (LT_EFFECTIVE_TRACK_WIDTH_M / (2.0f * LT_TRACK_CURVE_RADIUS_M));
    }

    float feedback_mps = (out->correction / LT_LINE_CORRECTION_LIMIT) *
                         steer_limit_mps;
    float steer_target_mps = feedforward_mps + feedback_mps;
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
    CHASSIS_DUTY duty = Chassis_GetDuty();
    float distance_m = Chassis_GetDistance();

    DebugUart_Printf(
        "[TRK] t=%lu req=%u seg=%s st=%s sen=%u mask=%02X n=%u err=%.1f cor=%.2f vc=%.3f ac=%.3f "
        "vl=%.3f vr=%.3f dl=%.1f dr=%.1f sl=%.3f sr=%.3f s=%.3f "
        "rem=%.3f drop=%lu\r\n",
        (unsigned long)(now - lt_start_ms),
        (unsigned int)lt_profile->requirement, LineFollowTest_SegmentName(),
        LineFollowTest_StateName(),
        sensor_ready ? 1U : 0U, (unsigned int)lt_detected_mask,
        (unsigned int)out.black_count, out.error, out.correction,
        lt_speed_command_mps, lt_accel_command_mps2,
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
    LineFollow_Init(NULL);            /* 默认配置（含陀螺增稳），并复位控制状态。 */
    LineSensor_Enter();
    lt_last_ui = 0U;
    lt_last_telemetry = 0U;
    lt_stop_ms = 0U;
    lt_state = LT_STATE_FOLLOW;
    lt_finish_detect_distance_m = 0.0f;
    lt_speed_command_mps = 0.0f;
    lt_accel_command_mps2 = 0.0f;
    lt_steer_command_mps = 0.0f;
    lt_straight_b_passed = false;
    lt_straight_b_ms = 0U;
    lt_segment = LT_SEGMENT_S1;

    DebugUart_Printf(
        "[TRK] --- enter req=%u rear->axle=%.3fm rear->sensor=%.3fm "
        "sensor->axle=%.3fm v=%.3f a=%.3f pipe=%s ---\r\n",
        (unsigned int)profile->requirement,
        APP_TRACK_MEASURE_TO_AXLE_M, APP_TRACK_MEASURE_TO_SENSOR_M,
        APP_TRACK_MEASURE_TO_SENSOR_M - APP_TRACK_MEASURE_TO_AXLE_M,
        profile->cruise_speed_mps, profile->accel_limit_mps2,
        (profile->requirement >= 4U) ? "off" : "n/a");
}

static void H2_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H2); }
static void H4_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H4); }
static void H5_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H5); }
static void H6_Enter(void){ LineFollowTask_EnterCommon(&LT_PROFILE_H6); }

static APP_TASK_STATUS LineFollowTest_Tick(float dt){
    uint8_t detected_mask;
    bool sensor_ready = LineSensor_Tick(&detected_mask);
    uint32_t now = BSP_Time_GetMs();
    float distance_m = Chassis_GetDistance();

    LineFollowTest_UpdateSegment(now, distance_m);

    if ((lt_profile->route == LT_ROUTE_STRAIGHT) &&
        !lt_straight_b_passed &&
        (distance_m >= LT_STRAIGHT_B_DISTANCE_M)){
        lt_straight_b_passed = true;
        lt_straight_b_ms = now;
        DebugUart_Printf("[TRK] pass B req=4 t=%lu s=%.3f\r\n",
            (unsigned long)(now - lt_start_ms), distance_m);
    }

    if ((lt_profile->route == LT_ROUTE_LAP) &&
        (lt_segment == LT_SEGMENT_S4) &&
        (lt_state == LT_STATE_FOLLOW) && sensor_ready &&
        LineFollowTest_FinishArmed(distance_m) &&
        ((detected_mask & LT_FINISH_LINE_MASK) == LT_FINISH_LINE_MASK)){
        /*
         * 灰度阵列在车尾测量点前方 19.5 cm。看到 A 点横线时只锁存里程，
         * 继续循迹这段几何偏置后再刹车，才能让车尾而不是灰度板停在基准线上。
         */
        lt_finish_detect_distance_m = distance_m;
        lt_state = LT_STATE_FINISH_OFFSET;
        DebugUart_Printf("[TRK] finish detected t=%lu s=%.3f offset=%.3f\r\n",
            (unsigned long)(now - lt_start_ms), distance_m,
            APP_TRACK_MEASURE_TO_SENSOR_M);
    }

    float finish_remaining_m = LineFollowTest_FinishRemaining(distance_m);
    bool finish_position_reached =
        ((lt_profile->route == LT_ROUTE_STRAIGHT) &&
         (distance_m >= (LT_STRAIGHT_STOP_DISTANCE_M -
                          LT_STOP_DISTANCE_TOLERANCE_M))) ||
        ((lt_profile->route == LT_ROUTE_LAP) &&
         (lt_state == LT_STATE_FINISH_OFFSET) &&
         (finish_remaining_m <= LT_STOP_DISTANCE_TOLERANCE_M)) ||
        ((lt_profile->route == LT_ROUTE_LAP) &&
         (lt_state == LT_STATE_FOLLOW) &&
         (distance_m >= LT_NOMINAL_LAP_DISTANCE_M));
    if (finish_position_reached &&
        (fabsf(Chassis_GetSpeed()) <= LT_STOP_SPEED_TOLERANCE_MPS)){
        (void)Chassis_Brake();
        lt_stop_ms = now;
        lt_state = LT_STATE_STOPPED;
        DebugUart_Printf("[TRK] stopped t=%lu s=%.3f\r\n",
            (unsigned long)(lt_stop_ms - lt_start_ms), distance_m);
    }

    if (lt_state != LT_STATE_STOPPED){
        LINE_FOLLOW_OUTPUT control_out;
        BSP_STATUS st = sensor_ready
                            ? LineFollow_EvaluateDetectedMask(
                                  detected_mask, dt, &control_out)
                            : BSP_STATUS_NOT_READY;
        if (st != BSP_STATUS_OK){
            (void)Chassis_Brake();    /* 测试期安全策略：观测恢复后可继续，不盲搜线。 */
            lt_speed_command_mps = 0.0f;
            lt_accel_command_mps2 = 0.0f;
            lt_steer_command_mps = 0.0f;
        } else{
            LineFollowTest_UpdateLongitudinal(distance_m, dt);
            LineFollowTest_UpdateSteering(&control_out, distance_m, dt);
            /* 左右差速反对称，平均轮速始终等于纵向 S 曲线速度。 */
            Chassis_SetWheelSpeed(
                lt_speed_command_mps + lt_steer_command_mps,
                lt_speed_command_mps - lt_steer_command_mps);
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

    n = LtPutStr(l2, "err ");
    AppFmt_Fixed(&l2[n], out.error, 1);
    n = LtPutStr(l3, "v ");
    AppFmt_Fixed(&l3[n], lt_speed_command_mps, 3);
    n = LtPutStr(l4, "s ");
    AppFmt_Fixed(&l4[n], Chassis_GetDistance(), 2);

    const char *status;
    if (lt_state == LT_STATE_STOPPED){
        status = "DONE / BACK";
    } else if (!sensor_ready){
        status = "SENSOR WAIT";
    } else if (out.line_lost){
        status = "LINE LOST";
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

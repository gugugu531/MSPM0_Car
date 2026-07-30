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
/** 弯道灰度瞬时全白时，允许纯曲率/陀螺保持的最长时间。 */
#define LT_CURVE_GYRO_ONLY_GRACE_MS 200U
#define LT_LINE_REACQUIRE_GRACE_MS 120U
#define LT_LINE_REACQUIRE_EDGE_ERROR_MM 28.0f
#define LT_LINE_REACQUIRE_OMEGA_DEGPS 15.0f
#define LT_LINE_REACQUIRE_STEER_SLEW_MPS2 1.0f
/** 标称环线中心线周长：2*1.5 + 2*pi*0.5。 */
#define LT_NOMINAL_LAP_DISTANCE_M 6.1416f
/** 终点识别只在标称触发位置前 0.4 m 内开放，避免普通宽线/干扰误触发。 */
#define LT_FINISH_ARM_MARGIN_M 0.400f
/** 实测平行通过 A 点横线可稳定命中 4 路：连续 3 帧确认，回到普通 1~2 路后取中心。 */
#define LT_FINISH_ENTER_MIN_COUNT 4U
#define LT_FINISH_ENTER_CONFIRM_FRAMES 3U
#define LT_FINISH_EXIT_MAX_COUNT 2U
/** line_follow 默认 correction 的最终半幅（DIFFERENTIAL_LIMIT/2）。 */
#define LT_LINE_CORRECTION_LIMIT \
    (LINE_FOLLOW_DEFAULT_DIFFERENTIAL_LIMIT * 0.5f)
/** 进入终点停车态的剩余距离门限。 */
#define LT_STOP_DISTANCE_TOLERANCE_M 0.005f
/** 要求 4 的 B 点标称里程，以及通过 B 后的安全停车点。 */
#define LT_STRAIGHT_B_DISTANCE_M 1.500f
#define LT_STRAIGHT_STOP_DISTANCE_M 1.600f
#define LT_STRAIGHT_TERMINAL_BLIND_START_M 1.550f
#define LT_FINISH_CAPTURE_SPEED_EMPTY_MPS 0.150f
#define LT_FINISH_CAPTURE_SPEED_LOADED_MPS 0.120f
/** 制动包络只使用纵向加速度权限的一半，为 jerk 建立和执行延迟留距离。 */
#define LT_TERMINAL_BRAKE_ENVELOPE_RATIO 0.50f
/** 四段状态机的赛道标称几何，以及两轮接地点中心的实测轮距。 */
#define LT_TRACK_STRAIGHT_LENGTH_M 1.500f
#define LT_TRACK_CURVE_RADIUS_M 0.500f
#define LT_EFFECTIVE_TRACK_WIDTH_M 0.206f
/*
 * 弯道前馈比例。两次手推实测真实转弯半径 ~0.55–0.58 m（弧长/π），大于标称 0.5，
 * 前馈按 R=0.5 算差速会偏大 → 驱动实测“切内圈、线甩到外侧左缘”出弯丢线。
 * 用 scale≈0.5/0.55≈0.90 补偿到实测半径，减少过转（原 1.10/1.00 是抬轮空载旧标定）。
 */
#define LT_CURVE_FEEDFORWARD_SCALE_S2 0.90f
#define LT_CURVE_FEEDFORWARD_SCALE_S4 0.90f
#define LT_PI 3.1415926f
#define LT_RAD_TO_DEG (180.0f / LT_PI)
/** H 题实测专用灰度角速度外环：降低增益并限制其残差权限。 */
#define LT_GYRO_LINE_KP 1.25f
#define LT_GYRO_LINE_LIMIT_EMPTY_DEGPS 20.0f
#define LT_GYRO_LINE_LIMIT_LOADED_DEGPS 15.0f
#define LT_GYRO_REF_LIMIT_EMPTY_DEGPS 75.0f
#define LT_GYRO_REF_LIMIT_LOADED_DEGPS 50.0f
/** 提前撤除弯道前馈时额外预留的纵向安全距离。 */
#define LT_CURVE_TAPER_MARGIN_M 0.030f
#define LT_CURVE_EXIT_SETTLE_DISTANCE_M \
    (APP_TRACK_MEASURE_TO_SENSOR_M - APP_TRACK_MEASURE_TO_AXLE_M)
/*
 * 编码器里程从车尾位于 A 时清零。灰度阵列比轮轴前探 12.5 cm，四段控制边界以
 * 灰度阵列到达 B/C/D/A 为准，给差速变化率限制留出预瞄建立距离。
 * S4 保持到任务结束；S2/S4 的曲率前馈都在物理出弯点前按当前速度和差速斜率
 * 动态进入 EXIT 阶段，保证到达切线点时前馈已基本归零。
 */
/*
 * ===== 四段编码器切换里程（手推一圈实测标定，沿前进方向里程，单位 m）=====
 * 2026-07-30 手推标定：整圈 s=0..6.28，两个同向 180° 半圆 + 两段直道，轮距 0.206 实测吻合。
 *   实测入弯/出弯里程(平均轮里程 s)：弯1 入 1.43 出 3.03；弯2 入 4.52 出 6.24；停车 ≈6.28。
 * 关键修正：原标称把两处“出弯点”都踩早了（S2_END/S4_CURVE_END），会在弯未走完时
 * 就撤掉弯道前馈导致出弯发飘；这里改成实测出弯里程。S1_END 已标定且准确，保持不变。
 */
#define LT_SEGMENT_S1_END_M \
    (LT_TRACK_STRAIGHT_LENGTH_M - APP_TRACK_MEASURE_TO_SENSOR_M)  /* ≈1.305，实测入弯 1.43，保留入弯预瞄 */
#define LT_SEGMENT_S2_END_M        3.10f    /* 弯1出弯：两次手推 3.03/3.14，取 3.10（保前馈到出弯） */
#define LT_SEGMENT_S3_END_M        4.40f    /* 弯2入弯：两次手推 ~4.50，留 ~0.10 入弯预瞄 */
#define LT_SEGMENT_S4_CURVE_END_M  6.28f    /* 弯2出弯：两次手推 6.28/6.32，取 6.28 */

/*
 * 停车里程：手推“出弯即停、航向归零(yaw≈0)”两次落在 s≈6.28/6.34；取 6.30 让车在弯2 出弯处停稳。
 */
#define LT_LAP_STOP_DISTANCE_M     6.30f

/*
 * 改动2（按手推数据修正）：本赛道弯2 出弯≈终点，几乎没有末端直道，无法“回正后再减速”。
 * 改为——终点前 LT_LAP_TERMINAL_ARM_M 起把巡航压到低速爬行 LT_FINISH_CAPTURE_SPEED_*，
 * 并按“到 LT_LAP_STOP_DISTANCE_M 的剩余距离”走制动包络，允许在弯2 内就开始减速；
 * 低速下弯道几何自然把航向带回 0，车在出弯处刹停，复现手推的出弯/停车效果。
 *   刹停距离 v^2/2a：巡航 0.45 需 ~0.675 m，低速 0.15 只需 ~0.075 m —— 低速爬行是关键。
 * ARM 需 >= 由巡航减速到爬行速度所需的里程，给足弯内减速余量。
 */
#define LT_LAP_TERMINAL_ARM_M      0.60f


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
    0.260f, 0.120f, 0.240f, 0.045f, 0.025f, 0.070f, 0.100f,
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
    LT_STATE_FINISH_LINE,
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
static uint8_t lt_finish_enter_count;
static float lt_finish_enter_candidate_m;
static float lt_finish_enter_distance_m;
static float lt_finish_target_distance_m;
static float lt_speed_command_mps;
static float lt_terminal_speed_ceiling_mps;
static float lt_accel_command_mps2;
static float lt_steer_command_mps;
static float lt_curve_feedforward_mps;
static uint32_t lt_last_line_seen_ms;
static bool lt_curve_gyro_only;
static float lt_last_line_error_mm;
static bool lt_line_reacquire_active;
static const LT_PROFILE *lt_profile;
static bool lt_straight_b_passed;
static uint32_t lt_straight_b_ms;
static LT_SEGMENT lt_segment;
/* 当前弯道已锁存进入提前撤前馈阶段，避免触发边界随速度变化反复开关。 */
static bool lt_curve_taper_active;
static float lt_curve_taper_release_distance_m;
/* 空推标定模式：全部逻辑照跑并发遥测，但不驱动电机（电机 Coast，供手推采数据）。 */
static bool lt_dry_run;

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
        case LT_STATE_FINISH_LINE:
            return "XLINE";
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

/*
 * LAP 终点减速区：里程进入“停车点前 ARM”即开始压速/制动。允许落在弯2 内（本赛道弯2
 * 出弯≈终点、无末端直道），配合低速爬行让车在出弯处停稳。仅在 S4 段判定，避免误触发。
 */
static bool LineFollowTest_InLapTerminal(float distance_m){
    return (lt_profile != NULL) && (lt_profile->route == LT_ROUTE_LAP) &&
        (lt_segment == LT_SEGMENT_S4) &&
        (distance_m >= (LT_LAP_STOP_DISTANCE_M - LT_LAP_TERMINAL_ARM_M));
}

static void LineFollowTest_UpdateSegment(uint32_t now, float distance_m){
    if ((lt_profile == NULL) || (lt_profile->route != LT_ROUTE_LAP)){
        return;
    }

    LT_SEGMENT previous = lt_segment;
    if ((lt_segment == LT_SEGMENT_S1) &&
        (distance_m >= LT_SEGMENT_S1_END_M)){
        lt_segment = LT_SEGMENT_S2;
        lt_curve_taper_active = false;
    }
    if ((lt_segment == LT_SEGMENT_S2) &&
        (distance_m >= LT_SEGMENT_S2_END_M)){
        lt_segment = LT_SEGMENT_S3;
    }
    if ((lt_segment == LT_SEGMENT_S3) &&
        (distance_m >= LT_SEGMENT_S3_END_M)){
        lt_segment = LT_SEGMENT_S4;
        lt_curve_taper_active = false;
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

    /* LAP：进入终点减速区后，向标定停车里程逼近（可落在弯2 内）。 */
    if (LineFollowTest_InLapTerminal(distance_m)){
        float remaining_m = LT_LAP_STOP_DISTANCE_M - distance_m;
        return (remaining_m > 0.0f) ? remaining_m : 0.0f;
    }
    return 0.0f;
}

/*
 * A 线灰度横线识别已改为“编码器里程停车 + 终点低速爬行”（见 LT_LAP_STOP_DISTANCE_M
 * 与 LineFollowTest_InLapTerminal），此处暂时停用；保留占位便于日后需要横线校正时恢复。
 */
static void LineFollowTest_UpdateFinishLine(uint32_t now, float distance_m,
                                             bool sensor_ready,
                                             uint8_t detected_mask){
    (void)now;
    (void)distance_m;
    (void)sensor_ready;
    (void)detected_mask;
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
    /*
     * 改动2（按手推数据修正）：LAP 在终点减速区(可落在弯2 内)压到低速爬行并走制动包络，
     * 出弯处停稳；不再等“末端直道回正”。直道题(H4)沿用原终点补偿。
     */
    bool lap_terminal = LineFollowTest_InLapTerminal(distance_m);
    bool terminal_approach = (lt_profile->route == LT_ROUTE_STRAIGHT) ||
        (lt_state == LT_STATE_FINISH_OFFSET) || lap_terminal;
    if (terminal_approach){
        float terminal_limit_mps = lt_profile->cruise_speed_mps;
        /* LAP 终点低速爬行：低速下刹停距离极短，是“出弯即停”的关键。 */
        if (lap_terminal){
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
            /* 到达/越过停车里程时柔和停住，绝不重新加速冲出赛道。 */
            terminal_limit_mps = 0.0f;
        }
        /* 横线中心会把目标向前修正，终点阶段速度上限只能下降，禁止再次加速。 */
        if (terminal_limit_mps < lt_terminal_speed_ceiling_mps){
            lt_terminal_speed_ceiling_mps = terminal_limit_mps;
        }
        speed_limit_mps = lt_terminal_speed_ceiling_mps;
    } else{
        lt_terminal_speed_ceiling_mps = lt_profile->cruise_speed_mps;
    }

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
        /* 新上限低于当前速度时先按 jerk/accel 斜坡减速，只在真正穿越时钳位。 */
        lt_speed_command_mps = speed_limit_mps;
    }
}

static bool LineFollowTest_IsRightCurve(float distance_m){
    return (lt_profile->route == LT_ROUTE_LAP) &&
        ((lt_segment == LT_SEGMENT_S2) ||
         ((lt_segment == LT_SEGMENT_S4) &&
          (distance_m < LT_SEGMENT_S4_CURVE_END_M)));
}

static float LineFollowTest_CurveExitDistance(void){
    if (lt_segment == LT_SEGMENT_S2){
        return LT_SEGMENT_S2_END_M;
    }
    if (lt_segment == LT_SEGMENT_S4){
        return LT_SEGMENT_S4_CURVE_END_M;
    }
    return 0.0f;
}

static void LineFollowTest_UpdateCurveFeedforward(uint32_t now,
                                                   float distance_m,
                                                   float dt_s){
    float target_mps = 0.0f;
    if (LineFollowTest_IsRightCurve(distance_m)){
        float feedforward_scale = (lt_segment == LT_SEGMENT_S4)
                                      ? LT_CURVE_FEEDFORWARD_SCALE_S4
                                      : LT_CURVE_FEEDFORWARD_SCALE_S2;
        float curve_target_mps = lt_speed_command_mps *
            (LT_EFFECTIVE_TRACK_WIDTH_M / (2.0f * LT_TRACK_CURVE_RADIUS_M)) *
            feedforward_scale;

        if (!lt_curve_taper_active &&
            (lt_profile->steer_slew_mps2 > 0.0f)){
            float exit_distance_m = LineFollowTest_CurveExitDistance();
            float decay_time_s = fabsf(curve_target_mps) /
                                 lt_profile->steer_slew_mps2;
            float decay_distance_m = lt_speed_command_mps * decay_time_s +
                                     LT_CURVE_TAPER_MARGIN_M;
            float remaining_m = exit_distance_m - distance_m;
            if (remaining_m <= decay_distance_m){
                lt_curve_taper_active = true;
                lt_curve_taper_release_distance_m = exit_distance_m +
                    LT_CURVE_EXIT_SETTLE_DISTANCE_M;
                DebugUart_Printf(
                    "[TRK] curve taper run=%lu seg=%s t=%lu s=%.3f exit=%.3f decay=%.3f cff=%.3f\r\n",
                    (unsigned long)lt_run_id, LineFollowTest_SegmentName(),
                    (unsigned long)(now - lt_start_ms), distance_m,
                    exit_distance_m, decay_distance_m,
                    lt_curve_feedforward_mps);
            }
        }

        if (!lt_curve_taper_active){
            target_mps = curve_target_mps;
        }
    }
    lt_curve_feedforward_mps = LineFollowTest_MoveTowards(
        lt_curve_feedforward_mps, target_mps,
        lt_profile->steer_slew_mps2 * dt_s);

    /* 离开物理弯道且前馈确已归零后释放锁存，供下一弯重新触发。 */
    if (lt_curve_taper_active &&
        (distance_m >= lt_curve_taper_release_distance_m) &&
        (fabsf(lt_curve_feedforward_mps) <= 0.0001f)){
        lt_curve_taper_active = false;
    }
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
    /* EXIT may brake/reverse, but must not rebuild same-direction steer. */
    if (lt_curve_taper_active &&
        (steer_target_mps > lt_curve_feedforward_mps)){
        steer_target_mps = lt_curve_feedforward_mps;
    }
    if (steer_target_mps > steer_limit_mps){
        steer_target_mps = steer_limit_mps;
    } else if (steer_target_mps < -steer_limit_mps){
        steer_target_mps = -steer_limit_mps;
    }
    lt_steer_command_mps = LineFollowTest_MoveTowards(
        lt_steer_command_mps, steer_target_mps,
        lt_profile->steer_slew_mps2 * dt_s);
}

static void LineFollowTest_UpdateReacquireSteering(float dt_s){
    float omega_rad_s = LT_LINE_REACQUIRE_OMEGA_DEGPS / LT_RAD_TO_DEG;
    float steer_target_mps =
        0.5f * LT_EFFECTIVE_TRACK_WIDTH_M * omega_rad_s;
    if (lt_last_line_error_mm > 0.0f){
        steer_target_mps = -steer_target_mps;
    }

    /* Cross the stale curve command quickly, with a bounded slew rate. */
    lt_curve_feedforward_mps = LineFollowTest_MoveTowards(
        lt_curve_feedforward_mps, 0.0f,
        LT_LINE_REACQUIRE_STEER_SLEW_MPS2 * dt_s);
    lt_steer_command_mps = LineFollowTest_MoveTowards(
        lt_steer_command_mps, steer_target_mps,
        LT_LINE_REACQUIRE_STEER_SLEW_MPS2 * dt_s);
}

/*
 * 电机出力统一走这两个包装：空推标定模式(lt_dry_run)下不下发任何驱动/刹车指令，
 * 电机保持进入时设置的 Coast 状态，便于手推；正常任务与原行为一致。
 */
static void LineFollowTest_DriveWheels(float left_mps, float right_mps){
    if (!lt_dry_run){
        Chassis_SetWheelSpeed(left_mps, right_mps);
    }
}

static void LineFollowTest_BrakeChassis(void){
    if (!lt_dry_run){
        (void)Chassis_Brake();
    }
}

/*
 * 控制子阶段标签，用于遥测里一眼定位“异常发生在哪个阶段”：
 *   CURVE  = 弯道前馈生效中（IsRightCurve）
 *   EXIT   = 物理出弯点前，曲率前馈正在提前归零
 *   STR    = 直道灰度循迹
 *   GYRO   = 弯内灰度全白时的纯 gz 兜底
 *   TERM   = 终点低速爬行减速区
 *   OFFSET = 直道题横线终点补偿
 *   LOST/STOP = 已锁存丢线 / 已停车
 * 出弯过转的典型特征：seg 已到 S3/直道，但 ph 仍为 CURVE 或 cff 未归零、wz 未跟随 wref 下降。
 */
static const char *LineFollowTest_PhaseName(float distance_m){
    if (lt_state == LT_STATE_STOPPED){ return "STOP"; }
    if (lt_state == LT_STATE_LINE_LOST){ return "LOST"; }
    if (lt_state == LT_STATE_FINISH_OFFSET){ return "OFFSET"; }
    if (lt_line_reacquire_active){ return "RECAP"; }
    if (lt_curve_gyro_only){ return "GYRO"; }
    if (lt_curve_taper_active){ return "EXIT"; }
    if (LineFollowTest_InLapTerminal(distance_m)){ return "TERM"; }
    if (LineFollowTest_IsRightCurve(distance_m)){ return "CURVE"; }
    return "STR";
}

static void LineFollowTest_Telemetry(uint32_t now, bool sensor_ready){
    LINE_FOLLOW_OUTPUT out = LineFollow_GetOutput();
    JY61P_I2C_SAMPLE imu_sample;
    float yaw_deg = JY61P_I2C_GetSnapshot(&imu_sample)
                        ? imu_sample.data.attitude_deg.yaw
                        : 0.0f;
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
        "[TRK] t=%lu run=%lu req=%u seg=%s ph=%s st=%s fs=%s gm=%u sen=%u mask=%02X n=%u err=%.1f cor=%.2f "
        "cff=%.3f vc=%.3f ac=%.3f "
        "vs=%.3f wref=%.1f wz=%.1f yaw=%.1f vl=%.3f vr=%.3f dl=%.1f dr=%.1f sl=%.3f sr=%.3f s=%.3f "
        "rem=%.3f drop=%lu\r\n",
        (unsigned long)(now - lt_start_ms),
        (unsigned long)lt_run_id, (unsigned int)lt_profile->requirement,
        LineFollowTest_SegmentName(), LineFollowTest_PhaseName(distance_m),
        LineFollowTest_StateName(),
        LineFollowTest_FinishSourceName(),
        lt_curve_gyro_only ? 1U : 0U,
        sensor_ready ? 1U : 0U, (unsigned int)lt_detected_mask,
        (unsigned int)black_count, line_error, out.correction,
        lt_curve_feedforward_mps,
        lt_speed_command_mps, lt_accel_command_mps2,
        lt_steer_command_mps,
        out.omega_ref_deg_s, out.omega_measured_deg_s, yaw_deg,
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
    lt_finish_enter_count = 0U;
    lt_finish_enter_candidate_m = 0.0f;
    lt_finish_enter_distance_m = 0.0f;
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
    lt_last_line_error_mm = 0.0f;
    lt_line_reacquire_active = false;
    lt_straight_b_passed = false;
    lt_straight_b_ms = 0U;
    lt_segment = LT_SEGMENT_S1;
    lt_curve_taper_active = false;
    lt_curve_taper_release_distance_m = 0.0f;
    DebugUart_Printf(
        "[TRK] --- enter run=%lu req=%u dry=%u rear->axle=%.3fm rear->sensor=%.3fm "
        "sensor->axle=%.3fm track=%.3fm radius=%.3fm v=%.3f a=%.3f "
        "gkp=%.2f glim=%.1f ff2=%.2f ff4=%.2f pipe=%s ---\r\n",
        (unsigned long)lt_run_id, (unsigned int)profile->requirement,
        lt_dry_run ? 1U : 0U,
        APP_TRACK_MEASURE_TO_AXLE_M, APP_TRACK_MEASURE_TO_SENSOR_M,
        APP_TRACK_MEASURE_TO_SENSOR_M - APP_TRACK_MEASURE_TO_AXLE_M,
        LT_EFFECTIVE_TRACK_WIDTH_M, LT_TRACK_CURVE_RADIUS_M,
        profile->cruise_speed_mps, profile->accel_limit_mps2,
        line_config.gyro_line_kp, line_config.omega_line_limit,
        LT_CURVE_FEEDFORWARD_SCALE_S2, LT_CURVE_FEEDFORWARD_SCALE_S4,
        (profile->requirement >= 4U) ? "off" : "n/a");
}

static void H2_Enter(void){
    lt_dry_run = false;
    LineFollowTask_EnterCommon(&LT_PROFILE_H2);
}
static void H4_Enter(void){
    lt_dry_run = false;
    LineFollowTask_EnterCommon(&LT_PROFILE_H4);
}
static void H5_Enter(void){
    lt_dry_run = false;
    LineFollowTask_EnterCommon(&LT_PROFILE_H5);
}
static void H6_Enter(void){
    lt_dry_run = false;
    LineFollowTask_EnterCommon(&LT_PROFILE_H6);
}

/*
 * H2 空推标定：与 H2 同一套逻辑/参数/遥测，但不驱动电机。进入时把底盘置为 Coast
 * 让轮子自由转动，随后每拍都不下发驱动/刹车指令；编码器、IMU、灰度照常读取并发
 * [TRK] 遥测，供手推小车采集分段里程标定 S2/S3/S4 与停车点。
 */
static void H2_DryRun_Enter(void){
    lt_dry_run = true;
    LineFollowTask_EnterCommon(&LT_PROFILE_H2);
    (void)Chassis_Coast();
}

static APP_TASK_STATUS LineFollowTest_Tick(float dt){
    uint8_t detected_mask;
    bool sensor_ready = LineSensor_Tick(&detected_mask);
    uint32_t now = BSP_Time_GetMs();
    float distance_m = Chassis_GetDistance();

    lt_curve_gyro_only = false;
    lt_line_reacquire_active = false;
    if (sensor_ready && (detected_mask != 0U)){
        lt_last_line_seen_ms = now;
    }

    LineFollowTest_UpdateSegment(now, distance_m);

    if ((lt_profile->route == LT_ROUTE_STRAIGHT) &&
        !lt_straight_b_passed &&
        (distance_m >= LT_STRAIGHT_B_DISTANCE_M)){
        lt_straight_b_passed = true;
        lt_straight_b_ms = now;
        DebugUart_Printf("[TRK] pass B req=4 t=%lu s=%.3f\r\n",
            (unsigned long)(now - lt_start_ms), distance_m);
    }

    LineFollowTest_UpdateFinishLine(
        now, distance_m, sensor_ready, detected_mask);

    if ((lt_profile->route == LT_ROUTE_STRAIGHT) &&
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

    /* LAP：低速爬到标定停车里程即刹停（编码器停车，出弯处停稳）。 */
    bool finish_position_reached =
        ((lt_profile->route == LT_ROUTE_STRAIGHT) &&
         (distance_m >= (LT_STRAIGHT_STOP_DISTANCE_M -
                          LT_STOP_DISTANCE_TOLERANCE_M))) ||
        ((lt_profile->route == LT_ROUTE_LAP) &&
         (lt_segment == LT_SEGMENT_S4) &&
         (distance_m >= LT_LAP_STOP_DISTANCE_M));
    if (finish_position_reached){
        if (lt_finish_source == LT_FINISH_SOURCE_NONE){
            lt_finish_source = LT_FINISH_SOURCE_ODOM;
        }
        LineFollowTest_BrakeChassis();
        lt_speed_command_mps = 0.0f;
        lt_accel_command_mps2 = 0.0f;
        lt_steer_command_mps = 0.0f;
        lt_curve_feedforward_mps = 0.0f;
        lt_stop_ms = now;
        lt_state = LT_STATE_STOPPED;
        DebugUart_Printf(
            "[TRK] stopped run=%lu fs=%s t=%lu s=%.3f target=%.3f\r\n",
            (unsigned long)lt_run_id, LineFollowTest_FinishSourceName(),
            (unsigned long)(lt_stop_ms - lt_start_ms), distance_m,
            lt_finish_target_distance_m);
    }

    if ((lt_state != LT_STATE_STOPPED) &&
        (lt_state != LT_STATE_LINE_LOST)){
        if (lt_state == LT_STATE_FINISH_OFFSET){
            /* 横线中心锁存后只走几何补偿距离，直行可避免宽线退出后的错误转向。 */
            LineFollowTest_UpdateLongitudinal(distance_m, dt);
            lt_curve_feedforward_mps = LineFollowTest_MoveTowards(
                lt_curve_feedforward_mps, 0.0f,
                lt_profile->steer_slew_mps2 * dt);
            lt_steer_command_mps = LineFollowTest_MoveTowards(
                lt_steer_command_mps, 0.0f,
                lt_profile->steer_slew_mps2 * dt);
            LineFollowTest_DriveWheels(
                lt_speed_command_mps + lt_steer_command_mps,
                lt_speed_command_mps - lt_steer_command_mps);
        } else{
            LINE_FOLLOW_OUTPUT control_out;
            LineFollowTest_UpdateCurveFeedforward(now, distance_m, dt);
            float curve_omega_ff_deg_s =
                LineFollowTest_CurveOmegaFeedforwardDegS(distance_m);
            /*
             * 出弯（含 S2->S3）直接用灰度+gz 串级循迹；弯道前馈本就会衰减到 0。
             * 只有在弯道内灰度瞬时全白时，才有界降级为纯 gz 保持（原兜底逻辑）。
             * （撤销早前的 S2->S3 陀螺回正致盲窗口：实测它挡住出弯灰度纠偏导致丢线。）
             */
            BSP_STATUS st = sensor_ready
                     ? LineFollow_EvaluateDetectedMaskWithOmegaFeedforward(
                           detected_mask, dt, curve_omega_ff_deg_s,
                           &control_out)
                     : BSP_STATUS_NOT_READY;
            bool edge_reacquire_allowed =
                (st != BSP_STATUS_OK) && sensor_ready &&
                (detected_mask == 0U) && lt_line_acquired &&
                (fabsf(lt_last_line_error_mm) >=
                 LT_LINE_REACQUIRE_EDGE_ERROR_MM) &&
                ((uint32_t)(now - lt_last_line_seen_ms) <=
                 LT_LINE_REACQUIRE_GRACE_MS);
            if (edge_reacquire_allowed){
                float reacquire_omega_deg_s =
                    (lt_last_line_error_mm > 0.0f)
                        ? -LT_LINE_REACQUIRE_OMEGA_DEGPS
                        : LT_LINE_REACQUIRE_OMEGA_DEGPS;
                st = LineFollow_EvaluateOmegaFeedforwardOnly(
                    reacquire_omega_deg_s, &control_out);
                lt_line_reacquire_active = (st == BSP_STATUS_OK);
            }
            bool gyro_only_allowed =
                !edge_reacquire_allowed && (st != BSP_STATUS_OK) &&
                sensor_ready &&
                (detected_mask == 0U) && lt_line_acquired &&
                LineFollowTest_IsRightCurve(distance_m) &&
                ((uint32_t)(now - lt_last_line_seen_ms) <=
                 LT_CURVE_GYRO_ONLY_GRACE_MS);
            if (gyro_only_allowed){
                st = LineFollow_EvaluateOmegaFeedforwardOnly(
                    curve_omega_ff_deg_s, &control_out);
                lt_curve_gyro_only = (st == BSP_STATUS_OK);
            }
            if (st != BSP_STATUS_OK){
                LineFollowTest_BrakeChassis();
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
                if (lt_line_reacquire_active){
                    LineFollowTest_UpdateReacquireSteering(dt);
                } else{
                    lt_last_line_error_mm = control_out.error;
                    LineFollowTest_UpdateSteering(&control_out, distance_m, dt);
                }
                /* 左右差速反对称，平均轮速始终等于纵向 S 曲线速度。 */
                LineFollowTest_DriveWheels(
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

    n = LtPutStr(l2, "err ");
    AppFmt_Fixed(&l2[n], out.error, 1);
    n = LtPutStr(l3, "v ");
    AppFmt_Fixed(&l3[n], lt_speed_command_mps, 3);
    n = LtPutStr(l4, "s ");
    AppFmt_Fixed(&l4[n], Chassis_GetDistance(), 2);

    const char *status;
    if (lt_state == LT_STATE_STOPPED){
        status = "DONE / BACK";
    } else if (lt_state == LT_STATE_LINE_LOST){
        status = "LOST / BACK";
    } else if (!sensor_ready){
        status = "SENSOR WAIT";
    } else if (lt_curve_gyro_only){
        status = "GYRO HOLD";
    } else if (out.line_lost){
        status = "LINE LOST";
    } else if (lt_state == LT_STATE_FINISH_LINE){
        status = "A LINE";
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

    Ui_RenderLines(lt_dry_run ? "H2 Push Calib" : lt_profile->title,
                   bits, l2, l3, l4, status, "BACK: exit");
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
const APP_TASK_DESC APP_H2_EMPTY_LAP_DRYRUN = {
    "H2 Push Calib", H2_DryRun_Enter, LineFollowTest_Tick, NULL
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

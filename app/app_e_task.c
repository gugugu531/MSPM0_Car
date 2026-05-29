#include "app_e_task.h"

#include "bsp_time.h"
#include "chassis.h"
#include "gimbal.h"
#include "gimbal_tracking/gimbal_tracking.h"
#include "key.h"
#include "line_follow.h"
#include "line_tracking/line_tracking.h"
#include "motion/motion.h"
#include "ui.h"

#include <stdbool.h>
#include <stdio.h>

#define APP_E_MAX_LAPS 5U
#define APP_E_EDGES_PER_LAP 4U
#define APP_E_LINE_TIMEOUT_MS 20000U
#define APP_E_AIM_2S_TIMEOUT_MS 2000U
#define APP_E_AIM_4S_TIMEOUT_MS 4000U
#define APP_E_RECT_SCAN_SPEED_DEG_S 120.0f
#define APP_E_RECT_SCAN_ANGLE_DEG 360.0f
#define APP_E_LINE_LOST_GRACE_MS 300U
#define APP_E_LINE_EDGE_MIN_DISTANCE_M 0.15f
#define APP_E_CORNER_INNER_DUTY_PERCENT 6.0f
#define APP_E_CORNER_OUTER_DUTY_PERCENT 30.0f
#define APP_E_LOOP_DELAY_MS 10U

typedef enum {
    APP_E_LINE_STATE_FOLLOW = 0,
    APP_E_LINE_STATE_CORNER_ARC
} APP_E_LINE_STATE;

typedef enum {
    APP_E_CORNER_NONE = 0,
    APP_E_CORNER_LEFT,
    APP_E_CORNER_RIGHT
} APP_E_CORNER_DIR;

static BSP_STATUS AppE_ApplyCornerArc(APP_E_CORNER_DIR corner_dir){
    if (corner_dir == APP_E_CORNER_LEFT){
        return Chassis_SetDuty(APP_E_CORNER_INNER_DUTY_PERCENT,
                               APP_E_CORNER_OUTER_DUTY_PERCENT);
    }

    if (corner_dir == APP_E_CORNER_RIGHT){
        return Chassis_SetDuty(APP_E_CORNER_OUTER_DUTY_PERCENT,
                               APP_E_CORNER_INNER_DUTY_PERCENT);
    }

    return BSP_STATUS_INVALID_ARG;
}

static uint32_t AppE_ElapsedMs(uint32_t start_ms){
    return BSP_Time_GetMs() - start_ms;
}

static void AppE_WaitBack(void){
    Key_ClearAllEvents();

    while (!Key_IsLongPress(KEY_ID_1)){
        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    Key_ClearAllEvents();
}

static uint8_t AppE_NormalizeLapCount(uint8_t lap_count){
    if (lap_count == 0U){
        return 1U;
    }

    if (lap_count > APP_E_MAX_LAPS){
        return APP_E_MAX_LAPS;
    }

    return lap_count;
}

static bool AppE_IsLineActive(const LINE_FOLLOW_SENSOR_STATE *sensor,
                              uint8_t index){
    if ((sensor == NULL) || (index >= LINE_FOLLOW_SENSOR_COUNT)){
        return false;
    }

    if ((LINE_TRACKING_ACTIVE_SENSOR_MASK & (1U << index)) == 0U){
        return false;
    }

    return sensor->value[index] == 0U;
}

static bool AppE_IsLineInnerActive(const LINE_FOLLOW_SENSOR_STATE *sensor){
    return AppE_IsLineActive(sensor, 2U) ||
           AppE_IsLineActive(sensor, 4U);
}

static uint8_t AppE_CountLineActiveRange(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                         uint8_t start,
                                         uint8_t end){
    uint8_t count = 0U;

    for (uint8_t i = start; i <= end && i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (AppE_IsLineActive(sensor, i)){
            count++;
        }
    }

    return count;
}

static uint8_t AppE_CountRawLineActive(const LINE_FOLLOW_SENSOR_STATE *sensor){
    uint8_t count = 0U;

    if (sensor == NULL){
        return 0U;
    }

    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (sensor->value[i] == 0U){
            count++;
        }
    }

    return count;
}

static APP_E_CORNER_DIR AppE_DetectCorner(const LINE_FOLLOW_SENSOR_STATE *sensor){
    uint8_t left_count = AppE_CountLineActiveRange(sensor, 0U, 2U);
    uint8_t right_count = AppE_CountLineActiveRange(sensor, 5U, 7U);
    bool inner_active = AppE_IsLineInnerActive(sensor);

    /*
     * 逻辑通道 3 已由有效通道掩码忽略，因此用 2/4 作为内侧参考。
     * 当前优先保证能够进入转弯：只要某一侧外侧至少一路触发，
     * 且该侧触发数量不小于另一侧，就判定为对应方向直角弯。
     */
    if (!inner_active && left_count >= 1U && left_count >= right_count){
        return APP_E_CORNER_LEFT;
    }

    if (!inner_active && right_count >= 1U && right_count > left_count){
        return APP_E_CORNER_RIGHT;
    }

    return APP_E_CORNER_NONE;
}

void AppE_RunLineFollow(uint8_t lap_count){
    char line0[24];
    char line1[24];
    char line2[24];
    uint8_t target_laps = AppE_NormalizeLapCount(lap_count);
    int32_t target_edges = (int32_t)(target_laps * APP_E_EDGES_PER_LAP);
    bool half_latched = false;
    float last_edge_distance = 0.0f;
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;
    uint32_t line_lost_start_ms = start_ms;
    uint32_t corner_count = 0U;
    bool line_lost_pending = false;
    APP_E_LINE_STATE line_state = APP_E_LINE_STATE_FOLLOW;
    APP_E_CORNER_DIR corner_dir = APP_E_CORNER_NONE;
    MOTION_COMMAND line_follow_command = Motion_CommandLineFollow();

    /*
     * E1 只使用底盘巡线，进入任务前停止可能残留的云台视觉跟踪。
     * 圈数换算为边线数：当前场地按每圈 4 条边线估计。
     */
    (void)GimbalTracking_Stop();
    LineFollow_Reset();
    LineTracking_Init(NULL);
    Motion_Init();
    Chassis_ResetDistance();
    Ui_RenderLines("E1 Line",
                   "Running...",
                   "Lap:0",
                   "Edge:0",
                   "Time:0.0",
                   "Long:stop",
                   NULL);

    while (AppE_ElapsedMs(start_ms) < APP_E_LINE_TIMEOUT_MS){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;

        BSP_STATUS status = BSP_STATUS_OK;
        LINE_FOLLOW_SENSOR_STATE sensor = {0};

        if (LineFollow_GetSensor(&sensor) != BSP_STATUS_OK){
            (void)LineFollow_UpdateSensor();
            (void)LineFollow_GetSensor(&sensor);
        }

        if (line_state == APP_E_LINE_STATE_FOLLOW){
            /* Motion_Apply() 只执行巡线运动原语，圈数和丢线仍由 E1 任务状态机判断。 */
            status = Motion_Apply(&line_follow_command, dt_s);
            (void)LineFollow_GetSensor(&sensor);

            APP_E_CORNER_DIR detected_corner = AppE_DetectCorner(&sensor);
            if (detected_corner != APP_E_CORNER_NONE){
                corner_dir = detected_corner;
                line_state = APP_E_LINE_STATE_CORNER_ARC;
                corner_count++;
                status = AppE_ApplyCornerArc(corner_dir);
            }
        } else{
            status = AppE_ApplyCornerArc(corner_dir);
            (void)LineFollow_UpdateSensor();
            (void)LineFollow_GetSensor(&sensor);

            if (AppE_IsLineInnerActive(&sensor)){
                line_state = APP_E_LINE_STATE_FOLLOW;
                corner_dir = APP_E_CORNER_NONE;
                line_lost_pending = false;
                LineTracking_Reset();
            }
        }

        float distance_m = Chassis_GetDistance();

        /*
         * 半线检测容易在同一条边上连续触发，因此同时使用 latch 和最小距离
         * 做去抖，避免车辆还没离开边线就重复计数。
         */
        if ((line_state == APP_E_LINE_STATE_FOLLOW) && LineFollow_IsHalfDetected()){
            if (!half_latched &&
                ((distance_m - last_edge_distance) >= APP_E_LINE_EDGE_MIN_DISTANCE_M)){
                LineFollow_IncrementEdge();
                last_edge_distance = distance_m;
                half_latched = true;
            }
        } else{
            half_latched = false;
        }

        uint8_t raw_active_count = AppE_CountRawLineActive(&sensor);
        bool line_missing = (line_state == APP_E_LINE_STATE_FOLLOW) &&
                            (raw_active_count < 2U);

        if (line_missing){
            if (!line_lost_pending){
                line_lost_start_ms = now_ms;
                line_lost_pending = true;
            }
        } else{
            line_lost_pending = false;
        }

        if (line_missing &&
            ((now_ms - line_lost_start_ms) >= APP_E_LINE_LOST_GRACE_MS)){
            char turn_line[24];

            /* 丢线属于题目流程故障，在 app 层刹车并等待用户长按返回。 */
            (void)Motion_Stop();
            snprintf(turn_line, sizeof(turn_line), "Turn:%lu", (unsigned long)corner_count);
            Ui_RenderLines("E1 Line",
                           "[WARN]",
                           "Line lost",
                           turn_line,
                           "Long:back",
                           NULL,
                           NULL);
            AppE_WaitBack();
            return;
        }

        int32_t edge_count = LineFollow_GetEdgeCount();
        uint8_t completed_laps = (uint8_t)(edge_count / APP_E_EDGES_PER_LAP);

        snprintf(line0, sizeof(line0), "Lap:%u/%u", completed_laps, target_laps);
        snprintf(line1, sizeof(line1), "Edge:%ld/%ld", (long)edge_count, (long)target_edges);
        snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        if (edge_count >= target_edges){
            (void)Motion_Stop();
            Ui_RenderStatusPage("E1 Line", UI_STATUS_OK, "Finished", "Long:back");
            AppE_WaitBack();
            return;
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)Motion_Stop();
    Ui_RenderStatusPage("E1 Line", UI_STATUS_WARN, "Stopped/timeout", "Long:back");
    AppE_WaitBack();
}

static void AppE_RunAimCenter(uint32_t timeout_ms, const char *title){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;

    (void)Chassis_Brake();
    GimbalTracking_Init(NULL);
    Ui_RenderLines(title,
                   "Aiming center",
                   "Laser:WAIT",
                   "Err:0,0",
                   "Time:0.0",
                   "Long:stop",
                   NULL);

    while (AppE_ElapsedMs(start_ms) < timeout_ms){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        BSP_STATUS status = GimbalTracking_UpdateLaserCenter(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();

        snprintf(line0, sizeof(line0), "Laser:%s", status == BSP_STATUS_OK ? "OK" : "WAIT");
        snprintf(line1, sizeof(line1), "Err:%0.0f,%0.0f", state.error.x, state.error.y);
        snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)GimbalTracking_Stop();
    Ui_RenderStatusPage(title, UI_STATUS_OK, "Aim finished", "Long:back");
    AppE_WaitBack();
}

void AppE_RunAimCenter2s(void){
    AppE_RunAimCenter(APP_E_AIM_2S_TIMEOUT_MS, "E2 Aim 2s");
}

static bool AppE_ScanForRect(void){
    char line0[24];
    char line1[24];
    uint32_t start_ms = BSP_Time_GetMs();

    GimbalTracking_Init(NULL);
    Gimbal_ResetAxisPosition(GIMBAL_AXIS_YAW);
    (void)Gimbal_SetSpeed(APP_E_RECT_SCAN_SPEED_DEG_S, 0.0f);
    Ui_RenderLines("E3 Rect",
                   "Scanning...",
                   "Yaw:0",
                   "Rect:WAIT",
                   "Long:stop",
                   NULL,
                   NULL);

    while (Gimbal_GetAngle().yaw_deg < APP_E_RECT_SCAN_ANGLE_DEG){
        (void)Gimbal_Update();

        if (GimbalTracking_IsRectValid()){
            (void)Gimbal_Stop();
            return true;
        }

        snprintf(line0, sizeof(line0), "Yaw:%0.0f", Gimbal_GetAngle().yaw_deg);
        snprintf(line1, sizeof(line1), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(3U, line1);

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)Gimbal_Stop();
    return false;
}

static void AppE_RunRectTrack(uint32_t timeout_ms){
    char line0[24];
    char line1[24];
    char line2[24];
    uint32_t start_ms = BSP_Time_GetMs();
    uint32_t last_ms = start_ms;

    GimbalTracking_Reset();
    Ui_RenderLines("E3 Rect",
                   "Tracking...",
                   "Rect:OK",
                   "Err:0,0",
                   "Time:0.0",
                   "Long:stop",
                   NULL);

    while (AppE_ElapsedMs(start_ms) < timeout_ms){
        uint32_t now_ms = BSP_Time_GetMs();
        float dt_s = (float)(now_ms - last_ms) / 1000.0f;

        if (dt_s <= 0.0f){
            dt_s = 0.001f;
        }

        last_ms = now_ms;
        BSP_STATUS status = GimbalTracking_UpdateRectCenter(dt_s);
        GIMBAL_TRACKING_STATE state = GimbalTracking_GetState();

        snprintf(line0, sizeof(line0), "Rect:%s", status == BSP_STATUS_OK ? "OK" : "WAIT");
        snprintf(line1, sizeof(line1), "Err:%0.0f,%0.0f", state.error.x, state.error.y);
        snprintf(line2, sizeof(line2), "Time:%lu.%01lu",
                 (unsigned long)(AppE_ElapsedMs(start_ms) / 1000U),
                 (unsigned long)((AppE_ElapsedMs(start_ms) / 100U) % 10U));
        Ui_UpdateContentLine(1U, line0);
        Ui_UpdateContentLine(2U, line1);
        Ui_UpdateContentLine(3U, line2);

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            break;
        }

        BSP_DelayMs(APP_E_LOOP_DELAY_MS);
    }

    (void)GimbalTracking_Stop();
}

void AppE_RunAimCenter4s(void){
    (void)Chassis_Brake();

    if (!AppE_ScanForRect()){
        Ui_RenderStatusPage("E3 Rect", UI_STATUS_WARN, "Rect not found", "Long:back");
        AppE_WaitBack();
        return;
    }

    AppE_RunRectTrack(APP_E_AIM_4S_TIMEOUT_MS);
    Ui_RenderStatusPage("E3 Rect", UI_STATUS_OK, "Track finished", "Long:back");
    AppE_WaitBack();
}

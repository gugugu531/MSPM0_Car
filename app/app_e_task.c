#include "app_e_task.h"

#include "bsp_time.h"
#include "chassis.h"
#include "gimbal_tracking/gimbal_tracking.h"
#include "key.h"
#include "line_follow.h"
#include "line_tracking/line_tracking.h"
#include "ui.h"

#include <stdbool.h>
#include <stdio.h>

#define APP_E_MAX_LAPS 5U
#define APP_E_EDGES_PER_LAP 4U
#define APP_E_LINE_TIMEOUT_MS 20000U
#define APP_E_AIM_2S_TIMEOUT_MS 2000U
#define APP_E_AIM_4S_TIMEOUT_MS 4000U
#define APP_E_LINE_EDGE_MIN_DISTANCE_M 0.15f
#define APP_E_LOOP_DELAY_MS 10U

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

    (void)GimbalTracking_Stop();
    LineFollow_Reset();
    LineTracking_Init(NULL);
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

        BSP_STATUS status = LineTracking_Update(dt_s);
        float distance_m = Chassis_GetDistance();

        if (LineFollow_IsHalfDetected()){
            if (!half_latched &&
                ((distance_m - last_edge_distance) >= APP_E_LINE_EDGE_MIN_DISTANCE_M)){
                LineFollow_IncrementEdge();
                last_edge_distance = distance_m;
                half_latched = true;
            }
        } else{
            half_latched = false;
        }

        if (status == BSP_STATUS_NOT_READY || LineFollow_IsEmpty()){
            (void)Chassis_Brake();
            Ui_RenderStatusPage("E1 Line", UI_STATUS_WARN, "Line lost", "Long:back");
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
            (void)Chassis_Brake();
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

    (void)Chassis_Brake();
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

void AppE_RunAimCenter4s(void){
    AppE_RunAimCenter(APP_E_AIM_4S_TIMEOUT_MS, "E3 Aim 4s");
}

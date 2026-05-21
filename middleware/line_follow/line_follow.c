/**
 * @file  line_follow.c
 * @brief Middleware line-follow runtime state implementation.
 */
#include "line_follow.h"
#include "grayscale_sensor.h"
#include <stddef.h>
#include <string.h>

static LINE_FOLLOW_STATE s_line_follow_state;

static uint8_t LineFollow_BuildMask(const uint8_t value[LINE_FOLLOW_SENSOR_COUNT]){
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (value[i] != 0U){
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

BSP_STATUS LineFollow_Init(void){
    LineFollow_Reset();
    return LineFollow_UpdateSensor();
}

void LineFollow_Reset(void){
    memset(&s_line_follow_state, 0, sizeof(s_line_follow_state));
}

BSP_STATUS LineFollow_Update(void){
    return LineFollow_UpdateSensor();
}

BSP_STATUS LineFollow_UpdateSensor(void){
    GrayscaleSensor_Read(s_line_follow_state.sensor.value);
    s_line_follow_state.sensor.mask = LineFollow_BuildMask(s_line_follow_state.sensor.value);
    return BSP_STATUS_OK;
}

BSP_STATUS LineFollow_GetState(LINE_FOLLOW_STATE *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    *out = s_line_follow_state;
    return BSP_STATUS_OK;
}

BSP_STATUS LineFollow_GetSensor(LINE_FOLLOW_SENSOR_STATE *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    *out = s_line_follow_state.sensor;
    return BSP_STATUS_OK;
}

uint8_t LineFollow_GetSensorMask(void){
    return s_line_follow_state.sensor.mask;
}

uint8_t LineFollow_GetSensorValue(uint8_t index){
    if (index >= LINE_FOLLOW_SENSOR_COUNT){
        return 0U;
    }

    return s_line_follow_state.sensor.value[index];
}

uint8_t LineFollow_GetActiveCount(void){
    uint8_t active_count = 0U;

    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (s_line_follow_state.sensor.value[i] == 0U){
            active_count++;
        }
    }

    return active_count;
}

bool LineFollow_IsActiveCountInRange(uint8_t min_count, uint8_t max_count){
    uint8_t active_count = LineFollow_GetActiveCount();

    return (active_count >= min_count) && (active_count <= max_count);
}

bool LineFollow_IsEmpty(void){
    return LineFollow_GetActiveCount() == 0U;
}

bool LineFollow_IsHalfDetected(void){
    return LineFollow_IsActiveCountInRange(3U, 6U);
}

bool LineFollow_IsCrossDetected(void){
    return LineFollow_IsActiveCountInRange(7U, 8U);
}

bool LineFollow_IsCenterActive(void){
    return (s_line_follow_state.sensor.value[3] == 0U) ||
           (s_line_follow_state.sensor.value[4] == 0U);
}

int32_t LineFollow_GetEdgeCount(void){
    return s_line_follow_state.edge_count;
}

void LineFollow_SetEdgeCount(int32_t edge_count){
    s_line_follow_state.edge_count = edge_count;
}

void LineFollow_IncrementEdge(void){
    s_line_follow_state.edge_count++;
}

void LineFollow_ResetEdge(void){
    s_line_follow_state.edge_count = 0;
}

bool LineFollow_IsTurning(void){
    return s_line_follow_state.turning;
}

void LineFollow_SetTurning(bool turning){
    s_line_follow_state.turning = turning;
}

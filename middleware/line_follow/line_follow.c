/**
 * @file  line_follow.c
 * @brief Middleware 层巡线运行状态服务实现。
 */
#include "line_follow.h"
#include "grayscale_sensor.h"
#include <stddef.h>
#include <string.h>

static LINE_FOLLOW_STATE line_follow_state;

static uint8_t LineFollow_BuildMask(const uint8_t value[LINE_FOLLOW_SENSOR_COUNT]){
    uint8_t mask = 0U;

    /*
     * mask 只保存最近一次传感器快照的 bit 表达，供调试显示和快速判断使用。
     * 具体“多少路触发算半线/十字”的语义放在下面的查询函数中。
     */
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
    memset(&line_follow_state, 0, sizeof(line_follow_state));
}

BSP_STATUS LineFollow_Update(void){
    return LineFollow_UpdateSensor();
}

BSP_STATUS LineFollow_UpdateSensor(void){
    /*
     * middleware 层只缓存 BSP 读数，不做 PID、里程或任务状态跳转。
     * 这样 app/core 可以按自己的周期重复读取同一份快照。
     */
    GrayscaleSensor_Read(line_follow_state.sensor.value);
    line_follow_state.sensor.mask =
        LineFollow_BuildMask(line_follow_state.sensor.value);

    return BSP_STATUS_OK;
}

BSP_STATUS LineFollow_GetSensor(LINE_FOLLOW_SENSOR_STATE *out){
    if (out == NULL){
        return BSP_STATUS_NULL;
    }

    *out = line_follow_state.sensor;
    return BSP_STATUS_OK;
}

uint8_t LineFollow_GetSensorMask(void){
    return line_follow_state.sensor.mask;
}

uint8_t LineFollow_GetActiveCount(void){
    uint8_t active_count = 0U;

    /*
     * 当前上层巡线算法沿用旧 Digital[] 语义：value == 0 表示检测到黑线。
     * 若后续统一为 BSP 的“有效为 1”语义，应同步调整这里和 line_tracking。
     */
    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (line_follow_state.sensor.value[i] == 0U){
            active_count++;
        }
    }

    return active_count;
}

int32_t LineFollow_GetEdgeCount(void){
    return line_follow_state.edge_count;
}

void LineFollow_IncrementEdge(void){
    line_follow_state.edge_count++;
}

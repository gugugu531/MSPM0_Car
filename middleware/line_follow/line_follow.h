/**
 * @file  line_follow.h
 * @brief Middleware line-follow runtime state interface.
 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "bsp_common.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_FOLLOW_SENSOR_COUNT 8U

typedef struct {
    uint8_t value[LINE_FOLLOW_SENSOR_COUNT];
    uint8_t mask;
} LINE_FOLLOW_SENSOR_STATE;

typedef struct {
    LINE_FOLLOW_SENSOR_STATE sensor;
    int32_t edge_count;
    bool turning;
} LINE_FOLLOW_STATE;

BSP_STATUS LineFollow_Init(void);
void LineFollow_Reset(void);

BSP_STATUS LineFollow_Update(void);
BSP_STATUS LineFollow_UpdateSensor(void);

BSP_STATUS LineFollow_GetState(LINE_FOLLOW_STATE *out);
BSP_STATUS LineFollow_GetSensor(LINE_FOLLOW_SENSOR_STATE *out);
uint8_t LineFollow_GetSensorMask(void);
uint8_t LineFollow_GetSensorValue(uint8_t index);

int32_t LineFollow_GetEdgeCount(void);
void LineFollow_SetEdgeCount(int32_t edge_count);
void LineFollow_IncrementEdge(void);
void LineFollow_ResetEdge(void);

bool LineFollow_IsTurning(void);
void LineFollow_SetTurning(bool turning);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_H */

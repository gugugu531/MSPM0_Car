#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include "bsp_common.h"
#include "line_follow.h"
#include "pid/pid.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_TRACKING_DEFAULT_BASE_DUTY 35.0f
#define LINE_TRACKING_DEFAULT_OUTPUT_LIMIT 100.0f
#define LINE_TRACKING_DEFAULT_CORRECTION_LIMIT 60.0f
#define LINE_TRACKING_DEFAULT_POSITION_SCALE 10.0f

typedef struct {
    float base_duty;
    float sensor_position_scale;
    float output_limit;
    PID_CONFIG pid_config;
} LINE_TRACKING_CONFIG;

typedef struct {
    float error;
    float correction;
    float left_duty;
    float right_duty;
    uint8_t active_count;
    bool line_lost;
} LINE_TRACKING_OUTPUT;

void LineTracking_Init(const LINE_TRACKING_CONFIG *config);
void LineTracking_Reset(void);
BSP_STATUS LineTracking_Update(float dt_s);
BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                float dt_s,
                                LINE_TRACKING_OUTPUT *out);
LINE_TRACKING_OUTPUT LineTracking_GetOutput(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_TRACKING_H */

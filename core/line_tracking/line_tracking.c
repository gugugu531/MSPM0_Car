#include "line_tracking.h"

#include "chassis.h"
#include "kinematics/kinematics.h"

#include <stddef.h>

static const float s_sensor_position[LINE_FOLLOW_SENSOR_COUNT] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f,
};

static LINE_TRACKING_CONFIG s_line_tracking_config;
static LINE_TRACKING_OUTPUT s_line_tracking_output;
static PID_CONTROLLER s_line_tracking_pid;
static bool s_line_tracking_initialized;

static LINE_TRACKING_CONFIG LineTracking_DefaultConfig(void){
    LINE_TRACKING_CONFIG config = {
        .base_duty = LINE_TRACKING_DEFAULT_BASE_DUTY,
        .sensor_position_scale = LINE_TRACKING_DEFAULT_POSITION_SCALE,
        .output_limit = LINE_TRACKING_DEFAULT_OUTPUT_LIMIT,
        .pid_config = {
            .kp = 30.0f,
            .ki = 0.0f,
            .kd = 1.5f,
            .integral_limit = 500.0f,
            .output_limit = LINE_TRACKING_DEFAULT_CORRECTION_LIMIT,
            .mode = PID_MODE_POSITION,
        },
    };

    return config;
}

static void LineTracking_EnsureInitialized(void){
    if (!s_line_tracking_initialized){
        LineTracking_Init(NULL);
    }
}

void LineTracking_Init(const LINE_TRACKING_CONFIG *config){
    if (config == NULL){
        s_line_tracking_config = LineTracking_DefaultConfig();
    } else{
        s_line_tracking_config = *config;
    }

    PID_Init(&s_line_tracking_pid, &s_line_tracking_config.pid_config);
    LineTracking_Reset();
    s_line_tracking_initialized = true;
}

void LineTracking_Reset(void){
    PID_Reset(&s_line_tracking_pid);

    s_line_tracking_output.error = 0.0f;
    s_line_tracking_output.correction = 0.0f;
    s_line_tracking_output.left_duty = 0.0f;
    s_line_tracking_output.right_duty = 0.0f;
    s_line_tracking_output.active_count = 0U;
    s_line_tracking_output.line_lost = true;
}

BSP_STATUS LineTracking_Update(float dt_s){
    LineTracking_EnsureInitialized();

    BSP_STATUS status = LineFollow_Update();
    if (status != BSP_STATUS_OK){
        return status;
    }

    LINE_FOLLOW_SENSOR_STATE sensor;
    status = LineFollow_GetSensor(&sensor);
    if (status != BSP_STATUS_OK){
        return status;
    }

    status = LineTracking_Compute(&sensor, dt_s, &s_line_tracking_output);
    if (status != BSP_STATUS_OK){
        return status;
    }

    if (s_line_tracking_output.line_lost){
        return BSP_STATUS_NOT_READY;
    }

    return Chassis_SetDuty(s_line_tracking_output.left_duty,
                           s_line_tracking_output.right_duty);
}

BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                float dt_s,
                                LINE_TRACKING_OUTPUT *out){
    if ((sensor == NULL) || (out == NULL)){
        return BSP_STATUS_NULL;
    }

    LineTracking_EnsureInitialized();

    float position_sum = 0.0f;
    uint8_t active_count = 0U;

    /* 灰度传感器在当前硬件上为低有效，value 为 0 表示检测到线。 */
    for (uint8_t i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (sensor->value[i] == 0U){
            position_sum += s_sensor_position[i];
            active_count++;
        }
    }

    out->active_count = active_count;
    out->line_lost = (active_count == 0U);

    if (out->line_lost){
        out->error = 0.0f;
        out->correction = 0.0f;
        out->left_duty = 0.0f;
        out->right_duty = 0.0f;
        return BSP_STATUS_OK;
    }

    /* 用有效传感器位置均值描述线中心偏移，避免多路同时触发时只取单点。 */
    float average_position = position_sum / (float)active_count;
    out->error = average_position * s_line_tracking_config.sensor_position_scale;
    out->correction = PID_Update(&s_line_tracking_pid, out->error, 0.0f, dt_s);

    /* forward 为基础占空比，turn 为 PID 修正量，最终由运动学工具做统一限幅。 */
    KINEMATICS_DIFFERENTIAL_OUTPUT duty =
        Kinematics_DifferentialMix(s_line_tracking_config.base_duty,
                                   out->correction,
                                   s_line_tracking_config.output_limit);

    out->left_duty = duty.left;
    out->right_duty = duty.right;

    return BSP_STATUS_OK;
}

LINE_TRACKING_OUTPUT LineTracking_GetOutput(void){
    return s_line_tracking_output;
}

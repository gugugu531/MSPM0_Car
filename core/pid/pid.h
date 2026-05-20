#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_MODE_POSITION = 0,
    PID_MODE_INCREMENTAL
} PID_MODE;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    PID_MODE mode;
} PID_CONFIG;

typedef struct {
    float target;
    float feedback;
    float error;
    float last_error;
    float prev_error;
    float integral;
    float derivative;
    float increment;
    float output;
} PID_STATE;

typedef struct {
    PID_CONFIG config;
    PID_STATE state;
} PID_CONTROLLER;

void PID_Init(PID_CONTROLLER *pid, const PID_CONFIG *config);
void PID_Reset(PID_CONTROLLER *pid);
void PID_SetConfig(PID_CONTROLLER *pid, const PID_CONFIG *config);
float PID_Update(PID_CONTROLLER *pid, float target, float feedback, float dt_s);
float PID_GetOutput(const PID_CONTROLLER *pid);
float PID_GetError(const PID_CONTROLLER *pid);
float PID_GetIncrement(const PID_CONTROLLER *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */

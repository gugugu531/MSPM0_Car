#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#define KINEMATICS_PI 3.1415926f
#define KINEMATICS_DEG_TO_RAD(deg) ((deg) * (KINEMATICS_PI / 180.0f))
#define KINEMATICS_RAD_TO_DEG(rad) ((rad) * (180.0f / KINEMATICS_PI))

#define DEG_TO_RAD(deg) KINEMATICS_DEG_TO_RAD(deg)
#define RAD_TO_DEG(rad) KINEMATICS_RAD_TO_DEG(rad)

typedef enum {
    KINEMATICS_DIR_LEFT = 0,
    KINEMATICS_DIR_RIGHT,
    KINEMATICS_DIR_FORWARD,
    KINEMATICS_DIR_BACKWARD,
    KINEMATICS_DIR_UNSTABLE
} KINEMATICS_DIR;

typedef struct {
    float yaw;
    float pitch;
    float roll;
} KINEMATICS_ATTITUDE;

typedef struct {
    float x_m;
    float y_m;
    float heading_deg;
} KINEMATICS_POSE;

typedef struct {
    float linear_mps;
    float angular_deg_s;
} KINEMATICS_VELOCITY;

typedef struct {
    float left;
    float right;
} KINEMATICS_DIFFERENTIAL_OUTPUT;

typedef KINEMATICS_ATTITUDE RotationAngles;

float Kinematics_Clamp(float value, float min_value, float max_value);
float Kinematics_NormalizeAngleDeg(float angle_deg);
float Kinematics_AngleDiffDeg(float target_deg, float current_deg);
float Kinematics_Distance2D(float x0_m, float y0_m, float x1_m, float y1_m);

KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward,
                                                          float turn,
                                                          float output_limit);

void Kinematics_PoseInit(KINEMATICS_POSE *pose);
void Kinematics_PoseUpdate(KINEMATICS_POSE *pose,
                           float linear_mps,
                           float heading_deg,
                           float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */

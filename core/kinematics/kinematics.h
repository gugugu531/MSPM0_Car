/**
 * @file  kinematics.h
 * @brief Core 层二维运动学和差速混控计算接口。
 *
 * 纯算法模块，不读传感器/不驱动执行器。当前唯一消费者是 middleware/line_tracking，
 * 只用到 Kinematics_Clamp 与 Kinematics_DifferentialMix；其余类型/宏/函数标注为
 * 「预留」——已实现并保留，但当前工程暂无调用者，供后续路径/姿态/动作策略模块复用。
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* 预留：角度/弧度换算宏，当前工程暂无调用者。 */
#define KINEMATICS_PI 3.1415926f
#define KINEMATICS_DEG_TO_RAD(deg) ((deg) * (KINEMATICS_PI / 180.0f))
#define KINEMATICS_RAD_TO_DEG(rad) ((rad) * (180.0f / KINEMATICS_PI))

/**
 * @brief 相对运动方向判定结果。
 * @note 预留：当前工程暂无调用者，供后续路径/动作策略模块表达方向。
 */
typedef enum {
    KINEMATICS_DIR_LEFT = 0,
    KINEMATICS_DIR_RIGHT,
    KINEMATICS_DIR_FORWARD,
    KINEMATICS_DIR_BACKWARD,
    KINEMATICS_DIR_UNSTABLE
} KINEMATICS_DIR;

/**
 * @brief 欧拉姿态角，单位 deg。
 * @note 预留：当前工程暂无调用者。
 */
typedef struct {
    float yaw;
    float pitch;
    float roll;
} KINEMATICS_ATTITUDE;

/**
 * @brief 二维平面位姿。
 * @note 预留：当前工程暂无调用者。
 */
typedef struct {
    /** X 坐标，单位 m。 */
    float x_m;
    /** Y 坐标，单位 m。 */
    float y_m;
    /** 航向角，单位 deg。 */
    float heading_deg;
} KINEMATICS_POSE;

/**
 * @brief 底盘平面速度指令。
 * @note 预留：当前工程暂无调用者。
 */
typedef struct {
    /** 线速度，单位 m/s。 */
    float linear_mps;
    /** 角速度，单位 deg/s。 */
    float angular_deg_s;
} KINEMATICS_VELOCITY;

/**
 * @brief 差速左右轮混控输出。
 */
typedef struct {
    /** 左轮输出，具体单位由调用方定义。 */
    float left;
    /** 右轮输出，具体单位由调用方定义。 */
    float right;
} KINEMATICS_DIFFERENTIAL_OUTPUT;

/**
 * @brief 将数值限制在指定闭区间内。
 */
float Kinematics_Clamp(float value, float min_value, float max_value);

/**
 * @brief 将角度归一化到 [-180, 180)。
 * @param angle_deg 输入角度，单位 deg。
 * @return 归一化后的角度，单位 deg。
 * @note 预留：当前工程暂无调用者（仅被 Kinematics_AngleDiffDeg 内部使用）。
 */
float Kinematics_NormalizeAngleDeg(float angle_deg);

/**
 * @brief 计算目标角到当前角的最短角度误差。
 * @return target_deg - current_deg 的归一化结果，单位 deg，范围 [-180, 180)。
 * @note 预留：当前工程暂无调用者。
 */
float Kinematics_AngleDiffDeg(float target_deg, float current_deg);

/**
 * @brief 将前进量和转向量混合为左右轮输出。
 * @param forward 前进分量。
 * @param turn 转向分量。
 * @param output_limit 左右轮输出绝对值限幅；<= 0 时不启用限幅。
 * @return 左右轮混控输出。
 */
KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward,
                                                          float turn,
                                                          float output_limit);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H */

# core/kinematics 接口说明

## 模块职责

`core/kinematics` 提供限幅、角度归一化和差速混控等纯算法计算能力，属于纯 `core` 模块。

该模块不读取传感器、不控制电机、不调用 BSP 或 middleware。调用方负责提供输入数据，并根据计算结果调用对应执行接口。

> 使用现状：当前唯一消费者是 `middleware/line_tracking`，只用到 `Kinematics_Clamp` 与
> `Kinematics_DifferentialMix`。本文标注为「预留」的类型/宏/函数已实现并保留，但当前工程
> 暂无调用者，供后续路径/姿态/动作策略模块复用。

## 基础宏（预留）

```c
#define KINEMATICS_PI 3.1415926f
#define KINEMATICS_DEG_TO_RAD(deg) ((deg) * (KINEMATICS_PI / 180.0f))
#define KINEMATICS_RAD_TO_DEG(rad) ((rad) * (180.0f / KINEMATICS_PI))
```

角度/弧度换算宏。**预留，当前工程暂无调用者。**

## 数据结构

```c
typedef enum {
    KINEMATICS_DIR_LEFT = 0,
    KINEMATICS_DIR_RIGHT,
    KINEMATICS_DIR_FORWARD,
    KINEMATICS_DIR_BACKWARD,
    KINEMATICS_DIR_UNSTABLE
} KINEMATICS_DIR;
```

通用方向枚举。**预留，当前工程暂无调用者**，供后续路径和动作策略模块表达方向。

```c
typedef struct {
    float yaw;
    float pitch;
    float roll;
} KINEMATICS_ATTITUDE;
```

姿态角，单位为度。**预留，当前工程暂无调用者。**

```c
typedef struct {
    float x_m;
    float y_m;
    float heading_deg;
} KINEMATICS_POSE;
```

二维位姿。`x_m` 和 `y_m` 单位为米，`heading_deg` 单位为度。**预留，当前工程暂无调用者。**

```c
typedef struct {
    float linear_mps;
    float angular_deg_s;
} KINEMATICS_VELOCITY;
```

平面运动速度。线速度单位为米每秒，角速度单位为度每秒。**预留，当前工程暂无调用者。**

```c
typedef struct {
    float left;
    float right;
} KINEMATICS_DIFFERENTIAL_OUTPUT;
```

差速混控输出。`left` 和 `right` 的单位由调用方输入决定：如果输入为占空比百分比，输出也为占空比百分比；如果输入为物理速度，输出也为对应物理量。

## 接口

### `float Kinematics_Clamp(float value, float min_value, float max_value)`

将 `value` 限制在 `[min_value, max_value]` 范围内。如果最小值大于最大值，函数内部会自动交换二者。

### `float Kinematics_NormalizeAngleDeg(float angle_deg)`（预留）

将角度归一化到 `[-180, 180)` 范围。**预留，当前工程暂无调用者**（仅被 `Kinematics_AngleDiffDeg` 内部使用）。

### `float Kinematics_AngleDiffDeg(float target_deg, float current_deg)`（预留）

返回从 `current_deg` 到 `target_deg` 的最短角度差，结果范围为 `[-180, 180)`。**预留，当前工程暂无调用者。**

### `KINEMATICS_DIFFERENTIAL_OUTPUT Kinematics_DifferentialMix(float forward, float turn, float output_limit)`

根据前进量和转向量计算差速左右输出：

```text
left  = forward + turn
right = forward - turn
```

当 `output_limit > 0.0f` 且任一侧输出超过限制时，函数按比例缩放左右输出，保持差速比例不变。该函数只计算输出，不调用 `Chassis_SetDuty()`。

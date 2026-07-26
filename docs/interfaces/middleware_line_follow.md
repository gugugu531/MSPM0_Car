# middleware/line_follow 接口说明

## 模块职责

`middleware/line_follow` 是完整巡线闭环控制器。它直接从 `bsp/grayscale_sensor` 获取已经
标准化的 GPIO 灰度读数，计算线中心、误差和差速修正，并通过 `middleware/chassis` 输出左右轮
占空比。

该模块同时提供硬件无关的 `LineFollow_Compute()`，便于使用人工构造的观测做算法验证。

数据流：

```text
bsp/grayscale_sensor
        ↓  level[i]: 0=检测到黑线，1=未检测到黑线
middleware/line_follow
        ↓  left/right duty
middleware/chassis
```

该模块不负责直角弯、圈数、边线计数、丢线搜索或任务完成条件；这些属于 app 任务状态机。

## 输入语义

```c
typedef struct {
    uint8_t level[LINE_FOLLOW_SENSOR_COUNT];
} LINE_FOLLOW_INPUT;
```

- `level[i] == 0`：通道 `i` 检测到黑线。
- `level[i] != 0`：通道 `i` 未检测到黑线。
- 通道顺序与 BSP 的 `GRAYSCALE_SENSOR_CHANNEL_0..7` 一致。

BSP 已处理板级输入反相和物理探头到逻辑通道的映射；上述语义来自 Device Check 实机显示，
middleware 不再二次翻转。

## 参数分组

```c
/* 传感器与输出。 */
#define LINE_FOLLOW_SENSOR_COUNT                       GRAYSCALE_SENSOR_CHANNEL_COUNT
#define LINE_FOLLOW_ACTIVE_SENSOR_MASK                 0xFFU
#define LINE_FOLLOW_DEFAULT_POSITION_SCALE             10.0f
#define LINE_FOLLOW_DEFAULT_BASE_DUTY                  34.0f
#define LINE_FOLLOW_DEFAULT_OUTPUT_LIMIT               100.0f
#define LINE_FOLLOW_DEFAULT_DIFFERENTIAL_LIMIT         16.0f

/* 误差预处理。 */
#define LINE_FOLLOW_ERROR_LPF_ALPHA                    0.5f
#define LINE_FOLLOW_ERROR_DEADBAND                     10.0f

/* 关闭陀螺增稳时使用的纯位置 PID。 */
#define LINE_FOLLOW_DEFAULT_PID_KP                     1.0f
#define LINE_FOLLOW_DEFAULT_PID_KI                     0.0f
#define LINE_FOLLOW_DEFAULT_PID_KD                     0.0f
#define LINE_FOLLOW_DEFAULT_PID_INTEGRAL_LIMIT         500.0f
#define LINE_FOLLOW_DEFAULT_PID_OUTPUT_LIMIT           60.0f
#define LINE_FOLLOW_DEFAULT_PID_MODE                   PID_MODE_POSITION

/* 默认启用的陀螺串级。 */
#define LINE_FOLLOW_DEFAULT_GYRO_STAB_ENABLED          true
#define LINE_FOLLOW_DEFAULT_GYRO_LINE_KP               3.0f
#define LINE_FOLLOW_DEFAULT_GYRO_STAB_KP               0.20f
#define LINE_FOLLOW_DEFAULT_OMEGA_REF_LIMIT            60.0f
#define LINE_FOLLOW_DEFAULT_GYRO_Z_SIGN                (1.0f)
```

默认使用“灰度偏差 P 外环 + 角速度 P 内环”，只有关闭 `gyro_stab_enabled` 时才调用通用
位置式 PID。`DIFFERENTIAL_LIMIT=16` 表示左右轮占空比差值不超过 16，因此最终
`correction` 会限制在 `±8`。

## 配置与输出

`LINE_FOLLOW_CONFIG` 保存基础占空比、误差缩放、输出/差值限幅、退化 PID 配置及陀螺串级
配置。

```c
typedef struct {
    uint8_t level_mask;
    uint8_t black_count;
    float error;
    float correction;
    float left_duty;
    float right_duty;
    bool line_lost;
} LINE_FOLLOW_OUTPUT;
```

- `level_mask`：本拍数字电平掩码；置 1 表示对应通道未检测到黑线，与 Device Check 一致。
- `black_count`：启用通道中检测到黑线（`level==0`）的数量。
- `error`：未经 EMA/死区处理的缩放误差。
- `correction`：陀螺串级或退化 PID 产生并经过差值限幅的修正量。
- `line_lost`：没有任何启用通道检测到线。

## 接口

### `LINE_FOLLOW_CONFIG LineFollow_GetDefaultConfig(void)`

返回默认配置副本，供上层覆盖个别参数后传给 `LineFollow_Init()`。

### `void LineFollow_Init(const LINE_FOLLOW_CONFIG *config)`

初始化控制器和运行状态；传 `NULL` 使用默认配置。

### `void LineFollow_Reset(void)`

复位 PID、EMA 滤波状态和最近输出，不改变配置。

### `BSP_STATUS LineFollow_Update(float dt_s)`

完整闭环入口：

1. 调用 `GrayscaleSensor_Read()` 获取 GPIO 灰度。
2. 从 JY61P 缓存读取 `gz`（启用陀螺增稳时）。
3. 调用 `LineFollow_Compute()`。
4. 未丢线时调用 `Chassis_SetDuty()`。

丢线时返回 `BSP_STATUS_NOT_READY`，不自行刹车或搜索。JY61P 的初始化与
`JY61P_I2C_Poll()` 调度仍由 app 任务负责。

### `BSP_STATUS LineFollow_Compute(...)`

```c
BSP_STATUS LineFollow_Compute(const LINE_FOLLOW_INPUT *input,
                              float dt_s,
                              float omega_deg_s,
                              LINE_FOLLOW_OUTPUT *out);
```

纯计算入口，不读取灰度硬件、不驱动底盘。处理顺序为：

```text
level[]（0=黑线）→ 线中心均值 → 缩放误差 → EMA → 中心死区
           → 陀螺串级或位置 PID → 差值限幅 → 差速混控
```

### `LINE_FOLLOW_OUTPUT LineFollow_GetOutput(void)`

返回最近一次输出和本拍观测摘要，供 app 显示和诊断。

## 已移除的旧状态包装

原 `middleware/line_follow` 仅缓存灰度快照，并保存无人使用的 `edge_count`；这层包装已合并。
以下接口不再存在：

- `LineFollow_UpdateSensor()`
- `LineFollow_GetSensor()` / `GetSensorMask()` / `GetActiveCount()`
- `LineFollow_GetEdgeCount()` / `IncrementEdge()`

如未来需要 GPIO/I2C 灰度源切换、带时间戳快照、坏道屏蔽或异步采样，再单独引入
`line_sensing` 观测层。

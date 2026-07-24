# middleware/chassis 接口说明

## 模块职责

`middleware/chassis` 是底盘组合服务，负责把 `bsp/motor/tb6612fng` 和 `bsp/motor/hall_encoder` 组合成上层可调用的底盘接口。

该模块负责：

- 初始化底盘电机驱动和编码器。
- 设置左右轮输出占空比百分比。
- 底盘刹车和滑行。
- 更新编码器采样。
- 提供底盘速度、距离、方向和左右轮输出状态快照。

该模块不负责：

- 巡线传感器快照。
- `Digital[]`、`sInedge`、`edge`、`turning` 等巡线运行状态。
- PID、运动学和题目流程。
- 错误消息存储和错误显示。

## 公开类型

### `CHASSIS_STOP_MODE`

```c
typedef enum {
    CHASSIS_STOP_MODE_COAST = 0,
    CHASSIS_STOP_MODE_BRAKE
} CHASSIS_STOP_MODE;
```

用于指定停止方式：

- `CHASSIS_STOP_MODE_COAST`：滑行，调用 `TB6612FNG_CoastAll()`。
- `CHASSIS_STOP_MODE_BRAKE`：刹车，调用 `TB6612FNG_BrakeAll()`。

### `CHASSIS_DUTY`

```c
typedef struct {
    float left_percent;
    float right_percent;
} CHASSIS_DUTY;
```

保存左右轮占空比百分比。正值表示前进方向，负值表示后退方向。实际限幅由 BSP `TB6612FNG_SetDuty()` 完成。

### `CHASSIS_STATUS`

```c
typedef struct {
    CHASSIS_DUTY duty;
    float speed_mps;
    float distance_m;
    HALL_ENCODER_DIR encoder_dir;
    TB6612FNG_OUTPUT left_output;
    TB6612FNG_OUTPUT right_output;
} CHASSIS_STATUS;
```

底盘状态快照。

工程现为**左右双轮**霍尔编码器。`Chassis_GetSpeed()`/`Chassis_GetDistance()` 返回**车体量**（左右轮均值），每轮独立值用 `Chassis_GetWheelSpeed(id)`/`Chassis_GetWheelDistance(id)` 获取。

## 公开接口

### `BSP_STATUS Chassis_Init(void)`

初始化底盘组合服务。内部调用：

- `TB6612FNG_Init()`
- `HallEncoder_Init()`

### `BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent)`

一次性设置左右轮占空比百分比。

示例：

```c
Chassis_SetDuty(40.0f, 40.0f);
Chassis_SetDuty(-40.0f, -40.0f);
Chassis_SetDuty(-30.0f, 30.0f);
```

### `BSP_STATUS Chassis_Stop(CHASSIS_STOP_MODE mode)`

按指定方式停止底盘。成功后会清零当前软件记录的 duty。

### `BSP_STATUS Chassis_Brake(void)`

刹车停止，等价于 `Chassis_Stop(CHASSIS_STOP_MODE_BRAKE)`。

### `BSP_STATUS Chassis_Coast(void)`

滑行停止，等价于 `Chassis_Stop(CHASSIS_STOP_MODE_COAST)`。

### `CHASSIS_DUTY Chassis_GetDuty(void)`

返回当前软件记录的左右轮占空比。

### `float Chassis_GetSpeed(void)`

返回车体估计线速度（左右轮均值），单位 m/s。内部用 `core/kinematics` 的 `Kinematics_WheelToBody` 取 `linear` 分量（角速度分量需 `track_width`，暂缺，故只取线速度）。

### `float Chassis_GetWheelSpeed(HALL_ENCODER_ID wheel)`

返回指定轮估计线速度，单位 m/s。

### `float Chassis_GetDistance(void)`

返回车体估计距离（左右轮均值），单位 m。

### `float Chassis_GetWheelDistance(HALL_ENCODER_ID wheel)`

返回指定轮估计距离，单位 m。

> 编码器采样与 GPIO/定时器中断由 `bsp/motor/hall_encoder.c` 内的 `GROUP1_IRQHandler()`（分别服务右轮 GPIOB / 左轮 GPIOA A 相）/ `TIMER_0_INST_IRQHandler()` 直接处理，middleware 不再转发。

### `void Chassis_ResetDistance(void)`

复位两轮编码器距离累计。

## 迁移说明

本轮重写不保留旧 `Motor_*`、`Encoder_*` 和 `UpdateSInedge()` 接口。

后续应将上层调用从：

- `Motor_SystemInit()`
- `Motor_SetLeft()`
- `Motor_SetRight()`
- `Motor_Brake()`
- `Encoder_GetSpeed()`
- `UpdateSInedge()`

迁移到 `Chassis_*` 或后续 `line_follow` 模块。

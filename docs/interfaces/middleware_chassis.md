# middleware/chassis 接口说明

## 模块职责

`middleware/chassis` 是底盘组合服务，负责把 `bsp/motor/tb6612fng` 和 `bsp/motor/hall_encoder` 组合成上层可调用的底盘接口。

该模块负责：

- 初始化底盘电机驱动和编码器。
- 开环设置左右轮输出占空比百分比。
- **闭环：每轮速度环 PID 跟踪目标轮速**（见「速度闭环」节）。
- 底盘刹车和滑行。
- 提供底盘/每轮速度、距离和左右轮输出状态快照。

该模块不负责：

- 巡线传感器快照。
- `Digital[]`、`sInedge`、`edge`、`turning` 等巡线运行状态。
- 运动学模型和题目流程（速度环复用 `core/pid`，但不含差速/位姿逻辑）。
- 错误消息存储和错误显示。

## 速度闭环

底盘有两种控制模式（`CHASSIS_CONTROL_MODE`）：

- `CHASSIS_CONTROL_DUTY`（默认，开环）：`Chassis_SetDuty()` 直接下发占空比。
- `CHASSIS_CONTROL_SPEED`（闭环）：`Chassis_SetWheelSpeed()`/`Chassis_SetSpeed()` 设定目标轮速后，
  由 **每轮独立的位置式速度环 PID**（复用 `core/pid`）跟踪：setpoint=目标轮速(m/s)、feedback=
  `HallEncoder_GetSpeed(id)`、output=占空比%(±100 限幅)。目标可为负（倒转）。

调用 `Chassis_SetWheelSpeed/SetSpeed` 进入闭环（从开环切入时复位 PID）；`Chassis_SetDuty/Stop/
Brake/Coast` 切回开环并清目标。实际出力由 `Chassis_UpdateSpeedControl(dt_s)` 周期驱动——当前
接在 `App_ControlTick`（RUN 态、20ms）：任务每拍设目标速度，速度环随后跑一步。退出 RUN 时
`App_ExitRun` 会 `Chassis_Brake` 自动停环。

默认增益（`chassis.h`，**须上板整定**，可用 Device Check「Speed PID」自检）：

```c
#define CHASSIS_SPEED_KP             200.00f
#define CHASSIS_SPEED_KI             17.0f
#define CHASSIS_SPEED_KD             0.0f
#define CHASSIS_SPEED_INTEGRAL_LIMIT 96.0f
#define CHASSIS_SPEED_OUTPUT_LIMIT   100.0f
```

`CHASSIS_SPEED_INTEGRAL_LIMIT` 限制的是 PID 内部积分累计值本身，不是已乘 `Ki` 的积分项；
最终占空比仍由 `CHASSIS_SPEED_OUTPUT_LIMIT` 限制。

> 差速转向（车体 v/ω → 双轮目标）需 `track_width`（暂缺）；有了轮距后用 `core/kinematics`
> 的 `Kinematics_BodyToWheel` 在 `Chassis_SetWheelSpeed` 之上加一层即可。

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

工程现为**左右双轮**霍尔编码器。`Chassis_GetSpeed()`/`Chassis_GetDistance()` 返回**车体量**（左右轮均值），每轮独立值用 `Chassis_GetWheelSpeed(id)`/`Chassis_GetWheelDistance(id)` 获取。

## 公开接口

### `BSP_STATUS Chassis_Init(void)`

初始化底盘组合服务。内部调用：

- `TB6612FNG_Init()`
- `HallEncoder_Init()`

### `BSP_STATUS Chassis_SetDuty(float left_percent, float right_percent)`

一次性设置左右轮占空比百分比（开环，切回 `CHASSIS_CONTROL_DUTY`）。

示例：

```c
Chassis_SetDuty(40.0f, 40.0f);
Chassis_SetDuty(-40.0f, -40.0f);
Chassis_SetDuty(-30.0f, 30.0f);
```

### `void Chassis_SetWheelSpeed(float left_mps, float right_mps)`

设置左右轮目标线速度（m/s，负为倒转）并进入速度闭环。仅设定目标，实际出力由 `Chassis_UpdateSpeedControl()` 周期驱动；从开环切入时复位 PID。

### `void Chassis_SetSpeed(float body_mps)`

设置车体直行目标线速度（两轮同速），进入速度闭环。差速转向需 `track_width`（暂缺）。

### `BSP_STATUS Chassis_UpdateSpeedControl(float dt_s)`

速度闭环单步更新：闭环模式下跑两轮 PID 并出力；开环模式为空操作（恒返回 `BSP_STATUS_OK`）。须由控制周期任务周期调用（当前接在 `App_ControlTick`，20ms）。

### `CHASSIS_CONTROL_MODE Chassis_GetControlMode(void)`

返回当前控制模式（开环/闭环）。

### `float Chassis_GetWheelSpeedTarget(HALL_ENCODER_ID wheel)`

返回指定轮当前目标线速度（m/s），仅闭环模式有意义。

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

> 编码器采样与 GPIO/定时器中断由 `bsp/motor/hall_encoder.c` 内的 `GROUP1_IRQHandler()`（服务 GPIOA 上的两轮 A 相）/ `TIMER_0_INST_IRQHandler()` 直接处理，middleware 不再转发。

### `void Chassis_ResetDistance(void)`

复位两轮编码器距离累计。

## 迁移说明

早期的 `Motor_*`、`Encoder_*` 和 `UpdateSInedge()` 接口在底盘层重写时已废弃，上层调用
**均已迁移完毕**，现统一走 `Chassis_*`（底盘组合服务）与 `HallEncoder_*`（BSP 层测速）。
仓库内已无这些旧接口的定义与调用者。

# core/gimbal_tracking 接口说明

## 模块职责

`core/gimbal_tracking` 提供基于 CanMV 视觉目标的云台跟踪控制。

该模块负责：

- 从 `bsp/canmv` 获取激光点和矩形角点数据。
- 使用 `core/geometry` 计算纸面圆点到图像矩形的映射。
- 使用 `core/pid` 计算 yaw/pitch 跟踪速度。
- 调用 `middleware/gimbal` 输出云台速度。

该模块不直接调用 BSP 步进电机驱动，不访问旧 `Laser_Loc`、`Rect_Loc` 全局变量，不处理 LED 调试和巡线阶段距离积分。

## 数据结构

```c
typedef struct {
    PID_CONFIG yaw_pid;
    PID_CONFIG pitch_pid;
    float image_height;
    float paper_width;
    float paper_height;
    float circle_radius;
    float yaw_output_sign;
    float pitch_output_sign;
} GIMBAL_TRACKING_CONFIG;
```

- `yaw_pid` / `pitch_pid`：yaw 和 pitch 两个方向的 PID 参数。
- `image_height`：图像高度，用于将旧图像坐标转换为当前使用的 y 轴方向；默认值为 `240`，对齐当前 `k230/rect_07.py` 的 `320x240` 输出。
- `paper_width` / `paper_height`：纸面尺寸。
- `circle_radius`：纸面圆路径半径。
- `yaw_output_sign` / `pitch_output_sign`：输出方向系数，用于适配安装方向。

```c
typedef struct {
    CORE_POINT2F target;
    CORE_POINT2F laser;
    CORE_POINT2F error;
    float yaw_speed;
    float pitch_speed;
    CANMV_STATUS laser_status;
    CANMV_STATUS rect_status;
    bool target_valid;
    bool laser_valid;
} GIMBAL_TRACKING_STATE;
```

保存最近一次跟踪状态，供调试显示和上层状态判断使用。

## 接口

### `void GimbalTracking_Init(const GIMBAL_TRACKING_CONFIG *config)`

初始化云台跟踪控制器。传入 `NULL` 时使用默认配置。

### `void GimbalTracking_Reset(void)`

清空 PID 状态和最近一次跟踪状态。

### `BSP_STATUS GimbalTracking_UpdateLaserCenter(float dt_s)`

从 `CANMV_TARGET_LASER` 读取目标中心和激光点，并执行一次云台跟踪控制。

该接口对应旧流程中的：

```text
SetLaserPosition()
SetTargetCenter()
PID_SMotor_Cont()
```

### `BSP_STATUS GimbalTracking_UpdateRectCircle(int32_t edge_index, float angle_offset_deg, float dt_s)`

从 `CANMV_TARGET_RECT` 读取矩形角点，根据 `edge_index` 和 `angle_offset_deg` 计算纸面圆点，并映射到图像矩形后执行跟踪。

该接口不读取旧 `edge` 和 `sInedge`，调用方负责传入阶段参数。

### `BSP_STATUS GimbalTracking_TrackPoints(CORE_POINT2F target, CORE_POINT2F laser, float dt_s)`

直接根据目标点和激光点执行一次 PID 控制并调用 `Gimbal_SetSpeed()`。

### `BSP_STATUS GimbalTracking_Stop(void)`

调用 `Gimbal_Stop()` 停止云台运动。

### `GIMBAL_TRACKING_STATE GimbalTracking_GetState(void)`

返回最近一次云台跟踪状态。

## 边界说明

- 旧 `Compute_excur()` 不迁入本模块。该逻辑混合巡线转弯状态、编码器速度和 LED GPIO 调试，后续应按实际任务重新设计。
- 旧 `getDistance()` 不迁入本模块。阶段距离逻辑后续基于 `Chassis_GetDistance()` 或任务上下文重新设计。
- 本模块不直接操作 GPIO，不直接调用 `StepMotor_*` 或 `YP_SMotor_*`。

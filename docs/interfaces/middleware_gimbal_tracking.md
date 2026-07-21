# middleware/gimbal_tracking 接口说明

## 模块职责

`middleware/gimbal_tracking` 提供基于 CanMV 视觉目标的云台跟踪控制。

该模块负责：

- 从 `bsp/canmv` 获取 K230 的 yaw/pitch 角度误差帧。
- 使用 `core/pid` 计算 yaw/pitch 跟踪速度。
- 调用 `middleware/gimbal` 输出云台速度。

该模块不直接调用 BSP 步进电机驱动，不访问旧 `Laser_Loc`、`Rect_Loc` 全局变量，不处理 LED 调试和巡线阶段距离积分。

## 数据结构

```c
typedef struct {
    PID_CONFIG yaw_angle_pid;
    PID_CONFIG pitch_angle_pid;
    float yaw_output_sign;
    float pitch_output_sign;
    uint32_t link_timeout_ms;
} GIMBAL_TRACKING_CONFIG;
```

- `yaw_angle_pid` / `pitch_angle_pid`：角度帧（deg 误差）使用的独立 PID 参数，输出为 deg/s；须在实机上整定（上位机 `s ykp/...` 调的即这两组）。
- `yaw_output_sign` / `pitch_output_sign`：输出方向系数，用于适配安装方向。
- `link_timeout_ms`：K230 有效目标数据超时时间，默认 `1000ms`；在该时间内没有收到可用于跟踪的有效目标数据时，视觉跟踪会立即停止云台输出。

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
    bool link_timeout;
} GIMBAL_TRACKING_STATE;
```

保存最近一次跟踪状态，供调试显示和上层状态判断使用。

## 接口

### `void GimbalTracking_Init(const GIMBAL_TRACKING_CONFIG *config)`

初始化云台跟踪控制器。传入 `NULL` 时使用默认配置。

初始化会记录当前系统时间，作为后续有效目标数据超时判断的起点。

### `void GimbalTracking_Reset(void)`

清空 PID 状态和最近一次跟踪状态。

> 旧的像素激光跟踪路径（`UpdateLaserCenter` / `TrackPoints` / `ReadLaser` 及像素 PID）已随 K230 改用角度协议而移除；`UpdateAngle` 为唯一视觉闭环入口。

### `BSP_STATUS GimbalTracking_Stop(void)`

调用 `Gimbal_Stop()` 停止云台运动。

### `GIMBAL_TRACKING_STATE GimbalTracking_GetState(void)`

返回最近一次云台跟踪状态。

## 边界说明

- 旧 `Compute_excur()` 不迁入本模块。该逻辑混合巡线转弯状态、编码器速度和 LED GPIO 调试，后续应按实际任务重新设计。
- 旧 `getDistance()` 不迁入本模块。阶段距离逻辑后续基于 `Chassis_GetDistance()` 或任务上下文重新设计。
- 本模块不直接操作 GPIO，不直接调用 `StepMotor_*` 或 `YP_SMotor_*`。

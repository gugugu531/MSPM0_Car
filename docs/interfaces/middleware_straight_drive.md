# middleware/straight_drive 接口说明

## 模块职责

`middleware/straight_drive` 组合 JY61P 缓存、通用 PID、差速混控与底盘服务，集中实现四种
直行测试方式。它不负责菜单、按键、OLED、串口格式化或 I2C 轮询调度；这些仍由 app 任务编排。

支持模式：

- `STRAIGHT_DRIVE_MODE_DUTY_OPEN`：左右轮相同占空比。
- `STRAIGHT_DRIVE_MODE_SPEED`：调用 chassis 双轮速度闭环。
- `STRAIGHT_DRIVE_MODE_GYRO_RATE`：基础占空比叠加 `gz -> 0` 差速修正。
- `STRAIGHT_DRIVE_MODE_GYRO_HEADING`：基础占空比叠加巡航角差速修正。

## 公开接口

- `StraightDrive_Init(mode)`：清空运行状态与里程、初始化 PID，并装载模式默认指令。
- `StraightDrive_AdjustCommand(steps)`：按照当前模式的占空比或速度步长调整指令并限幅。
- `StraightDrive_ZeroCommand()`：停车并清除巡航角基准；下次启动时重新捕获。
- `StraightDrive_Update(dt_s)`：读取 BSP 缓存，计算闭环并向 chassis 下发控制。
- `StraightDrive_GetOutput()`：返回显示和遥测所需的状态快照，不触发硬件访问或控制计算。

## 姿态有效性与安全行为

控制器通过 `JY61P_I2C_IsDataFresh(STRAIGHT_DRIVE_IMU_MAX_AGE_MS)` 判断最近完整
angle + gyro 样本是否仍有效，不再从轮询次数与错误次数推断 I2C 事务结果。姿态模式在首帧
有效样本前或样本超时后将两轮占空比置零并复位 PID；短暂失效不会清除已锁定的巡航角。

巡航角模式在本次运动首次真正输出非零占空比前捕获当前 yaw。只有指令归零后再次启动，或
重新初始化任务时才重新捕获。角度误差使用 `Kinematics_AngleDiffDeg()` 限定到
`[-180, 180)`。

## 参数位置

默认指令、步长、指令/输出限幅、IMU 最大时延、两组 PID 与 yaw/gz 反馈符号均集中在
`middleware/straight_drive/straight_drive.h`，宏统一使用 `STRAIGHT_DRIVE_*` 前缀。

当前实车反馈方向为：

- `STRAIGHT_DRIVE_RATE_GYRO_SIGN = 1.0f`
- `STRAIGHT_DRIVE_HEADING_YAW_SIGN = -1.0f`

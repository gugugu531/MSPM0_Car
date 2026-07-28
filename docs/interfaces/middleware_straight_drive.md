# middleware/straight_drive 接口说明

## 模块职责

`middleware/straight_drive` 组合 JY61P 缓存、通用 PID、差速混控与底盘服务，集中实现九种
直行测试方式。它不负责菜单、按键、OLED、串口格式化或 I2C 轮询调度；这些仍由 app 任务编排。

支持模式：

- `STRAIGHT_DRIVE_MODE_DUTY_OPEN`：左右轮相同占空比。
- `STRAIGHT_DRIVE_MODE_SPEED`：调用 chassis 双轮速度闭环。
- `STRAIGHT_DRIVE_MODE_GYRO_RATE`：基础占空比叠加 `gz -> 0` 差速修正。
- `STRAIGHT_DRIVE_MODE_GYRO_HEADING`：基础占空比叠加巡航角差速修正。
- `STRAIGHT_DRIVE_MODE_RAMP_HEADING`：占空比线性上升，全程保持启动航向。
- `STRAIGHT_DRIVE_MODE_RATE_THEN_HEADING`：前 1 s 保持 `gz=0`，随后保持切换航向。
- `STRAIGHT_DRIVE_MODE_ENCODER_THEN_HEADING`：前 1 s 闭环左右轮累计路程差，随后保持切换航向。
- `STRAIGHT_DRIVE_MODE_INTEGRATED_THEN_HEADING`：80% 对照版，前 500 ms 用固定控制周期积分
  `gz` 得到 A，随后用融合角 B 巡航。
- `STRAIGHT_DRIVE_MODE_FULL_INTEGRATED_THEN_HEADING`：100% 速度优先版，前 500 ms 仅按
  IMU 新样本实际间隔积分 A，随后用融合角 B 巡航。

## 公开接口

- `StraightDrive_Init(mode)`：清空运行状态与里程、初始化 PID，并装载模式默认指令。
- `StraightDrive_AdjustCommand(steps)`：按照当前模式的占空比或速度步长调整指令并限幅。
- `StraightDrive_ZeroCommand()`：停车并清除巡航角基准；下次启动时重新捕获。
- `StraightDrive_Update(dt_s)`：读取 BSP 缓存，计算闭环并向 chassis 下发控制。
- `StraightDrive_GetOutput()`：返回显示和遥测所需的状态快照，不触发硬件访问或控制计算。

## 姿态有效性与安全行为

控制器通过 `JY61P_I2C_IsDataFresh(STRAIGHT_DRIVE_IMU_MAX_AGE_MS)` 判断最近完整
angle + gyro 样本是否仍有效，不再从轮询次数与错误次数推断 I2C 事务结果。姿态模式在首帧
有效样本前或样本超时后将两轮占空比置零并复位 PID。基础巡航角模式保留已锁定参考角；
带启动阶段的实验模式会重新开始该阶段，避免在样本缺口上继续积分或计时。

巡航角模式在本次运动首次真正输出非零占空比前捕获当前 yaw。只有指令归零后再次启动，或
重新初始化任务时才重新捕获。角度误差使用 `Kinematics_AngleDiffDeg()` 限定到
`[-180, 180)`。

## A/B 航向切换

纯积分角记为 A，经过 JY61P 姿态融合的航向记为 B。启动首个完整样本建立 `A0=B0`；切换时
取得 `A1/B1`，第二阶段参考角为：

```text
reference = normalize(B0 + shortest_angle(B1 - A1))
```

该式将猛烈加速期间 B 相对 A 产生的偏移补偿到初始航向上。它不能消除 A 自身的积分误差，
因此保留 `Device Check -> Yaw A/B` 无电机页面用于先观察 `A`、`B`、`B-A` 和 `gz`。

100% 模式无法再提高快侧占空比。控制器保持原本 `2*correction` 的轮间差，并把两轮整体
下移到快侧为 100%；修正量限制为 ±10%，所以最强修正对应 `100%/80%`。

## 参数位置

默认指令、步长、指令/输出限幅、启动阶段时长、IMU 最大时延、各组 PID 与 yaw/gz 符号均集中在
`middleware/straight_drive/straight_drive.h`，宏统一使用 `STRAIGHT_DRIVE_*` 前缀。

当前实车反馈方向为：

- `STRAIGHT_DRIVE_RATE_GYRO_SIGN = 1.0f`：只用于已验证的 `gz -> 0` 角速度闭环；
- `STRAIGHT_DRIVE_INTEGRATION_GYRO_SIGN = -1.0f`：只用于 A/B 航向积分；
- `STRAIGHT_DRIVE_HEADING_YAW_SIGN = -1.0f`

角速度闭环反馈极性和航向积分坐标转换是两种独立语义，不得共用同一个符号宏。积分 `gz`
与融合 yaw 必须同向，确保转动车体时纯积分角 A 与融合角 B 同号变化。

当前已识别的快照一致性、模式分支扩散、A/B 重复实现和测试缺口统一登记在
[`../todo.md`](../todo.md#直行控制与-jy61p-数据链维护债务)。后续修复必须遵守其中的隔离约束，
不得把 Straight Test 的阶段切换或专用混控耦合进 chassis、通用 PID、循迹或其他检查任务。

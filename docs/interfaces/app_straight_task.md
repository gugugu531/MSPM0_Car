# app/app_straight_task 直行测试适配说明

任务层负责九个菜单入口、JY61P I2C 轮询调度、按键映射、OLED 显示和 Debug_Ex 遥测。
模式状态、指令限幅、PID、IMU 新鲜度判断、巡航角锁定和底盘输出统一由
`middleware/straight_drive` 实现，详见 `middleware_straight_drive.md`。

## 入口与操作

`Main Menu -> Straight Test` 下有九个任务：

| 菜单项 | 控制方式 | 指令单位 |
|---|---|---|
| `Duty Open` | 左右轮相同占空比，无反馈 | `%` |
| `Speed Closed` | 左右轮独立速度 PID | `m/s` |
| `Duty+Gyro Rate` | 基础占空比 + `gz -> 0` 角速度 PID | `%` |
| `Duty+Yaw Hold` | 基础占空比 + 锁定 yaw 巡航角 PID | `%` |
| `Ramp Yaw Hold` | 占空比线性上升，全程锁定启动 yaw | `%` |
| `80 Rate->Yaw` | 80%，前 1 s 角速度闭环，随后锁定切换 yaw | `%` |
| `80 Enc->Yaw` | 80%，前 1 s 左右轮累计路程差闭环，随后锁定切换 yaw | `%` |
| `80 Int->Yaw` | 80%，前 500 ms 纯积分航向闭环，随后使用误差修正的融合 yaw | `%` |
| `100 Int->Yaw` | 100% 速度优先版，按 IMU 新样本积分后切换到修正融合 yaw | `%` |

选中具体任务后会直接运动（仅浏览 `Straight Test` 子菜单不会驱动电机）：

- 常规占空比模式以 `80%` 基础占空比启动，速度闭环以 `1.06 m/s` 目标启动。
- `Ramp Yaw Hold` 从 0 开始按 `50%/s` 上升，约 1.6 s 达到 80%。
- `100 Int->Yaw` 以 100% 启动；其满占空比差速修正只降低慢侧，不尝试把快侧提高到
  100% 以上。
- 所有模式都以左右轮绝对累计路程平均值估算车体距离，达到 `3.0 m` 后主动归零并由
  app 框架刹车返回菜单。

- `UP` / `DOWN`：增加/减少指令。
- `ENTER`：指令归零。
- `BACK`：退出任务，由 app 框架统一主动刹车。

OLED 显示指令、实际左右占空比与编码器速度；陀螺模式额外显示 `gz`、
`yaw`、巡航角或差速修正量。

## Debug_Ex 遥测

九种任务都在每个 20ms 控制拍通过 `Debug_Ex/UART1`（115200 8N1）非阻塞发送一行：

```text
[STR] t=<ms> m=<0..8> imu=<0|1> cmd=<target> act=<applied> phase=<0|1> dl=<%> dr=<%> vl=<m/s> vr=<m/s> xl=<m> xr=<m> yaw=<deg> gz=<deg/s> iyaw=<deg> corr=<%>
```

- `m`：依次对应上述表格，取值 `0..8`。
- `imu`：JY61P 完整 angle + gyro 样本有效标志；所有模式都轮询 JY61P，姿态闭环模式
  会把它用于控制。
- `cmd/act`：目标指令与本拍应用的基础指令；斜坡模式二者不同。速度模式单位为 `m/s`，
  其余模式为 `%`。
- `phase`：`0` 表示启动/斜坡阶段，`1` 表示该阶段完成；无阶段模式初始化后为 `1`。
- `dl/dr`：左/右轮已应用占空比。速度闭环中是上一控制拍的 PID 输出。
- `vl/vr`：左/右轮线速度。
- `xl/xr`：左/右轮累计距离。
- `yaw/gz`：分别经 `STRAIGHT_DRIVE_HEADING_YAW_SIGN` 和
  `STRAIGHT_DRIVE_RATE_GYRO_SIGN` 校正的融合航向角与角速度闭环反馈。A/B 纯积分另用
  `STRAIGHT_DRIVE_INTEGRATION_GYRO_SIGN`，避免改变已验证的角速度闭环极性。
- `iyaw`：从启动时融合角 `B0` 起算的纯 `gz` 积分角 `A`；非积分模式为 0。
- `corr`：当前闭环给出的差速修正；开环和速度模式为 0。

上位机工具：

```powershell
python tools/straight_test_viz.py --list
python tools/straight_test_viz.py --port COM7
python tools/straight_test_viz.py --port COM7 --csv straight.csv --log straight_raw.txt
```

依赖为 `pyserial` 和 `matplotlib`。窗口实时显示左右轮占空比、速度、距离及
融合 yaw/积分 yaw/gz；空格暂停，`c` 清空，`q` 退出。

## 闭环结构

### 角速度闭环

```text
target_gz = 0
correction = PID(target_gz, corrected_gz)
left  = base_duty + correction
right = base_duty - correction
```

### 基础巡航角闭环

`Duty+Yaw Hold` 进入任务或指令从 `0` 变为非零时，在真正下发第一拍非零占空比前，
把当前 yaw 锁定为本次运动的基准角。运动期间即使 IMU 短暂失效，该基准角也不会改变；
只有 `ENTER` 归零后再次启动，或重新进入任务时才重新采集。角度误差通过
`Kinematics_AngleDiffDeg()` 限定到 `[-180, 180)`，避免跨越 `-180/180` 时绕远旋转。

```text
error = shortest_angle(target_yaw - current_yaw)
correction = PID(error, 0)
left  = base_duty + correction
right = base_duty - correction
```

### 纯积分角与融合角切换

记启动时纯积分角与 JY61P 融合角为 `A0`、`B0`，代码初始化为 `A0=B0`；切换时两者为
`A1`、`B1`。第二阶段参考角为：

```text
reference = normalize(B0 + shortest_angle(B1 - A1))
```

前 500 ms 用 `A` 相对 `B0` 做航向闭环。`80 Int->Yaw` 保留按 20 ms 控制周期积分的旧版，
用于实车对照；`100 Int->Yaw` 仅在 `JY61P_I2C_GetSampleCount()` 变化时，按相邻完整样本的
实际时间间隔积分，确保 `A1` 和 `B1` 来自同一采样时序。

100% 基础占空比没有向上的调节余量，因此普通 `base±correction` 会被上限截断。该模式先
保留 `2*correction` 的轮间差，再将两轮整体下移到快侧恰为 100%；修正限幅为 ±10%，极限
输出为 `100%/80%`。

## 安全与整定

- 九种模式进入后都启动 JY61P 非阻塞 I2C 轮询。需要姿态的模式在至少得到一帧有效完整
  样本前，即使已有非零指令，两轮仍保持零占空比。
- 运行中若 IMU 完整样本超过 `STRAIGHT_DRIVE_IMU_MAX_AGE_MS` 未更新，则把输出置零并清除
  PID 动态状态。`Duty+Yaw Hold` 保留开始运动时锁定的参考角；带斜坡或定时切换阶段的模式
  会清除阶段参考并在采样恢复后重新开始启动阶段，避免跨越样本缺口继续积分或计时。
- 常规占空比基础指令限制为 `±80%`，100% 实验模式允许到 `±100%`；单轮输出统一限制
  为 `±100%`。
- 角速度、累计路程差与航向 PID 默认为 P 控制，完整 `Kp/Ki/Kd`、积分和输出限幅宏集中在
  `middleware/straight_drive/straight_drive.h`，须根据实车整定。
- yaw、角速度闭环反馈和航向积分 gz 使用独立符号宏。实车观察到原始 yaw 会使巡航角环先掉头约
  `180°`，因此 `STRAIGHT_DRIVE_HEADING_YAW_SIGN=-1.0f`；但 gz 同步反相后角速度模式会持续
  转圈，因此 `STRAIGHT_DRIVE_RATE_GYRO_SIGN=1.0f`。A/B 积分必须与修正后的融合 yaw 同向，
  单独使用 `STRAIGHT_DRIVE_INTEGRATION_GYRO_SIGN=-1.0f`。后续更换 IMU 安装方向时应分别验证。

维护风险和后续重构边界见 [`../todo.md`](../todo.md#直行控制与-jy61p-数据链维护债务)。app
任务不得复制 A/B 解算、阶段切换或混控算法；这些控制策略继续由 `straight_drive` 单点拥有。

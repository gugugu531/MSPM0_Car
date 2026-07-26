# app/app_straight_task 直行测试适配说明

任务层负责四个菜单入口、JY61P I2C 轮询调度、按键映射、OLED 显示和 Debug_Ex 遥测。
模式状态、指令限幅、PID、IMU 新鲜度判断、巡航角锁定和底盘输出统一由
`middleware/straight_drive` 实现，详见 `middleware_straight_drive.md`。

## 入口与操作

`Main Menu -> Straight Test` 下有四个任务：

| 菜单项 | 控制方式 | 指令单位 |
|---|---|---|
| `Duty Open` | 左右轮相同占空比，无反馈 | `%` |
| `Speed Closed` | 左右轮独立速度 PID | `m/s` |
| `Duty+Gyro Rate` | 基础占空比 + `gz -> 0` 角速度 PID | `%` |
| `Duty+Yaw Hold` | 基础占空比 + 锁定 yaw 巡航角 PID | `%` |

选中具体任务后会直接运动（仅浏览 `Straight Test` 子菜单不会驱动电机）：

- 开环和两种陀螺模式以 `50%` 基础占空比启动。
- 速度闭环以 `0.65 m/s` 目标启动；根据现有 `spd.csv` 整定数据，该速度的稳态占空比
  约为 `45.5%–51.5%`，用作 50% 基础速度的闭环等价初值。

- `UP` / `DOWN`：增加/减少指令。
- `ENTER`：指令归零。
- `BACK`：退出任务，由 app 框架统一主动刹车。

OLED 显示指令、实际左右占空比与编码器速度；陀螺模式额外显示 `gz`、
`yaw`、巡航角或差速修正量。

## Debug_Ex 遥测

四种任务都在每个 20ms 控制拍通过 `Debug_Ex/UART1`（115200 8N1）非阻塞发送一行：

```text
[STR] t=<ms> m=<0..3> imu=<0|1> cmd=<value> dl=<%> dr=<%> vl=<m/s> vr=<m/s> xl=<m> xr=<m> yaw=<deg> gz=<deg/s> corr=<%>
```

- `m`：`0=Duty Open`，`1=Speed Closed`，`2=Duty+Gyro Rate`，
  `3=Duty+Yaw Hold`。
- `imu`：姿态数据有效标志。四种模式都采集 JY61P 用于对比，但仅两个陀螺模式
  会把 IMU 用于控制。
- `cmd`：当前指令；速度模式单位为 `m/s`，其余模式单位为 `%`。
- `dl/dr`：左/右轮已应用占空比。速度闭环中是上一控制拍的 PID 输出。
- `vl/vr`：左/右轮线速度。
- `xl/xr`：左/右轮累计距离。
- `yaw/gz`：分别经 `STRAIGHT_DRIVE_HEADING_YAW_SIGN` 和
  `STRAIGHT_DRIVE_RATE_GYRO_SIGN`
  校正的航向角与 z 轴角速度。
- `corr`：两种陀螺闭环的差速修正；其他模式为 `0`。

上位机工具：

```powershell
python tools/straight_test_viz.py --list
python tools/straight_test_viz.py --port COM7
python tools/straight_test_viz.py --port COM7 --csv straight.csv --log straight_raw.txt
```

依赖为 `pyserial` 和 `matplotlib`。窗口实时显示左右轮占空比、速度、距离及
yaw/gz；空格暂停，`c` 清空，`q` 退出。

## 闭环结构

### 角速度闭环

```text
target_gz = 0
correction = PID(target_gz, corrected_gz)
left  = base_duty + correction
right = base_duty - correction
```

### 巡航角闭环

进入任务或指令从 `0` 变为非零时，在真正下发第一拍非零占空比前，
把当前 yaw 锁定为本次运动的基准角。运动期间即使 IMU 短暂失效，该基准角也不会改变；
只有 `ENTER` 归零后再次启动，或重新进入任务时才重新采集。角度误差通过
`Kinematics_AngleDiffDeg()` 限定到 `[-180, 180)`，避免跨越 `-180/180` 时绕远旋转。

```text
error = shortest_angle(target_yaw - current_yaw)
correction = PID(error, 0)
left  = base_duty + correction
right = base_duty - correction
```

## 安全与整定

- 四种模式进入后都启动 JY61P 非阻塞 I2C 轮询供遥测对比。两个陀螺控制模式在可以
  确定至少一轮采样成功前，即使已有非零指令，左右轮仍保持零占空比。
- 运行中若 IMU 完整样本超过 `STRAIGHT_DRIVE_IMU_MAX_AGE_MS` 未更新，则把输出置零并清除
  PID 动态状态，但保留开始运动时锁定的基准角；采样恢复后仍围绕原基准角继续闭环。
- 占空比基础指令限制为 `±80%`，修正后的单轮输出限制为 `±100%`。
- 角速度与巡航角 PID 默认为 P 控制，完整 `Kp/Ki/Kd`、积分和输出限幅宏集中在
  `middleware/straight_drive/straight_drive.h`，须根据实车整定。
- yaw 和 gz 使用独立符号宏。实车观察到原始 yaw 会使巡航角环先掉头约
  `180°`，因此 `STRAIGHT_DRIVE_HEADING_YAW_SIGN=-1.0f`；但 gz 同步反相后角速度模式会持续
  转圈，因此 `STRAIGHT_DRIVE_RATE_GYRO_SIGN=1.0f`。后续更换 IMU 安装方向时应分别验证两个宏。

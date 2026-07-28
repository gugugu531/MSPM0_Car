# middleware/turn_drive 接口说明

`turn_drive` 实现两种前进转向实验：两者均直行 2 m、无刹车左转 90°、再直行 1 m，并均按 IMU 新样本实际间隔积分和 A/B 修正参考角。80% 版左轮在 250 ms 内从 80% 降至 -80%；满速版在 300 ms 内从 100% 降至 -100%。两种版本随后都调用 TB6612FNG 主动制动左轮，右轮维持对应基础占空比。

## 接口

- `TurnDrive_Init(mode)`：按 80% 或满速模式复位 PID、阶段状态与编码器里程。
- `TurnDrive_Update(dt_s)`：消费 JY61P 缓存，执行一次状态推进与底盘输出。
- `TurnDrive_GetOutput()`：返回 OLED 与 UART 遥测快照。
- `TurnDrive_IsComplete()`：最后 1 m 完成后返回 `true`。

## 参数

`turn_drive.h` 集中定义两种模式的基础占空比、左轮减速时长和终点占空比，以及直行距离、左转角、IMU 最大时延与航向 PID。左转先线性降低左轮占空比，随后固定左轮 `0%`、右轮保持对应基础占空比，并调用 `TB6612FNG_Brake(LEFT)` 抱死左轮。满速模式使用 `YawEstimator`、JY61P 一致快照和速度优先混控，与 `100 Int->Yaw` 对齐。

`TURN_DRIVE_LEFT_YAW_DELTA_DEG` 设为 `-90°`，与当前逻辑 yaw 的正方向和物理左转方向相反的安装关系匹配。融合 yaw 与积分 gz 分别使用 `TURN_DRIVE_YAW_SIGN=-1.0f` 和 `TURN_DRIVE_INTEGRATION_GYRO_SIGN=-1.0f`，保证 A/B 同向；该积分符号不参与直行模块的 `gz -> 0` 角速度闭环。JY61P 无完整新鲜样本时，启动前保持零输出；运行中丢失则任务进入可恢复故障，框架主动刹车。

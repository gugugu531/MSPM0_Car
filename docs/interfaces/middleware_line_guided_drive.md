# middleware/line_guided_drive 接口说明

## 模块职责

`line_guided_drive` 是独立的 80% Yahboom 线引导直行实验，不替换现有 `line_follow`。它组合：

- `line_follow` 对 Yahboom 归一化黑线掩码的纯质心观测；
- 启动 500 ms 的 JY61P yaw 修正标定；
- 直接灰度 PID 与航向保持 PID；
- chassis 占空比输出。

控制状态：

```text
WAIT_IMU → HEADING_HOLD ⇄ LINE_PID
```

## 控制规则

1. 首个有效 IMU 样本到达后立即以 80% 基础占空比运动。起步后前 500 ms 按 IMU 样本时间戳积分 `gz`，以积分值与 JY61P yaw 的角度差更新 yaw 修正量；500 ms 后该修正量冻结。积分 `gz` 与融合 yaw 分别由 `LINE_GUIDED_INTEGRATION_GYRO_SIGN`、`LINE_GUIDED_HEADING_YAW_SIGN` 转到同一航向坐标系，当前安装方向下二者均为 `-1.0f`。
2. Yahboom 的 `bit0..7` 对应 `X1..X8`，置 1 表示黑线；中间两路 X4/X5 为位 3/4。
3. 外侧六路掩码 `0xE7` 中任一路检测到黑线时，立即进入 `LINE_PID`。灰度质心误差经 EMA 后直接送入独立 PID，其输出直接作为左右轮占空比差速修正量；航向 PID 不参与该模式。
4. 外侧六路均未检测到黑线时进入 `HEADING_HOLD`，并仅在切换瞬间捕获当前修正后 yaw 作为参考角；航向 PID 保持该角度。再次检测到外侧黑线后立即切回 `LINE_PID`。
5. IMU 或 Yahboom 读数未建立、超过 60 ms 未更新时，两轮立即归零并重新等待完整启动；阻塞驱动不会进入控制状态机。

## I2C0 分时

Yahboom 使用阻塞接口，JY61P 使用中断状态机。`app_line_task` 每拍先检查 `JY61P_I2C_IsIdle()`；只有状态机空闲时才挂起 JY61P、读取一次 Yahboom、恢复 JY61P，然后轮询下一轮 IMU。控制器消费最近一帧完整 IMU 快照，不会在进行中的 JY61P 事务上抢占总线。

## 隔离边界

阶段切换、500 ms yaw 修正、直接灰度 PID 与航向参考角捕获只存在于本模块。`line_follow` 仅提供 `LineFollow_ObserveDetectedMask()` 纯观测，`yaw_estimator` 仅提供纯数学积分，`chassis` 不感知寻迹状态。因此该实验不会改变现有基础循迹、直行测试或其他设备任务的控制逻辑。

菜单入口为 `Main Menu -> Line Guided 80`。OLED 显示 Yahboom 黑线位图（1=黑线）、误差、修正后 yaw/参考角、修正量、左右占空比和当前状态；任务持续运行直到按 BACK 退出。

# middleware/line_guided_drive 接口说明

## 模块职责

`line_guided_drive` 是独立的 80% Yahboom 线引导直行实验，不替换现有 `line_follow`。它组合：

- `line_follow` 对 Yahboom 归一化黑线掩码的纯质心观测；
- `yaw_estimator` 的 A/B 偏移估计；
- JY61P 一致快照；
- 起步角速度 PID 与巡航航向 PID；
- chassis 占空比输出。

实验状态机：

```text
WAIT_IMU → STARTUP_RATE → HEADING_HOLD ⇄ LINE_OUTER
```

## 控制规则

1. 首个有效 IMU 样本到达后立即以 80% 基础占空比运动，前 1000 ms 使用 `gz→0` 角速度
   闭环，同时从 `A0=B0` 开始按 IMU 样本时间戳积分 A。
2. 启动结束时冻结融合偏移 `startup_offset=B1-A1`。
3. Yahboom 的 `bit0..7` 对应 `X1..X8`，置 1 表示黑线；中间两路 X4/X5 为位 3/4。
   外侧六路掩码 `0xE7` 中任一路检测到黑线时进入
   `LINE_OUTER`；只有中间命中、或暂时没有通道命中时使用 `HEADING_HOLD`。
4. `LINE_OUTER` 中，灰度质心误差经过 EMA 后转换成相对当前航向的目标偏置：

   ```text
   target_yaw = current_yaw + clamp(line_kp * line_error, ±30°)
   ```

   航向 PID 内环再把目标角误差转换为左右轮差速。
5. 每次从启动角速度阶段或灰度外环进入 `HEADING_HOLD`，都捕获当时融合角 `Bentry`，参考角为：

   ```text
   reference = normalize(Bentry + startup_offset)
   startup_offset = shortest_angle(B1 - A1)
   ```

6. IMU 或 Yahboom 读数未建立/超过 60 ms 未更新时，两轮立即归零并重新等待完整启动，
   避免停车等待时间被误计入 1 s 启动阶段；阻塞驱动不会进入控制状态机。

## I2C0 分时

Yahboom 使用阻塞接口，JY61P 使用中断状态机。`app_line_task` 每拍先检查
`JY61P_I2C_IsIdle()`；只有状态机和控制器都空闲时才挂起 JY61P、读取一次 Yahboom、恢复
JY61P，然后 kick 下一轮 IMU。控制器本拍消费上一帧完整 IMU 快照，因此不会在进行中的
JY61P 事务上强制抢占总线。

## 隔离边界

阶段切换、外侧通道触发、80% 指令和参考角策略只存在于本模块。`line_follow` 仅提供
`LineFollow_ObserveDetectedMask()` 纯观测，`yaw_estimator` 仅提供纯数学积分，`chassis` 不感知寻迹状态。
因此该实验不会改变现有基础循迹、直行测试或其他设备任务的控制逻辑。

菜单入口为 `Main Menu -> Line Guided 80`，OLED 显示 Yahboom 黑线位图（1=黑线）、误差、yaw/ref、修正量、
左右占空比和当前阶段；任务持续运行直到按 BACK 退出。

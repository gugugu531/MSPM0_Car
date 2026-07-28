# middleware/line_guided_drive 接口说明

## 模块职责

`line_guided_drive` 是独立的 80% Yahboom 线引导直行实验，不替换现有 `line_follow`。它使用 Yahboom 黑线质心观测、直接灰度 PID、JY61P 航向保持 PID 和 chassis 占空比输出。

控制状态：

```text
WAIT_IMU → HEADING_HOLD ⇄ LINE_PID
```

## 控制规则

1. 控制器直接使用 JY61P 的当前 yaw；不执行陀螺仪积分，也不应用启动阶段 yaw 修正。
2. Yahboom 的 `bit0..7` 对应 `X1..X8`，置 1 表示黑线；中间两路 X4/X5 为位 3/4。
3. 外侧六路掩码 `0xE7` 中任一路检测到黑线时，立即进入 `LINE_PID`。灰度质心误差经 EMA 后直接送入独立 PID，其输出直接作为左右轮占空比差速修正量；航向 PID 不参与该模式。
4. 外侧六路均未检测到黑线、但仍有中间黑线时进入 `HEADING_HOLD`，并仅在切换瞬间捕获当前 yaw 作为参考角；航向 PID 保持该角度。
5. 全部八路均未检测到黑线时，控制器立即将两轮指令置零并报告正常结束；app 框架刹车并退出 `Line Guided 80` 返回菜单。
6. IMU 或 Yahboom 数据未建立、超过 60 ms 未更新时，两轮立即归零；这属于通信异常，不会被当作正常丢线退出。

## I2C0 分时

Yahboom 使用阻塞接口，JY61P 使用中断状态机。`app_line_task` 只在 `JY61P_I2C_IsIdle()` 时挂起 JY61P、读取一次 Yahboom、恢复 JY61P，然后轮询下一轮 IMU。控制器消费最近一帧完整 IMU 快照，不会抢占进行中的 JY61P 事务。

## 隔离边界

阶段切换、直接灰度 PID、航向参考角捕获和丢线退出策略只存在于本模块。`line_follow` 仅提供 `LineFollow_ObserveDetectedMask()` 纯观测，`chassis` 不感知循迹状态。

菜单入口为 `Main Menu -> Line Guided 80`。OLED 显示 Yahboom 黑线位图（1=黑线）、误差、yaw/参考角、修正量、左右占空比和当前状态；任务持续运行直到丢线或按 BACK 退出。

# Todo

> 二维云台/瞄准子系统与原 E 题任务框架已移除。当前 app 已重建为菜单驱动的协作式调度
> 框架，待办集中在底盘闭环整定、外设上板验证和构建工程一致性。

- 重新导入并验证 CCS/ticlang 构建；projectspec 的 `G3507.syscfg`、debug UART include 和
  仓库内 SDK 路径已经同步，但尚未用 CCS 工具链完成回归构建。
- 后续根据真实系统时钟复核 `BSP_TIME_CPUCLK_HZ`，必要时改为从时钟配置自动派生。
- 后续如增加第二个实体按键，在 `KEY_ID`、硬件映射宏和 `bsp/key` 内部配置表中同步扩展。
- 后续构建具体 BSP 外设时，继续按 SDK 风格统一公开类型和枚举成员命名。
- 如需要更彻底的 API 统一，后续可把公开 C 函数也改为 snake_case。
- 上板验证菜单/按键/SysTick、双轮编码器方向与比例、速度闭环、GPIO 灰度、感为 I2C 灰度、
  Yahboom I2C 循线模块、I2C0 四设备分时、蓝牙 UART0 和调试 UART1 遥测链路。
- Yahboom 专项上板检查：确认 400kHz 下地址 `0x12` 应答稳定；核对 X1=bit7、X8=bit0 和
  黑线低有效极性；等待上电预热至少 20 秒后验证黑/白场校准及断电保存。
- MPU6050 专项上板检查：确认地址 `0x68` 的 WHO_AM_I 可稳定读取；静止平放时合加速度接近
  `9.81 m/s²`、三轴角速度接近零，翻转板卡核对加速度轴向及 pitch/roll 符号；分别验证
  Device Check 基础模式和 DMP bring-up，并检查占用 I2C0 时 JY61P 的挂起/恢复。

## 速度闭环待查漏洞（未验证推测）

> 以下为对 `middleware/chassis` 速度闭环 + `bsp/motor/hall_encoder` 测速链路的**分析推测，
> 均尚未上板验证**，按怀疑程度排序。采样周期 100ms/10ms 不一致已修复（commit `ee6a779`）；
> 下列为其余待查项，动手前建议先以遥测/实验证实。

1. **编码器脉冲「读-清」竞态（未验证）**：GPIO 编码器 A 相中断优先级 `0` 可抢占采样定时器
   中断优先级 `3`；`HallEncoder_UpdateSample` 里 `raw = pending_count; pending_count = 0;`
   非原子，抢占若落在两句之间会丢 1 个脉冲。频率低（故距离仍准），**推测**给速度添噪、里程极缓漂移。
   根治设想：GPIO ISR 只维护**只增累计计数**，采样时取快照算增量、不清零（32 位读在 M0+ 原子）。
2. **抗积分饱和限幅错配（未验证）**：`CHASSIS_SPEED_INTEGRAL_LIMIT` 宜 ≈ `output_limit/ki`，
   否则输出饱和后积分过度累积、恢复时超调。改 `ki` 时需同步调整。**推测**。
3. **反馈量化 + 无低通（未验证）**：20ms 窗低速时脉冲少（分辨率 ~0.03 m/s/脉冲），实测速度未过
   低通，`kp` 直接放大量化跳变成占空比抖。设想加 `core/filter` 的 `Filter_LowpassEma`，或 A 相
   双边沿 / AB 四倍频解码提分辨率。**推测**。
4. **控制 dt 固定 vs 调度抖动；采样与控制不同步（未验证）**：`App_ControlTick` 用**固定** `dt=0.02`，
   协作调度实测会抖到 31ms；编码器**硬件** 20ms 采样与控制**软件** 20ms 拍未相位锁定 → 拍频漂移，
   偶尔重复读同一采样或跳过。设想给 PID 传实际经过 dt / 由采样事件驱动控制。**推测**。
5. **过零反转冲击（未验证）**：PID 输出过零转反向时 TB6612 硬切方向，近停转「读 B 相定向」最不可靠，
   易在零点抖动/误判向。设想加输出斜率限制或零点小死区。**推测**。
6. **死区 / 无前馈（局限，非 bug）**：电机有起转死区，纯 PI 靠积分慢慢爬过 → 起步慢。设想用
   `Duty Sweep` 测的「目标→稳态占空比」曲线做前馈 `duty≈kff·target`，PI 只补小误差。

## 延后的性能优化

### OLED 刷新 DMA 化 (Step 2, 已延后)

现状: `bsp/oled` 已完成帧缓冲重构 (commit `31baeec`, Step 1)——绘制只写 1KB RAM,
`OLED_Flush()` 用水平寻址**一次事务**发整帧 `[0x40][1024]`, 已消除旧版分包低效。
但该 Flush 目前仍是**阻塞**传输 (I2C1@400kHz, 整帧 ~26ms), 会占用控制环。

背景评估: MSPM0G3507 主频仅 **32MHz**(max 80MHz, 有余量), DMA **完全未用**(通道全空),
SRAM 32KB(仅用 ~8.5KB)。OLED 传输是 I2C 带宽瓶颈, 非 CPU 瓶颈; DMA 不缩短传输时间,
但能把那 ~26ms 从"CPU 忙等"变成"CPU 空闲"。

Step 2 待办 (让 Flush 全异步、CPU 近零):
1. **SysConfig**: 新增 DMA 模块 + 1 个通道, 配 I2C1(OLED) 的 TX DMA 触发事件; 给
   I2C1 加 STOP 中断(intController); 用 SysConfig CLI 重生成
   `ti_msp_dl_config` (仅确认新增 DMA/中断配置, 勿污染其它)。
2. **oled.c**: OLED_Flush 改为——设窗口(阻塞小命令) → 配 DMA(src=oled_fb, dst=I2C1 TX
   FIFO, 1025 字节, 8bit) → 启动 I2C 传输 → 立即返回; 加 busy 标志防重入(DMA 未完时新
   Flush 等待或跳过, 渲染周期 400ms 远大于 ~26ms 传输, 实际不重叠)。
3. **新增 `I2C1_IRQHandler`** (BSP 自持, 见 architecture.md 中断策略): STOP 中断清 busy。

可选进一步: I2C1 升 Fast Mode Plus (1MHz) 把整帧传输 ~26ms 降到 ~10ms (需 SSD1306
与走线支持); 属独立小改, 与 DMA 正交。

建议时机: 集成阶段(与 MPU6050/JY61P 的 I2C 统筹)一并做, 或控制环对 OLED 阻塞敏感时再做。

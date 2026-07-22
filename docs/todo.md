# Todo

> 二维云台/瞄准子系统与原 E 题任务框架已移除，新任务待定。原与云台/瞄准/E 题标定相关的
> 待办及历史文档已随之删除。

- **清理 `bsp/canmv` 死代码**：视觉 UART 的 `@deprecated` 旧 API（`g_canmv_uart_*` 裸全局
  与手动解码路径）在原消费者（app_e_task/gimbal_tracking/auto_aim/app_device_check）删除后
  已无调用点，可连同 `CANMV_TARGET_LASER/RECT` 坐标帧死链一并评估退役（保留 `CanMvUart_GetAngle`
  等干净 getter）。
- 后续根据真实系统时钟复核 `BSP_TIME_CPUCLK_HZ`，必要时改为从时钟配置自动派生。
- 后续如增加第二个实体按键，在 `KEY_ID`、硬件映射宏和 `bsp/key` 内部配置表中同步扩展。
- 后续构建具体 BSP 外设时，继续按 SDK 风格统一公开类型和枚举成员命名。
- 如需要更彻底的 API 统一，后续可把公开 C 函数也改为 snake_case。
- 重建 app 后，上板验证 UART0 IMU、UART2 视觉、SysTick 调度等基础链路。

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
   I2C1 加 STOP 中断(intController); 用 SysConfig CLI (`C:/ti/sysconfig_1.26.2`) 重生成
   `ti_msp_dl_config` (仅确认新增 DMA/中断配置, 勿污染其它)。
2. **oled.c**: OLED_Flush 改为——设窗口(阻塞小命令) → 配 DMA(src=oled_fb, dst=I2C1 TX
   FIFO, 1025 字节, 8bit) → 启动 I2C 传输 → 立即返回; 加 busy 标志防重入(DMA 未完时新
   Flush 等待或跳过, 渲染周期 400ms 远大于 ~26ms 传输, 实际不重叠)。
3. **新增 `I2C1_IRQHandler`** (BSP 自持, 见 architecture.md 中断策略): STOP 中断清 busy。

可选进一步: I2C1 升 Fast Mode Plus (1MHz) 把整帧传输 ~26ms 降到 ~10ms (需 SSD1306
与走线支持); 属独立小改, 与 DMA 正交。

建议时机: 集成阶段(与 MPU6050/JY61P 的 I2C 统筹)一并做, 或控制环对 OLED 阻塞敏感时再做。

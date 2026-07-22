# 分层架构

## 分层规则

工程控制路径采用单向依赖结构：

```text
app ─► middleware ─► bsp
          │
          └────────► core（纯计算）
```

`core` 不读取硬件、不调用 middleware；`app` 只编排任务，不重复实现算法。

> 二维云台/瞄准子系统（gimbal / auto_aim / aim_solver / aim_fusion / localization /
> bldc / step_motor）已整体移除；原任务框架的 app 已清空为极简启动骨架。下述职责与依赖
> 边界规则不变，模块清单反映移除后的当前状态。

## 模块职责

- `app`：固件入口。当前仅极简启动骨架（SysConfig 初始化后空循环），待按新需求重建。
- `core`：PID、运动学等纯计算能力。
- `middleware`：组合 core 与 BSP，包括底盘、巡线（line_follow/line_tracking）、UI 和故障处理。
- `bsp`：直接面向板级外设的驱动，包括直流电机(TB6612)、霍尔编码器、OLED、按键、JY61P IMU、MPU6050 和灰度巡线传感器。

## 支撑目录

- `board`：SysConfig 生成文件、输入文件和启动/链接资源。生成文件内容不手写业务逻辑。
- `project`：CCS 和 Keil 工程元数据。
- `tools`：调试、烧录和诊断脚本。
- `docs`：架构、接口、变更记录、待办和构建说明。

## 依赖边界

- `bsp` 只能包含标准头、厂商/板级头和 `bsp` 内部头文件。
- `middleware` 可以包含 `middleware`、`core` 和 `bsp` 头文件。
- `core` 只能包含标准头和 `core` 内部头文件。
- `app` 可以包含任意下层公开头文件。
- 跨模块状态应优先由明确拥有该状态的模块维护，并通过公开接口读取；不要重新引入裸全局状态兼容头。

## 中断分发策略

中断入口按"谁拥有该外设/职责，谁定义 ISR"划分，`middleware` 不做中断转发：

- **BSP 驱动自持其专属外设中断**：
  - `bsp/motor/hall_encoder.c` → `GROUP1_IRQHandler`（编码器 GPIO）、`TIMER_0_INST_IRQHandler`（编码器采样定时器）
  - `bsp/imu/wit_sdk.c` → `I2C0_IRQHandler`（JY61P I2C 中断驱动状态机）
- **`app` 持有需跨子系统分发或属应用调度的中断**：当前 app 为极简骨架，仅提供空
  `SysTick_Handler`（占位以避免 startup weak 死循环）。重建 app 时在此接入 UART 命令路由
  （蓝牙/调试上位机等）与分频调度（按键扫描、传感器轮询、系统计时）。

> 新增外设时遵循同一原则：仅该驱动使用的中断放进对应 BSP 源文件；需要唤醒多个上层子系统或承担应用级调度的中断放进 `app`。不要在 `middleware` 里写"转发到下层驱动"的空壳 ISR 入口。

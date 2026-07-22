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
> bldc / step_motor）已整体移除。app 层已重建为「菜单选择 → 执行任务 → 退回菜单」的
> 协作式调度框架（见 `docs/app-design.md`）。下述职责与依赖边界规则不变。

## 模块职责

- `app`：固件入口与应用框架——上电初始化(`app_init`)、时间触发调度器(`app_scheduler`)、
  顶层状态机 INIT/MENU/RUN/FAULT(`app_mode`)、任务注册表(`app_tasks`)。
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
- **`app` 持有需跨子系统分发或属应用调度的中断**：
  - `app/app_scheduler.c` → `SysTick_Handler`（1ms：`BSP_Time_TickInc` 时基递增 + `Key_Scan`
    按键消抖）。`tick_active` 门控确保初始化完成前不误触发。调度器时基即取自此。
  - 后续接入 UART 命令路由（蓝牙/调试上位机）时，同样将其 `UARTx_IRQHandler` 放在 app 层。

> 新增外设时遵循同一原则：仅该驱动使用的中断放进对应 BSP 源文件；需要唤醒多个上层子系统或承担应用级调度的中断放进 `app`。不要在 `middleware` 里写"转发到下层驱动"的空壳 ISR 入口。

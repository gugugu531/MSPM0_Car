# 接口说明

> 二维云台/瞄准子系统（gimbal / gimbal_tracking / auto_aim / aim_solver / aim_fusion /
> localization / bldc / step_motor）已整体移除，原任务框架的 app 亦已清空。下表仅列出当前
> 仍存在的模块。

## app

- `main.c`：极简启动骨架，`SYSCFG_DL_init()` 后 `__enable_irq()` 进入空循环，并提供空 `SysTick_Handler`。

## core

- `pid/pid.h`：位置式和增量式 PID 控制器，详细说明见 `docs/interfaces/core_pid.md`。
- `common/core_types.h`：core 层基础二维点类型，详细说明见 `docs/interfaces/core_common.md`。
- `kinematics/kinematics.h`：角度、位姿、差速混控和二维几何计算，详细说明见 `docs/interfaces/core_kinematics.md`。

## middleware

- `chassis.h`：底盘组合服务，详细说明见 `docs/interfaces/middleware_chassis.md`。
- `line_follow.h`：巡线运行状态服务，详细说明见 `docs/interfaces/middleware_line_follow.md`。
- `line_tracking/line_tracking.h`：巡线偏差计算、PID 修正和底盘输出接口，详细说明见 `docs/interfaces/middleware_line_tracking.md`。
- `ui.h`：轻量 OLED UI 渲染层，详细说明见 `docs/interfaces/middleware_ui.md`。
- `system_fault.h`：系统故障处理服务，详细说明见 `docs/interfaces/middleware_system_fault.md`。

## bsp

- `common/bsp_common.h`：BSP 公共状态，详细说明见 `docs/interfaces/bsp_common.md`。
- `time/bsp_time.h`：BSP 系统时间和阻塞延时服务，详细说明见 `docs/interfaces/bsp_time.md`。
- `key.h`：按键读取、消抖和事件生成，详细说明见 `docs/interfaces/bsp_key.md`。
- `oled.h`：SSD1306 OLED 帧缓冲显示接口（I2C1），详细说明见 `docs/interfaces/bsp_oled.md`。
- `wit_sdk.h`：JY61P（WIT 协议）I2C0 中断驱动、数据解析和结构体读取接口，详细说明见 `docs/interfaces/bsp_wit_sdk.md`。
- `mpu6050.h`：MPU6050 DMP 姿态驱动（I2C0，与 JY61P 共总线）。
- `tb6612fng.h`：TB6612FNG 双路直流电机驱动芯片接口，详细说明见 `docs/interfaces/bsp_tb6612fng.md`。
- `hall_encoder.h`：霍尔编码器计数、速度和距离估计接口，详细说明见 `docs/interfaces/bsp_hall_encoder.md`。
- `grayscale_sensor.h`：8 路光敏灰度传感器数字量接口，详细说明见 `docs/interfaces/bsp_grayscale_sensor.md`。
- `debug_uart.h`：调试 UART 收发接口。

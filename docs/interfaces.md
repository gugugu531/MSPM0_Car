# 接口说明

## app

- `main.c`：完成 SysConfig、IRQ、BSP 设备和中间层 provider 初始化，然后进入 `App_Launch()`。
- `app_launcher.h`：提供 `App_Launch()` 和 `App_DebugUartHandler()`。
- `menu.h`、`mode.h`：提供菜单与题目流程入口。

## core

- `pid.h`：PID 控制器初始化、更新、计算和复位。
- `kinematics.h`：坐标、运动学和里程相关计算。
- `tracking.h`：巡线控制入口。
- `sensor_proc.h`：目标点提取、纸面坐标和相机坐标转换。
- `step_motor_ctrl.h`：基于视觉目标的云台控制逻辑。

## middleware

- `chassis.h`：底盘组合服务，详细说明见 `docs/interfaces/middleware_chassis.md`。
- `gimbal.h`：云台组合服务，详细说明见 `docs/interfaces/middleware_gimbal.md`。
- `line_follow.h`：巡线运行状态服务，详细说明见 `docs/interfaces/middleware_line_follow.md`。
- `ui.h`：轻量 OLED UI 渲染层，详细说明见 `docs/interfaces/middleware_ui.md`。
- `system_error_state.h`：声明错误显示所需的 `error_message`。
- `vision_state.h`：当前仍是视觉状态兼容层，后续需迁移到 `bsp/canmv/canmv_uart.h`。
- `error_handler.h`：提供错误处理入口，当前行为为调用 `Chassis_Brake()` 并在 OLED 上显示错误信息。

## bsp

- `common/bsp_common.h`：BSP 公共状态和基础数据结构，详细说明见 `docs/interfaces/bsp_common.md`。
- `time/bsp_time.h`：BSP 系统时间和阻塞延时服务，详细说明见 `docs/interfaces/bsp_time.md`。
- `key.h`：按键读取、消抖和事件生成，详细说明见 `docs/interfaces/bsp_key.md`。
- `oled.h`：SSD1306 OLED 软件 I2C 显示接口，详细说明见 `docs/interfaces/bsp_oled.md`。
- `step_motor.h`：步进电机开环速度、阻塞定时运行和估计位置接口，详细说明见 `docs/interfaces/bsp_step_motor.md`。
- `canmv_uart.h`：CanMV UART 收发、帧解析和目标状态接口，详细说明见 `docs/interfaces/bsp_canmv_uart.md`。
- `wit_sdk.h`：WitMotion IMU 厂家驱动、JY61P 数据解析和结构体读取接口，详细说明见 `docs/interfaces/bsp_wit_sdk.md`。
- `tb6612fng.h`：TB6612FNG 双路直流电机驱动芯片接口，详细说明见 `docs/interfaces/bsp_tb6612fng.md`。
- `hall_encoder.h`：霍尔编码器计数、速度和距离估计接口，详细说明见 `docs/interfaces/bsp_hall_encoder.md`。
- `grayscale_sensor.h`：8 路光敏灰度传感器数字量接口，详细说明见 `docs/interfaces/bsp_grayscale_sensor.md`。

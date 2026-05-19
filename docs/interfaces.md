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

- `system_time.h`：声明 `tick` 和 `System_GetTickMs()`。
- `system_error_state.h`：声明错误显示所需的 `error_message`。
- `tracking_runtime.h`：声明巡线和任务运行时共享状态，包括 `Digital`、`sInedge`、`edge`、`turning`。
- `vision_state.h`：兼容层，转发 `bsp/laser/laser_usart.h` 导出的视觉状态。
- `delay.h`：提供毫秒和微秒延时。
- `motor_system.h`：提供左右底盘电机组合初始化、原始占空比控制和带符号占空比控制。
- `step_motor_system.h`：提供 yaw/pitch 双轴步进电机组合初始化、速度控制、状态更新和位置查询。
- `error_handler.h`：提供错误处理入口，当前行为为刹车并在 OLED 上显示错误信息。

## bsp

- `key.h`：按键状态机。时间源通过 `Key_SetTimeProvider()` 注入。
- `oled.h`：SSD1306 OLED 驱动。延时函数通过 `OLED_SetDelayProvider()` 注入。
- `step_motor.h`：步进电机底层驱动。状态更新由调用方显式传入当前时间。
- `laser_usart.h`：CanMV UART 解析和激光点/矩形点状态。
- `wit_sdk.h`：IMU 协议解析和陀螺仪通道数据。
- `tb6612fng.h`、`hall_encoder.h`、`tracking_sensor.h`：底盘电机驱动、编码器和巡线传感器接口。


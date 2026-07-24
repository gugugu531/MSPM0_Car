# 接口说明

> 二维云台/瞄准子系统（gimbal / gimbal_tracking / auto_aim / aim_solver / aim_fusion /
> localization / bldc / step_motor）已整体移除，原任务框架的 app 亦已清空。下表仅列出当前
> 仍存在的模块。

## app

菜单驱动的协作式调度框架，详见 `docs/app-design.md`。

- `app_init.h`：`App_Init()` 集中式上电初始化并注册调度任务。
- `app_scheduler.h`：`Scheduler_Init/AddTask/Run/EnableTick` 时间触发调度器；自持 `SysTick_Handler`。
- `app_mode.h`：状态机 `APP_MODE`（INIT/MENU/RUN/FAULT）与 `App_Mode_Init/Get`、`App_ControlTick/UiTick`。
- `app_task.h`：任务生命周期契约——`APP_TASK_STATUS` 与 `APP_TASK_DESC`（`on_enter/on_tick/on_exit`），不含具体任务。
- `app_menu.h`：菜单树类型 `MENU_NODE`/`MENU_ITEM` 与 `Menu_Tick` 导航；`app_menu_def.c` 定义菜单树实例 `APP_ROOT_MENU`。
- `app_checks.h`：外设自检任务描述符 `APP_CHK_*`（JY61P / MPU6050 / Grayscale / Gray I2C / TB6612 / Encoder / Speed PID）。

## core

- `pid/pid.h`：位置式和增量式 PID 控制器，详细说明见 `docs/interfaces/core_pid.md`。
- `filter/filter.h`：一阶低通(EMA)与中心死区等通用信号调理算法，详细说明见 `docs/interfaces/core_filter.md`。
- `common/core_types.h`：core 层基础二维点类型，详细说明见 `docs/interfaces/core_common.md`。
- `kinematics/kinematics.h`：限幅、角度归一化和差速混控计算，详细说明见 `docs/interfaces/core_kinematics.md`。

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
- `grayscale_sensor.h`：8 路光敏灰度传感器数字量接口（GPIO），详细说明见 `docs/interfaces/bsp_grayscale_sensor.md`。
- `ganv_gray.h`：感为 8 路灰度传感器 I2C 驱动（I2C0，默认地址 `0x4F`），详细说明见 `docs/interfaces/bsp_ganv_gray.md`。
- `debug_uart.h`：调试 UART 收发接口。

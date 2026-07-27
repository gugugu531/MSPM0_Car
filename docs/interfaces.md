# 接口说明

> 二维云台/瞄准子系统（gimbal / gimbal_tracking / auto_aim / aim_solver / aim_fusion /
> localization / bldc / step_motor）已整体移除。`app` 已重建为菜单驱动的协作式调度框架；
> 下表仅列出当前仍存在并参与构建的模块。

## app

菜单驱动的协作式调度框架，详见 `docs/app-design.md`。

- `app_init.h`：`App_Init()` 集中式上电初始化并注册调度任务。
- `app_scheduler.h`：`Scheduler_Init/AddTask/Run/EnableTick` 时间触发调度器；自持 `SysTick_Handler`。
- `app_mode.h`：状态机 `APP_MODE`（INIT/MENU/RUN/FAULT）与 `App_Mode_Init/Get`、`App_ControlTick/UiTick`。
- `app_task.h`：任务生命周期契约——`APP_TASK_STATUS` 与 `APP_TASK_DESC`（`on_enter/on_tick/on_exit`），不含具体任务。
- `app_menu.h`：菜单树类型 `MENU_NODE`/`MENU_ITEM` 与 `Menu_Tick` 导航；`app_menu_def.c` 定义菜单树实例 `APP_ROOT_MENU`。
- `app_checks.h`：外设自检任务描述符 `APP_CHK_*`（JY61P / Yaw A/B / MPU6050 / Grayscale / Gray I2C / Yahboom I2C / TB6612 / Encoder / Speed PID / Duty Sweep）。
- `app_line_task.h`：两个 Yahboom 循迹任务及共享 I2C0 分时调度（挂根菜单）。
- `app_straight_task.h`：九种直行测试任务、3 m 自动停车与遥测配置，详见
  `docs/interfaces/app_straight_task.md`。
- `app_bt_task.h`：蓝牙串口收发测试任务 `APP_CHK_BLUETOOTH`（挂 Device Check）。

## core

- `pid/pid.h`：位置式和增量式 PID 控制器，详细说明见 `docs/interfaces/core_pid.md`。
- `filter/filter.h`：一阶低通(EMA)与中心死区等通用信号调理算法，详细说明见 `docs/interfaces/core_filter.md`。
- `common/core_types.h`：core 层基础二维点类型，详细说明见 `docs/interfaces/core_common.md`。
- `kinematics/kinematics.h`：限幅、角度归一化、差速混控与两轮差速运动学模型（正/逆运动学、转弯半径、位姿积分），详细说明见 `docs/interfaces/core_kinematics.md`。
- `yaw_estimator/yaw_estimator.h`：纯角速度积分航向 A 与融合角偏移 `B-A`，详细说明见
  `docs/interfaces/core_yaw_estimator.md`。

## middleware

- `chassis.h`：底盘组合服务（开环占空比 + 每轮速度闭环 PID），详细说明见 `docs/interfaces/middleware_chassis.md`。
- `line_follow.h`：标准化八路黑线观测、巡线偏差计算、PID/陀螺修正和底盘输出，详细说明见
  `docs/interfaces/middleware_line_follow.md`。
- `line_guided_drive.h`：80% 直接起步的启动角速度控制、Yahboom 外环与航向内环，详细说明见
  `docs/interfaces/middleware_line_guided_drive.md`。
- `straight_drive.h`：九种直行模式的指令、PID、启动阶段切换、姿态反馈和底盘输出，详细说明见
  `docs/interfaces/middleware_straight_drive.md`。
- `ui.h`：轻量 OLED UI 渲染层，详细说明见 `docs/interfaces/middleware_ui.md`。
- `system_fault.h`：系统故障处理服务，详细说明见 `docs/interfaces/middleware_system_fault.md`。

## bsp

- `common/bsp_common.h`：BSP 公共状态，详细说明见 `docs/interfaces/bsp_common.md`。
- `time/bsp_time.h`：BSP 系统时间和阻塞延时服务，详细说明见 `docs/interfaces/bsp_time.md`。
- `key.h`：按键读取、消抖和事件生成，详细说明见 `docs/interfaces/bsp_key.md`。
- `oled.h`：SSD1306 OLED 帧缓冲显示接口（I2C1），详细说明见 `docs/interfaces/bsp_oled.md`。
- `wit_sdk.h`：JY61P（WIT 协议）I2C0 中断驱动、数据解析和结构体读取接口，详细说明见 `docs/interfaces/bsp_wit_sdk.md`。
- `mpu6050.h`：MPU6050 基础六轴/温度/静态倾角与 DMP 姿态驱动（I2C0，共享总线），详细说明见
  `docs/interfaces/bsp_mpu6050.md`。
- `tb6612fng.h`：TB6612FNG 双路直流电机驱动芯片接口，详细说明见 `docs/interfaces/bsp_tb6612fng.md`。
- `hall_encoder.h`：左右双轮霍尔编码器计数、速度和距离估计接口（按 `HALL_ENCODER_ID` 选轮），详细说明见 `docs/interfaces/bsp_hall_encoder.md`。
- `grayscale_sensor.h`：8 路光敏灰度传感器数字量接口（GPIO），详细说明见 `docs/interfaces/bsp_grayscale_sensor.md`。
- `ganv_gray.h`：感为 8 路灰度传感器 I2C 驱动（I2C0，默认地址 `0x4F`），详细说明见 `docs/interfaces/bsp_ganv_gray.md`。
- `yahboom_track.h`：Yahboom 8 路循线模块 I2C 驱动（I2C0，地址 `0x12`），详细说明见 `docs/interfaces/bsp_yahboom_track.md`。
- `debug_uart.h`：调试串口（Debug_Ex/UART1，115200）**非阻塞发送**——环形缓冲 + TX 中断排空，供遥测输出，无接收。
- `bluetooth.h`：蓝牙串口（BlueTooth/UART0，9600 8N1）收发——RX 中断 + 环形缓冲，发送直接写 TX FIFO。

### 三种灰度接口的位序与极性

三种驱动保留各自硬件/协议语义，当前没有一个跨驱动统一的 `mask` ABI。调用方不得只根据
“灰度掩码”名称假设位序和极性：

| 接口 | 位序 | 置 `1` 的含义 |
|---|---|---|
| `GrayscaleSensor_ReadMask()` | `bit0=逻辑通道0` … `bit7=逻辑通道7` | 未检测到黑线（实机输入电平语义） |
| `GanvGray_ReadDigital()` | `bit0=第1路` … `bit7=第8路` | 该路检测到，设备端 LED 亮 |
| `YahboomTrack_ReadRaw()` | `bit7=X1` … `bit0=X8` | 白底，设备端 LED 灭 |
| `YahboomTrack_ReadDetectedMask()` | `bit0=X1` … `bit7=X8` | 检测到黑线 |

当前两个实际循迹任务均调用 `YahboomTrack_ReadDetectedMask()`，并把“黑线置 1”的归一化掩码
传入 middleware。`LineFollow_Update()` 仍保留为 GPIO 兼容入口，但菜单任务不再调用它。

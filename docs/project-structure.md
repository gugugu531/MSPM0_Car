# 工程结构

## 顶层结构

当前工程采用“分层源码目录 + IDE 工程目录 + 板级配置目录”的组织方式：

- `app/`：应用入口与框架（初始化/调度器/状态机/菜单树/任务）
- `core/`：控制算法和数据处理
- `middleware/`：系统组合能力与共享运行时状态
- `bsp/`：板级外设驱动
- `board/`：SysConfig、启动和链接资源
- `project/`：CCS 和 Keil 工程文件
- `docs/`：工程文档
- `tools/`：调试和烧录脚本

> 说明：二维云台/瞄准子系统（step_motor / bldc / gimbal / gimbal_tracking / auto_aim /
> aim_solver / aim_fusion / localization）已整体移除。app 层已重建为菜单驱动的协作式调度
> 框架（详见 `docs/app-design.md`）；下层 bsp/middleware/core 能力保留供任务复用。

## 运行路径

1. `main.c` 调 `App_Init()`：SysConfig → BSP → 中间件 → 框架初始化并注册调度任务。
2. `__enable_irq()` 后进入 `while(1) Scheduler_Run()` 超循环。
3. `SysTick_Handler`(1ms) 递增时基并扫描按键；调度器按周期分派 `App_ControlTick`(20ms) 与
   `App_UiTick`(50ms)。
4. 状态机 INIT→MENU→RUN→FAULT：MENU 委派 `app_menu` 的嵌套菜单导航，短按选中任务进入 RUN
   委派任务 `on_tick`，任务返回 DONE/中止/故障后退回 MENU/FAULT。

## app

`app` 是顶层应用层与框架，按职责拆分：

- `main.c`：入口，初始化后进入调度超循环。
- `app_init.c/.h`：集中式上电时序（Ui/Chassis 先于任何可能 fault 的步骤）。
- `app_scheduler.c/.h`：时间触发任务表 + `Scheduler_Run` 分派；自持 `SysTick_Handler`。
- `app_mode.c/.h`：顶层状态机 INIT/MENU/RUN/FAULT 与全部状态转移入口。
- `app_task.h`：任务生命周期契约（`APP_TASK_STATUS` + `APP_TASK_DESC` 三钩子），不含具体任务。
- `app_menu.c/.h` 与 `app_menu_def.c`：菜单树（`MENU_NODE`/`MENU_ITEM`）导航与菜单树实例定义。
- `app_checks.c/.h`：外设自检任务描述符，挂在 Device Check 子菜单。
- `app_line_task.c/.h`：循迹测试任务（完整巡线闭环 + 陀螺增稳），挂在根菜单。
- `app_straight_task.c/.h`：直行测试的任务生命周期、按键、OLED 与遥测适配，挂在
  `Straight Test` 子菜单；具体控制由 `middleware/straight_drive` 承担。
- `app_bt_task.c/.h`：蓝牙串口收发测试任务，挂在 Device Check 子菜单。
- `app_fmt.c/.h`：定点数字格式化（不引浮点 printf），供自检显示。

> 外设自检集中在 `app_checks`；功能性任务各占一个 `app_*_task.c/.h`，避免自检文件无限膨胀。

新增任务只需实现三个钩子（`on_enter/on_tick/on_exit`）并在 `app_menu_def.c` 的菜单树挂一项，
调度/进出清理自动接入。该层可直接调用下层公开接口，但不应把纯算法或底层驱动细节塞入应用流程。

## core

`core` 放置和硬件无关或弱硬件相关的控制逻辑：

- `common/`：core 层基础数据类型。
- `pid/`：位置式和增量式 PID 控制器。
- `filter/`：一阶低通（EMA）与中心死区等通用信号调理算法。
- `kinematics/`：角度、位姿、差速混控与两轮差速运动学模型（车体参数一律传参，不在层内固化）。

该层保持硬件无关，不包含 `app`、`middleware` 或 `bsp` 头文件。

## middleware

`middleware` 放置多个 BSP 外设组合后的系统能力：

- `chassis/`：底盘组合服务（开环占空比 + 每轮速度闭环 PID）
- `line_follow/`：GPIO 灰度读取、巡线偏差计算、PID/陀螺修正和底盘输出
- `straight_drive/`：四种直行模式的指令、姿态反馈、PID 与底盘输出
- `ui/`：轻量 OLED UI 渲染层
- `fault/`：系统故障处理服务

> 注：`line_follow` 因需调用 chassis、读取 BSP 灰度和 IMU 缓存，
> 本质是“组合 core 算法 + 硬件观测 + 驱动执行器”的中间件能力，置于 `middleware` 以消除
> `core ↔ middleware` 循环依赖、保持 `core` 纯计算。

## bsp

`bsp` 是最低层，直接面对板级外设：

- `common/`
- `imu/`（JY61P，WIT 协议，I2C0 中断驱动）
- `mpu6050/`（MPU6050 DMP 姿态，I2C0，与 JY61P 共总线）
- `key/`
- `motor/`（TB6612 直流电机 + 左右双轮霍尔编码器）
- `oled/`（SSD1306，帧缓冲模型）
- `grayscale_sensor/`（数字量 GPIO 版 8 路灰度）
- `ganv_gray/`（感为 8 路灰度，I2C0，与 MPU6050/JY61P 共总线）
- `debug_uart/`（Debug_Ex/UART1，非阻塞遥测输出）
- `bluetooth/`（BlueTooth/UART0，9600 8N1 收发）
- `time/`

该层禁止包含 `app`、`core`、`middleware` 头文件。需要基础时间或阻塞延时能力时，统一调用 `bsp/time`。

## board

`board` 保存板级配置和启动资源：

- `sys_config/`
- `startup/`

其中 SysConfig 生成文件内容不手改；调整外设需改 `board/sys_config/G3507.syscfg` 后用
SysConfig CLI 重新生成。

## project

`project` 保存 IDE 工程入口：

- `ccs/`
- `keil/`

源码路径重构后必须同步维护这两个目录中的工程文件。

## tools

- `speed_pid_viz.py`：可视化 Device Check 的 `[SPD]` 速度 PID 遥测。
- `straight_test_viz.py`：可视化 Straight Test 的 `[STR]` 左右轮占空比、速度、
  距离、yaw 与 gz，支持原始日志和 CSV 导出。

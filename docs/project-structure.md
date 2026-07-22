# 工程结构

## 顶层结构

当前工程采用“分层源码目录 + IDE 工程目录 + 板级配置目录”的组织方式：

- `app/`：应用入口
- `core/`：控制算法和数据处理
- `middleware/`：系统组合能力与共享运行时状态
- `bsp/`：板级外设驱动
- `board/`：SysConfig、启动和链接资源
- `project/`：CCS 和 Keil 工程文件
- `docs/`：工程文档
- `tools/`：调试和烧录脚本

> 说明：二维云台/瞄准子系统（step_motor / bldc / gimbal / gimbal_tracking / auto_aim /
> aim_solver / aim_fusion / localization）已整体移除，原任务框架的 app 亦已清空。当前
> app 仅保留极简启动骨架，下层 bsp/middleware/core 能力保留但暂无调用者，等待按新需求重建。

## 运行路径

当前 `app/main.c` 为极简骨架：`SYSCFG_DL_init()` 完成 SysConfig 外设/中断初始化后
`__enable_irq()` 进入空 `while(1)`，并提供空 `SysTick_Handler` 以避免落入 startup 的
weak 死循环。重建 app 时在此接入调度与任务流程。

## app

`app` 是顶层应用层。当前仅含 `main.c`（启动骨架）。重建时该层可以直接调用下层公开接口，
但不应把纯算法或底层驱动细节继续塞入应用流程。

## core

`core` 放置和硬件无关或弱硬件相关的控制逻辑：

- `common/`：core 层基础数据类型。
- `pid/`：位置式和增量式 PID 控制器。
- `kinematics/`：角度、位姿、差速混控和二维几何计算。

该层保持硬件无关，不包含 `app`、`middleware` 或 `bsp` 头文件。

## middleware

`middleware` 放置多个 BSP 外设组合后的系统能力：

- `chassis/`：底盘组合服务
- `line_follow/`：巡线运行状态服务
- `line_tracking/`：巡线偏差计算、PID 修正和底盘输出
- `ui/`：轻量 OLED UI 渲染层
- `fault/`：系统故障处理服务

> 注：`line_tracking` 等模块因需直接调用 chassis/line_follow 等下层服务并读取硬件观测，
> 本质是“组合 core 算法 + 驱动执行器”的中间件能力，置于 `middleware` 以消除
> `core ↔ middleware` 循环依赖、保持 `core` 纯计算。

## bsp

`bsp` 是最低层，直接面对板级外设：

- `common/`
- `imu/`（JY61P，WIT 协议，I2C0 中断驱动）
- `mpu6050/`（MPU6050 DMP 姿态，I2C0，与 JY61P 共总线）
- `key/`
- `motor/`（TB6612 直流电机 + 霍尔编码器）
- `oled/`（SSD1306，帧缓冲模型）
- `grayscale_sensor/`
- `debug_uart/`
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

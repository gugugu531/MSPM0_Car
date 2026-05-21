# MSPM0 Car Project

基于 `TI MSPM0G3507` 的嵌入式小车工程，当前采用四层源码结构：

`app -> core -> middleware -> bsp`

工程仍同时维护：

- `CCS + ticlang`
- `Keil MDK + ArmClang`

两套 IDE 工程共享同一套源码，但启动文件、链接配置、工程元数据和构建输出目录彼此独立。

## 当前入口

系统主入口位于 `app/main.c`：

- 调用 `SYSCFG_DL_init()` 并使能中断
- 初始化 `Ui`、按键、底盘、云台、巡线状态、CanMV UART 和故障状态
- 使能编码器采样定时器、UART0 IMU 接收中断和 UART2 CanMV 接收中断
- 进入 `app/app_launcher.c` 的 `App_Launch()`

## 分层目录

- `app/`
  - 主入口、中断分发、E 题前三项任务入口和设备自检页面
- `core/`
  - PID、运动学、几何、旋转、巡线控制和视觉云台控制等算法逻辑
- `middleware/`
  - 底盘、云台、巡线状态、轻量 UI 和故障处理等组合能力
- `bsp/`
  - `common/`、`canmv/`、`grayscale_sensor/`、`imu/`、`key/`、`motor/`、`oled/`、`step_motor/`、`time/`
- `board/`
  - `sys_config/`：SysConfig 输入和生成代码
  - `startup/`：启动和链接相关资源
- `project/`
  - `ccs/`：CCS projectspec 和目标配置
  - `keil/`：Keil 工程文件
- `docs/`
  - 架构、接口、构建、结构、changelog、todo 文档
- `tools/`
  - J-Link 调试和烧录脚本

## 依赖规则

源码层级只能自顶向下调用：

- `app` 可以调用 `core`、`middleware`、`bsp`
- `core` 可以调用 `middleware`、`bsp`
- `middleware` 可以调用 `bsp`
- `bsp` 不能调用任何上层模块

当前已把 BSP 中原本依赖上层的时间、延时和共享状态访问改为明确的下层接口或由对应模块自己导出状态。

## 任务与调试入口

固件启动后进入启动页，当前顶层菜单提供：

- `E1 Line 1 lap` 到 `E1 Line 5 laps`
- `E2 Aim 2s`
- `E3 Aim 4s`
- `Device check`

设备自检页包含底盘、云台、IMU、CanMV 视觉和 8 路灰度传感器检查。

## 构建入口

- CCS 导入规格：`project/ccs/NUEDC2025_MSPM0G3507_ticlang.projectspec`
- Keil 工程文件：`project/keil/NUEDC2025_MSPM0G3507.uvprojx`

修改源码布局后需要同步维护两套工程文件。当前工程文件已指向新的分层目录。

最近一次验证：

- CCS/ticlang 直接交叉编译通过，使用 `C:\ti\ti_cgt_arm_llvm_4.0.2.LTS\bin\tiarmclang.exe`
- Keil/ArmClang rebuild 通过，使用 `D:\Keil_v5\UV4\UV4.exe`

## 文档

- `docs/architecture.md`
- `docs/interfaces.md`
- `docs/build-guide.md`
- `docs/project-structure.md`
- `docs/changelog.md`
- `docs/todo.md`

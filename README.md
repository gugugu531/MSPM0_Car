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
- 初始化编码器、激光串口、OLED、底盘电机、按键
- 为 BSP 注入必要 provider，例如按键时间源和 OLED 延时函数
- 进入 `app/app_launcher.c` 的 `App_Launch()`

## 分层目录

- `app/`
  - 主入口、中断分发、启动器、菜单、题目模式和设备自检页面
- `core/`
  - PID、运动学、巡线控制、传感处理、视觉云台控制等算法逻辑
- `middleware/`
  - `runtime/`：系统时间、错误信息、巡线运行时状态等共享状态声明
  - `system/`：延时、错误处理、底盘电机组合控制、云台组合控制
- `bsp/`
  - `common/`、`imu/`、`key/`、`laser/`、`motor/`、`oled/`、`step_motor/`、`tracking_sensor/`
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

当前已把 BSP 中原本依赖上层的时间、延时和共享状态访问改为 provider 注入或由 BSP 自己导出状态。

## 任务与调试入口

固件启动后进入启动页，提供两个一级入口：

- `Task flow`：进入正式任务菜单
- `Device check`：进入设备自检页

正式任务菜单包含 `Task B`、`Task H` 相关流程；设备自检页包含底盘电机、云台、IMU、视觉定位和巡线传感器检查。

## 构建入口

- CCS 导入规格：`project/ccs/NUEDC2025_MSPM0G3507_ticlang.projectspec`
- Keil 工程文件：`project/keil/NUEDC2025_MSPM0G3507.uvprojx`

修改源码布局后需要同步维护两套工程文件。当前工程文件已指向新的分层目录。

## 文档

- `docs/architecture.md`
- `docs/interfaces.md`
- `docs/build-guide.md`
- `docs/project-structure.md`
- `docs/changelog.md`
- `docs/todo.md`


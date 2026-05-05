# MSPM0 Car Project

基于 `TI MSPM0G3507` 的嵌入式小车工程，当前仓库同时维护：

- `CCS + ticlang` 工程
- `Keil MDK + ArmClang` 工程

两套工程共用同一套业务源码，但各自保持独立的启动文件、链接配置和构建输出目录。

## 当前运行入口

系统主入口位于 [Core/Src/main.c](Core/Src/main.c)：

- 完成 `SYSCFG_DL_init()`、中断使能和基础外设初始化
- 初始化编码器、激光串口、OLED、底盘电机和按键
- 最后进入 [Modules/Mission/AppLauncher.c](Modules/Mission/AppLauncher.c) 的 `App_Launch()`

当前固件的顶层交互不是直接进入比赛模式，而是先进入启动页，在 OLED 上提供两个一级入口：

- `Task flow`：进入正式任务菜单
- `Device check`：进入设备自检页

## 任务与调试结构

### Task flow

正式任务菜单由 [Modules/Mission/Menu.c](Modules/Mission/Menu.c) 和 [Modules/Mission/ModeTree.c](Modules/Mission/ModeTree.c) 构建，当前模式树如下：

- `Task B`
  - `B1 Circle`
    - `1 Lap` 到 `5 Laps`
  - `B23 Laser`
- `Task H`
  - `H1 Circle`
    - `1 Lap`、`2 Laps`
  - `H2 Track`

对应任务实现位于 [Modules/Mission/Mode.c](Modules/Mission/Mode.c)：

- `mode_problem_b_1()`：按圈数执行巡线绕圈
- `mode_problem_b_2_3()`：步进云台配合视觉做激光目标控制
- `mode_problem_h_1()`：巡线与视觉联动
- `mode_problem_h_2()`：巡线并跟踪目标圆

### Device check

设备自检入口由 `App_Launch()` 调起，当前包含 5 个测试页：

- `Motor speed`：底盘前进、后退、原地转向和编码器速度观测
- `Pan tilt`：云台步进电机角速度与姿态角显示
- `Gyro angle`：IMU 串口接收、姿态页切换和帧计数
- `Vision loc`：视觉串口激光点和矩形定位数据显示
- `Track sensor`：8 路巡线数字量与车速显示

## 目录概览

- `Core/`
  - `Src/`：主入口和中断入口
  - `Inc/`：全局共享状态、编译配置与跨模块声明
- `Board/`
  - `SysConfig/`：`SysConfig` 输入与生成代码
  - `Startup/`：链接脚本和板级启动相关资源
- `Drivers/`
  - `BSP/`：编码器、电机、OLED、按键、IMU、巡线、激光串口等外设驱动
  - `Platform/`：延时、错误处理、初始化等平台层服务
- `Modules/`
  - `Control/`：巡线、PID、运动学、视觉控制和步进控制算法
  - `Mission/`：启动器、菜单树、题目流程与任务编排
- `Project/`
  - `CCS/`：`CCS` 导入规格与工程元数据
  - `CCS/targetConfigs/`：`CCS` 目标连接配置
  - `Keil/`：`Keil` 工程文件
- `Docs/`
  - 构建说明、目录结构和维护约定
- `Tools/`
  - `J-Link` 调试与烧录脚本

## 工程入口文件

- `CCS` 导入规格文件：
  [Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec](Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec)
- `Keil` 工程文件：
  [Project/Keil/NUEDC2025_MSPM0G3507.uvprojx](Project/Keil/NUEDC2025_MSPM0G3507.uvprojx)

当前仓库不再保留根目录 `CCS` 工程元数据文件：

- 不跟踪 `.project`
- 不跟踪 `.cproject`
- 不跟踪 `.ccsproject`

如需在 `CCS` 中打开本工程，应通过 `projectspec` 重新导入。
当前 `projectspec` 采用“链接仓库源码”的方式导入，不再复制一份源码树。
若使用 `CCS` 图形界面直接导入，生成工程目录通常会落在当前 workspace 中。
如需避免把导入工程目录生成在仓库根目录，推荐：

- 使用仓库外独立 workspace
- 或将导入生成目录固定放在 `Project/CCS/Workspace/` 这类专用位置，并作为本地产物忽略

## 构建边界

- `CCS` 使用 TI SDK 自带的 `ticlang` 启动文件
- `Keil` 使用 TI SDK 自带的 `startup_mspm0g350x_uvision.s`
- `Debug/` 仅属于 `CCS` 构建输出
- `Project/Keil/Objects/` 仅属于 `Keil` 构建输出
- 两套工程都应只引用仓库中的共享源码，不应把对方的构建产物目录当作输入

## 工作区清理约定

以下目录或文件默认视为可再生的本地产物，不应作为稳定源码内容维护：

- `Debug/`
- `Project/Keil/Objects/`
- `Project/Keil/*.map`
- `Project/Keil/*.uvoptx`
- `Project/Keil/*.uvguix.*`
- `NUEDC2025_MSPM0G3507_nortos_ticlang/`
- `.settings/`
- `.theia/`
- `.clangd/`

## 开发约定

- 新增任务流程或菜单逻辑，优先放在 `Modules/Mission/`
- 新增控制算法或传感处理，优先放在 `Modules/Control/`
- 新增硬件驱动，优先放在 `Drivers/BSP/`
- 新增系统级初始化或公共平台能力，优先放在 `Drivers/Platform/`
- 修改源码布局后，同步检查 `CCS` 和 `Keil` 两套工程文件
- `SysConfig` 生成文件仅在必要时重生成，不在其中写业务逻辑

## 相关文档

- [Docs/build-guide.md](Docs/build-guide.md)
- [Docs/project-structure.md](Docs/project-structure.md)

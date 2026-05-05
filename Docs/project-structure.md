# MSPM0 Car Project Structure

## 1. Layout Overview

本工程采用“共享源码目录 + 独立 IDE 工程目录”的组织方式，同时服务于：

- `CCS + ticlang`
- `Keil + ArmClang`

目录职责如下。

- `Core/`
  - `Src/`：系统主入口与中断入口
  - `Inc/`：全局共享状态、编译配置和跨模块头文件
- `Board/`
  - `SysConfig/`：`SysConfig` 输入与生成代码
  - `Startup/`：散装链接脚本、命令文件和板级启动资源
- `Drivers/`
  - `BSP/`：外设和板级驱动
  - `Platform/`：平台公共层和系统服务
- `Modules/`
  - `Control/`：巡线、运动学、PID、云台控制和传感处理
  - `Mission/`：启动页、菜单树、题目模式和流程编排
- `Project/`
  - `CCS/`：`CCS` 导入规格和工程元数据
  - `CCS/targetConfigs/`：`CCS` 目标连接配置
  - `Keil/`：`Keil` 工程文件
- `Docs/`
  - 结构说明、构建说明、维护约定
- `Tools/`
  - `J-Link` 辅助脚本

当前根目录不再保留 `.project`、`.cproject`、`.ccsproject`，因此仓库形态更接近“共享源码 + `CCS` 导入模板 + `Keil` 工程文件”，而不是“直接可打开的根目录 `CCS` 工程”。
`CCS` 若基于 `projectspec` 重新生成导入工程目录，该目录应视为本地工程产物，而不是仓库源码目录的一部分。

## 2. Runtime Layering

当前源码的运行层次可以按下面理解：

1. `Core`
   [Core/Src/main.c](../Core/Src/main.c) 负责系统启动、中断入口和调度起点。
2. `Drivers/Platform`
   [Drivers/Platform/System/Initialize.c](../Drivers/Platform/System/Initialize.c) 提供电机系统初始化、共享状态定义和基础运动接口。
3. `Drivers/BSP`
   提供编码器、电机、OLED、按键、激光串口、IMU、巡线和步进电机底层驱动。
4. `Modules/Control`
   承载巡线、速度控制、视觉控制、运动学和步进控制算法。
5. `Modules/Mission`
   负责把底层能力拼装成“启动页 -> 菜单 -> 比赛模式 / 自检模式”的完整流程。

这个分层比“按文件夹看功能”更接近当前固件的实际执行路径，后续新增逻辑建议继续沿这个方向维护。

## 3. Core And Shared State

`Core/Inc/` 当前除了公共头文件，也承担了共享运行时状态的声明职责，主要包括：

- [Core/Inc/AppState.h](../Core/Inc/AppState.h)
- [Core/Inc/SystemTime.h](../Core/Inc/SystemTime.h)
- [Core/Inc/TrackingRuntime.h](../Core/Inc/TrackingRuntime.h)
- [Core/Inc/VisionState.h](../Core/Inc/VisionState.h)
- [Core/Inc/project_build_config.h](../Core/Inc/project_build_config.h)

这些头文件被 `Platform`、`Control`、`Mission` 多层共同使用，因此：

- 适合放跨模块共享的状态声明
- 不适合塞入只服务单一驱动的私有接口

## 4. Mission Layer Structure

`Modules/Mission/` 是当前仓库与早期“直接从 `main()` 进入某个模式函数”相比变化最大的部分。

### 4.1 AppLauncher

[Modules/Mission/AppLauncher.c](../Modules/Mission/AppLauncher.c) 是顶层应用启动器，职责包括：

- 提供启动首页
- 在 `Task flow` 和 `Device check` 之间切换
- 承接 `UART0` 的 IMU 调试串口接收
- 提供设备自检页的显示与交互

### 4.2 Menu + ModeTree

[Modules/Mission/Menu.c](../Modules/Mission/Menu.c) 与 [Modules/Mission/ModeTree.c](../Modules/Mission/ModeTree.c) 共同实现树形菜单：

- `Menu.c` 负责构建菜单树、OLED 展示和选择逻辑
- `ModeTree.c` 提供静态内存池式树节点管理
- `CircleList.c` 提供循环链表形式的菜单遍历支撑

### 4.3 Mode

[Modules/Mission/Mode.c](../Modules/Mission/Mode.c) 存放任务模式与部分测试模式的主循环逻辑，当前重点入口有：

- `mode_problem_b_1()`
- `mode_problem_b_2_3()`
- `mode_problem_h_1()`
- `mode_problem_h_2()`

这部分代码依赖 `Modules/Control/` 和多个 `BSP` 驱动，属于“流程编排层”，不建议再把纯算法细节继续塞回这里。

## 5. Control Layer Structure

`Modules/Control/` 当前主要承载以下职责：

- `Tracking.*`：巡线控制策略
- `SensorProc.*`：传感数据处理
- `PID.*`：PID 控制器
- `Kinematics.*`：运动学与坐标相关处理
- `Rotation.*`：旋转相关能力
- `StepMotorCtrl.*`：步进云台控制

如果后续增加新的控制闭环或传感融合逻辑，优先放在这里，再由 `Mission` 层调用。

## 6. BSP And Platform Boundaries

当前仓库已经把“硬件驱动”和“系统服务”分开，建议继续保持：

- `Drivers/BSP/`
  - 面向具体外设和引脚资源
  - 例如 `Motor`、`OLED`、`Key`、`Laser`、`IMU`、`TrackingSensor`
- `Drivers/Platform/System/`
  - 面向系统启动、错误处理、延时和跨驱动初始化
  - 不直接承载题目流程

例如 [Drivers/Platform/System/Initialize.c](../Drivers/Platform/System/Initialize.c) 当前既提供 `Motor_SystemInit()`，也集中定义若干共享运行时变量；如果继续扩展平台层，最好保持这种“平台公共能力”定位，不要让它回退成业务杂项目录。

## 7. Startup Strategy

为避免双 IDE 互相污染，当前采用独立启动文件策略：

- `CCS`
  - 使用 TI SDK 自带 `ticlang` 启动文件
- `Keil`
  - 使用 TI SDK 自带 `uVision` 启动汇编文件

配套边界如下：

- `Debug/` 只属于 `CCS`
- `Project/Keil/Objects/` 只属于 `Keil`
- 两套工程不共享对方的启动文件和构建产物

## 8. Local Artifact Boundary

以下内容属于本地环境或构建产物边界，默认不应视为长期维护对象：

- `Debug/`
- `Project/Keil/Objects/`
- `Project/Keil/*.map`
- `Project/Keil/*.uvoptx`
- `Project/Keil/*.uvguix.*`
- `NUEDC2025_MSPM0G3507_nortos_ticlang/`
- `.settings/`
- `.theia/`
- `.clangd/`

若后续需要在仓库内保留 `CCS` 导入工程目录，也建议收纳到 `Project/CCS/Workspace/` 这类专用位置，而不是继续放在仓库根目录。

## 9. Maintenance Guidance

- 新增业务流程，优先考虑放在 `Modules/Mission/`
- 新增控制算法，优先放在 `Modules/Control/`
- 新增硬件驱动，优先放在 `Drivers/BSP/`
- 新增系统级初始化或公共封装，优先放在 `Drivers/Platform/`
- 修改目录结构或新增源码后，务必同步维护 `CCS` 与 `Keil` 两套工程文件
- `SysConfig` 生成文件尽量少手改，优先修改输入源并重生成

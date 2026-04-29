# MSPM0 Car Project

基于 `TI MSPM0G3507` 的嵌入式小车工程，当前同时维护：

- `CCS + ticlang` 工程
- `Keil MDK + ArmClang` 工程

本仓库已经按“源码与工程文件分离”的方式整理，并明确区分了 `CCS` 与 `Keil` 的启动文件、构建输出和工程元数据。

## 目录概览

- `Core/`
  - `Src/`：主入口与中断入口
  - `Inc/`：全局公共头文件与共享状态声明
- `Board/`
  - `SysConfig/`：`SysConfig` 输入与生成代码
  - `Startup/`：链接脚本、命令文件等板级启动资源
  - `Target/`：目标连接配置
- `drivers/`
  - `BSP/`：硬件外设驱动
  - `Platform/`：平台公共层与系统服务
- `Modules/`
  - `Control/`：控制、巡线、运动学、传感处理
  - `Mission/`：菜单、模式树、任务流程
- `Project/`
  - `CCS/`：CCS 工程导入与构建相关文件
  - `Keil/`：Keil 工程文件
- `docs/`
  - 工程结构、构建说明、维护约定

## 工程入口

- `CCS` 工程名：`NUEDC2025_MSPM0G3507_nortos_ticlang`
- `Keil` 工程文件：[Project/Keil/NUEDC2025_MSPM0G3507.uvprojx](/E:/Learning/MSPM0_Car/Project/Keil/NUEDC2025_MSPM0G3507.uvprojx:1)
- `CCS` 导入规格文件：[Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec](/E:/Learning/MSPM0_Car/Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec:1)

## 构建约定

- `CCS` 使用 TI SDK 自带的 `ticlang` 启动文件
- `Keil` 使用 TI SDK 自带的 `uVision` 启动汇编文件
- 两个 IDE 不共享对方的启动文件
- `Keil` 输出目录 `Project/Keil/Objects/` 不参与 `CCS` 构建
- `Debug/`、`Project/Keil/Objects/` 属于构建产物目录，不作为手工源码编辑区域

## 开发建议

- 新增业务逻辑优先放入 `Modules/`
- 新增硬件驱动优先放入 `drivers/BSP/`
- 新增系统级能力优先放入 `drivers/Platform/`
- `SysConfig` 生成文件仅在必要时由工具重新生成，不手工改业务逻辑
- 修改目录或新增源码后，要同步检查 `CCS` 与 `Keil` 工程文件

## 相关文档

- [docs/project-structure.md](/E:/Learning/MSPM0_Car/docs/project-structure.md:1)
- [docs/build-guide.md](/E:/Learning/MSPM0_Car/docs/build-guide.md:1)

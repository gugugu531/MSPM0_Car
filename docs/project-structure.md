# MSPM0 Car Project Structure

## 1. Directory Layout

本工程采用“源码目录”和“IDE 工程目录”分离的组织方式，兼顾：

- `CCS + ticlang`
- `Keil + ArmClang`

目录职责如下。

- `Core/`
  - `Src/`：系统入口、中断入口、主循环入口
  - `Inc/`：全局共享头文件、状态声明、编译开关
- `Board/`
  - `SysConfig/`：`SysConfig` 输入与生成代码
  - `Startup/`：与板级启动、链接脚本相关的公共资源
  - `Target/`：目标连接配置
- `drivers/`
  - `BSP/`：外设驱动与板级硬件驱动
  - `Platform/`：系统服务、公共底层类型与平台封装
- `Modules/`
  - `Control/`：控制、巡线、运动学、传感处理
  - `Mission/`：菜单、模式树、题目流程与任务编排
- `Project/`
  - `CCS/`：`CCS` 工程导入和构建规格
  - `Keil/`：`Keil` 工程文件
- `docs/`
  - 工程结构、构建说明、维护规则

## 2. Startup File Strategy

为避免工具链互相污染，当前采用双工程独立启动文件策略。

- `CCS`
  - 使用 TI SDK 自带 `ticlang` 启动文件
- `Keil`
  - 使用 TI SDK 自带 `uVision` 启动汇编文件

约束如下。

- 不在仓库内长期维护 `Keil` 专属启动汇编副本
- 不让 `CCS` 扫描 `Keil` 产物目录
- 不让 `Keil` 依赖 `CCS` 生成目录

## 3. Build Output Boundary

- `Debug/`
  - `CCS` 自动生成目录
- `Project/Keil/Objects/`
  - `Keil` 自动生成目录

这两个目录都属于构建产物目录，不应：

- 作为手工源码目录
- 被另一套 IDE 链接为输入

## 4. Module Placement Rules

- 主流程与中断入口放在 `Core/`
- 硬件驱动放在 `drivers/BSP/`
- 平台公共逻辑放在 `drivers/Platform/`
- 算法与控制逻辑放在 `Modules/Control/`
- 菜单与题目流程放在 `Modules/Mission/`
- `SysConfig` 相关内容放在 `Board/SysConfig/`

## 5. Maintenance Notes

- 新增源码后要同步检查 `CCS` 与 `Keil` 工程文件
- 修改目录结构后要同步检查头文件路径和 IDE 分组
- 自动生成文件尽量少手改，优先修改其输入源
- 若出现双工程构建异常，优先排查是否存在：
  - 启动文件混用
  - 构建产物互相污染
  - `SysConfig` 生成缓存未刷新

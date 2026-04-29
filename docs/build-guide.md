# Build Guide

## 1. Toolchains

本工程当前维护两套独立构建链：

- `CCS 2020 + ticlang`
- `Keil MDK 5 + ArmClang`

两套工程共享同一份业务源码，但：

- 启动文件分别独立
- 构建输出目录分别独立
- IDE 工程文件分别独立

## 2. CCS Build

### 2.1 关键文件

- 工程规格文件：
  [Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec](/E:/Learning/MSPM0_Car/Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec:1)
- `SysConfig`：
  [Board/SysConfig/empty.syscfg](/E:/Learning/MSPM0_Car/Board/SysConfig/empty.syscfg:1)

### 2.2 启动与链接

- 启动文件使用 TI SDK 自带：
  `startup_mspm0g350x_ticlang.c`
- 链接相关资源位于：
  [Board/Startup/stack_heap.cmd](/E:/Learning/MSPM0_Car/Board/Startup/stack_heap.cmd:1)

### 2.3 构建输出

- `Debug/`

该目录是自动生成目录，出现异常时可直接清理后重建。

### 2.4 注意事项

- `CCS` 不应链接 `Project/Keil/Objects/` 下的任何 `.o`
- 如果修改了 `.project`、`.cproject` 或 `projectspec`，建议：
  1. 刷新工程
  2. `Clean`
  3. `Build`

## 3. Keil Build

### 3.1 关键文件

- 工程文件：
  [Project/Keil/NUEDC2025_MSPM0G3507.uvprojx](/E:/Learning/MSPM0_Car/Project/Keil/NUEDC2025_MSPM0G3507.uvprojx:1)

### 3.2 启动与链接

- 启动文件使用 TI SDK 自带 `Keil` 版本：
  `C:\ti\mspm0_sdk_2_10_00_04\source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s`
- Scatter 文件：
  [Board/Startup/mspm0g3507.sct](/E:/Learning/MSPM0_Car/Board/Startup/mspm0g3507.sct:1)

### 3.3 构建输出

- `Project/Keil/Objects/`

该目录只属于 `Keil`，不应被 `CCS` 当作输入参与链接。

## 4. SysConfig Generated Files

以下文件为工具生成或与工具强相关：

- [Board/SysConfig/ti_msp_dl_config.c](/E:/Learning/MSPM0_Car/Board/SysConfig/ti_msp_dl_config.c:1)
- [Board/SysConfig/ti_msp_dl_config.h](/E:/Learning/MSPM0_Car/Board/SysConfig/ti_msp_dl_config.h:1)
- `Debug/` 内的 `device.opt`、`device_linker.cmd`、`device.cmd.genlibs`

建议：

- 业务逻辑不要写入这些自动生成文件
- 如果 `SysConfig` 重生成后出现差异，优先检查是否为工具生成导致

## 5. Maintenance Rules

- 新增源码后，同时检查 `CCS` 与 `Keil` 是否都纳入工程
- 不要把某个 IDE 的构建产物目录纳入另一套 IDE 的源码扫描范围
- 不要在 `Project/Keil/Objects/`、`Debug/` 内手工维护源码
- 遇到双 IDE 构建异常时，优先排查：
  - 是否误链接了对方的 `.o`
  - 是否误用了对方的启动文件
  - 是否是自动生成缓存未刷新

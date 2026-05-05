# Build Guide

## 1. Toolchains

本工程当前维护两套独立构建链：

- `CCS + ticlang`
- `Keil MDK 5 + ArmClang`

两套工程共享同一套业务源码，但以下内容彼此独立：

- 启动文件
- 链接配置
- IDE 工程元数据
- 构建输出目录

## 2. Shared Source Set

两套工程共同使用的核心源码主要分布在以下位置：

- [Core/Src/main.c](../Core/Src/main.c)
- `Core/Inc/`
- `Board/SysConfig/`
- `Drivers/Platform/System/`
- `Drivers/BSP/`
- `Modules/Control/`
- `Modules/Mission/`

当前主流程入口为：

1. `main()` 完成系统和外设初始化
2. `App_Launch()` 进入启动页
3. 用户在 `Task flow` 与 `Device check` 之间选择

这意味着构建验证时，不仅要确认能通过编译，还要关注 `AppLauncher`、`Menu`、`Mode` 相关文件是否都已被两套工程纳入。

## 3. CCS Build

### 3.1 Key Files

- 导入规格文件：
  [Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec](../Project/CCS/NUEDC2025_MSPM0G3507_ticlang.projectspec)
- `SysConfig` 输入文件：
  [Board/SysConfig/empty.syscfg](../Board/SysConfig/empty.syscfg)
- `SysConfig` 生成代码：
  [Board/SysConfig/ti_msp_dl_config.c](../Board/SysConfig/ti_msp_dl_config.c)
  [Board/SysConfig/ti_msp_dl_config.h](../Board/SysConfig/ti_msp_dl_config.h)

当前仓库不再保留根目录 `CCS` 工程元数据文件，因此 `CCS` 的推荐入口是 `projectspec` 导入，而不是直接打开仓库根目录。

### 3.2 Startup And Link

- 启动文件使用 TI SDK 自带 `startup_mspm0g350x_ticlang.c`
- 链接相关资源位于：
  [Board/Startup/stack_heap.cmd](../Board/Startup/stack_heap.cmd)

### 3.3 Build Output

- `Debug/`

该目录属于自动生成目录，可在构建异常时清理后重建。

### 3.4 CCS Maintenance Notes

- 新增源码后，确认 `projectspec` 已包含对应文件
- 若修改 `projectspec` 或重新导入工程，建议执行 `Refresh`、`Clean`、`Build`
- 不要让 `CCS` 扫描 `Project/Keil/Objects/` 或引用其中的 `.o/.obj`

## 4. Keil Build

### 4.1 Key Files

- 工程文件：
  [Project/Keil/NUEDC2025_MSPM0G3507.uvprojx](../Project/Keil/NUEDC2025_MSPM0G3507.uvprojx)

### 4.2 Startup And Link

- 启动文件使用 TI SDK 自带 `Keil` 版本：
  `C:\ti\mspm0_sdk_2_10_00_04\source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s`
- Scatter 文件：
  [Board/Startup/mspm0g3507.sct](../Board/Startup/mspm0g3507.sct)

### 4.3 Build Output

- `Project/Keil/Objects/`

该目录只属于 `Keil`，不应被 `CCS` 当作输入参与编译或链接。

### 4.4 Keil Maintenance Notes

- 新增源码后，确认 `.uvprojx` 已纳入对应文件
- 保持 `OutputDirectory` 指向 `.\Objects\`
- 不要把 `Debug/` 里的自动生成文件作为 `Keil` 工程输入
- `uvoptx`、`uvguix.*` 属于本地会话文件，不作为稳定仓库内容依赖
- `uvprojx` 中引用的 TI SDK 启动文件和 `driverlib.a` 仍可能是工具链安装路径，这类外部 SDK 路径不属于仓库内相对路径范围

## 5. SysConfig And Generated Files

以下内容与工具生成强相关：

- [Board/SysConfig/empty.syscfg](../Board/SysConfig/empty.syscfg)
- [Board/SysConfig/ti_msp_dl_config.c](../Board/SysConfig/ti_msp_dl_config.c)
- [Board/SysConfig/ti_msp_dl_config.h](../Board/SysConfig/ti_msp_dl_config.h)
- `Debug/` 内的 `device.opt`、`device_linker.cmd`、`device.cmd.genlibs`

建议：

- 业务逻辑不要写入自动生成文件
- 若重生成后出现差异，优先判断是否只是工具版本或生成参数变化

## 6. Runtime Verification Checklist

仅通过编译并不能说明固件行为正确，建议至少验证以下运行路径：

- 上电后是否进入 `Mode select`
- `Task flow` 能否进入任务菜单
- `Device check` 的 5 个测试页能否正常切换
- `UART0_IRQHandler` 的 IMU 接收是否正常累计帧数
- `UART2_IRQHandler` 的视觉串口接收是否仍能更新 `Laser_Loc` / `Rect_Loc`
- `SysTick_Handler` 的按键扫描节拍是否正常

## 7. Maintenance Rules

- 新增源码后，同时更新 `CCS` 和 `Keil` 工程
- 不要混用两套 IDE 的启动文件
- 不要把某个 IDE 的构建产物目录纳入另一套 IDE 的源码扫描范围
- 不要在 `Debug/` 或 `Project/Keil/Objects/` 中手工维护源码
- 若本地重新生成了 `.settings/`、`Debug/`、`uvoptx`、`uvguix.*`，提交前应先清理
- 若出现双 IDE 构建异常，优先排查：
  - 是否误引用了对方的构建产物
  - 是否误用了对方的启动文件
  - 是否 `SysConfig` 或工程缓存未刷新

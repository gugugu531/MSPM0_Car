# 构建说明

## 工具链

本工程维护两套构建链：

- `CCS + ticlang`
- `Keil MDK 5 + ArmClang`

两套工程共享 `app`、`core`、`middleware`、`bsp`、`board/sys_config` 中的源码和配置，但各自维护启动文件、链接配置和输出目录。

## 共享源码

主要源码目录如下：

- `app/`
- `core/`
- `middleware/`
- `bsp/`
- `board/sys_config/`

构建时必须同时包含这些 include path。按照当前重写流程，新增源码先完成分层框架和接口收敛；整体框架确认后，再统一维护 `project/ccs/NUEDC2025_MSPM0G3507_ticlang.projectspec` 和 `project/keil/NUEDC2025_MSPM0G3507.uvprojx`。

当前两套工程文件已同步到分层源码树。

## CCS 构建

入口文件：

- `project/ccs/NUEDC2025_MSPM0G3507_ticlang.projectspec`

关键约定：

- 使用 `projectspec` 导入，不在仓库根目录维护 `.project`、`.cproject`、`.ccsproject`
- 推荐使用仓库外 CCS workspace
- 若必须在仓库内生成导入工程，应放入专用本地产物目录并保持忽略
- SysConfig 输入和生成文件位于 `board/sys_config/`
- CCS 构建输出目录视为可再生本地产物

本地已验证的直接交叉编译命令使用：

- 编译器：`C:\ti\ti_cgt_arm_llvm_4.0.2.LTS\bin\tiarmclang.exe`
- SDK：`C:\ti\mspm0_sdk_2_10_00_04`
- 产物：`build/ccs/NUEDC2025_MSPM0G3507.out`

## Keil 构建

入口文件：

- `project/keil/NUEDC2025_MSPM0G3507.uvprojx`

关键约定：

- Keil 输出目录为 `project/keil/Objects/`
- Keil 使用 SDK 中的 uVision 启动文件
- Scatter 文件位于 `board/startup/mspm0g3507.sct`
- `uvoptx`、`uvguix.*` 属于本地会话文件，不作为稳定源码依赖

本地已验证的 Keil 命令：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -r 'NUEDC2025_MSPM0G3507.uvprojx' -o 'keil_build.log'
```

运行目录为 `project/keil/`，构建结果为 `Objects/NUEDC2025_MSPM0G3507.axf` 和对应 hex，最近一次 rebuild 为 `0 Error(s), 0 Warning(s)`。

命令行烧录（调试器已连接）：

```bash
"D:/Keil_v5/UV4/UV4.exe" -f "NUEDC2025_MSPM0G3507.uvprojx" -o "flash_log.txt"
```

成功日志应含 `Programming Done. Verify OK.`。

## 烧录已知问题：镜像须 8 字节对齐

**现象**：`Erase Done.` 之后立即 `Programming Failed! Error: Flash Download failed - "Cortex-M0+"`
（擦除成功、写入失败）。与镜像大小/内容相关，与调试时钟无关（降频无效）。

**根因**：MSPM0 Flash 按 **64 位（8 字节）字 + ECC** 编程，整镜像长度必须是 8 的倍数，
否则最后不足一整字的部分无法写入。分散加载文件的 `ALIGNALL 8` 只对齐各段**起始**、不保证
整镜像**尾长**对齐；TI 官方 `.sct` 亦如此，本质是 Keil 的 MSPM0 烧录算法(FLM)未补齐末字。
社区有同现象反馈（见文末 TI E2E 帖）。

**当前修复**：`app/main.c` 定义 8 字节对齐的尾填充 `g_flash_tail_pad`（`__ARMCC_VERSION`
限定仅 Keil），经 `board/startup/mspm0g3507.sct` 的 `*(.flash_tail_pad, +Last)` 放在
`ER_IROM1` 末尾，强制该段 8 对齐。

**⚠ 局限与复发条件**：该填充位于 `ER_IROM1`（代码+只读段）末尾。当前 `RW-data=0`，故它就是
整镜像尾部、有效。但 Flash 镜像 = `ER_IROM1(含填充) + RW 初值数据`，RW 初值数据排在填充**之后**
且无对齐保护。因此：

- 只改代码/函数/`const`/零值或未初始化全局（`RW-data` 保持 0）→ **永远安全，不会复发**。
- 新增**带非零初值的全局/静态变量**（`RW-data > 0`）→ 风险回归（概率性，约 1/8 的改动会触发）。

**每次编译自查**：看 Keil `Program Size` 行——`RW-data=0` 即稳；若 `RW-data` 非 0，检查
`.map` 的 `Total ROM Size` 是否为 8 的倍数，不是则会烧录失败。

**长期根治选项（暂未采用，记录备选）**：
1. 升级 Keil 的 MSPM0 DFP（器件包）——若新版 FLM 已补齐末字则问题消失。
2. 改用 J-Link 的 SEGGER 内部 flash 下载器（自动补齐末字）。
3. post-build 步骤把最终 `.bin/.hex` 补齐到 8/16 字节（工具链无关，对任意 RW 大小都成立）。

## SysConfig 和生成文件

以下文件由工具或配置生成，业务逻辑不要写入其中：

- `board/sys_config/empty.syscfg`
- `board/sys_config/ti_msp_dl_config.c`
- `board/sys_config/ti_msp_dl_config.h`

如重生成后产生差异，优先判断是否来自工具版本或配置变更。

## 运行验证清单

编译通过后仍需上板验证（当前 app 框架，详见 `docs/app-design.md`）：

- 上电后 OLED 显示 `Main Menu`
- 短按 UP/DOWN 移动选择、ENTER 进入、BACK 返回上级
- `Device Check` 子菜单内 6 个自检（Gyro JY61P / Gyro MPU6050 / Grayscale / Gray I2C / TB6612 / Encoder）可进入并刷新数据
- `Gray I2C` 进入后显示 8 路数字量二进制与 `online`/固件版本；断开传感器应显示 `OFFLINE`
- TB6612 自检短按发单次脉冲、编码器计数随之变化
- `SysTick_Handler` 可按周期扫描按键（菜单响应正常即证明）

## 本地产物边界

以下内容默认视为本地产物或 IDE 会话文件，不作为稳定源码维护：

- `Debug/`
- `project/keil/Objects/`
- `project/keil/*.map`
- `project/keil/*.uvoptx`
- `project/keil/*.uvguix.*`
- `.settings/`
- `.theia/`
- `.clangd/`

## 参考

- 烧录 8 字节对齐问题的社区反馈：TI E2E “MSPM0G3507: Keil: *Specific* code can be compiled
  but cannot be downloaded”
  <https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/1407593>

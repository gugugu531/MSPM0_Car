# MSPM0G3507 车载固件

基于 `TI MSPM0G3507` 的分层车载固件工程。

> **当前状态**：原 2025E 二维云台/瞄准子系统已整体移除；`app` 已重建为菜单驱动的裸机
> 协作式调度框架。当前可从 OLED 菜单运行 Yahboom 八路循迹测试、80% 航向循迹实验、K230 红线视觉循迹、
> 九种直行控制/启动实验、两种前进左转实验，
> 以及 JY61P、MPU6050、GPIO/感为/Yahboom 三种灰度传感器、TB6612、双轮编码器、
> 速度 PID、占空比扫描和蓝牙串口等设备检查任务。
>
> **视觉循迹状态**：K230 红线识别、LCD/Preview 和约 90 FPS 板端运行已验证，但尚未安装到车上进行
> 整车循迹实测；K230 控制 UART 仍关闭，MSPM0 控制增益均为待整车标定的初始值。
> 现阶段重点是底盘测速/速度闭环整定与各外设上板验证。

## 硬件构成

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 主控 | MSPM0G3507 (LQFP-64) | — | — |
| 底盘轮 | 直流电机 ×2 (TB6612FNG) + 霍尔编码器 | PWM / GPIO | `bsp/motor` |
| 姿态 | JY61P 六轴 (WIT 协议) | I2C0（与感为灰度共总线） | `bsp/imu` (wit_sdk) |
| 循迹 | 感为 8 路灰度传感器 | I2C0（地址 `0x4F`，与 JY61P 共总线） | `bsp/ganv_gray` |
| 摆杆执行器 | 步进电机 + 驱动器 | STEP PA29（TIMG6）/ DIR PB14 / EN PB11 | `bsp/step_motor` |
| 人机 | OLED + 独立按键 | I2C1 / GPIO | `bsp/oled`, `bsp/key` |
| 调试遥测 | Debug_Ex | UART1 115200（PA8 TX / PA9 RX） | `bsp/debug_uart` |

## 分层架构

```text
app ─► middleware ─► bsp
          │
          └────────► core（纯计算）
```

- `app`：初始化、协作式调度器、状态机、菜单树与具体测试任务。
- `core`：PID、滤波、运动学与纯角速度积分等纯计算能力。
- `middleware`：组合 core 与 BSP 的系统能力（`chassis` / `line_follow` /
  `line_guided_drive` / `vision_line_drive` / `straight_drive` / `turn_drive` /
  `ui` / `fault`）。
- `bsp`：直接面向板级外设的驱动（见上表 + `time` / `common`）。
- `board`：SysConfig 源文件与生成代码、启动/链接资源。
- `third_party/mspm0-sdk`：TI 官方 MSPM0 SDK 2.10.00.04 submodule，供各工程使用相对路径。

依赖单向：`core` 不读硬件、不调用 middleware；`app` 只编排、不重复实现算法。

## 快速开始

### 1. 获取源码与 SDK

首次克隆建议同时初始化子模块：

```powershell
git clone --recurse-submodules <仓库地址> MSPM0_Car
cd MSPM0_Car
```

若已经克隆过仓库，执行：

```powershell
git submodule update --init --depth 1 third_party/mspm0-sdk
```

SDK 固定为 `mspm0_sdk_2_10_00_04`，位于 `third_party/mspm0-sdk`。Keil 与 CCS 工程均通过
仓库相对路径引用它，不需要另外配置 SDK 绝对路径。

### 2. 选择开发环境

推荐工具组合：

- Keil MDK 5：uVision + Arm Compiler 6.22。
- Keil MDK 6：VS Code + Keil Studio Pack + CMSIS Toolbox + Arm Compiler 6.22。
- CCS：保留 projectspec，但当前版本尚未完成重新导入后的编译回归。
- Python 3：仅运行遥测可视化工具时需要。

## 编译

### Keil MDK 6 / VS Code

1. 用 VS Code 打开仓库根目录。
2. 打开 `project/keil/NUEDC2025_MSPM0G3507.csolution.yml`，将其设为活动 Solution。
3. 等待 Keil Studio Pack 完成 CMSIS Pack 和工具环境解析。
4. 在 CMSIS 侧栏选择当前 context，执行 **Build Solution** 或 **Rebuild Solution**。

`project/keil/vcpkg-configuration.json` 可让 Arm Tools Environment Manager 获取匹配版本的
AC6、CMSIS Toolbox、CMake 与 Ninja；需要时可右键该文件并执行 **Activate Environment**。
若希望复用本机 Keil 已安装的 AC6，在 VS Code 设置中搜索
`CMSIS Solution: Environment Variables`，向工作区 `settings.json` 加入：

```json
"cmsis-csolution.environmentVariables": {
  "AC6_TOOLCHAIN_6_22_0": "<KEIL>\\ARM\\ARMCLANG\\bin"
}
```

本机 Keil 安装在 D 盘时，`<KEIL>` 例如 `D:\Keil_v5`。修改后执行
`Developer: Reload Window`，再重新构建。成功日志应包含：

```text
Using AC6 V6.22.0 compiler
Build summary: 1 succeeded, 0 failed
```

命令行等价构建方式：

```powershell
$env:AC6_TOOLCHAIN_6_22_0 = "<KEIL>\ARM\ARMCLANG\bin"
cbuild project/keil/NUEDC2025_MSPM0G3507.csolution.yml --rebuild --toolchain AC6
```

输出位于 `project/keil/out/`。TI 启动文件使用 legacy armasm 语法，当前会出现一条 `A1950W`
弃用警告，不影响链接产物生成。

### Keil MDK 5 / uVision

打开：

```text
project/keil/NUEDC2025_MSPM0G3507.uvprojx
```

选择 Arm Compiler 6.22 后执行 Build/Rebuild。也可以在 PowerShell 中运行：

```powershell
Push-Location project/keil
& "<KEIL>\UV4\UV4.exe" -r "NUEDC2025_MSPM0G3507.uvprojx" -o "keil_build.log"
Pop-Location
```

输出位于 `project/keil/Objects/`。当前工程已使用 D 盘 Keil 5.41 / AC6 6.22 验证为
`0 Error(s), 0 Warning(s)`。

更完整的 CCS、SysConfig、命令行构建和 Flash 对齐说明见
[`docs/build-guide.md`](docs/build-guide.md)。

## 烧录与首次运行

连接调试器并确认目标器件为 `MSPM0G3507`：

- MDK 5：在 uVision 中执行 Download，或使用工程已配置的下载按钮。
- MDK 6：在 CMSIS 视图中选择正确调试适配器后执行 Load/Run。

首次上板请先架空驱动轮并准备随时断电。`Straight Test` 的具体任务在进入后会立即运行：

- 常规占空比模式默认以 `80%` 启动；`Speed Closed` 默认目标为 `1.06 m/s`。
- `100 Int->Yaw` 会立即以 `100%` 输出，前 500 ms 使用纯角速度积分航向，随后切换到
  经启动误差修正的 JY61P 航向闭环。所有直行模式达到 3 m 后自动停车并返回菜单。
- `Speed PID`、`Duty Sweep` 和 `TB6612` 同样属于电机测试，必须先架空车轮。

上电初始化成功后，OLED 显示 `Main Menu`。四个按键均使用短按：

| 按键 | 菜单中 | 任务中 |
|---|---|---|
| `UP` / `DOWN` | 移动选择 | 调整支持该操作的速度或占空比指令 |
| `ENTER` | 进入子菜单或启动任务 | 按任务定义执行，直行/速度测试中通常将指令归零 |
| `BACK` | 返回上级 | 中止任务、主动刹车并返回菜单 |

主菜单功能：

| 入口 | 用途 |
|---|---|
| `Line Follow` | Yahboom 八路循线外环与角速度内环测试 |
| `Line Guided 80` | 80% 直接起步，外侧黑线直控灰度 PID，未命中时航向保持 |
| `Line->Left->Line` | 首段循迹丢线后制动 250 ms，再左转至 X4/X5 重获线 |
| `Vision Red` | K230 红线位置/方向融合循迹；1 s 角速度起步，稳定阶段持续视觉控制 |
| `Straight Test` | 4 种基础直行控制与 5 种斜坡/启动阶段切换实验 |
| `Turn Test` | 80% 与满速两种转向：直行 2 m 后左轮反转、右轮降至 0%，到达左转 90° 后沿新航向直行 1 m |
| `Device Check` | JY61P、Yaw A/B、MPU6050、三种灰度、TB6612、编码器、速度 PID、占空比扫描和蓝牙检查 |

两个循迹页面均按 `X1 → X8` 显示 Yahboom 归一化掩码：`1` 表示该路检测到黑线。

## 串口遥测与可视化

调试遥测使用 `Debug_Ex/UART1`，参数为 `115200 8N1`。先安装 Python 依赖：

```powershell
python -m pip install pyserial matplotlib
```

列出串口并启动直行测试可视化：

```powershell
python tools/visualizers/straight_test_viz.py --list
python tools/visualizers/straight_test_viz.py --port COM7
```

界面同时显示两轮占空比、速度、距离以及融合 yaw、积分 yaw 和角速度。需要保存采样时可运行：

```powershell
python tools/visualizers/straight_test_viz.py --port COM7 --csv straight.csv --log straight_raw.txt
```

速度 PID 与占空比扫描使用：

```powershell
python tools/visualizers/speed_pid_viz.py --port COM7 --csv spd.csv --log raw.txt
```

`raw.txt`、`spd.csv`、`straight_raw.txt` 和 `straight.csv` 已默认忽略。蓝牙测试使用
UART0，参数为 `9600 8N1`；它与 UART1 调试遥测是两条不同通道。

## 修改工程

- 修改引脚或外设时，只编辑 `board/sys_config/G3507.syscfg`，随后用 SysConfig CLI 重生成
  `ti_msp_dl_config.c/h`；不要直接修改生成文件。
- 新增或删除源码时，同时维护 MDK 5 `.uvprojx`、MDK 6 `.cproject.yml` 和 CCS projectspec。
- 修改 Keil 工程输入后运行：

```powershell
python tools/checks/check_keil_project_sync.py
python tools/checks/check_docs.py
```

- JY61P、MPU6050、感为灰度与 Yahboom 循线模块共用 I2C0；循迹任务只在 JY61P 异步事务
  空闲时短暂读取 Yahboom，其他阻塞式检查任务通过 `JY61P_I2C_SetSuspended()` 独占总线。
- 不可同时使用无线调试器的虚拟串口和 Ex Uart。

## 文档索引

- 架构与依赖：[`docs/architecture.md`](docs/architecture.md)
- app 菜单和调度：[`docs/app-design.md`](docs/app-design.md)
- 构建与 SysConfig：[`docs/build-guide.md`](docs/build-guide.md)
- 目录职责：[`docs/project-structure.md`](docs/project-structure.md)
- 模块接口：[`docs/interfaces/`](docs/interfaces/)
  - 感为 I2C 灰度：[`bsp_ganv_gray.md`](docs/interfaces/bsp_ganv_gray.md)
  - JY61P：[`bsp_wit_sdk.md`](docs/interfaces/bsp_wit_sdk.md)
- 当前待办和上板风险：[`docs/todo.md`](docs/todo.md)
- K230 v1.8 固件更新、VS Code CanMV 扩展和 MCP/Preview 连接流程：
  [`docs/k230-development.md`](docs/k230-development.md)
- K230 Wi-Fi 程序上传、远程执行、调试输出与恢复流程：
  [`docs/k230-remote-development.md`](docs/k230-remote-development.md)
- 电脑端工具、K230 探针、J-Link 脚本和遥测可视化索引：[`tools/README.md`](tools/README.md)

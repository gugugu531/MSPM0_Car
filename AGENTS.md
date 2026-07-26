# AGENTS.md — MSPM0G3507 车载固件 开发工作流

## 项目概述

- **平台**: MSPM0G3507 (LQFP-64) 车载固件
- **IDE**: Keil MDK 5 + MDK 6/Keil Studio (ARMCLANG V6.22) / CCS (TICLANG)
- **SDK**: `third_party/mspm0-sdk` submodule，固定 mspm0_sdk_2_10_00_04
- **当前状态**: 原 2025E 二维云台/瞄准子系统已移除；`app` 已重建为菜单驱动的裸机协作式
  调度框架，现有循迹测试、四种直行控制测试和 9 项设备检查。当前重点为底盘闭环整定与上板验证。

> **注意**: 外部工具路径（Keil、SysConfig）需向用户确认实际安装位置；SDK 使用仓库内
> submodule，不应恢复成本机绝对路径。

## 硬件外设映射

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 底盘轮 | 直流电机 ×2 + 霍尔编码器 | PWM/GPIO | `bsp/motor` (TB6612) |
| 姿态 | JY61P (WIT 协议) | I2C0 | `bsp/imu` (wit_sdk, 中断驱动) |
| 姿态 | MPU6050（基础六轴 + DMP） | I2C0（共享总线） | `bsp/mpu6050` |
| 循迹 | 8 路灰度 | GPIO | `bsp/grayscale_sensor` |
| 循迹 | 感为 8 路灰度 | I2C0（与两种 IMU 共总线） | `bsp/ganv_gray` |
| 循迹 | Yahboom 8 路循线模块 | I2C0（地址 `0x12`，与两种 IMU/感为共总线） | `bsp/yahboom_track` |
| 显示/输入 | OLED / 按键 | I2C1/GPIO | `bsp/oled`, `bsp/key` |
| 通信 | 蓝牙 / 调试遥测 | UART0/UART1 | `bsp/bluetooth`, `bsp/debug_uart` |

## 目录结构

```
MSPM0_Car/
├── app/                  # 初始化、调度、状态机、菜单与测试任务
├── bsp/                  # 板级驱动 (motor, imu, grayscale, uart, oled, key, time...)
├── board/
│   ├── startup/          # 启动文件, 链接脚本
│   └── sys_config/       # SysConfig 源文件及生成代码
├── core/                 # 纯计算算法 (pid, filter, kinematics, common)
├── middleware/           # 中间件 (chassis, line_follow, straight_drive, ui, fault)
├── project/{keil,ccs}/     # Keil（MDK 5 + MDK 6/CMSIS Solution）与 CCS 工程
├── third_party/mspm0-sdk/ # TI 官方 SDK submodule（2.10.00.04）
├── docs/                 # 架构/接口/构建文档
└── tools/                # 调试/烧录脚本
```

## 分层与依赖边界

```text
app ─► middleware ─► {core (纯计算), bsp}
```

- `bsp` 只含标准头、厂商/板级头和 `bsp` 内部头。
- `middleware` 可含 `middleware` / `core` / `bsp` 头。
- `core` 只含标准头和 `core` 内部头（不读硬件）。
- `app` 可含任意下层公开头。
- 中断按“谁拥有该外设，谁定义 ISR”划分，`middleware` 不做中断转发；跨子系统分发/调度类中断放 `app`。

## 问题排查思路

1. **查官方例程**: `<sdk>/examples/nortos/LP_MSPM0G3507/`。
2. **读底层实现**: 深入 DriverLib 头 (如 `dl_uart.h`, `dl_i2c.h`) 的源码注释。
3. **搜 TI E2E 论坛**: 芯片型号 + 特性词。
4. **对比验证**: 找已知可工作参考逐行对比。
5. **先简后优**: 先验证连通性再切目标方案。
6. **加诊断手段**: 运行时计数器 + OLED/串口观察。

## 开发操作流程

### 修改引脚/外设配置
1. 编辑 `board/sys_config/G3507.syscfg`
2. 运行 SysConfig CLI 重生成 `ti_msp_dl_config.c/h`
3. 检查生成代码（勿手改生成文件）

> I2C0 实例在 syscfg 中名为 `MPU6050_JY61P_Tracking`（历史命名，未重命名以免牵连重生成），
> 现由 JY61P / MPU6050 / 感为灰度 / Yahboom 循线模块共用。SR04 的 GPIO 定义为历史残留，未使用。

### 编译
```
<UV4.exe> -b "NUEDC2025_MSPM0G3507.uvprojx" -o "build_log.txt"
```
> 运行前 `cd` 到 `project/keil/`，UV4.exe 路径需向用户确认。

MDK 6 使用 `project/keil/NUEDC2025_MSPM0G3507.csolution.yml`，通过 Keil Studio Pack
或 `cbuild` 构建。新增/删除编译输入后运行 `python tools/check_keil_project_sync.py`。

### Keil 工程维护
- `.uvprojx` 是 XML，可手编增删 `<File>` 条目；漏加会在链接阶段报 `L6218E`。
- 已删文件引用须同步维护 MDK 5、MDK 6 与 CCS 三套工程。
- MDK 5 与 MDK 6 的源码/库输入必须通过 `tools/check_keil_project_sync.py` 核对一致。
- CCS projectspec 已改用仓库内 SDK 并修正已知元数据，但重新构建验证前不要宣称 CCS 可用。

### 提交
- 格式: `type: 中文描述`（`feat` | `fix` | `docs` | `refactor`）

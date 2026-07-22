# MSPM0G3507 车载固件

基于 `TI MSPM0G3507` 的分层车载固件工程。

> **当前状态**：原 2025E「自动寻迹小车 + 二维激光瞄准云台」任务框架与二维云台/瞄准子系统
> 已整体移除，新任务待定。`app` 现为极简启动骨架（SysConfig 初始化后进入空循环）；下层
> bsp / middleware / core 能力保留但暂无调用者，等待按新需求重建。

## 硬件构成

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 主控 | MSPM0G3507 (LQFP-64) | — | — |
| 底盘轮 | 直流电机 ×2 (TB6612FNG) + 霍尔编码器 | PWM / GPIO | `bsp/motor` |
| 姿态 | JY61P 六轴 (WIT 协议) | I2C0 | `bsp/imu` (wit_sdk) |
| 姿态 | MPU6050 (DMP) | I2C0（与 JY61P 共总线） | `bsp/mpu6050` |
| 视觉 | K230 (CanMV) | UART2 | `bsp/canmv` |
| 循迹 | 8 路灰度传感器 | GPIO | `bsp/grayscale_sensor` |
| 人机 | OLED + 独立按键 | I2C1 / GPIO | `bsp/oled`, `bsp/key` |

## 分层架构

```text
app ─► middleware ─► bsp
          │
          └────────► core（纯计算）
```

- `app`：固件入口。当前仅极简启动骨架，待按新需求重建。
- `core`：PID、运动学等纯计算能力（`pid` / `kinematics` / `common`）。
- `middleware`：组合 core 与 BSP 的系统能力（`chassis` / `line_follow` / `line_tracking` / `ui` / `fault`）。
- `bsp`：直接面向板级外设的驱动（见上表 + `time` / `common` / `debug_uart`）。
- `board`：SysConfig 源文件与生成代码、启动/链接资源。

依赖单向：`core` 不读硬件、不调用 middleware；`app` 只编排、不重复实现算法。

## 说明

- JY61P 与 MPU6050 共用 I2C0 总线，同时使用需分时（见 `bsp/imu` 的挂起接口）。
- 不可同时使用无线调试器的虚拟串口和 Ex Uart。
- 修改引脚/外设需改 `board/sys_config/G3507.syscfg` 后用 SysConfig CLI 重新生成，生成代码不手改。
- 详见 `docs/architecture.md`、`docs/project-structure.md` 与 `docs/interfaces/`。

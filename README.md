# MSPM0G3507 车载固件

基于 `TI MSPM0G3507` 的分层车载固件工程。

> **当前状态**：原 2025E 二维云台/瞄准子系统已整体移除；`app` 已重建为菜单驱动的裸机
> 协作式调度框架。当前可从 OLED 菜单运行 GPIO 灰度循迹测试，以及 JY61P、MPU6050、
> 两种灰度传感器、TB6612、双轮编码器、速度 PID、占空比扫描和蓝牙串口等设备检查任务。
> 现阶段重点是底盘测速/速度闭环整定与各外设上板验证。

## 硬件构成

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 主控 | MSPM0G3507 (LQFP-64) | — | — |
| 底盘轮 | 直流电机 ×2 (TB6612FNG) + 霍尔编码器 | PWM / GPIO | `bsp/motor` |
| 姿态 | JY61P 六轴 (WIT 协议) | I2C0 | `bsp/imu` (wit_sdk) |
| 姿态 | MPU6050 (DMP) | I2C0（与 JY61P 共总线） | `bsp/mpu6050` |
| 循迹 | 8 路灰度传感器 | GPIO | `bsp/grayscale_sensor` |
| 循迹 | 感为 8 路灰度传感器 | I2C0（与两种 IMU 共总线） | `bsp/ganv_gray` |
| 人机 | OLED + 独立按键 | I2C1 / GPIO | `bsp/oled`, `bsp/key` |
| 通信 | 蓝牙 / 调试遥测 | UART0 9600 / UART1 115200 | `bsp/bluetooth`, `bsp/debug_uart` |

## 分层架构

```text
app ─► middleware ─► bsp
          │
          └────────► core（纯计算）
```

- `app`：初始化、协作式调度器、状态机、菜单树与具体测试任务。
- `core`：PID、滤波和运动学等纯计算能力（`pid` / `filter` / `kinematics` / `common`）。
- `middleware`：组合 core 与 BSP 的系统能力（`chassis` / `line_follow` / `ui` / `fault`）。
- `bsp`：直接面向板级外设的驱动（见上表 + `time` / `common`）。
- `board`：SysConfig 源文件与生成代码、启动/链接资源。

依赖单向：`core` 不读硬件、不调用 middleware；`app` 只编排、不重复实现算法。

## 说明

- JY61P、MPU6050 与感为灰度共用 I2C0；阻塞式 MPU6050/感为任务运行时通过
  `JY61P_I2C_SetSuspended()` 与 JY61P 分时。
- 不可同时使用无线调试器的虚拟串口和 Ex Uart。
- 修改引脚/外设需改 `board/sys_config/G3507.syscfg` 后用 SysConfig CLI 重新生成，生成代码不手改。
- 当前 Keil 工程可用；CCS projectspec 尚有已知元数据问题，详见 `docs/build-guide.md`。
- 详见 `docs/architecture.md`、`docs/app-design.md`、`docs/project-structure.md` 与 `docs/interfaces/`。

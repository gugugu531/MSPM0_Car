# bsp/mpu6050 接口说明

## 模块职责

`bsp/mpu6050` 是地址 `0x68` 的 MPU6050 阻塞式 I2C0 驱动，提供两条用途不同的路径：

- **基础模式**：`MPU6050_Init()` 配置 ±2g、±250°/s 和 X 轴陀螺 PLL 时钟；可读取原始
  六轴数据，或直接取得加速度、角速度、温度及静态 pitch/roll。
- **DMP 模式**：`MPU6050_DmpInitialize()` 加载 MotionApps v2.0 固件，随后从 FIFO 读取
  四元数解算的 yaw/pitch/roll 与角速度。

两种模式不是可随意交叉调用的等价视图。DMP 初始化会把陀螺量程切换为 ±2000°/s，基础物理量
接口仍按 ±250°/s 换算；执行 DMP 初始化后若需返回基础模式，必须重新调用 `MPU6050_Init()`。

## 总线与调用约束

- SysConfig 实例：`MPU6050_JY61P_Tracking_INST`，`PA0=SDA / PA1=SCL`，400 kHz。
- I2C0 同时连接 JY61P (`0x50`)、感为灰度 (`0x4F`) 和 Yahboom 循线 (`0x12`)。
- MPU6050 驱动为阻塞式，只能在线程上下文调用，禁止在 ISR 中调用。
- JY61P 使用异步中断驱动。app 在访问 MPU6050 前必须执行
  `JY61P_I2C_SetSuspended(true)`，结束后执行 `JY61P_I2C_SetSuspended(false)`；仅地址不同
  不能防止两个控制器状态机同时发起事务。

## 基础模式接口

```c
BSP_STATUS MPU6050_Init(void);
bool MPU6050_TestConnection(void);
BSP_STATUS MPU6050_GetMotion6(MPU6050_MOTION6 *out);
BSP_STATUS MPU6050_GetMeasurement(MPU6050_MEASUREMENT *out);
```

- `Init` 退出睡眠并建立基础模式量程。
- `TestConnection` 读取 `WHO_AM_I[6:1]`，期望值为 `0x34`。
- `GetMotion6` 返回加速度计与陀螺仪原始 LSB。驱动实际一次读取 14 字节，跳过中间温度值。
- `GetMeasurement` 用同一次 14 字节快照输出以下物理量，避免各通道跨采样：

| 字段 | 单位/换算 |
|---|---|
| `accel_*_mps2` | `raw / 16384 × 9.80665`，单位 m/s² |
| `accel_magnitude_mps2` | 三轴加速度向量模，单位 m/s² |
| `gyro_*_deg_s` | `raw / 131`，单位 °/s，仅适用 ±250°/s |
| `temperature_c` | `raw / 340 + 36.53`，单位 °C |
| `pitch_deg`, `roll_deg` | 仅由重力方向估算，单位 ° |

静态倾角不能给出绝对 yaw，并且在车辆加减速、振动或碰撞时会把线性加速度误认为重力方向；
它适合接线、轴向和静态姿态检查，不应直接当作动态融合姿态。

## DMP 接口

```c
uint8_t MPU6050_DmpInitialize(void);
void MPU6050_SetDMPEnabled(bool enable);
BSP_STATUS MPU6050_DmpGetAttitude(MPU6050_ATTITUDE *out);
```

`MPU6050_DmpInitialize()` 为秒级阻塞初始化，返回 `0` 表示成功，`1` 表示固件写入校验失败，
`2` 表示配置校验失败，`3` 表示等待 FIFO 超时。成功后还需调用
`MPU6050_SetDMPEnabled(true)`。`MPU6050_DmpGetAttitude()` 轮询 FIFO；无完整新包或 FIFO
溢出复位时返回 `BSP_STATUS_NOT_READY`，成功时输出角度（°）与角速度（°/s）。

`MPU6050_RunDmpTest()` 是独立 bring-up 工具：连接或初始化失败时返回，成功后持续从调试串口
输出姿态且不再返回。调用者必须预先挂起 JY61P。目前 app 的协作式任务没有直接使用该入口。

## Device Check

路径：`Main Menu -> Device Check -> Gyro MPU6050`。

- 进入页面时挂起 JY61P、初始化 MPU6050 基础模式；退出时恢复 JY61P。
- 第 1 页显示三轴加速度、合加速度和温度。
- 第 2 页显示三轴角速度及静态 pitch/roll；短按 UP 或 DOWN 切页。
- `conn FAIL` 表示初始化或 WHO_AM_I 检查失败，`read FAIL` 表示连续数据读取失败。
- 静止平放时合加速度应接近 `9.81 m/s²`，角速度应接近零；实际零偏需以实测为准。

## 实现依据

- [TDK MPU-6000/MPU-6050 Register Map and Descriptions](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [TDK MPU-6000/MPU-6050 Product Specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf)
- [Jeff Rowberg i2cdevlib MPU6050 实现](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)

DMP 固件与初始化流程源自 i2cdevlib 的 MotionApps v2.0 移植；基础量程、灵敏度和温度换算以
TDK 官方寄存器手册为准。

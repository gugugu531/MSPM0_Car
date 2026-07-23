# bsp/ganv_gray 接口说明

## 模块职责

`bsp/ganv_gray` 是感为(GANV)8 路灰度传感器的 I2C 驱动，主用途是一次读回 8 路数字量，辅以
ping / 版本 / 错误 / 重启等诊断命令。

- 复用已由 SysConfig 初始化的 **I2C0** 总线（实例 `MPU6050_JY61P_Tracking_INST`，
  `PA0=SDA / PA1=SCL`），作为总线上地址 `0x4F` 的第三个从机，与 MPU6050(`0x68`)、
  JY61P(`0x50`) 不冲突，**无需修改 SysConfig**。
- 底层为**阻塞 I2C**，采用手册方法 2/3 的「写一次命令、反复读」：内部维护命令指针缓存，仅当
  命令变化时才写 1 字节命令（独立事务，带 STOP），连读同一命令直接读 N 字节（START+RX+STOP）、
  跳过写命令；软件重启为纯写 1 字节命令。**每次读都重发命令会触发写阶段隔次 NACK**（见使用约束）。
- 仅可在线程上下文调用，**不可在 ISR 内调用**；与 JY61P 异步中断驱动同时主动发起事务会在
  总线上冲突，须由上层分时（与 MPU6050 自检同法：进挂起 / 出恢复对方）。

> 校准为设备端**按键操作**（白 / 黑场按按钮），I2C 侧无校准命令，本驱动不涉及校准。

## 地址与命令

7 位从机地址：高 5bit 软件地址位出厂为 `0b10011`，低 2bit 由板上 `AD1`/`AD0` 跳线帽决定。

```c
/* AD1=AD0=1(双跳线帽) → 0x4F；都不装 → 0x4C。改跳线帽后覆盖此宏即可。 */
#ifndef GANV_GRAY_I2C_ADDR_7BIT
#define GANV_GRAY_I2C_ADDR_7BIT 0x4FU
#endif
```

命令符（见手册 7.17，本驱动实现其中数字量与诊断部分）：

| 功能 | 命令符 | 说明 |
|---|---|---|
| 读数字量 | `0xDD` | 读 1 字节，`bit0`=第 1 路 … `bit7`=第 8 路，置 1=检测到 |
| ping | `0xAA` | 期望应答 `0x66` 表示在线 |
| 固件版本 | `0xC1` | 高 4bit.低 4bit，如 `0x3E` 表示 V3.14 |
| 错误寄存器 | `0xDE` | `bit1`=按键长期短路，`bit0`=对管过曝；读后设备自动清零 |
| 软件重启 | `0xC0` | 纯写命令，无应答 |

## 接口

所有接口返回 `BSP_STATUS`（`BSP_STATUS_OK` 为成功）。

```c
BSP_STATUS GanvGray_Init(void);
```

初始化并与传感器同步：有限次（默认 100 次，间隔 1ms）ping 等待其上电就绪。收到 `0x66` 返回
`BSP_STATUS_OK`；未就绪返回 `BSP_STATUS_TIMEOUT`。I2C0 外设本体由 SysConfig 初始化，本函数只做
上电同步，不重配外设。灰度为可选外设，建议调用方将失败作为可恢复处理，不进入致命停机。

```c
BSP_STATUS GanvGray_Ping(void);
```

ping 探测在线（`0xAA` → `0x66`）。应答值不符返回 `BSP_STATUS_NOT_READY`。

```c
BSP_STATUS GanvGray_ReadDigital(uint8_t *mask);
```

读取 8 路数字量位掩码（`0xDD`）。`mask` 的 `bit0`=第 1 路 … `bit7`=第 8 路，置 1 表示该路检测
到。`mask` 为 `NULL` 时返回 `BSP_STATUS_NULL`。

```c
BSP_STATUS GanvGray_ReadVersion(uint8_t *version);
BSP_STATUS GanvGray_ReadError(uint8_t *err);
BSP_STATUS GanvGray_Reboot(void);
```

分别为读固件版本（`0xC1`）、读错误寄存器（`0xDE`，读后设备自清零）、触发软件重启（`0xC0`）。
重启后设备需重新就绪，调用方应随后调用 `GanvGray_Init` / `GanvGray_Ping`。

## 使用约束

- 只能在线程上下文调用，禁止在中断内调用（阻塞 I2C）。
- 与同总线的 MPU6050 / JY61P 主动事务须分时，避免总线冲突。
- 数字量的 bit 顺序为设备定义的第 1..8 路探头，物理安装方向的通道映射由上层巡线逻辑处理。
- **传感器对高频写命令会间歇 NACK**：手册方法 2/3 要求「写一次、反复读」。驱动用命令指针
  缓存实现——连读同一命令不重发命令，故高频读稳定；仅切换命令时才写，写命令保留
  `GANV_GRAY_RETRY` 次重试兜底。**切勿改成每次读都重发命令**（曾因此出现 online/offline
  各半的隔次交替）。若写阶段仍频繁失败，检查 I2C 总线上拉——三从机共总线时手册建议外置约
  10K 上拉到 3.3V。

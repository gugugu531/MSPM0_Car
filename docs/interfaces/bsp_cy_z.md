# bsp/cy_z 接口说明

`bsp/cy_z` 适配创源启明 CY-Z 串口陀螺仪。模块通过 115200 8N1 输出积分角度与滤波角速度，
驱动独占 `CY_Z/UART3`，UART ISR 只负责把 RX 字节放入环形缓冲，协议解析在任务上下文执行。

## 接线

- CY-Z TX → MSPM0 PB13（`CY_Z/UART3 RX`）
- CY-Z RX ← MSPM0 PB2（`CY_Z/UART3 TX`，需要使用清零/校准命令时连接）
- 两板 GND 共地
- VCC 按模块丝印或厂家说明供电，不要仅凭串口电平推断供电电压

CY-Z 使用独立 UART3，不占用 UART0 蓝牙和 UART1 调试/K230 链路。诊断信息显示在 OLED 上。

## 数据帧

固定 16 字节、小端：

```text
AA 55  Seq:u16  AngleDeg:f32  GyroDps:f32  CRC16:u16  55 AA
```

CRC 为 CRC-16/MODBUS，仅覆盖 `Seq + AngleDeg + GyroDps` 共 10 字节。解析器使用滑动窗口，
丢字节或插入噪声后可重新同步，并分别记录有效帧、ACK、CRC 错误和丢弃字节数。

## 测试

上电进入 `Device Check > Gyro CY-Z`：

- OLED 应持续显示 `angle`、`gyro`，`frame` 计数递增；静止时角速度应接近 0。
- `UP`：角度清零，发送时保持模块静止。
- `DOWN`：重新估计零偏，发送后约 2 秒保持静止。
- `ENTER`：主动请求一帧，便于模块处于查询模式时测试。
- `BACK`：退出测试。

退出页面会关闭 UART3 RX 中断；再次进入时清空旧数据并重新启用，避免模块持续上报影响其他任务。

命令执行结果显示为 `ack 命令/结果`；结果 `00` 为成功，`01` 表示模块检测到未静止。

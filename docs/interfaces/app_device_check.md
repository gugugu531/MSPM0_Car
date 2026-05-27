# app/app_device_check 接口说明

## 模块职责

`app/app_device_check` 提供应用层设备检查页面，用于在完整题目流程前快速确认主要硬件链路。

当前检查页面：

- 电机：停止、左轮转、右轮转、双轮转。切换测试项目时会清空编码器计数、速度和距离。
- yaw 步进电机：短按释放后立即正方向运行 1 秒，再负方向运行 1 秒。
- pitch 步进电机：短按释放后立即正方向运行 1 秒，再负方向运行 1 秒。
- 灰度：实时显示 8 路传感器二进制状态，Device Check 页面按 bit0 到 bit7 的顺序显示。
- IMU：实时显示 roll、pitch、yaw 和三轴角速度，并保留接收状态页。

## 公开接口

### `void AppDeviceCheck_Run(void)`

进入设备检查页面。双击切换测试模块，单击切换或触发当前模块内测试项目，长按返回顶层菜单并停止底盘和步进电机输出。

yaw/pitch 步进电机页面为了降低测试动作延迟，会使用 `Key_IsShortRelease()` 在释放消抖完成后立即触发点动，不等待普通短按的双击窗口超时。

### `void AppDeviceCheck_ProcessImuByte(uint8_t byte)`

处理 UART0 接收到的 IMU 字节。该函数由 `UART0_IRQHandler()` 调用，内部按 JY61P 标准 11 字节子帧滑动组包和校验，通过后调用 `GYROSCOPE_DATA_Decoder()` 更新 WitMotion 数据。

该函数只负责调试串口字节处理，不主动打开 UART 中断；UART 初始化和中断使能由 `app/main.c` 完成。当前 UART0 接收只使用中断路径，`app/main.c` 会将 RX FIFO 中断阈值设置为 1 字节，进入 `UART0_IRQHandler()` 后一次性排空当前 RX FIFO。

设备检查中 IMU 拆分为姿态角、角速度、加速度和状态页。状态页显示 UART0 收到的字节数、有效帧数、无效帧数和最近一次有效子帧类型，便于上板判断故障在串口输入、帧校验还是数据显示链路。

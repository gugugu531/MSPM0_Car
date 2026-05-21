# app/app_device_check 接口说明

## 模块职责

`app/app_device_check` 提供应用层设备检查页面，用于在完整题目流程前快速确认主要硬件链路。

当前检查页面：

- 底盘：左右轮占空比、编码器速度、编码器距离。
- 云台：yaw/pitch 估计角度和速度。
- 灰度：8 路传感器 mask、有效通道数量、半线和十字粗略判定。
- 视觉：CanMV 激光点和矩形目标状态、坐标。
- IMU：姿态角和接收到的有效帧数量。

## 公开接口

### `void AppDeviceCheck_Run(void)`

进入设备检查页面。短按切换页面，长按返回顶层菜单。

### `void AppDeviceCheck_ProcessImuByte(uint8_t byte)`

处理 UART0 接收到的 IMU 字节。该函数由 `UART0_IRQHandler()` 调用，内部组 33 字节帧并校验三段和校验，通过后调用 `GYROSCOPE_DATA_Decoder()` 更新 WitMotion 数据。

该函数只负责调试串口字节处理，不主动打开 UART 中断；UART 初始化和中断使能由 `app/main.c` 完成。

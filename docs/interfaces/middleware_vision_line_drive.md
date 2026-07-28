# middleware/vision_line_drive 接口说明

> 当前模块尚未进行整车视觉循迹实测。下述增益、符号、超时和基础占空比均为初始值，上车前保持
> K230 `UART_ENABLED = False`，首次联调必须架空车轮确认负反馈与失效停车。

`vision_line_drive` 解析 K230 红线视觉帧，并实现 `WAIT_IMU → STARTUP_RATE → TRACK` 控制。启动 1 秒
使用 `gz=0` 角速度闭环；稳定阶段每拍联合位置与方向偏差生成目标角速度，再由 IMU 角速度内环产生
左右轮差速。K230 只上报红色竖线；地图中的红色横支线在视觉端作为宽段干扰剔除。

- `VisionLineDrive_Init()`：复位协议解析、统计、PID 和阶段状态。
- `VisionLineDrive_Update(dt)`：排空 UART RX、检查帧龄/IMU、计算并写一次底盘。
- `VisionLineDrive_Stop()`：清控制动态并把双轮指令归零。
- `VisionLineDrive_GetOutput()`：返回阶段、帧质量、误差、角速度和轮占空比。
- `VisionLineDrive_IngestByte()`：协议单字节入口，供离线测试；正常运行无需直接调用。

协议和标定流程见 `docs/vision-red-line-follow.md`。

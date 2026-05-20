# 变更记录

## 未发布

- 重写 `bsp/motor/hall_encoder`：公开 `HallEncoder_*` 接口，使用明确的编码器物理参数宏提供采样计数、方向、速度和距离估计。
- 为霍尔编码器补充中文接口文档 `docs/interfaces/bsp_hall_encoder.md`，并记录后续上层从旧 `Encoder_*` 接口迁移。
- 重写 `bsp/motor/tb6612fng`：移除旧 `Motor_*` 类型和速度换算接口，新增 `TB6612FNG_*` 通道接口，使用百分比占空比控制 IN1/IN2/PWM 输出。
- 为 TB6612FNG 驱动补充可覆盖硬件映射宏和中文接口文档 `docs/interfaces/bsp_tb6612fng.md`。
- 将 TB6612FNG 输出状态查询接口命名为 `TB6612FNG_GetOutputStatus()`，避免 `GetOutput()` 语义过宽。
- 将 `bsp/tracking_sensor` 重命名并重写为 `bsp/grayscale_sensor`，公开 `GrayscaleSensor_Read()`、`GrayscaleSensor_ReadMask()` 和 `GrayscaleSensor_ReadSingle()`。
- 为 8 路光敏灰度传感器补充可覆盖引脚宏和中文接口文档 `docs/interfaces/bsp_grayscale_sensor.md`。
- 重写 `bsp/key`：公开类型改为 `KEY_ID`、`KEY_EVENT`，硬件映射改为可覆盖的 `KEY1_PORT`、`KEY1_PIN`、`KEY1_ACTIVE_LOW` 宏，并用内部配置表预留多按键扩展。
- 将按键正式说明集中到 `docs/interfaces/bsp_key.md`，移除 `bsp/key` 目录下的旧说明文档。
- 新增 `bsp/time`，统一提供 `BSP_Time_GetMs()`、`BSP_DelayUs()` 和 `BSP_DelayMs()`。
- 删除 `middleware/system/delay.*` 和 `middleware/runtime/system_time.h`，系统时间与阻塞延时下沉到 BSP 层。
- 将按键消抖和 OLED 软件 I2C 延时改为直接使用 `bsp/time`，移除对应 provider 注入接口。
- 重写 `bsp/common` 公共契约：统一使用 SDK 风格的全大写类型名 `BSP_STATUS`、`BSP_POINT2F`、`BSP_ATTITUDE2F`。
- 将 CanMV 状态码从 `bsp/common` 移入 `bsp/laser/laser_usart.h`，由对接 CanMV 的驱动自行维护。
- 将工程源码重构为 `app`、`core`、`middleware`、`bsp` 四层结构。
- 将自维护源码和目录统一为小写加下划线命名。
- 将 SysConfig 和启动资源迁入 `board`，IDE 工程元数据迁入 `project`，脚本目录统一为 `tools`。
- 移除 BSP 对上层的反向依赖：按键和 OLED 改为依赖 `bsp/time`，步进电机状态更新时间改为显式传参，视觉和 IMU 状态由 BSP 接口导出。
- 更新 CCS 和 Keil 工程路径，使其指向新的分层源码树。

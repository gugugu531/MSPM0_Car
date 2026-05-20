# Todo

- 后续根据真实系统时钟复核 `BSP_TIME_CPUCLK_HZ`，必要时改为从时钟配置自动派生。
- 后续如增加第二个实体按键，在 `KEY_ID`、硬件映射宏和 `bsp/key` 内部配置表中同步扩展。
- 后续整体框架确认后，将 CCS、Keil 和编辑器配置中的 `bsp/tracking_sensor` 路径统一更新为 `bsp/grayscale_sensor`。
- 后续构建具体 BSP 外设时，继续按 SDK 风格统一公开类型和枚举成员命名。
- 在确认本地 MSPM0 SDK 环境变量后运行 CCS/ticlang 构建。
- 在确认本地 Keil MDK/ArmClang 安装路径后运行 Keil 构建。
- 继续把 `middleware/runtime` 中的全局运行时状态收敛为显式上下文结构。
- 如需要更彻底的 API 统一，后续可把公开 C 函数也改为 snake_case。
- 复查 `middleware/system/error_handler.c` 对 OLED 的直接依赖，必要时改成显示回调。
- 上板验证启动页、任务菜单、设备自检、UART0 IMU、UART2 视觉和 SysTick 按键扫描。

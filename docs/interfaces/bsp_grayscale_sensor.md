# bsp/grayscale_sensor 接口说明

## 模块职责

`bsp/grayscale_sensor` 负责读取 8 路光敏灰度传感器的数字量输出。当前硬件接口读取的是传感器模块比较器输出后的数字电平，不是 ADC 模拟灰度值。

该模块属于 BSP 层，只依赖 SysConfig 生成的 GPIO 宏和 TI DriverLib GPIO 读取接口，不依赖 `app`、`core` 或 `middleware`。

## 命名说明

模块命名为 `grayscale_sensor`，强调外设本身是“光敏灰度传感器”。完整巡线控制由上层
`middleware/line_follow` 负责。

## 通道顺序

公开逻辑通道保持兼容原 `Digital[8]` 数组语义：

| 逻辑通道 | 数组下标 | 默认硬件输入 |
| --- | --- | --- |
| `GRAYSCALE_SENSOR_CHANNEL_0` | `digital_array[0]` | `Tracking_Tracking_8` |
| `GRAYSCALE_SENSOR_CHANNEL_1` | `digital_array[1]` | `Tracking_Tracking_7` |
| `GRAYSCALE_SENSOR_CHANNEL_2` | `digital_array[2]` | `Tracking_Tracking_6` |
| `GRAYSCALE_SENSOR_CHANNEL_3` | `digital_array[3]` | `Tracking_Tracking_5` |
| `GRAYSCALE_SENSOR_CHANNEL_4` | `digital_array[4]` | `Tracking_Tracking_4` |
| `GRAYSCALE_SENSOR_CHANNEL_5` | `digital_array[5]` | `Tracking_Tracking_3` |
| `GRAYSCALE_SENSOR_CHANNEL_6` | `digital_array[6]` | `Tracking_Tracking_2` |
| `GRAYSCALE_SENSOR_CHANNEL_7` | `digital_array[7]` | `Tracking_Tracking_1` |

这样重写后不会改变现有巡线算法看到的数组顺序。

## 硬件映射宏

`grayscale_sensor.h` 提供 `GRAYSCALE_SENSOR_1_PORT/PIN` 到 `GRAYSCALE_SENSOR_8_PORT/PIN` 的可覆盖宏，默认映射到 SysConfig 生成的 `Tracking_Tracking_x` 宏。

板级输入反相由统一宏控制：

```c
#define GRAYSCALE_SENSOR_INVERT_INPUT 1U
```

当前实机已经通过 Device Check 确认：接口返回 `0` 表示检测到黑线，返回 `1` 表示未检测到
黑线。若更换传感器或接线，应先用自检页确认该语义，再调整反相配置。

## 公开类型

```c
#define GRAYSCALE_SENSOR_CHANNEL_COUNT 8U

typedef enum {
    GRAYSCALE_SENSOR_CHANNEL_0 = 0,
    GRAYSCALE_SENSOR_CHANNEL_1,
    GRAYSCALE_SENSOR_CHANNEL_2,
    GRAYSCALE_SENSOR_CHANNEL_3,
    GRAYSCALE_SENSOR_CHANNEL_4,
    GRAYSCALE_SENSOR_CHANNEL_5,
    GRAYSCALE_SENSOR_CHANNEL_6,
    GRAYSCALE_SENSOR_CHANNEL_7,
    GRAYSCALE_SENSOR_CHANNEL_MAX
} GRAYSCALE_SENSOR_CHANNEL;
```

## 公开接口

### `void GrayscaleSensor_Read(uint8_t digital_array[GRAYSCALE_SENSOR_CHANNEL_COUNT])`

读取全部 8 路数字量。当前实机数组元素语义为：

- `0`：检测到黑线
- `1`：未检测到黑线

传入空指针时函数直接返回。

### `uint8_t GrayscaleSensor_ReadMask(void)`

读取全部 8 路并返回 bit mask。bit0 对应 `GRAYSCALE_SENSOR_CHANNEL_0`，bit7 对应
`GRAYSCALE_SENSOR_CHANNEL_7`；置 1 表示对应通道未检测到黑线。

### `uint8_t GrayscaleSensor_ReadSingle(GRAYSCALE_SENSOR_CHANNEL channel)`

读取单个逻辑通道。当前实机上 0 表示检测到黑线，1 表示未检测到黑线；非法通道也返回 0，
因此调用方不得用该接口的非法参数结果判断黑线。

## 兼容接口

当前保留一条兼容宏，便于旧代码短期迁移：

```c
#define TrackingSensor_Read(digital_array) GrayscaleSensor_Read(digital_array)
```

新代码应直接使用 `GrayscaleSensor_*` 接口。

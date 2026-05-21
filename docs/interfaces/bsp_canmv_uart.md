# bsp/canmv/canmv_uart 接口说明

## 模块职责

`bsp/canmv/canmv_uart` 负责 CanMV 视觉模块的 UART 收发和帧解析。该模块只处理字节流、帧边界、目标数据解析和 CanMV 状态，不负责视觉业务决策、目标选择或坐标转换。

## 帧格式

当前协议使用固定帧头和帧尾：

```c
#define CANMV_FRAME_START 0x12U
#define CANMV_FRAME_END   0x5BU
```

已知数据段：

- 激光点数据：从 `CANMV_LASER_BEGIN` 开始，共 `CANMV_LASER_BYTE_COUNT` 字节，解析为 4 个 `uint16_t`
- 矩形点数据：从 `CANMV_RECT_BEGIN` 开始，共 `CANMV_RECT_BYTE_COUNT` 字节，解析为 8 个 `uint16_t`

每个 `uint16_t` 按高字节在前、低字节在后的方式组合。

## 硬件映射宏

```c
#define CANMV_UART_INST UART2
#define CANMV_UART_IRQN UART2_INT_IRQn
#define CANMV_RX_BUFFER_LEN 100U
#define CANMV_MIN_FRAME_LEN (CANMV_RECT_BEGIN + CANMV_RECT_BYTE_COUNT + 1U)
```

如果后续 UART 实例或缓冲区大小变化，应通过覆盖这些宏调整。

## 公开类型

### `CANMV_TARGET`

```c
typedef enum {
    CANMV_TARGET_LASER = 0,
    CANMV_TARGET_RECT,
    CANMV_TARGET_MAX
} CANMV_TARGET;
```

### `CANMV_STATUS`

```c
typedef enum {
    CANMV_STATUS_INIT = -1,
    CANMV_STATUS_OK = 0,
    CANMV_STATUS_NOT_FOUND = 1,
    CANMV_STATUS_LOST = 2,
    CANMV_STATUS_FRAME_DROP = 3,
} CANMV_STATUS;
```

状态说明：

- `CANMV_STATUS_INIT`：尚未收到有效帧。
- `CANMV_STATUS_OK`：目标数据有效。
- `CANMV_STATUS_NOT_FOUND`：从未识别到目标。
- `CANMV_STATUS_LOST`：曾经识别到目标，但当前帧目标为空。
- `CANMV_STATUS_FRAME_DROP`：帧格式异常、接收溢出或非帧数据进入解析器。

### `CANMV_TARGET_DATA`

```c
typedef struct {
    uint16_t value[CANMV_TARGET_VALUE_CAPACITY];
    uint8_t count;
    CANMV_STATUS status;
} CANMV_TARGET_DATA;
```

`value` 保存目标数据，`count` 表示当前目标实际有效的 `uint16_t` 数量，`status` 表示该目标状态。

## 公开接口

### `BSP_STATUS CanMvUart_Init(void)`

清空接收状态和目标数据，初始化目标状态为 `CANMV_STATUS_INIT`，并使能 CanMV UART 中断。

### `void CanMvUart_ProcessByte(uint8_t byte)`

处理一个输入字节。该接口只做协议解析，不直接读取 UART 硬件，便于后续统一中断分发或测试代码喂入数据。

### `void CanMvUart_ProcessRx(void)`

从 `CANMV_UART_INST` 读取一个字节，并调用 `CanMvUart_ProcessByte()`。

### `BSP_STATUS CanMvUart_SendByte(uint8_t byte)`

等待 UART 空闲后发送一个字节。

### `BSP_STATUS CanMvUart_SendString(const char *str)`

发送以 `\0` 结尾的字符串。`str == NULL` 时返回 `BSP_STATUS_NULL`。

### `CANMV_STATUS CanMvUart_GetStatus(CANMV_TARGET target)`

返回指定目标状态。非法目标返回 `CANMV_STATUS_INIT`。

### `uint8_t CanMvUart_GetData(CANMV_TARGET target, uint16_t *out, uint8_t max_count)`

将指定目标数据复制到调用方缓冲区，返回实际复制数量。非法目标、空指针或 `max_count == 0` 时返回 `0`。

### `const CANMV_TARGET_DATA *CanMvUart_GetTargetData(CANMV_TARGET target)`

返回指定目标内部数据的只读指针。非法目标返回 `NULL`。

## 对接说明

旧 `Laser_*`、`Rect_*`、`CanMV_Process()` 和全局视觉状态接口已不再作为当前应用层入口使用。新的视觉数据读取统一通过 `CanMvUart_*` 接口完成。

# API 迁移与旧接口退役记录

> 逐层核对/重写期间, 用 strangler(渐进替换)法引入干净 API: 先 **新增** 干净接口(非破坏、
> 立即可编译), 待上层消费者逐个切换后再 **删除** 旧接口。本文件记录每个模块的
> "旧 API → 新 API" 映射、调用点、以及退役状态, 作为删除旧接口的检查清单。
>
> 代码中旧接口以 `@deprecated` 标注; 删除前对照本表确认所有调用点已迁移。

## 状态图例
- 🟡 新旧并存 (新 API 已加, 旧 API 待迁移调用点)
- 🟢 已全部迁移, 旧 API 可删
- ⚪ 已删除

---

## bsp/canmv — 视觉角度接口

引入 commit: (本次) · 状态: 🟡 新旧并存

### 新 API (已加, `canmv_uart.h/.c`)
```c
typedef struct { float yaw_deg, pitch_deg; CANMV_STATUS status; bool valid; uint32_t frame_id; } CANMV_ANGLE;
CANMV_ANGLE CanMvUart_GetAngle(void);
bool        CanMvUart_TakeNewAngle(uint32_t *token, CANMV_ANGLE *out);
uint32_t    CanMvUart_GetAngleFrameCount(void);
uint32_t    CanMvUart_GetRxByteCount(void);
uint32_t    CanMvUart_GetValidFrameCount(void);
uint32_t    CanMvUart_GetDropCount(void);
uint8_t     CanMvUart_GetLastByte(void);
```

### 旧 API → 新 API 映射
| 旧接口 (`@deprecated`) | 新接口 | 说明 |
|------|------|------|
| `g_canmv_uart_angle_frame_count`(裸全局) | `CanMvUart_TakeNewAngle()` / `GetAngleFrameCount()` | 判新帧不再自持 last_vf |
| `g_canmv_uart_rx_byte_count` | `CanMvUart_GetRxByteCount()` | |
| `g_canmv_uart_valid_frame_count` | `CanMvUart_GetValidFrameCount()` | |
| `g_canmv_uart_drop_count` | `CanMvUart_GetDropCount()` | |
| `g_canmv_uart_last_byte` | `CanMvUart_GetLastByte()` | |
| `CanMvUart_GetTargetData(ANGLE)` + 手动 `(int16_t)value[]/CANMV_ANGLE_SCALE` | `CanMvUart_GetAngle()` | 解码集中到模块内 |

### 待迁移调用点 (核对时逐个切到新 API)
| 文件 | 读全局帧计数 | 手动解码 | 备注 |
|------|:---:|:---:|------|
| `app/app_e_task.c` | 10 | 0 | 判新帧样板 ×多处 |
| `middleware/gimbal_tracking/gimbal_tracking.c` | 3 | 2 | `ReadAngleError` |
| `middleware/auto_aim/auto_aim.c` | 2 | 2 | `ReadNewVision` |
| `app/app_device_check.c` | 1 | 2 | 视觉诊断页 |

### 退役条件 (全部满足后删除旧 API, 状态→🟢)
- [ ] 上述 4 文件全部改用 `CanMvUart_TakeNewAngle`/`GetAngle`/`GetXxxCount`
- [ ] 删除 `extern volatile g_canmv_uart_*` 五个全局 (改为文件内 static)
- [ ] `CanMvUart_GetTargetData`/`GetData`/`GetStatus` 仅剩坐标帧(LASER/RECT)用途;
      若坐标帧链最终移除, 一并退役

### 关联: 坐标帧(LASER/RECT)死链
- 现状: 像素/矩形跟踪已删, `CANMV_TARGET_LASER/RECT` 无 app 层消费者。
- 决策: 按"子系统暂时全部移植"**保留**解析链; 待任务确定后再评估是否移除
  (`ParseFrame`/`ParseTarget`/`CombineCoordinate`/`s_canmv_parse_config` + 相关 getter)。

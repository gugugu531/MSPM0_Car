# bsp/step_motor 接口说明

摆杆执行器的步进电机开环控制。STEP 脉冲由定时器 PWM 产生，DIR/EN 为普通 GPIO。

## 硬件映射

| 信号 | SysConfig 符号 | 引脚 | 外设 |
|---|---|---|---|
| STEP | `SMotor_INST` + `DL_TIMER_CC_0_INDEX` | PA29 | TIMG6_CCP0 |
| DIR | `SMotor_IO_DIR1_PIN` | PB14 | GPIO |
| EN | `SMotor_IO_EN1_PIN` | PB11 | GPIO |

配套的 `SMotor_QEI`（TIMG8 / PA26 / PA27）当前仅在 SysConfig 中占位保留引脚，
本驱动不消费，摆杆角度闭环反馈需另行实现。

## 时钟约束（改配置前必读）

驱动把速度换算成定时器重载值：

```c
arr = STEP_MOTOR_TIMER_CLOCK_HZ / (step_freq * STEP_MOTOR_TIMER_PRESCALER_FACTOR)
```

其中 `STEP_MOTOR_TIMER_PRESCALER_FACTOR = 32 × 8 × 2 = 512`，即公式**假定定时器
实际时钟为 32MHz / 512 = 62500 Hz**。因此 SysConfig 中 `SMotor` 必须配
`clockDivider = 8`、`clockPrescale = 64`，生成的 `SMotor_INST_CLK_FREQ` 应为 `62500`。

> 二者不一致时不会有编译错误，但转速会整体偏离 —— 例如缺失分频配置时时钟为
> 32MHz，脉冲频率会高出 512 倍，电机直接堵转。改动任一侧后务必核对
> `ti_msp_dl_config.h` 中的 `SMotor_INST_CLK_FREQ`。

## 参数

| 宏 | 默认 | 说明 |
|---|---|---|
| `STEP_MOTOR_STEP_ANGLE_DEG` | 1.8 | 电机固有步距角 |
| `STEP_MOTOR_MICROSTEP` | 32 | 细分数，**须与驱动器拨码一致** |
| `STEP_MOTOR_MAX_SPEED_DEG_S` | 240 | 速度限幅 |
| `STEP_MOTOR_MIN/MAX_POSITION_DEG` | ∓30 | 开环估计位置限位，**电机轴角非摆杆角** |
| `STEP_MOTOR_BEAM_ENABLE_HIGH` | 1 | 使能脚高有效；驱动器低有效时改 0 |
| `STEP_MOTOR_BEAM_POSITIVE_DIR_HIGH` | 1 | 正方向对应 DIR 高电平；上板确认转向后按需翻转 |

## 接口

```c
BSP_STATUS StepMotor_Init(void);
BSP_STATUS StepMotor_SetSpeed(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s);
BSP_STATUS StepMotor_RunFor(STEP_MOTOR_CHANNEL channel, float speed_deg_per_s, uint32_t duration_ms);
BSP_STATUS StepMotor_Stop(STEP_MOTOR_CHANNEL channel);
BSP_STATUS StepMotor_StopAll(void);
BSP_STATUS StepMotor_UpdateState(STEP_MOTOR_CHANNEL channel, uint32_t now_ms);
BSP_STATUS StepMotor_UpdateAllState(uint32_t now_ms);
float      StepMotor_GetSpeed(STEP_MOTOR_CHANNEL channel);
float      StepMotor_GetEstimatedPosition(STEP_MOTOR_CHANNEL channel);
void       StepMotor_ResetEstimatedPosition(STEP_MOTOR_CHANNEL channel);
BSP_STATUS StepMotor_SetPositionLimit(const STEP_MOTOR_POSITION_LIMIT *limit);
STEP_MOTOR_POSITION_LIMIT StepMotor_GetPositionLimit(void);
```

通道枚举当前只有 `STEP_MOTOR_CHANNEL_BEAM`，保留枚举形式便于后续扩展。

## 使用约束

- `StepMotor_SetSpeed()` 内部会先推进一次位置积分，控制拍内直接调用即可。
- **`StepMotor_RunFor()` 是阻塞接口**（内部 `BSP_DelayMs`），只能用于自检/标定，
  不可在控制拍内调用。
- 位置为**速度积分的开环估计**，丢步不会被察觉；限位只能防止指令超程，
  不能替代物理限位开关。
- 位置限位默认值仅为兜底，须按实际连杆减速比与摆杆行程标定后用
  `StepMotor_SetPositionLimit()` 覆盖。

## 移植来源

移植自 `NUEDC_2026/2026H` 的双轴（YAW/PITCH）云台驱动，算法逻辑未改。适配点：

- 裁剪为单通道（本工程只有摆杆一路步进电机）
- 原 PITCH 专用的限位逻辑改为通用位置限位：
  `StepMotor_SetPitchLimit` → `StepMotor_SetPositionLimit`
- 宏前缀 `STEP_MOTOR_PITCH_*` → `STEP_MOTOR_BEAM_*`
- 补齐源工程缺失的定时器分频配置（见上文时钟约束）

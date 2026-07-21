# 重写回归基准 (Rewrite Baseline)

> 本文件是重写/大重构前建立的**行为与标定基准**。任何重写在完成后都必须逐条对照本文件，
> 保证：① 外部行为（每个菜单项做什么）不变；② 所有实车标定常量逐字保留；③ 坐标系与符号
> 约定不变。**代码是行为的实现，标定常量与符号约定是实车辨识结果——后者一旦搬错，只在物理
> 车上暴露，编译无法发现。**
>
> 基准建立时的固件状态：commit `4318b3b` 之后（死代码/Motion/像素路径已清理）。

---

## 0. 使用方式

- 重写**自底向上**（bsp → core → middleware → app），每层完成即用 Keil 编译（0 error/0 warning）。
- 每完成一个模块，回到本文件对应小节逐条打勾。
- **Golden Constants（§6）以源文件为权威**：重写时从被点名的源文件**原样复制**数值，不要凭本文
  记忆重敲（本文列出关键值仅供交叉核对与"是否漏项"检查）。
- 无实车台架时，重写后仍需**上车复验**符号类与时序类参数（§3、§6 标 ★ 者）。

---

## 1. 系统总览与分层

MCU: MSPM0G3507 (Cortex-M0+, 无 FPU, soft-float)。赛题: 2025 电赛 E「简易自瞄装置」
= 自动寻迹小车 + 二维激光瞄准云台。

```
app/        main / launcher / e_task / device_check / calibration / debug_cmd
  │  编排任务状态机、UI 菜单、ISR 分发/路由
  ▼
middleware/ chassis  gimbal  gimbal_tracking  auto_aim
            line_follow  line_tracking  ui  fault
  │  组合 core 算法 + bsp 驱动；运行时状态全为文件级 static 单例
  ▼
core/       pid  kinematics  localization  aim_solver  aim_fusion  common(纯计算)
  ▼
bsp/        tb6612(直流电机) hall_encoder  bldc(F32C无刷) wit_sdk(JY61P IMU)
            canmv(K230串口) grayscale(8路灰度) oled  key  debug_uart  time
  ▼
board/      SysConfig 生成代码 + 启动/链接脚本
```

依赖规则（重写必须维持）：`core` 只依赖标准库+core；`bsp` 只依赖厂商/板级+bsp；
`middleware` 可依赖 core+bsp+middleware；`app` 可依赖任意下层。**core 不读硬件。**

### 外设映射（不可改，硬件决定）

| 子系统 | 器件 | 接口 | 驱动 |
|--------|------|------|------|
| 底盘轮 | 直流×2 + 霍尔编码器 | PWM/GPIO | `bsp/motor` (TB6612) |
| 二维云台 | F32C 无刷×2 (yaw=addr1, pitch=addr2) | UART3 | `bsp/bldc` |
| 姿态 | JY61P (WIT 协议) | I2C0 | `bsp/imu` (wit_sdk) |
| 视觉 | K230 (CanMV) | UART2 | `bsp/canmv` |
| 循迹 | 8 路灰度 | GPIO | `bsp/grayscale_sensor` |
| 蓝牙/上位机 | — | UART0(BlueTooth) / UART1(Debug_Ex) | — |
| 显示/输入 | OLED / 按键 | I2C / GPIO | `bsp/oled`, `bsp/key` |

---

## 2. 中断与并发模型（★ 时序敏感，重写须原样保留）

| ISR | 位置 | 职责 |
|-----|------|------|
| `SysTick_Handler` | app/main.c | `BSP_Time_TickInc`；分频：每 10 tick `Key_Scan`，每 `JY61P_I2C_POLL_PERIOD_TICK` 调 `JY61P_I2C_Poll` |
| `UART0_IRQHandler` | app/main.c | 蓝牙 RX 排空（当前仅计数丢弃） |
| `Debug_Ex_INST_IRQHandler`(UART1) | app/main.c | TX：`DebugUart_TxIsr` 排环形缓冲；RX：逐字节喂 `AppDebugCmd_FeedByte` |
| `UART2_IRQHandler` | app/main.c | `CanMvUart_ProcessRx`（K230 视觉帧解析） |
| `BLDC_INST_IRQHandler`(UART3) | bsp/bldc | 无刷反馈接收 |
| `GROUP1_IRQHandler` | bsp/motor/hall_encoder | 编码器 GPIO 相位 |
| `TIMER_0_INST_IRQHandler`(TIMA1) | bsp/motor/hall_encoder | 编码器采样定时器 |

**策略（见 architecture.md）**：BSP 自持专属外设中断；app 持跨子系统路由/调度中断；middleware 不转发中断。

**关键初始化时序（main.c，不可乱序）**：
`SYSCFG_DL_init → BSP_Time_Init → JY61P_I2C_Init（必须在 __enable_irq 之前，否则 SysTick 的
I2C 轮询会与初始化竞态）→ __enable_irq → 各 UART 中断使能 → Ui/Key/Chassis/Gimbal/LineFollow/
CanMv Init → Gimbal_StartupElevatePitch（非阻塞发起 pitch 抬升）→ SystemFault_Clear →
TimerA start`。`BSP_DelayMs` 用 busy-wait，不依赖 SysTick。

---

## 3. 坐标系与符号约定（★★ 重写最易搬错处，逐条保留）

**世界系**：原点 = AB 边中点；+X 沿 A→B；+Y 指向靶（靶在场地外侧）；+Z 向上。逆时针行驶
`A→C→D→B→A`。四角坐标由边长 `side` 推出：`A(-side/2,0) B(+side/2,0) C(-side/2,-side) D(+side/2,-side)`。

**定位原点**：车体**轮轴中点**（不是车前沿！前沿到轮轴实测 0.18m）。

**航向 heading**：世界系角度（deg），由 IMU 融合航向 + 起点 offset + 陀螺前推得到。

**符号约定表（每一项都是实车辨识出来的，改一个就翻方向）**：

| 符号 | 值 | 含义 / 定罪依据 |
|------|-----|------|
| `gyro_z_sign` | **-1** | 本机 gz 与航向反号（左转 head 增而 gz 负）。影响速率前馈+车心重构 |
| `GIMBAL_YAW_DIR` | **-1** | 电机物理转向与几何 yaw 约定相反；只翻下发 F32C 的物理转向，逻辑角约定不变 |
| `vision_yaw_sign` | **+1** | 慢 bias 修正方向（2025-07-15 两次运行差分定罪；-1 会让 bias 单向贴限幅） |
| `vision_pitch_sign` | **+1** | 未经同法验证，待实测 |
| `gimbal_tracking.yaw_output_sign` | **-1** | 对准伺服方向（与上面是**独立**的两条链，互不作证） |
| `gimbal_tracking.pitch_output_sign` | **+1** | pitch 伺服方向 |
| `encoder_lateral_offset_m` | **-0.070** | 编码轮在右侧为负（轮距 14cm，右轮） |
| `laser_lateral_offset_m` | **+0.035** | 光束在 yaw 轴左侧为正 |
| `laser_vertical_offset_m` | **-0.040** | 束线在 pitch 轴上方为正，实测为负 |

> ★ 交接陷阱：起步 bias 计算里，yaw 对准角必须读**编码器反馈的逻辑角**
> `Gimbal_ReadYawFeedbackDeg()`，不能读开环估计 `Gimbal_GetAngle().yaw_deg`——后者在
> SetSpeed 期间按下发值积分，与 aim_solver 逻辑系差一个 `GIMBAL_YAW_DIR`，跨路径相减会得反号
> bias（起步误差翻倍）。pitch 无此问题。

---

## 4. 运行时数据流（三条控制链）

### 链 A：巡线（LineTracking_Update）
```
LineTracking_Update(dt)
  → LineFollow_Update → GrayscaleSensor_Read (bsp, 8 路, value==0 表示压黑线)
  → 质心估计 error = Σpos/active × scale
  → error 一阶低通(EMA α=0.5) + 中心死区(±10) 抑量化蛇形
  → 陀螺增稳串级: 外环 ω_ref=gyro_line_kp·error(限幅); 内环 corr=gyro_stab_kp·(ω_ref−gz)
     (增稳关或无 IMU → 退回纯位置 PID: kp=1,ki=0,kd=0)
  → correction 限幅(differential_limit/2)
  → Kinematics_DifferentialMix(base_duty, corr, output_limit) → 左右轮占空比
  → Chassis_SetDuty → TB6612FNG_SetDuty
  丢线(active==0): 返回 NOT_READY, 不自刹, 由 app 决定
```

### 链 B：瞄准前馈（AutoAim_Update，运动中连续瞄准，不依赖识别光斑）
```
AutoAim_Update(dt)
  → WitGetData(imu); ω = gyro_z_sign · imu.gz
  → Chassis_GetDistance 增量 → Localization_CorrectWheelDelta(用ω重构车心) → UpdateDelta(沿航向积分)
  → heading = imu.yaw + heading_offset + ω·heading_gyro_lead_s (陀螺前推补 JY61P 融合滞后)
  → 读新视觉角度帧(仅在 g_canmv_uart_angle_frame_count 变化时): yaw/pitch 误差
  → 弯中毒帧 ω 门控: |ω|>vision_freeze_omega 冻结慢 bias, 出弯滞留数帧
  → 视觉帧路由: startup_align 窗口内喂 startup_bias(大量程), 否则喂 vision_bias(慢跟踪衰减增益)
  → 目标点: CENTER=靶心; CIRCLE=lap_progress→相位(限速摊平 snap)→圆点
  → feedforward = AimSolver_Solve(pose, target, dz)  [静态几何角]
  → rate = AimFusion_MotionRate(...)  [运动引起的云台角速度需求, 含 ω×r 甩臂]
  → yaw_cmd = ff.yaw + startup_bias.yaw + vision_bias.yaw
  → pitch: 若 pitch_vision_only&&CENTER → 内联视觉 PID 伺服; 否则 ff.pitch+bias+rate·lead
  → Gimbal_SetAngle(yaw_cmd, pitch_cmd, rate.yaw)  [yaw 速度前馈级联; pitch 绝对位置]
```

### 链 C：纯视觉角度闭环（GimbalTracking_UpdateAngle，Aim track / E2 / E3 追踪段）
```
GimbalTracking_UpdateAngle(dt)
  → 仅在新角度帧到达时更新: 读 yaw/pitch 角度误差(deg)
  → TrackAngleErrors: 两轴独立 PID(位置式, 输入 deg 输出 deg/s)
     yaw_speed = yaw_output_sign · PID_yaw; pitch 同理
     yaw 越过整数 RPM 台阶: 输出非零时抬到 ≥MIN_MOVE_YAW(1 deg/s)
  → Gimbal_SetSpeed(yaw_speed, pitch_speed)
  帧间无新帧: 保持上一速度; 超 link_timeout(1000ms) 判超时
  丢失: yaw 停转, pitch 位置保持, 清角度 PID 积分
```

**Gimbal 层执行语义**（middleware/gimbal）：
- yaw：速度模式（F32C 速度内环）。`SetAngle` 走**速度前馈级联**：ω_cmd = 解析速率前馈·GAIN +
  VELFF_KP·(目标角−一步预估反馈角)，ΔΣ 量化整形下发 RPM，滑环多圈解缠。**惰性使能**（首次非零
  速度才使能）。**链路自愈**（周期重发使能+模式+RPM 防丢帧冻结）。
- pitch：**位置模式**。开机 `StartupElevatePitch` 非阻塞抬升到 home（默认 150°，即 `BLDC_PITCH_INIT_X10`），
  `EnsurePitchReady` 阻塞确认到位；归位后限位收紧到 home±30°=[120,180]。`SetSpeed` 的 pitch 分量
  积分成位置设定点；`SetAngle`/`SetPitchDeg` 直接置绝对设定点。

---

## 5. 任务状态机（行为契约，重写后逐项走查）

菜单项（app_launcher，K1/K4 移动，K3 进入，K2 长按返回）：
`E1 Line / Aim track / E2 Aim / E3 Scan aim / F1 Line+aim / F1 slow / F2 Line+aim / F3 Line+circle / Calibration / Device check`

### E1 Line（`AppE_RunLineFollow` → `AppE_RunLineTask(aim=NONE)`）
子菜单选 1~5 圈。纯底盘巡线。状态机：
- `FOLLOW`：`LineTracking_Update`；8 路全丢 → 计 `corner_enter_count`，连续 `ENTER_CONFIRM(2)` 帧确认入弯，
  丢线瞬间即降到 `corner_forward_duty` 防冲出。
- `CORNER_FORWARD`：按 forward_duty 前进，编码器距离达 `forward_dist(0.09m)` → 转 ARC。
- `CORNER_ARC`：原地左转（左轮反右轮正）；3/4 号传感器压黑线即结束，`LineTracking_Reset`，边数+1。
- 每圈 = 4 条边；到 `target_laps×4` 边完成。丢线超 `LOST_GRACE(1000ms)` → 刹车等返回。
  总超时 = `LINE_TIMEOUT(20000ms)×圈数`。

### Aim track（`AppE_RunContinuousAim`）
纯视觉角度闭环（链 C）。**不初始化 AutoAim**（不用世界系/IMU/编码器）。激光常亮。连续锁定判据：
仅在新有效角度帧且误差在 `aim_lock_yaw/pitch_deg` 内累计 `aim_lock_confirm_frames(8)`。丢目标停 yaw 保 pitch。

### E2 Aim（`AppE_RunAimCenter2s` → `AppE_RunVisualAim2s`）
2 秒瞄靶心。`GimbalTracking_UpdateAngle`（链 C）。锁定确认或 `e2_force_shot_ms(1800)` 强制 → 脉冲激光 `laser_pulse_ms(20)`。

### E3 Scan aim（`AppE_RunScanAim`）
子菜单选扫描方向 +/−。阶段1：yaw 单向匀速扫 `SCAN_YAW_SPEED(60 deg/s)`，直到新角度帧且 status OK →
发现靶。阶段2：停扫，链 C 追踪；锁定后发射一次并激光常亮继续追踪。无总超时。

### F1/F2/F3（`AppE_RunLineAim/LineAimSlow/LineCircle` → `AppE_RunLineTask(aim!=NONE)`）
巡线（链 A）+ 瞄准（链 B）并行。**起步序列**（★ 见 §6.交接）：
1. 几何预定位 `AIM_GEOMETRY_SETTLE_MS(1000)`：AutoAim 跑但禁视觉积分，车刹住。
2. 释放 AutoAim → GimbalTracking（链 C）纯视觉对齐；进入前 pitch 抬 `AIM_ENTRY_PITCH_UP_DEG(5°)` 让相机看到靶。
3. 连续锁定（或 `AIM_ALIGN_TIMEOUT_MS(15000)` 超时降级）：读逻辑 yaw 反馈算 `startup_bias = 对准角 − 起点几何角`。
4. 重启 AutoAim（链 B），带 startup_bias 无跳变切入；开激光跑巡线。
- F1=1圈, F1 slow=降速档+无总超时, F2=2圈, F3=CIRCLE 模式画圆。
- 起步在 A 处的首个左拐**不计边/不锚定**（`initial_start_turn`）；此后每确认一条边 `AutoAim_AnchorCorner`
  按序列 `[C,D,B,A]` 重锚位姿+弧长。

---

## 6. 双伺服职责边界与起步交接（★ 重写核心难点）

工程里**两套云台闭环并存**，重写时要么保持边界清晰，要么合并（方案 A）：

| | `middleware/auto_aim`（链 B） | `middleware/gimbal_tracking`（链 C） |
|---|---|---|
| 输入 | 世界位姿(IMU+编码器) + 视觉角度误差(慢校正) | 仅 K230 视觉角度误差 |
| 输出 | `Gimbal_SetAngle`（yaw 速度前馈级联 + pitch 绝对位置） | `Gimbal_SetSpeed`（两轴速度） |
| 用途 | F1/F2/F3 运动中前馈瞄准 | Aim track / E2 / E3追踪 / F1起步对齐 |
| yaw 符号 | `GIMBAL_YAW_DIR` 在 gimbal 层 | `yaw_output_sign` 在 tracking 层 |
| 视觉符号 | `vision_yaw/pitch_sign` | （PID 误差直接用，方向靠 output_sign） |

**交接时序（F1 起步，`AppE_RunLineTask` 内，行为必须逐拍等价）**：
`AutoAim_Start(禁视觉) → 1s几何刹停 → AutoAim_Stop → GimbalTracking_Init/Reset → pitch抬5° →
纯视觉对齐至 locked → 读 Gimbal_ReadYawFeedbackDeg + Gimbal_GetAngle → 算 startup_yaw/pitch_bias →
GimbalTracking_Stop → AutoAim_Init/Start → SetStartupAlign(true)+SetStartupBias+SetStartupAlign(false) →
SetVisionCorrectionEnabled(true) → 开激光跑`。
> 两个控制器**互斥**：同一任务固定用其一，切换前必须 Stop。yaw bias 跨路径相减的反号陷阱见 §3。

---

## 7. Golden Constants 清单（★★ 逐字保留；源文件为权威）

> 重写时**从源文件原样复制**。下表用于"是否漏项"核对与关键值交叉检查。

### 7.1 整车标定 `app/app_e_calibration.c`（最关键，带大量出处注释——注释一并保留）
- start_pose = (**-0.320, 0.000, 180°**)；track_side_m=**1.000**
- encoder_lateral_offset_m=**-0.070**；gyro_z_sign=**-1**；heading_gyro_lead_s=**0.038**
- solver.target_center=(**0.000, 0.500**)；height_diff_m=**0.030**；mount_x_m=**0.120**；mount_y_m=**0.000**
- solver.yaw_zero_offset_deg=**0.000**；laser_lateral_offset_m=**+0.035**
- solver.pitch_zero_offset_deg=**137.940**；pitch_beam_per_motor=**3.640**；laser_vertical_offset_m=**-0.040**
  （pitch 三元组 `(z0,k,ℓp)` 是**联合标定**，任一变动需整套重标；z0 不跨上电存活）
- vision_yaw={gain0=**0.500**, gain_min=**0.100**, limit=**8.0**}；vision_pitch={**0.250, 0.030, 2.0**}
- vision_yaw_startup_limit=**10.0**；vision_pitch_startup_limit=**5.0**；vision_freeze_omega=**40.0**
- vision_yaw_sign=**+1**；vision_pitch_sign=**+1**
- circle_radius_m=**0.060**；circle_phase0_deg=**0.0**；pitch_rate_lead_s=**0.0**；rate_ff_v_tau_s=**0.133**
- aim_lock_yaw_deg=**0.8**；aim_lock_pitch_deg=**0.8**；aim_lock_confirm_frames=**8**
- e2_force_shot_ms=**1800**；e3_force_shot_ms=**3800**；laser_pulse_ms=**20**；imu_fail_limit=**10**

### 7.2 云台 `middleware/gimbal/gimbal.c`（宏）
- YAW_ADDR=BLDC_ADDR_1, PITCH_ADDR=BLDC_ADDR_2；GEAR_RATIO=1.0；**YAW_DIR=-1**
- YAW_VELFF_KP=**7.0**；YAW_RATE_FF_GAIN=**1.10**；YAW_VELFF_MAX=**300 deg/s**；LINK_REFRESH=**500ms**
- PITCH_HOME=`BLDC_PITCH_INIT_X10`(默认150°)；HOME_TOL=3°；HOME_TIMEOUT=2000ms；RANGE=±30°→[120,180]

### 7.3 视觉角度 PID `middleware/gimbal_tracking/gimbal_tracking.h`
- ANGLE_YAW_PID: kp=**6.0** ki=**2.0** kd=**0.0** ilim=**20.0** olim=**120.0**
- ANGLE_PITCH_PID: kp=**2.0** ki=**0.8** kd=**0.0** ilim=**25.0** olim=**60.0**
- yaw_output_sign=**-1**；pitch_output_sign=**+1**；MIN_MOVE_YAW=**1.0**；link_timeout=**1000ms**
- 上位机 `s ykp/yki/ykd/pkp/pki/pkd/olim` 调的即这两组（olim 两轴输出限幅）

### 7.4 巡线 `middleware/line_tracking/*` 与 `app/app_e_task.c`
- LineTracking: ERROR_LPF_ALPHA=**0.5**；ERROR_DEADBAND=**10.0**；sensor_pos=-3.5..3.5；
  pid kp=**1.0** ki=0 kd=0 ilim=500；base_duty/output_limit/differential_limit/gyro 增稳参数见 `.h`
- app_e_task 行为：CORNER_TURN=**11.0**；FORWARD_DIST=**0.09m**；FORWARD_DUTY=**15.0**；
  SLOW{base=**22**, corner=**8**, forward=**12**}；ENTER_CONFIRM=**2**；EXIT_CONFIRM=**3**；
  LINE_TIMEOUT=**20000**；LOST_GRACE=**1000**；AIM_GEOMETRY_SETTLE=**1000**；ENTRY_PITCH_UP=**5.0**；
  ALIGN_TIMEOUT=**15000**；SCAN_YAW_SPEED=**60.0**；LOOP_DELAY=**10**；UI_REFRESH=**400**；TELEMETRY=**20**

### 7.5 视觉 bias 衰减增益模型 `core/aim_fusion`
`g(n)=max(gain_min, gain0/(1+n))`；`SetSteadyGain` 直接把 n 跳到稳态。startup/steady 两组独立 bias。

---

## 8. K230 视觉协议（`bsp/canmv`，不可改，K230 端约定）

两种定长帧，进入接收态后按长度采集（**中途不把载荷里的 0x12/0x5B 当帧边界**）：
- **角度帧（现役）**：`A5 5A status seq yaw_hi yaw_lo pitch_hi pitch_lo checksum`（9B）。
  yaw/pitch = `deg × CANMV_ANGLE_SCALE` 的 int16 **大端**；status: VALID/NOT_FOUND/LOST；
  checksum = 前 8 字节和的低 8 位。解析成功 `g_canmv_uart_angle_frame_count++`（上层以此判"新帧"）。
- **坐标帧（旧）**：`0x12 ... 0x5B`（27B），承载 laser/rect 像素坐标。含大小端自适应
  (`CombineCoordinate`)。**注意：像素/矩形跟踪路径已在固件删除**，此帧当前无固件消费者
  （§9），保留仅为兼容旧脚本/设备检查；重写可评估是否连带精简。

激光控制：MCU 经 UART2 发 `"LASER=1\n"/"LASER=0\n"` 字符串给 K230，由 K230 控激光 GPIO。

---

## 9. 已知潜在问题 / 重写时一并决策

1. **F1 slow 的 CORNER_FORWARD 不一致**（`app_e_task.c` CORNER_FORWARD 状态）：该状态硬编码常规档
   forward_duty(15.0)，未用 `prof.corner_forward_duty`，故慢速档在入弯直行段并未降到 12.0。
   待定：是刻意（保直行速度）还是疏漏。修正会改变慢速档行为，需上车验证。
2. **坐标帧解析已实质半死（已核实）**：像素/矩形跟踪删除后，全库 grep 确认 `CANMV_TARGET_LASER/RECT`
   在 app 层**无任何消费者**（坐标帧仍解析但无人读）。重写可精简：只保留角度帧解析链，
   坐标帧解析 + `CanMvUart_GetData`/`GetTargetData`(LASER/RECT 部分) 可评估删除。
3. **`AimFusion_Combine` 已确认死代码**：全库仅头文件声明、无调用者（auto_aim 内联组装命令）。重写时不迁入。
4. **开环 yaw 估计与逻辑系符号**：`Gimbal_GetAngle().yaw_deg` 与 `Gimbal_ReadYawFeedbackDeg()` 差
   一个 `GIMBAL_YAW_DIR`，交接算 bias 只能用后者（§3）。

---

## 10. 重写策略建议

1. **自底向上、保值、可回退**：bsp → core → middleware → app，每模块独立 commit，Keil 每步 0/0。
2. **常量原样搬运**：§7 各源文件的数值+出处注释**整体复制**，不重敲。
3. **符号/时序类先不动语义**：§3 符号表、§2 初始化时序、§6 交接时序按现状 1:1 迁移，重写完再谈优化。
4. **合并双伺服（方案 A）单独作为一步**：在其余重写稳定后再做，且必须上车复验。
5. **每完成一个任务，用 §5 行为契约走查**；上车后用 §3/§6/§7 的 ★ 项复验。

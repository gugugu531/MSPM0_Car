# 变更记录

## 未发布

- 修正编码器**测速采样周期严重不一致**的 bug。采样定时器 `TIMER_0`(TIMA1) SysConfig 实际配的是
  **100ms**,而固件 `HALL_ENCODER_SAMPLE_PERIOD_S` 假设 **10ms** → 速度 = 脉冲/0.01 被**放大 10 倍**
  (距离不含时间故不受影响, 正是"距离准/速度错"的成因);且速度仅 10Hz 刷新, 而速度环跑 50Hz(20ms)→
  连续 5 个控制拍吃同一陈旧值、~100ms 死区滞后, 是之前闭环发飘/难整定的重要推手。**方案 B**: 把
  `TIMER_0` 周期改到 **20ms**(SysConfig `timerPeriod="20ms"`, 生成 `LOAD=799`), `SAMPLE_PERIOD_S=0.02f`
  与之对齐——既纠正 10× 比例、又让反馈 50Hz 与控制同拍(滞后 100ms→20ms)。因无法运行 SysConfig 图形
  工具, 同步手改 `.syscfg` 与生成的 `ti_msp_dl_config.c/.h`。文档加"两者须一致"告诫。Keil 0/0。
  **注意: 速度数值缩小 10 倍且反馈变快, 之前整定的 `CHASSIS_SPEED_*` 增益须重调。**

- 主菜单新增「Line Track」循迹测试任务(`app/app_line_task.c`, 新建功能任务文件)。用
  `middleware/line_tracking` 完整闭环跑巡线(读 GPIO 数字灰度 → 计算差速 → 驱动底盘),
  on_enter 复位灰度/巡线状态并 Init JY61P、每拍 Poll 使默认配置的陀螺增稳生效; 丢线返回
  NOT_READY 时刹停(测试期安全)。OLED 显示 8 路命中图/误差/双轮占空比/track|LINE LOST。
  Keil/CCS 工程登记新文件。Keil 0/0。抬轮或置于线场运行。

- 新增 Device Check「Duty Sweep」自检: **开环**固定占空比(按键 ±2% 阶梯, 双轮同值)读回两轮
  编码器速度, OLED 显示 + 复用 `[SPD]` 遥测格式(tl/tr=0, dl/dr=占空比)供 `tools/speed_pid_viz.py`
  直接绘图。用于诊断电机死区与"占空比升高→编码器噪声爆表"的拐点(不走速度环, 排除闭环干扰)。Keil 0/0。

- Speed PID 自检加 **UART 遥测 + 上位机可视化整定**。补全一直未接的 debug 串口发送链:
  `DebugUart_Init` 使能 UART 中断线并新增 `Debug_Ex_INST_IRQHandler`(UART1) 排空 TX 环形缓冲、
  `App_Init` 调 `DebugUart_Init`(此前 debug 串口从未初始化、TX ISR 从未接, 实为不工作)。
  Device Check「Speed PID」每控制拍(20ms/50Hz)非阻塞输出遥测行
  `[SPD] t= tl= tr= l= r= dl= dr=`(目标/实测轮速 m/s + 应用占空比%)。新增上位机
  `tools/speed_pid_viz.py`(pyserial+matplotlib): 实时绘 左右轮 目标 vs 实测 曲线 + 占空比 +
  跟踪误差 RMS, 供整定 `chassis.h` 的 `CHASSIS_SPEED_*` 增益。Keil 0/0。

- `middleware/chassis` 接入**速度闭环控制**。新增控制模式 `CHASSIS_CONTROL_DUTY`(开环, 默认)/
  `CHASSIS_CONTROL_SPEED`(闭环): 每轮独立位置式速度环 PID(复用 `core/pid`), setpoint=目标轮速
  m/s、feedback=`HallEncoder_GetSpeed(id)`、output=占空比%(±100 限幅, 抗积分饱和)。API:
  `Chassis_SetWheelSpeed(l,r)`/`Chassis_SetSpeed(v)`(进闭环、设目标)、`Chassis_UpdateSpeedControl(dt)`
  (周期驱动出力)、`Chassis_GetControlMode`/`GetWheelSpeedTarget`;`SetDuty/Stop/Brake/Coast` 切回开环
  并清目标。tick 接在 `App_ControlTick`(RUN 态 20ms), 退出 RUN 时 `App_ExitRun` 自动 `Chassis_Brake`
  停环。默认增益在 `chassis.h`(须上板整定)。新增 Device Check「Speed PID」自检(按键给目标速度,
  OLED 显示两轮 目标 vs 实测, 供整定)。差速转向(车体 v/ω)待 `track_width`, 届时在 SetWheelSpeed
  之上加 `Kinematics_BodyToWheel` 即可。文档同步。Keil 0/0。**待上板整定增益并验证跟踪**。

- 编码器改为**左右双轮**并修正接线隐患。核实原实现非硬件 QEI，而是 GPIO 边沿中断 + 软件正交
  解码;且原单轮实现把右轮 B 相错接到 `PB22`(实为导航按键脚)→ B 相恒高、方向恒判前进(仅前进
  时看似正确)。SysConfig 引脚按轮改名 `E1A/E1B/E2A/E2B → ENC_R_A/ENC_R_B/ENC_L_A/ENC_L_B`
  (物理脚不变;因无法运行 SysConfig 图形工具,同步手改 `.syscfg` 与生成的 `ti_msp_dl_config.c/.h`
  保持一致)。`bsp/motor/hall_encoder` 重写为双实例:右轮 A=PB2/B=PA2、左轮 A=PA22/B=PA25,
  `GROUP1_IRQHandler` 分别服务 GPIOB(右)/GPIOA(左) A 相并各自清中断;API 加 `HALL_ENCODER_ID`
  参数;新增每轮"整车前进为正"符号宏(上板校正)。`middleware/chassis` 暴露 `Chassis_GetWheelSpeed/
  Distance(id)`,`Chassis_GetSpeed/Distance` 改返回车体量——车体线速度接 `Kinematics_WheelToBody`
  (差速运动学模型的首个消费者)。`Device Check` 的 Encoder/TB6612 自检显示双轮。文档同步。Keil 0/0。
  **待上板验证**:两轮计数/方向(尤其右轮真 B 相 PA2 的方向、及左右轮前进符号)。

- `core/kinematics` 建立两轮差速运动学模型（纯函数，车体参数传参，暂无消费者、标注「预留」）：
  正运动学 `WheelToBody`（轮速→车体 v/ω）、逆运动学 `BodyToWheel`（车体 v/ω→轮速，新类型
  `KINEMATICS_WHEEL_SPEED`）、轮线速度↔轮自转角速度换算、瞬时转弯半径 `TurnRadius`、里程推算
  `IntegratePose`（中点欧拉积分位姿）。约定：线速度 m/s、车体航向角速度 deg/s（逆时针为正，配陀螺
  `gz`）、轮自转 rad/s（配 `v=r·ω`）、位姿 x/y 米、航向 deg。激活原预留的 `KINEMATICS_POSE`/
  `KINEMATICS_VELOCITY` 类型与角度换算宏。新增 `docs/interfaces/core_kinematics.md` 模型章节
  （约定表+公式+接口）。Keil 0/0，预留函数经 section GC 零占用（Code size 不变）。

- 新增 `core/filter`：把 `line_tracking` 内联的两段通用信号处理下沉到 core——一阶低通
  `Filter_LowpassEma`（EMA）与中心死区 `Filter_Deadband`，均为无状态自由函数（状态由调用方
  持有，与 `kinematics` 同风格），按内联公式 1:1 实现、行为不变。`line_tracking.c` 改调 core
  函数，误差预处理六行塌成两行；同时合并冗余的 `LineTracking_DefaultConfig`/`GetDefaultConfig`
  为单一公开函数。配套新增 `docs/interfaces/core_filter.md`、更新接口索引与 Keil/CCS 工程登记。
  顺带修正 `middleware_line_tracking.md` 的默认参数漂移（`base_duty 34`、`differential_limit 16`、
  PID `kp=1.0`/`kd=0.0`、差值上限 16%），补齐陀螺增稳默认宏与低通/死区说明。Keil 0/0，Code size 不变。

- `core/pid` 增加 `PID_LIMIT_DISABLED` 哨兵宏（令限幅关闭的配置点自解释，沿用「<=0 关闭」
  语义、逻辑不变），并把当前无调用者的增量式路径标注「预留」，与 core 其余模块口径统一。

- `ganv_gray` 修复读取间歇失败（上板 + 诊断计数定位根因）：现象为 online/offline 约各半、
  RETRY=2 时 err=0（确定性隔次交替，非随机瞬态）。加写/读分阶段诊断计数后定位到失败**全在
  写命令阶段且为 NACK**、读阶段从不失败，根因是**每次读都重发命令 `0xDD`**——违背手册方法
  2/3「写一次、反复读」，传感器对高频写命令间歇 NACK。改为**命令指针缓存**：仅命令变化时才
  写命令，连读同一命令直接读、跳过写，高频读路径不再写命令，交替消除（上板验证 err≈0）。
  写命令保留 3 次重试兜底；底层 I2C 事务一并从 repeated-start 改为「写命令+STOP、独立读+STOP」
  两个完整事务。`Gray I2C` 自检显示 `ok`/`er` 与写/读阶段失败计数便于长期监控。Keil 0/0。

- `Device Check` 接入感为灰度 I2C 自检项 `Gray I2C`：OLED 实时显示 8 路数字量二进制、
  在线/离线状态与固件版本；与 JY61P/MPU6050 共 I2C0，进入时挂起 JY61P、退出时恢复。
  同时把传感器手册归档到 `docs/datasheets/`（大体积 PDF 经 `.gitignore` 排除，不入库，
  协议要点已固化进 `docs/interfaces/bsp_ganv_gray.md`）。

- 新增 `bsp/ganv_gray`：感为(GANV)8 路灰度传感器 I2C 驱动。复用已初始化的 I2C0 总线
  （SysConfig 实例 `MPU6050_JY61P_Tracking`，PA0=SDA/PA1=SCL），作为地址 `0x4F` 的第三个
  从机（与 MPU6050 `0x68`/JY61P `0x50` 不冲突，无需改 SysConfig）。底层为阻塞 I2C，与
  `mpu6050.c` 同构（写命令 + REPEATED START 读）。公开 `GanvGray_Init`（上电 ping 同步）、
  `GanvGray_ReadDigital`（0xDD，8 路数字量 mask，bit0=第1路）、`GanvGray_Ping`（0xAA→0x66）、
  `GanvGray_ReadVersion`（0xC1）、`GanvGray_ReadError`（0xDE）、`GanvGray_Reboot`（0xC0）。
  地址做成可覆盖宏 `GANV_GRAY_I2C_ADDR_7BIT`（默认 `0x4F`，随 AD1/AD0 跳线帽调整）。校准为
  设备端按键操作，无 I2C 命令。Keil/CCS 工程同步登记；保留原数字量 GPIO 版 `grayscale_sensor`。

- 梳理 `core`（纯算法层，唯一消费者 `middleware/line_tracking`）：按「保留代码、只标注」
  方针，对 `kinematics`/`common` 中当前无调用者的类型、宏与函数（`KINEMATICS_DIR/ATTITUDE/
  POSE/VELOCITY`、角度换算宏、`NormalizeAngleDeg`/`AngleDiffDeg`、`CORE_POINT2F`）统一加
  「预留」注释；删除 `kinematics.c` 冗余的 `math.h`/`stddef.h` include；修正角度归一化区间
  口径为 `[-180, 180)`。同步把 `core_kinematics`/`core_common`/`core_pid` 接口文档对齐到
  真实代码（删除文档中并不存在的 `Kinematics_Distance2D`/`PoseInit`/`PoseUpdate`、
  `PID_SetConfig`、`PID_Get*` 等接口描述）。

- 拆分 app 任务框架契约：`app_tasks.h` 原混装框架契约与具体 Timer Test 任务，现拆为
  `app_task.h`（仅 `APP_TASK_STATUS` + `APP_TASK_DESC` 生命周期契约，不含具体任务）；删除
  `app_tasks.c` 及 `APP_TASK_TIMER` 测试任务与其根菜单项，具体任务改由 `app_checks` 定义、
  经菜单树引用。同步更新各处 include 与 Keil/CCS 工程引用，并对齐
  `architecture`/`interfaces`/`project-structure`/`app-design`/`build-guide` 文档。

- 修复 Keil 烧录 `Programming Failed`（擦除成功但编程失败）：MSPM0 Flash 按 64 位(8 字节)字
  +ECC 编程，镜像总长须为 8 的倍数；此前镜像尾长 4-mod-8 导致最后一字编程失败（与镜像大小
  相关、与调试时钟无关——恰好旧镜像 8 对齐能烧、新镜像不对齐失败）。加一段 8 字节对齐的
  尾填充(`g_flash_tail_pad`)，经 `board/startup/mspm0g3507.sct` 以 `+Last` 放在镜像末尾，
  强制总长 8 对齐；镜像 `0x58dc→0x58e8`，`Programming Done. Verify OK.`。填充用
  `__ARMCC_VERSION` 限定仅 Keil 生效，不影响 CCS/GCC。

- app 层加入外设自检并改为嵌套菜单：
  - 新增 `app_menu`(菜单树 `MENU_NODE`/`MENU_ITEM` + 导航栈 + 渲染) 与 `app_menu_def`(菜单树实例)，
    取代原扁平任务注册表；`app_mode` 的 MENU 态委派 `app_menu`，选中任务由 `Menu_Tick` 返回
    给 `app_mode` 进入 RUN（app_menu 不反向依赖 app_mode）。BACK 短按=返回上级。
  - 新增 `app_checks`：5 个外设自检任务——Gyro JY61P / Gyro MPU6050 / Grayscale / TB6612 /
    Encoder，挂在「Device Check」子菜单。两个陀螺仪共 I2C0，靠 on_enter/on_exit 挂起/恢复分时。
    TB6612 为主动自检，短按发单次低速脉冲(20%/300ms 自动 Brake)+抬轮提示，并显示编码器响应。
  - 新增 `app_fmt`：定点数字格式化（不引浮点 printf）。
  - `app_tasks` 精简为业务任务（Timer Test），移除空占位任务与扁平注册表 API。
  - 按键仍仅短按。Keil/CCS 加入 4 个 app 源文件。Keil 0/0（Code 14326→19592）。

- 重建 app 层为裸机菜单调度框架（`docs/app-design.md`）：
  - `app_scheduler`：协作式时间触发任务表 + `Scheduler_Run` 分派；自持 `SysTick_Handler`
    (1ms：`BSP_Time_TickInc` + `Key_Scan`)，`tick_active` 门控防 init 期误触发。
  - `app_mode`：顶层状态机 INIT/MENU/RUN/FAULT；状态转移集中于 `App_EnterRun/ExitRun/RaiseFault`；
    进 RUN 必 ResetDistance+on_enter、出 RUN 必 Brake；两级故障（可恢复 FAULT 态 / 致命 Halt 终态）。
  - `app_tasks`：任务注册表 `TASK_REGISTRY[]` + `on_enter/on_tick/on_exit` 契约；含 3 个空占位
    任务与 1 个测试任务（Timer Test：5s 倒计时 DONE 回菜单）。
  - `app_init`：集中式上电时序（Ui/Chassis 先于任何可能 fault 的步骤）。
  - 按键仅用短按：MENU 用 UP/DOWN 选择、ENTER 进入；RUN 用 BACK 中止；FAULT 用 ENTER 复位。
  - Keil/CCS 工程加入 4 个 app 源文件。Keil 0/0（Code 7102→14326，框架拉入 ui/chassis/key/fault）。

- 删除 K230 视觉子系统：移除 `bsp/canmv`（原消费者删除后已无调用点），并从 `G3507.syscfg`
  移除 UART2(K230) 外设（释放 PA23/PA24，避免使能中断却无 ISR 的隐患），SysConfig CLI 重新
  生成 `ti_msp_dl_config`（K230/UART2 零残留）。同步删除接口文档 `bsp_canmv_uart` 与
  `k230-tool`，清理 Keil/CCS 工程条目与 include 路径、`architecture`/`interfaces`/
  `project-structure`/README/AGENTS 的视觉相关描述。Keil 0/0（Code 7246→7102）。
- 移除二维云台/瞄准子系统（分四步，各步 Keil `0 Error(s), 0 Warning(s)` 验证）：
  ① `app` 清空为极简启动骨架（仅 `main.c`：SysConfig 初始化后空循环 + 空 `SysTick_Handler`），
  删除全部原任务框架文件（`app_launcher`/`app_e_task`/`app_e_calibration`/`app_device_check`/
  `app_debug_cmd` 等）；② 删除 middleware 云台控制（`gimbal`/`gimbal_tracking`/`auto_aim`），
  `system_fault` 去掉 `Gimbal_Stop()` 依赖仅保留 `Chassis_Brake()`；③ 删除 core 瞄准/定位算法
  （`aim_solver`/`aim_fusion`/`localization`），保留 `common`/`kinematics`/`pid`；④ 删除
  `bsp/bldc`，并从 `G3507.syscfg` 移除 UART3(BLDC)/PWM2(SMotor_1)/GPIO1(SMotor_IO)，用
  SysConfig CLI 重新生成 `ti_msp_dl_config`（BLDC/SMotor 零残留）。同步清理 Keil/CCS 工程条目
  与 include 路径、删除对应接口文档，更新 `architecture`/`interfaces`/`project-structure`。
  释放引脚 PB3/PB12、PA29/PA30、PB14/PA15/PB11/PB20。Code 54056→7246 字节。
- 清理旧任务文档：删除 `docs/aim-tracking-plan`、`docs/angle_tracking_calibration`、
  `docs/calibration`、`docs/api-migration`、`docs/rewrite-baseline`（均为 2025E 云台/瞄准任务
  的历史资料，对应代码已移除）。`tools/aim_*.py` 旧可视化脚本暂留待评估。
- 消除 `core ↔ middleware` 循环依赖：将 `motion`、`gimbal_tracking`、`line_tracking` 三个模块从 `core/` 迁入 `middleware/`。这三者需直接调用 `Chassis_*`/`Gimbal_*`/`LineFollow_*` 并读取硬件观测，本质是中间件能力；迁移后 `core` 恢复为纯计算层（仅 `pid`/`kinematics`/`rotation`/`geometry`/`localization`/`aim_solver`/`common`），依赖方向恢复单向 `app → middleware → {core, bsp}`。同步更新 Keil/CCS 工程文件、include 路径、相关 include 语句为裸文件名，并重命名接口文档 `core_* → middleware_*`。编译验证 `0 Error(s), 0 Warning(s)`。
- 将 E3 拆分为 `E3 Yaw+` 和 `E3 Yaw-` 两个主菜单入口；矩形扫描识别成功后先停止云台约 `200ms`，随后直接复用 E2 的靶心瞄准逻辑，不再单独执行矩形中心跟踪。
- 收紧 E1 与 Corner test 的转弯结束条件：中心 3/4 重新检测到轨道后需要连续确认 3 次，才刹车退出转弯。
- 新增 `k230/rect_recognition.py` 与 MSPM0 的 UART 帧通信：按 `0x12 ... 0x5B` 固定帧发送目标中心、图像中心和矩形角点；考虑摄像头倒装，发送前对目标中心和矩形角点绕图像中心做中心对称修正，并在目标中心接近图像中心时开启 GPIO2 激光笔。
- 调整 `core/gimbal_tracking` 对当前 K230 通信协议的坐标处理：K230 端已发送倒装修正后的图像坐标，MCU 侧直接使用目标中心和图像中心，不再额外翻转 y 轴；E2 因此按当前 `rect_recognition.py` 的激光段数据执行云台瞄准。
- 为 `core/gimbal_tracking` 增加 K230 有效目标数据超时保护：默认约 `1000ms` 内没有新的有效目标数据时立即停止云台 yaw/pitch 输出，避免视觉链路断开或目标长时间丢失后云台继续按旧速度转动。
- 修正视觉云台跟踪 yaw 输出方向，将默认 `yaw_output_sign` 调整为正向，解决 E2/E3 中水平步进电机跟踪方向反的问题。
- 提高视觉云台跟踪默认速度响应：PID 比例系数调整为 `0.4`，输出限幅设为 `240 deg/s`；E2 改为持续调整云台，只有长按或连续约 `1000ms` 没有有效 K230 目标数据时才停止。
- 整理 E3 扫描和跟踪流程：扫描阶段 `GimbalTracking_IsRectValid()` 只查询矩形有效性，不再触发视觉超时停机；E3 增加扫描超时保护，识别到矩形后立即停止扫描并进入矩形中心跟踪，跟踪阶段仍由有效目标数据超时保护停止云台。
- 将 E3 云台扫描速度从 `120 deg/s` 降至 `60 deg/s`，并将扫描超时放宽到约 `8s`，保证慢速扫描时仍能覆盖最多一圈。
- 将 `Corner test` 的原地差速转弯逻辑应用到 E1 直角弯：确认拐角后先刹车，再用左轮 `-25%`、右轮 `+25%` 转向，中心 3/4 连续确认后刹车并恢复循线。
- E2/E3 进入任务后等待启动按键释放并清除事件，避免菜单长按残留导致任务刚启动就退出。
- 将 `Corner test` 左转停止策略改为两段式捕线和停稳复检：先用 2/3/4/5 宽范围捕线，再低速微调到中心 3/4 连续确认，刹车等待约 `80ms` 后复检；若中心离线则继续微调。
- 减小 E1 左转差速：转弯输出从内侧 `6%`、外侧 `30%` 调整为内侧 `12%`、外侧 `24%`，降低直角弯时的差速冲击。
- 调整 `Corner test` 左转停止条件：左转过程中持续读取灰度，中心 3/4 重新识别到轨道时立即刹车停止。
- 将 `Corner test` 从“识别拐角后立即刹车退出”扩展为“识别空线拐角后主动刹车，再开环原地左转约 90 度并刹车停住”，用于调试直角转弯动作参数。
- 将 E1 与 `Corner test` 的拐角识别条件切换为 8 路灰度全部未检测到轨道并连续确认 2 次；该条件不携带左右方向，E1 暂按逆时针任务默认进入左转流程。
- 主界面新增 `Corner test` 测试入口，进入后先循线运行，识别并连续确认到拐角后立即主动刹车并退出测试循环，用于单独验证拐角识别触发时机。
- E1 识别并确认直角弯后新增非阻塞主动刹车段，默认刹车约 `120ms` 后再进入低速差速圆弧转弯，用于降低入弯惯性。
- 灰度传感器更换完成后，恢复巡线控制为 8 路全通道参与：`LINE_TRACKING_ACTIVE_SENSOR_MASK` 改为 `0xFF`，E1 圆弧转弯重新捕线中心改回 3/4。
- 调整 E1 直角弯控制代码表达方式：进入确认、当前转向和退出确认仍在主循环中显式维护，并补充注释，方便继续上板调试。
- 优化 E1 直角弯状态机：外侧 0/1/2 或 5/6/7 需要同一方向连续确认 2 次才进入圆弧转弯；圆弧转弯退出为中心 3/4 连续确认 3 次，减少瞬时触发导致的误入弯或过早出弯。
- 修复 E1 难以从循迹跳转到转弯的问题：拐角入口不再要求中心 3/4 未检测到线，只由外侧 0/1/2 或 5/6/7 触发；3/4 仅用于转弯结束后重新捕线。
- 修正 E1 `Line lost` 判定：只要有效通道掩码内任意一路压线就不算丢线；拐角入口改为看外侧 0/1/2 或 5/6/7 中至少一路触发。
- 将 E1 `Line lost` 连续丢线宽限时间延长到 `1000ms`，避免短时间离线或转弯瞬间状态变化过早触发丢线。
- 优化 E1 `Line lost` 判定：丢线保护改用有效通道掩码判断，持续超过宽限时间才判定丢线。
- 移除 `middleware/line_follow` 的整组轨道识别消抖，传感器读取后立即发布最新 8 路状态；同时放宽 E1 直角弯识别条件，中心丢线且某侧外侧至少一路触发即可进入对应圆弧转弯。
- E1 `Line lost` 提示页新增转弯次数显示，进入直角弯圆弧状态时累加，用于定位丢线发生在第几次转弯之后。
- 将 E1 直角弯流程从“刹车后原地转向”改为低速差速圆弧转弯，当前内侧轮 `6%`、外侧轮 `30%`，不再设置最小或最大转弯持续时间，降低转弯时灰度阵列离线概率。
- 恢复 `middleware/line_follow` 的正常灰度传感器设计，移除逻辑通道 3 的坏道屏蔽和估计补偿，巡线与 Device Check 均直接使用 8 路真实读数。
- 为 `core/line_tracking` 增加循迹差速限幅，默认将左右轮占空比差值限制在 `10%` 以内，只影响巡线 PID 输出路径，不影响其他底盘运动原语。
- 放宽 E1 循线任务的丢线判定：普通循线状态下不再因单帧空线立即进入 `Line lost`，而是连续空线超过宽限时间后再刹车提示，降低灰度抖动和短暂离线导致的误停概率。
- 新增 `core/motion` 底盘运动原语执行器，统一提供刹车、滑行、直行、倒车、原地左右转和循线命令到 `Chassis` / `LineTracking` 的转换；该模块不包含时间、距离、捕线或任务完成条件判断。
- 将 E1 循迹任务的底盘运动输出切换为 `Motion_CommandLineFollow()` + `Motion_Apply()`，圈数、边线计数、丢线和超时判断仍保留在 app 任务流程中。
- 为 E1 任务增加直角弯状态机：普通循线识别到中心丢线且外侧多路触发后，执行低速差速圆弧转弯，中心重新捕线后恢复循线；`core/line_tracking` 保持纯 PID 巡线职责。
- 在 `Device Check` 中新增 `Yaw Hold` 页面，单击后锁定当前 IMU yaw 角并用云台 yaw 轴闭环保持该角度，再次单击停止；闭环激活时降低控制循环延迟并降低 OLED 刷新频率，减少测试界面对稳定响应的影响。
- 将 `Yaw Hold` 控制律调整为 IMU yaw 角速度前馈加 yaw 角度 PID 修正，并增加最小起转速度补偿、`±240 deg/s` 输出限幅和速度变化率限幅，用于改善云台起转迟滞和速度突变问题。
- 增大 `Yaw Hold` 角速度前馈和最小起转补偿，提高外部旋转初期的云台响应速度。
- 在实验分支中将 `Yaw Hold` 更新触发改为 IMU 有效帧驱动：UART0 中断只更新 IMU 数据和计数，Device Check 主循环检测到新帧后立即执行 yaw 保持控制，减少固定周期轮询带来的额外等待。
- 为 `bsp/canmv` 增加坐标字节序自动纠正和更严格的固定帧校验：当高字节在前解析得到明显异常的五位数、交换字节后落在合理图像坐标范围内时，采用交换后的坐标值；若帧长不匹配或坐标仍不合理，则拒绝整帧，避免随机串口数据被显示为有效 T/L 坐标。
- 将 E3 改为“正方向扫描寻找矩形，识别到有效矩形立即停止扫描并跟踪矩形中心”的流程，新增 `GimbalTracking_UpdateRectCenter()` 和 `GimbalTracking_IsRectValid()`。
- 将 `Device Check` 中 yaw/pitch 步进电机测试触发方式从短按释放恢复为普通单击，保持与其他模块测试交互一致。
- 将 pitch 开环估计位置限位下沉到 `bsp/step_motor`，默认限制为 `-30 deg` 到 `+30 deg`，新增 `StepMotor_SetPitchLimit()` / `StepMotor_GetPitchLimit()`，并让 `middleware/gimbal` 的限位配置同步到底层，避免直接调用 `StepMotor_*` 时绕过 pitch 旋转限幅。
- 新增 `k230/uart1_comm_test.py`，使用 K230 UART1（GPIO3 TX、GPIO4 RX）周期发送兼容旧 CanMV 协议的测试帧，用于验证 K230 与 MSPM0 之间的串口链路。
- 在 `Device Check` 中新增 K230 通信检查页面，显示接收字节数、有效帧数、丢帧数、最后接收字节和解析出的目标/激光坐标。
- 新增 `tools/k230_tool.py`，提供基于 pyserial/raw REPL 的 K230 脚本运行、写入和软复位辅助命令，并补充 `docs/k230-tool.md`。
- 验证 K230 可通过 `COM15` 进入 MicroPython raw REPL、写入 `/sdcard` 文件并执行脚本；将 K230 工具默认写入路径调整为 `/sdcard/main.py`。
- 为 `k230/rect_07.py` 增加 UART2 二进制帧发送，按旧 CanMV 协议向 MSPM0 回传靶心、图像中心和矩形角点。
- 调整 `bsp/canmv` UART2 接收配置，关闭 TX 中断、保留 RX/RX timeout 中断并排空 RX FIFO，同时增加接收字节、有效帧和丢帧诊断变量。
- 将 `core/gimbal_tracking` 默认图像高度调整为 `240`，对齐当前 K230 `320x240` 视觉输出。
- 回退 `Corner test` 中两段式捕线和停稳复检逻辑，恢复为循线识别空线拐角后刹车，再以左轮负值、右轮正值的原地差速左转，便于单独验证直角转弯方向和响应。
- 保留 `Corner test` 中心线检测逻辑：原地差速左转过程中中心 3/4 连续确认后立即刹车。
- 将 `Corner test` 左转差速适当增大为左轮 `-15%`、右轮 `+15%`，提高直角转弯响应。
- 移除 `Corner test` 左转阶段的固定时间兜底停止，转弯只由中心 3/4 连续确认或长按人工中止结束。
- 调整 `Device check` 中 yaw/pitch 步进电机测试触发方式，短按释放消抖完成后立即执行点动，不再等待双击窗口超时。
- 为 `bsp/key` 增加 `Key_IsShortRelease()` 即时释放事件接口，供低延迟测试动作使用。
- 修复 `bsp/step_motor` 初始化未拉起 `EN1/EN2` 的问题，将 yaw/pitch 步进电机使能脚纳入硬件映射宏，并默认按旧版行为置高使能。
- 收紧顶层启动菜单，将 E1 的 1 到 5 圈入口整合为 `E1 Line`，进入后在 E1 子菜单中选择具体圈数。
- 重写 `Device check` 测试程序交互：双击切换模块，单击切换或触发模块内测试；电机模块提供停止、左轮、右轮和双轮测试，yaw/pitch 步进电机拆分为独立模块，灰度实时显示 8 位二进制状态，IMU 实时显示姿态角和角速度，暂不提供 K230 通讯测试页。
- 重写 `app` 层为 2025 年电赛 E 题前三项任务入口，删除旧 `mode/menu/circle_list/mode_tree` 流程。
- 将 `bsp/oled` 从 GPIO 软件 I2C 适配为 SysConfig 生成的硬件 I2C 控制器，使用 `OLED_INST` 和 SSD1306 7 位地址 `0x3C` 完成阻塞式显示写入。
- 将 UART0/Debug 适配到 115200 波特率下的 JY61P 数据回传，IMU 接收从固定 33 字节组合帧改为 11 字节标准子帧滑动解析，并在设备检查页显示姿态角、角速度、加速度和接收状态；UART0 接收保持纯中断路径，并显式设置 1 字节 RX FIFO 中断阈值。
- 更新 CCS projectspec 和 Keil uvprojx，使两套工程文件指向当前 `app/core/middleware/bsp/board` 源码树。
- 验证 CCS/ticlang 直接交叉编译和 Keil/ArmClang rebuild 均通过。
- 为 `bsp/step_motor` 增加统一速度限幅宏 `STEP_MOTOR_MAX_SPEED_DEG_S`，默认限制 yaw/pitch 步进电机速度到 `240 deg/s`。
- 新增 `app/app_e_task.*`，提供寻迹 1 到 5 圈、2 秒靶心瞄准和 4 秒靶心瞄准任务。
- 新增 `docs/interfaces/app_launcher.md`、`docs/interfaces/app_e_task.md` 和 `docs/interfaces/app_device_check.md`，记录应用层当前公开边界。
- 新增 `app/app_device_check.*`，提供底盘、云台、灰度、视觉和 IMU 简化检查页面。
- 重写 `app/main.c`，统一初始化 `Chassis`、`Gimbal`、`LineFollow`、`CanMvUart`、`Ui` 和按键。
- 为 `middleware/line_follow` 增加 `LineFollow_GetActiveCount()`、空线、半线、十字和中心检测接口。
- 删除已无引用的 `middleware/runtime/vision_state.h` 和 `middleware/runtime/project_build_config.h`。
- 新增 `core/gimbal_tracking` 云台视觉跟踪模块，对接 `bsp/canmv`、`core/geometry`、`core/pid` 和 `middleware/gimbal`。
- 删除旧 `core/step_motor_ctrl.*`，移除直接调用 `YP_SMotor_*` 的云台控制路径。
- 旧 `Compute_excur()`、`getDistance()` 和 LED 调试逻辑不迁入 `gimbal_tracking`。
- 新增 `core/line_tracking` 巡线控制模块，对接 `middleware/line_follow`、`core/pid`、`core/kinematics` 和 `middleware/chassis`。
- 删除旧 `core/tracking.*`，移除 `PID_IR_Calc_Custom()` 和 `Motion_Car_Control()`。
- 将 `app/mode.c` 中 `lineWalking_low()` 调用最小替换为 `LineTracking_Update()`。
- 新增 `core/common/core_types.h`，将二维点和二维姿态类型迁移为 `CORE_POINT2F`、`CORE_ATTITUDE2F`。
- 新增 `core/geometry` 二维平面映射模块，提供矩形插值、纸面到矩形映射和圆点计算接口。
- 删除旧 `core/sensor_proc.*`，灰度巡线相关处理不再放入该模块，后续由 `core/line_tracking` 和 `middleware/line_follow` 承担。
- 从 `bsp/common/bsp_common.h` 移除 `BSP_POINT2F` 和 `BSP_ATTITUDE2F`，BSP common 回归为 BSP 状态公共契约。
- 重写 `core/rotation` 为独立子目录，使用 `ROTATION_EULER` 和 `ROTATION_MATRIX` 明确表达姿态角和旋转矩阵。
- 移除旧 `rotation_matrix()`、`matrix_multiplication()`、`matrix_transpose()` 和 `matrix_to_angles()` 接口。
- 为旋转数学模块补充中文接口文档 `docs/interfaces/core_rotation.md`。
- 重写 `core/kinematics` 为独立子目录，保留纯运动学和几何计算接口，不再声明任务流程式动作接口。
- 新增 `KINEMATICS_POSE`、`KINEMATICS_VELOCITY`、`KINEMATICS_DIFFERENTIAL_OUTPUT` 等运动学数据结构。
- 为运动学模块补充中文接口文档 `docs/interfaces/core_kinematics.md`。
- 重写 `core/pid` 为独立子目录，新增 `PID_MODE_POSITION` 和 `PID_MODE_INCREMENTAL` 两种控制模式。
- 将 PID 接口改为 `PID_CONTROLLER` / `PID_CONFIG` / `PID_STATE`，`PID_Update()` 直接返回当前输出并统一处理积分和输出限幅。
- 为 PID 控制器补充中文接口文档 `docs/interfaces/core_pid.md`。
- 新增 `middleware/fault` 系统故障处理服务，公开 `SystemFault_*` 接口。
- 故障停机时同步执行 `Chassis_Brake()` 和 `Gimbal_Stop()`，并通过 `Ui_RenderStatusPage()` 显示错误页。
- 删除旧 `middleware/runtime/system_error_state.h` 和 `middleware/system/error_handler.*`。
- 为系统故障处理服务补充中文接口文档 `docs/interfaces/middleware_system_fault.md`。
- 新增 `middleware/ui` 轻量 OLED UI 渲染层，提供文本页、状态页、列表页和行刷新接口。
- 为 UI 渲染层补充中文接口文档 `docs/interfaces/middleware_ui.md`。
- 新增 `middleware/line_follow` 巡线运行状态服务，公开传感器快照、边线计数和转弯状态接口。
- 删除旧 `middleware/runtime/tracking_runtime.h`，不再暴露 `Digital[]`、`edge`、`turning` 全局变量声明。
- 明确旧 `sInedge` 和 `UpdateSInedge()` 不迁入 `line_follow`，后续在具体使用点重新设计阶段距离逻辑。
- 为巡线运行状态服务补充中文接口文档 `docs/interfaces/middleware_line_follow.md`。
- 新增 `middleware/gimbal` 云台组合服务，对接 `StepMotor_*`，公开 `Gimbal_*` 接口并只保留 pitch 软件限位。
- 删除旧 `middleware/system/step_motor_system.*`。
- 为云台组合服务补充中文接口文档 `docs/interfaces/middleware_gimbal.md`。
- 新增 `middleware/chassis` 底盘组合服务，对接 `TB6612FNG_*` 与 `HallEncoder_*`，公开 `Chassis_*` 接口。
- 删除旧 `middleware/system/motor_system.*`，不再在底盘模块中混放巡线状态和错误消息全局变量。
- 为底盘组合服务补充中文接口文档 `docs/interfaces/middleware_chassis.md`。
- 重写 `bsp/oled` 公开边界：头文件只保留显示接口和硬件宏，软件 I2C 与 SSD1306 低层写入函数收敛为内部实现。
- 为 OLED 显示接口补充中文接口文档 `docs/interfaces/bsp_oled.md`。
- 为 `bsp/imu/wit_sdk` 追加 `WitGetAcc()`、`WitGetGyro()`、`WitGetAttitude()` 和 `WitGetData()`，保留厂家驱动主体和命名风格不变。
- 为 WitMotion IMU 读取接口补充中文接口文档 `docs/interfaces/bsp_wit_sdk.md`。
- 将 `bsp/laser/laser_usart` 重写为 `bsp/canmv/canmv_uart`，公开 `CanMvUart_*` 接口并移除旧 `Laser_*` / `Rect_*` 全局状态接口。
- 为 CanMV UART 协议解析补充中文接口文档 `docs/interfaces/bsp_canmv_uart.md`。
- 重写 `bsp/step_motor`：公开 `StepMotor_*` 通道接口，支持 yaw/pitch 两路速度设置、阻塞运行指定时间、停止和开环估计位置维护。
- 将步进电机位置接口命名为 `StepMotor_GetEstimatedPosition()` / `StepMotor_ResetEstimatedPosition()`，明确其不是物理归零或反馈位置。
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
- 重写 `bsp/common` 公共契约：统一使用 SDK 风格的全大写类型名 `BSP_STATUS`。
- 将 CanMV 状态码从 `bsp/common` 移入 CanMV UART 驱动，由对接 CanMV 的驱动自行维护。
- 将工程源码重构为 `app`、`core`、`middleware`、`bsp` 四层结构。
- 将自维护源码和目录统一为小写加下划线命名。
- 将 SysConfig 和启动资源迁入 `board`，IDE 工程元数据迁入 `project`，脚本目录统一为 `tools`。
- 移除 BSP 对上层的反向依赖：按键和 OLED 改为依赖 `bsp/time`，步进电机状态更新时间改为显式传参，视觉和 IMU 状态由 BSP 接口导出。
- 更新 CCS 和 Keil 工程路径，使其指向新的分层源码树。
- 为 `app`、`core`、`middleware`、`bsp` 主要公开头文件补充 Doxygen 风格中文注释，明确模块职责、结构体字段、函数参数、单位和调用约束。

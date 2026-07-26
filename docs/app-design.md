# app 层设计（裸机菜单调度框架）

## 目标形态

上电初始化后进入**任务选择菜单**；短按选择一个任务执行；任务完成或用户中止后退回菜单。
裸机（nortos）下用**协作式时间触发超循环**实现：ISR 只做时基/采样，主循环按周期分派
run-to-completion 任务。

## 顶层状态机

```
                ┌───────────── 致命故障 ──────────► SystemFault_Halt (终态)
                │
 INIT ──ok──► MENU ──短按ENTER──► RUN ──DONE/短按BACK──► MENU
                ▲                 │
                │                 └──on_tick 返回 FAULT──► FAULT ──短按ENTER──► MENU
                └──────────────────────────────────────────┘
```

- **INIT**：`App_Init()` 期间；完成后置 MENU。
- **MENU**：委派 `app_menu` 的嵌套菜单导航（见下）；选中任务时返回给 app_mode 进入 RUN。
- **RUN**：调度器每 20ms 调 `App_ControlTick` 委派 `current_task->on_tick`；短按 BACK 中止。
- **FAULT**：可恢复故障；显示错误页，短按 ENTER 复位回 MENU。

## 嵌套菜单（app_menu）

菜单是一棵 `MENU_NODE` 树，每个 `MENU_ITEM` 或指向子菜单、或指向任务；导航用一个
`node_stack`/`sel_stack` 深度栈。`app_menu` 不反向依赖 `app_mode`——选中任务时由
`Menu_Tick()` 返回该任务描述符，`app_mode` 再调 `App_EnterRun`。当前菜单树（`app_menu_def.c`）：

```
Main Menu
├── Line Follow         (task)   循迹测试: line_follow 完整闭环巡线 + 陀螺增稳, 电平图/误差/双轮占空比, 丢线刹停
├── Straight Test       (submenu)
│   ├── Duty Open       (task)   左右轮同占空比开环
│   ├── Speed Closed    (task)   左右轮独立速度 PID 直行
│   ├── Duty+Gyro Rate  (task)   基础占空比 + gz=0 角速度闭环
│   ├── Duty+Yaw Hold   (task)   基础占空比 + 启动航向锁定
│   ├── Ramp Yaw Hold   (task)   占空比斜坡，全程启动航向闭环
│   ├── 80 Rate->Yaw    (task)   1 s 角速度闭环后切换航向闭环
│   ├── 80 Enc->Yaw     (task)   1 s 双轮累计路程差闭环后切换航向闭环
│   ├── 80 Int->Yaw     (task)   500 ms 固定周期积分航向对照版
│   └── 100 Int->Yaw    (task)   500 ms 新样本积分 + 融合角修正，速度优先
└── Device Check        (submenu)
    ├── Gyro JY61P       (task)   JY61P 陀螺/姿态/温度 + 诊断计数
    ├── Yaw A/B          (task)   无电机对比 gz 积分角 A 与融合角 B
    ├── Gyro MPU6050     (task)   物理六轴/温度/静态倾角双页; 进挂起/出恢复 JY61P
    ├── Grayscale        (task)   数字量 GPIO 版, 8 路 mask 二进制 + 触发数
    ├── Gray I2C         (task)   感为 I2C 版(0x4F), 8 路数字量 + 在线/固件版本
    ├── Yahboom I2C      (task)   Yahboom I2C 版(0x12), X1→X8 位图 + 诊断计数
    ├── TB6612           (task)   短按单次低速脉冲(20%/300ms) + 编码器响应, 抬轮提示
    ├── Encoder          (task)   双轮 count/speed/dir
    ├── Speed PID        (task)   双轮速度闭环, 按键给目标 + 目标/实测对比, 整定用, 抬轮提示
    ├── Duty Sweep       (task)   开环占空比阶梯, 查各占空比下编码器读速(死区/噪声诊断), 抬轮提示
    └── BlueTooth        (task)   蓝牙串口(UART0,9600)收发测试: 显示收到 ASCII, EN 键发 "hello"
```

导航按键（仍仅短按）：`UP/DOWN` 移动、`ENTER` 进入子菜单/任务、`BACK` 返回上级。

所有状态转移集中在 `app_mode.c` 的入口函数（`App_EnterRun/App_ExitRun/App_RaiseFault`）；
进 RUN 必 `Chassis_ResetDistance`+`on_enter`，出 RUN 必 `Chassis_Brake`。任务只通过 `on_tick`
返回值表达迁移意图，不直接改模式/调度。

## 两级故障模型

- **致命/不可恢复**：`SystemFault_Handler → Halt`（刹车+显示+`__disable_irq`+死循环），
  用于 init 失败等。绕过状态机。
- **可恢复**：app 的 FAULT 态，`App_RaiseFault` 记录+刹车+进 FAULT，调度器继续跑，短按复位。

## 调度器

`app_scheduler`：任务表 `{cb, period_ms, last_ms}`，`Scheduler_Run()` 按 `BSP_Time_GetMs()`
分派到期任务。注册：`App_ControlTick`(20ms)、`App_UiTick`(50ms)。

`SysTick_Handler`(1ms，app 自持)：`BSP_Time_TickInc()` + `Key_Scan()`；`tick_active` 门控
避免初始化完成前误触发。

## 中断归属

| ISR | 归属 | 职责 |
|---|---|---|
| `SysTick_Handler` (1ms) | app/app_scheduler | 时基递增 + 按键扫描 |
| `GROUP1_IRQHandler` | bsp/hall_encoder | 编码器边沿 |
| `TIMER_0_INST_IRQHandler` (20ms) | bsp/hall_encoder | 编码器测速 |
| `I2C0_IRQHandler` | bsp/imu | JY61P I2C 状态机 |

## 按键交互（仅短按）

| 模式 | UP | DOWN | ENTER | BACK |
|---|---|---|---|---|
| MENU | 上移选择 | 下移选择 | 进入任务 | — |
| RUN | 任务定义 | 任务定义 | 任务定义 | 中止回菜单 |
| FAULT | — | — | 复位回菜单 | — |

`KEY_ID`：`KEY_ID_UP / KEY_ID_DOWN / KEY_ID_ENTER / KEY_ID_BACK`。用 `Key_GetEvent()==KEY_EVENT_SHORT_PRESS` 消费。

---

## 任务开发规范

### 三步接入
1. 在任务源文件（如 `app_checks.c`）写私有状态 + 三个钩子 `TaskXxx_Enter/Tick/Exit`，
   导出一个 `const APP_TASK_DESC`。
2. 在 `app_menu_def.c` 的菜单树挂一项 `{ .name="名字", .kind=MENU_ENTRY_TASK, .u.task=&描述符 }`（`on_exit` 可 NULL）。
3. 编译。菜单出现该项，其它文件不动。

### 钩子契约

| 钩子 | 必须 | 禁止 |
|---|---|---|
| `on_enter` | 复位本任务全部 `static` 私有状态（相位/计数/滤波/`PID_Reset`/基准） | 驱动执行器、写模式 |
| `on_tick(dt)` | 走取值→计算→执行→监督，返回 RUNNING/DONE/FAULT | 阻塞/延时、直接读慢速外设、写模式/调度 |
| `on_exit` | 仅任务专属收尾（可 NULL） | 把安全停机只放这里（框架已 Brake） |

返回值：`RUNNING` 继续；`DONE` 框架刹停回菜单；`FAULT` 进 FAULT 态（致命才用 `SystemFault_Handler`）。

### 硬性规范
1. **非阻塞**：多步等待拆成相位，用 `BSP_Time_GetMs()` 差值推进，不忙等。
2. **dt 权威**：只用传入 `dt`（=控制周期 0.02f），传给 `PID_Update` 等。
3. **输入取快照/getter**：经 `Chassis_Get*` / IMU getter / 采样快照，不在任务里做阻塞读。
4. **执行器单写者**：仅 `on_tick` 内、每拍至多一次 `Chassis_SetDuty/Brake`。
5. **不碰框架内部**：只靠返回值迁移，不写 `app_mode`、不调调度/迁移函数。
6. **进出对称**：`on_enter` Reset 的每项保证可重复启动（任务从菜单可反复进入）。
7. **每相位带超时**兜底 → 返回 FAULT。
8. **风格**：`static` 私有状态不加 `s_` 前缀；钩子命名 `TaskXxx_*`；魔法数提常量。
9. **UI**：RUN 运行页由任务 `on_tick` 低频自渲染；MENU/FAULT 页归框架。

### 参考实现
Device Check 共 11 项（`app_checks.c` 10 项诊断 + `app_bt_task.c` 蓝牙收发），演示了 enter 复位、tick 非阻塞采样、按变化节流刷屏、
仅靠 BACK 短按退出；MPU6050、感为灰度与 Yahboom 检查页还示范了 on_enter/on_exit
挂起/恢复 JY61P，以分时共用 I2C0。

### 公共支持
- `app_fmt`：`AppFmt_I32/AppFmt_Fixed` 定点数字→字符串，供自检显示，不引浮点 printf。

### 后续扩展点
- **传感器采样进 ISR**：灰度/编码器等便宜同步量可在 SysTick/专用定时器采进 volatile 快照，
  IMU 用「定时器 kick + I2C ISR 完成」；控制任务只消费快照（降低控制环输入抖动）。MPU6050
  DMP 阻塞重，留任务。
- **命令层**：蓝牙/调试 UART 的 RX/TX 中断 + 环形缓冲 + 命令解析，按需在 app 层新增
  `UARTx_IRQHandler`。
- **MPU6050 DMP 姿态**：当前 Device Check 使用基础模式 `MPU6050_GetMeasurement()`，
  UP/DOWN 在加速度/温度页与角速度/静态 pitch/roll 页之间切换。DMP 已有阻塞式 bring-up
  入口 `MPU6050_RunDmpTest()`；若要集成到协作式 app，仍需把初始化与 FIFO 消费改造成
  非阻塞任务状态机。

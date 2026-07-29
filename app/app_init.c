/**
 * @file  app_init.c
 * @brief 上电初始化时序：SysConfig → BSP → 中间件 → 框架 → 注册调度任务。
 */
#include "app_init.h"
#include "app_mode.h"
#include "app_scheduler.h"

#include "ti_msp_dl_config.h"
#include "bsp_time.h"
#include "debug_uart.h"
#include "ui.h"
#include "key.h"
#include "chassis.h"
#include "step_motor.h"
#include "system_fault.h"

/* 控制/UI 两个调度任务的周期，单位 ms。 */
#define APP_CONTROL_PERIOD_MS 20U
#define APP_UI_PERIOD_MS      50U

/*
 * 步进电机的调度周期，取自驱动给出的值。
 * 比控制周期快一倍：判越界晚一拍，摆杆就多冲一拍的距离，而那段距离要由
 * STEP_MOTOR_ENC_LIMIT_MARGIN_COUNTS 兜住。
 */
#define APP_STEP_TICK_PERIOD_MS STEP_MOTOR_TICK_PERIOD_MS

/*
 * 步进电机的周期入口：位置伺服 + 上电抬升 + 限位守护 + 编码器采样。
 * 与应用状态无关地跑——限位保护在菜单里、任务里、故障态下都得成立，
 * 而且**不调它电机根本不会动**（位置指令只登记目标，脉冲由这里的伺服下发）。
 */
static void App_StepMotorTick(void){
    StepMotor_Tick(BSP_Time_GetMs());
}

void App_Init(void){
    SYSCFG_DL_init();
    BSP_Time_Init();
    DebugUart_Init();   /* 非阻塞 debug 串口(Debug_Ex/UART1): Speed PID 等遥测输出用。 */

    /*
     * Ui/Chassis 先于任何可能触发 SystemFault 的步骤初始化，
     * 满足 SystemFault_Halt 需 Ui/Chassis 已就绪的前置契约。
     */
    Ui_Init();
    if (Chassis_Init() != BSP_STATUS_OK){
        SystemFault_Handler(SYSTEM_FAULT_HARDWARE, "Chassis init");  /* 终态，不返回 */
    }
    Key_Init();

    /*
     * 摆杆步进：上电即失能（见 StepMotor_Init），行程未标定前不给摆杆任何力矩。
     * 失败不算致命——底盘部分照常可用，摆杆故障会在 Device Check 页上暴露出来。
     */
    (void)StepMotor_Init();

    /* 框架：状态机 + 调度任务表。 */
    App_Mode_Init();
    Scheduler_Init();
    (void)Scheduler_AddTask(App_ControlTick, APP_CONTROL_PERIOD_MS);
    (void)Scheduler_AddTask(App_UiTick, APP_UI_PERIOD_MS);
    (void)Scheduler_AddTask(App_StepMotorTick, APP_STEP_TICK_PERIOD_MS);

    /* 全部就绪后放开 SysTick 时基/按键扫描。 */
    Scheduler_EnableTick();
}

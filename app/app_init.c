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
#include "system_fault.h"

/* 控制/UI 两个调度任务的周期，单位 ms。 */
#define APP_CONTROL_PERIOD_MS 20U
#define APP_UI_PERIOD_MS      50U

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

    /* 框架：状态机 + 调度任务表。 */
    App_Mode_Init();
    Scheduler_Init();
    (void)Scheduler_AddTask(App_ControlTick, APP_CONTROL_PERIOD_MS);
    (void)Scheduler_AddTask(App_UiTick, APP_UI_PERIOD_MS);

    /* 全部就绪后放开 SysTick 时基/按键扫描。 */
    Scheduler_EnableTick();
}

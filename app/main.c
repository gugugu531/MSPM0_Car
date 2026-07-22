/**
 * @file  main.c
 * @brief 固件入口：初始化后进入协作式调度超循环。
 *
 * 应用层为“菜单选择 → 执行选中任务 → 退回菜单”的状态机(见 app_mode)，
 * 由时间触发调度器(见 app_scheduler)驱动，任务注册见 app_tasks。
 * SysTick_Handler 定义在 app_scheduler.c（时基递增 + 按键扫描）。
 */
#include "app_init.h"
#include "app_scheduler.h"
#include "ti_msp_dl_config.h"

int main(void){
    App_Init();
    __enable_irq();

    while (1){
        Scheduler_Run();
    }
}

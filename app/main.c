/**
 * @file  main.c
 * @brief 极简骨架: SysConfig 外设/中断初始化后进入空循环。
 *
 * 云台/瞄准子系统与原任务框架已移除, 此处仅保留启动骨架, 供后续按新需求重建 app。
 * 底层驱动库(bsp)、通用中间件(chassis/line_follow/line_tracking/ui/fault)与
 * 算法(core: pid/kinematics)均保留, 但当前无调用者, 由链接器 GC。
 */
#include "ti_msp_dl_config.h"

int main(void){
    SYSCFG_DL_init();
    __enable_irq();

    while (1){
    }
}

/*
 * SysConfig 使能了 SysTick(1ms 周期中断)。提供空处理以避免落到 startup 的
 * weak SysTick_Handler(B . 死循环)。重建 app 时在此接入调度。
 */
void SysTick_Handler(void){
}

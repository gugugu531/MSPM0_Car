/**
 * @file  main.c
 * @brief 系统主入口，完成基础外设初始化并启动应用流程
 */
#include "AppState.h"
#include "SystemTime.h"
#include "TrackingRuntime.h"
#include "HallEncoder.h"
#include "Initialize.h"
#include "Key.h"
#include "LaserUsart.h"
#include "Mode.h"
#include "Oled.h"
#include "ti_msp_dl_config.h"

uint32_t tick;

int main(void){
    SYSCFG_DL_init();
    __enable_irq();

    Encoder_Init();
    Encoder_TimerInit();
    DL_TimerG_startCounter(TIMER_0_INST);

    Laser_USART_Init();
    OLED_Init();
    Motor_SystemInit();
    Key_Init();

    mode_problem_h_2();

    while (1){
    }
}

void UART2_IRQHandler(void){
    switch (DL_UART_getPendingInterrupt(LASER_UART)){
        case DL_UART_IIDX_RX:
            CanMV_Process();
            break;
        default:
            break;
    }
}

void SysTick_Handler(void){
    static int scan_divider = 0;

    scan_divider++;
    tick++;

    if (scan_divider == 10){
        Key_Scan();
        scan_divider = 0;
    }
}

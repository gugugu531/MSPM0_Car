#include "app_device_check.h"
#include "app_launcher.h"
#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "gimbal.h"
#include "key.h"
#include "line_follow.h"
#include "system_fault.h"
#include "ti_msp_dl_config.h"
#include "ui.h"

#include <stdint.h>

static void App_InitSystems(void){
    SYSCFG_DL_init();
    BSP_Time_Init();
    __enable_irq();

    Ui_Init();
    Key_Init();

    (void)Chassis_Init();
    (void)Gimbal_Init();
    (void)LineFollow_Init();
    (void)CanMvUart_Init();
    SystemFault_Clear();

    DL_TimerA_startCounter(TIMER_0_INST);
    DL_UART_Main_enableInterrupt(Debug_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(Debug_INST_INT_IRQN);
    NVIC_EnableIRQ(Debug_INST_INT_IRQN);
}

int main(void){
    App_InitSystems();
    App_Launch();

    while (1){
    }
}

void UART0_IRQHandler(void){
    if (DL_UART_getPendingInterrupt(Debug_INST) == DL_UART_IIDX_RX){
        AppDeviceCheck_ProcessImuByte((uint8_t)DL_UART_Main_receiveData(Debug_INST));
    }
}

void UART2_IRQHandler(void){
    if (DL_UART_getPendingInterrupt(CANMV_UART_INST) == DL_UART_IIDX_RX){
        CanMvUart_ProcessRx();
    }
}

void SysTick_Handler(void){
    static uint8_t scan_divider = 0U;

    BSP_Time_TickInc();

    scan_divider++;
    if (scan_divider >= 10U){
        Key_Scan();
        scan_divider = 0U;
    }
}

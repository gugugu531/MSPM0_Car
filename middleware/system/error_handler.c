#include "system_error_state.h"
#include "error_handler.h"
#include "motor_system.h"
#include "oled.h"

void error_handler(void){
    Motor_Brake();
    OLED_Clear();
    OLED_ShowString(0, 0, error_message, 8);
    while (1){
        ;
    }
}

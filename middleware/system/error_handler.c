#include "system_error_state.h"
#include "error_handler.h"
#include "chassis.h"
#include "oled.h"

void error_handler(void){
    (void)Chassis_Brake();
    OLED_Clear();
    OLED_ShowString(0, 0, error_message, 8);
    while (1){
        ;
    }
}

#include "AppState.h"
#include "ErrorHandler.h"
#include "Initialize.h"
#include "Oled.h"

void error_handler(void){
    Motor_Brake();
    OLED_Clear();
    OLED_ShowString(0, 0, error_message, 8);
    while (1){
        ;
    }
}

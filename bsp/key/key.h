/**
 * @file  key.h
 * @brief BSP 按键驱动接口。
 */
#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef KEY1_PORT
#define KEY1_PORT Key_PORT
#endif

#ifndef KEY1_PIN
#define KEY1_PIN Key_PIN_1_PIN
#endif

#ifndef KEY1_ACTIVE_LOW
#define KEY1_ACTIVE_LOW 1U
#endif

#ifndef KEY_DEBOUNCE_MS
#define KEY_DEBOUNCE_MS 20U
#endif

#ifndef KEY_SHORT_PRESS_MIN_MS
#define KEY_SHORT_PRESS_MIN_MS 50U
#endif

#ifndef KEY_LONG_PRESS_MS
#define KEY_LONG_PRESS_MS 1000U
#endif

#ifndef KEY_DOUBLE_CLICK_MS
#define KEY_DOUBLE_CLICK_MS 300U
#endif

typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_MAX
} KEY_ID;

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_DOUBLE_CLICK
} KEY_EVENT;

void Key_Init(void);
void Key_Scan(void);
KEY_EVENT Key_GetEvent(KEY_ID key_id);
bool Key_IsPressed(KEY_ID key_id);
bool Key_IsShortPress(KEY_ID key_id);
bool Key_IsLongPress(KEY_ID key_id);
bool Key_IsDoubleClick(KEY_ID key_id);
void Key_ClearEvent(KEY_ID key_id);
void Key_ClearAllEvents(void);

#define Key_Read()         Key_IsPressed(KEY_ID_1)
#define Key_short_press()  Key_IsShortPress(KEY_ID_1)
#define Key_long_press()   Key_IsLongPress(KEY_ID_1)
#define Key_double_click() Key_IsDoubleClick(KEY_ID_1)

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */

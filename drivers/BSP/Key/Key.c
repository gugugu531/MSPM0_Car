/**
 * @file  Key.c
 * @brief 按键驱动实现，支持消抖、短按、长按和双击检测
 */

#include "Key.h"
#include "SystemTime.h"

static key_info_t keys[KEY_ID_MAX];
static key_event_t key_events[KEY_ID_MAX];

/**
 * @brief 获取系统时间戳 (毫秒)
 * @return 当前时间戳
 */
static uint32_t Get_SystemTick(void){
    return tick;
}

/**
 * @brief 读取指定按键的GPIO状态
 * @param key_id 按键ID
 * @return GPIO引脚状态 (0或1)
 */
static uint8_t Key_ReadGPIO(key_id_t key_id){
    if (key_id >= KEY_ID_MAX){
        return KEY_RELEASED_LEVEL;
    }

    key_info_t *key = &keys[key_id];
    uint32_t pin_state = DL_GPIO_readPins(key->port, key->pin);

    return (pin_state != 0) ? 1 : 0;
}

/**
 * @brief 按键系统初始化
 */
void Key_Init(void){
    keys[KEY_ID_1].port = KEY_PORT;
    keys[KEY_ID_1].pin = KEY_PIN;

    for (uint8_t i = 0; i < KEY_ID_MAX; i++){
        keys[i].state = KEY_STATE_IDLE;
        keys[i].current_level = KEY_RELEASED_LEVEL;
        keys[i].last_level = KEY_RELEASED_LEVEL;
        keys[i].press_time = 0;
        keys[i].release_time = 0;
        keys[i].last_event_time = 0;
        keys[i].click_count = 0;
        keys[i].long_press_flag = false;
        key_events[i] = KEY_EVENT_NONE;
    }

    for (uint8_t i = 0; i < KEY_ID_MAX; i++){
        keys[i].current_level = Key_ReadGPIO((key_id_t)i);
        keys[i].last_level = keys[i].current_level;
    }
}

/**
 * @brief 单个按键状态机处理
 * @param key_id 按键ID
 */
static void Key_StateMachine(key_id_t key_id){
    if (key_id >= KEY_ID_MAX){
        return;
    }

    key_info_t *key = &keys[key_id];
    uint32_t current_time = Get_SystemTick();

    key->current_level = Key_ReadGPIO(key_id);

    switch (key->state){
        case KEY_STATE_IDLE:
            if (key->current_level == KEY_PRESSED_LEVEL && key->last_level == KEY_RELEASED_LEVEL){
                key->state = KEY_STATE_DEBOUNCE;
                key->press_time = current_time;
            }
            break;

        case KEY_STATE_DEBOUNCE:
            if (key->current_level == KEY_PRESSED_LEVEL){
                if (current_time - key->press_time >= KEY_DEBOUNCE_TIME){
                    key->state = KEY_STATE_PRESSED;
                    key_events[key_id] = KEY_EVENT_PRESS;
                    key->long_press_flag = false;
                }
            } else{
                key->state = KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_PRESSED:
            if (key->current_level == KEY_RELEASED_LEVEL){
                key->release_time = current_time;
                uint32_t press_duration = current_time - key->press_time;

                if (press_duration >= KEY_SHORT_PRESS_TIME && press_duration < KEY_LONG_PRESS_TIME){
                    key->click_count++;
                    key->last_event_time = current_time;
                    key->state = KEY_STATE_WAIT_DOUBLE;
                } else if (press_duration >= KEY_LONG_PRESS_TIME){
                    key->state = KEY_STATE_IDLE;
                    key->click_count = 0;
                } else{
                    key->state = KEY_STATE_IDLE;
                }

                key_events[key_id] = KEY_EVENT_RELEASE;
            } else{
                if (!key->long_press_flag && (current_time - key->press_time >= KEY_LONG_PRESS_TIME)){
                    key->long_press_flag = true;
                    key->state = KEY_STATE_LONG_PRESS;
                    key_events[key_id] = KEY_EVENT_LONG_PRESS;
                    key->last_event_time = current_time;
                }
            }
            break;

        case KEY_STATE_LONG_PRESS:
            if (key->current_level == KEY_RELEASED_LEVEL){
                key->state = KEY_STATE_IDLE;
                key->click_count = 0;
                key_events[key_id] = KEY_EVENT_RELEASE;
            } else{
                if (current_time - key->last_event_time >= KEY_REPEAT_TIME){
                    key_events[key_id] = KEY_EVENT_LONG_PRESS_REPEAT;
                    key->last_event_time = current_time;
                }
            }
            break;

        case KEY_STATE_WAIT_DOUBLE:
            if (key->current_level == KEY_PRESSED_LEVEL && key->last_level == KEY_RELEASED_LEVEL){
                if (current_time - key->release_time <= KEY_DOUBLE_CLICK_TIME){
                    key_events[key_id] = KEY_EVENT_DOUBLE_CLICK;
                    key->click_count = 0;
                    key->state = KEY_STATE_DEBOUNCE;
                    key->press_time = current_time;
                } else{
                    key_events[key_id] = KEY_EVENT_SHORT_PRESS;
                    key->click_count = 0;
                    key->state = KEY_STATE_DEBOUNCE;
                    key->press_time = current_time;
                }
            } else if (current_time - key->release_time > KEY_DOUBLE_CLICK_TIME){
                key_events[key_id] = KEY_EVENT_SHORT_PRESS;
                key->click_count = 0;
                key->state = KEY_STATE_IDLE;
            }
            break;

        default:
            key->state = KEY_STATE_IDLE;
            break;
    }
}

void Key_Scan(void){
    for (uint8_t i = 0; i < KEY_ID_MAX; i++){
        Key_StateMachine((key_id_t)i);
    }
}

/**
 * @brief 获取指定按键的事件
 * @param key_id 按键ID
 * @return 按键事件类型
 */
key_event_t Key_GetEvent(key_id_t key_id){
    if (key_id >= KEY_ID_MAX){
        return KEY_EVENT_NONE;
    }

    key_event_t event = key_events[key_id];
    key_events[key_id] = KEY_EVENT_NONE;
    return event;
}

/**
 * @brief 检查指定按键是否按下
 * @param key_id 按键ID
 * @return true=按下, false=释放
 */
bool Key_IsPressed(key_id_t key_id){
    if (key_id >= KEY_ID_MAX){
        return false;
    }

    return keys[key_id].current_level == KEY_PRESSED_LEVEL;
}

/**
 * @brief 检查指定按键的短按事件
 * @param key_id 按键ID
 * @return true=检测到短按, false=无短按
 */
bool Key_IsShortPress(key_id_t key_id){
    if (key_id >= KEY_ID_MAX){
        return false;
    }

    if (key_events[key_id] == KEY_EVENT_SHORT_PRESS){
        key_events[key_id] = KEY_EVENT_NONE;
        return true;
    }

    return false;
}

/**
 * @brief 检查指定按键的长按事件
 * @param key_id 按键ID
 * @return true=检测到长按, false=无长按
 */
bool Key_IsLongPress(key_id_t key_id){
    if (key_id >= KEY_ID_MAX){
        return false;
    }

    if (key_events[key_id] == KEY_EVENT_LONG_PRESS){
        key_events[key_id] = KEY_EVENT_NONE;
        return true;
    }

    return false;
}

/**
 * @brief 检查指定按键的双击事件
 * @param key_id 按键ID
 * @return true=检测到双击, false=无双击
 */
bool Key_IsDoubleClick(key_id_t key_id){
    if (key_id >= KEY_ID_MAX){
        return false;
    }

    if (key_events[key_id] == KEY_EVENT_DOUBLE_CLICK){
        key_events[key_id] = KEY_EVENT_NONE;
        return true;
    }

    return false;
}

/**
 * @brief 清除指定按键的所有事件
 * @param key_id 按键ID
 */
void Key_ClearEvent(key_id_t key_id){
    if (key_id < KEY_ID_MAX){
        key_events[key_id] = KEY_EVENT_NONE;
    }
}

/**
 * @brief 清除所有按键的所有事件
 */
void Key_ClearAllEvents(void){
    for (uint8_t i = 0; i < KEY_ID_MAX; i++){
        key_events[i] = KEY_EVENT_NONE;
    }
}

/**
 * @file  Key.h
 * @brief 按键驱动接口，支持消抖、短按、长按和双击检测
 */

#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 按键硬件配置 */
#define KEY_PORT                Key_PORT
#define KEY_PIN                 Key_PIN_1_PIN

/* 按键逻辑电平定义 */
#define KEY_PRESSED_LEVEL       0
#define KEY_RELEASED_LEVEL      (!KEY_PRESSED_LEVEL)

/* 按键时间参数，单位 ms */
#define KEY_DEBOUNCE_TIME       20
#define KEY_SHORT_PRESS_TIME    50
#define KEY_LONG_PRESS_TIME     1000
#define KEY_REPEAT_TIME         200
#define KEY_DOUBLE_CLICK_TIME   300

/* 按键事件类型 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS,
    KEY_EVENT_RELEASE,
    KEY_EVENT_SHORT_PRESS,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_LONG_PRESS_REPEAT,
    KEY_EVENT_DOUBLE_CLICK
} key_event_t;

/* 按键状态 */
typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_DEBOUNCE,
    KEY_STATE_PRESSED,
    KEY_STATE_LONG_PRESS,
    KEY_STATE_WAIT_DOUBLE
} key_state_t;

/* 按键 ID */
typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_MAX
} key_id_t;

/* 按键状态信息 */
typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    key_state_t state;
    uint8_t current_level;
    uint8_t last_level;
    uint32_t press_time;
    uint32_t release_time;
    uint32_t last_event_time;
    uint8_t click_count;
    bool long_press_flag;
} key_info_t;

/**
 * @brief 按键系统初始化
 * @note 初始化所有按键的 GPIO 和状态机
 */
void Key_Init(void);

/**
 * @brief 按键扫描处理
 * @note 建议在主循环或定时器中以 5 ms 到 10 ms 周期调用
 */
void Key_Scan(void);

/**
 * @brief 获取指定按键的事件
 * @param key_id 按键ID
 * @return 按键事件类型
 */
key_event_t Key_GetEvent(key_id_t key_id);

/**
 * @brief 检查指定按键是否按下
 * @param key_id 按键ID
 * @return true=按下, false=释放
 */
bool Key_IsPressed(key_id_t key_id);

/**
 * @brief 检查指定按键的短按事件
 * @param key_id 按键ID
 * @return true=检测到短按, false=无短按
 */
bool Key_IsShortPress(key_id_t key_id);

/**
 * @brief 检查指定按键的长按事件
 * @param key_id 按键ID
 * @return true=检测到长按, false=无长按
 */
bool Key_IsLongPress(key_id_t key_id);

/**
 * @brief 检查指定按键的双击事件
 * @param key_id 按键ID
 * @return true=检测到双击, false=无双击
 */
bool Key_IsDoubleClick(key_id_t key_id);

/**
 * @brief 清除指定按键的所有事件
 * @param key_id 按键ID
 */
void Key_ClearEvent(key_id_t key_id);

/**
 * @brief 清除所有按键的所有事件
 */
void Key_ClearAllEvents(void);

/* 兼容旧接口的便捷宏 */
#define Key_Read()                  Key_IsPressed(KEY_ID_1)
#define Key_short_press()           Key_IsShortPress(KEY_ID_1)
#define Key_long_press()            Key_IsLongPress(KEY_ID_1)
#define Key_double_click()          Key_IsDoubleClick(KEY_ID_1)

#endif /* KEY_H */

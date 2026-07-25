/**
 * @file  app_bt_task.h
 * @brief 蓝牙串口接收测试任务描述符(供菜单引用)。
 */
#ifndef APP_BT_TASK_H
#define APP_BT_TASK_H

#include "app_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 蓝牙串口接收测试(BlueTooth/UART0, 9600): OLED 显示收到的 ASCII 与字节/丢弃计数。 */
extern const APP_TASK_DESC APP_CHK_BLUETOOTH;

#ifdef __cplusplus
}
#endif

#endif /* APP_BT_TASK_H */

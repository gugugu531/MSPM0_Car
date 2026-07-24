/**
 * @file  app_menu_def.c
 * @brief 菜单树实例定义（根菜单 + Device Check 子菜单）。
 *
 * 加菜单项 = 改这两张表。任务描述符来自 app_checks.h（任务类型契约见 app_task.h）。
 */
#include "app_menu.h"
#include "app_checks.h"

/* --- Device Check 子菜单：8 个外设自检 --- */
static const MENU_ITEM device_check_items[] = {
    { .name = "Gyro JY61P",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GYRO_JY61P },
    { .name = "Gyro MPU6050", .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GYRO_MPU6050 },
    { .name = "Grayscale",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GRAYSCALE },
    { .name = "Gray I2C",     .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GRAY_I2C },
    { .name = "TB6612",       .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_TB6612 },
    { .name = "Encoder",      .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_ENCODER },
    { .name = "Speed PID",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_SPEED_PID },
    { .name = "Duty Sweep",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_DUTY_SWEEP },
};

static const MENU_NODE device_check_menu = {
    "Device Check",
    device_check_items,
    (uint8_t)(sizeof(device_check_items) / sizeof(device_check_items[0])),
};

/* --- 根菜单 --- */
static const MENU_ITEM root_items[] = {
    { .name = "Device Check", .kind = MENU_ENTRY_SUBMENU, .u.submenu = &device_check_menu },
};

const MENU_NODE APP_ROOT_MENU = {
    "Main Menu",
    root_items,
    (uint8_t)(sizeof(root_items) / sizeof(root_items[0])),
};

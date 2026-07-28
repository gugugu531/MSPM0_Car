/**
 * @file  app_menu_def.c
 * @brief 菜单树实例定义（根菜单 + 功能测试子菜单）。
 *
 * 加菜单项 = 改这两张表。任务描述符来自 app_checks.h（任务类型契约见 app_task.h）。
 */
#include "app_menu.h"
#include "app_checks.h"
#include "app_line_task.h"
#include "app_straight_task.h"
#include "app_turn_task.h"
#include "app_bt_task.h"

/* --- Device Check 子菜单：12 个外设自检 --- */
static const MENU_ITEM device_check_items[] = {
    { .name = "Gyro JY61P",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GYRO_JY61P },
    { .name = "Yaw A/B",       .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_YAW_AB },
    { .name = "Gyro MPU6050", .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GYRO_MPU6050 },
    { .name = "Gyro CY-Z",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GYRO_CY_Z },
    { .name = "Grayscale",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GRAYSCALE },
    { .name = "Gray I2C",     .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GRAY_I2C },
    { .name = "Yahboom I2C",  .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_YAHBOOM_I2C },
    { .name = "TB6612",       .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_TB6612 },
    { .name = "Encoder",      .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_ENCODER },
    { .name = "Speed PID",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_SPEED_PID },
    { .name = "Duty Sweep",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_DUTY_SWEEP },
    { .name = "BlueTooth",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_BLUETOOTH },
};

static const MENU_NODE device_check_menu = {
    "Device Check",
    device_check_items,
    (uint8_t)(sizeof(device_check_items) / sizeof(device_check_items[0])),
};

/* --- 直行测试子菜单：4 种基础方式 + 5 种启动/切换实验 --- */
static const MENU_ITEM straight_test_items[] = {
    { .name = "Duty Open",      .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_DUTY_OPEN_TEST },
    { .name = "Speed Closed",   .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_SPEED_TEST },
    { .name = "Duty+Gyro Rate", .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_GYRO_RATE_TEST },
    { .name = "Duty+Yaw Hold",  .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_GYRO_HEADING_TEST },
    { .name = "Ramp Yaw Hold",  .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_RAMP_HEADING_TEST },
    { .name = "80 Rate->Yaw",   .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_RATE_THEN_HEADING_TEST },
    { .name = "80 Enc->Yaw",    .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_ENCODER_THEN_HEADING_TEST },
    { .name = "80 Int->Yaw",    .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_INTEGRATED_THEN_HEADING_TEST },
    { .name = "100 Int->Yaw",   .kind = MENU_ENTRY_TASK, .u.task = &APP_STRAIGHT_FULL_INTEGRATED_THEN_HEADING_TEST },
};

static const MENU_NODE straight_test_menu = {
    "Straight Test",
    straight_test_items,
    (uint8_t)(sizeof(straight_test_items) / sizeof(straight_test_items[0])),
};

static const MENU_ITEM turn_test_items[] = {
    { .name = "Fwd2m L90 +1m", .kind = MENU_ENTRY_TASK,
      .u.task = &APP_TURN_FWD2M_LEFT90_POST1M_TEST },
    { .name = "Full Fwd2m L90", .kind = MENU_ENTRY_TASK,
      .u.task = &APP_TURN_FULL_FWD2M_LEFT90_POST1M_TEST },
};

static const MENU_NODE turn_test_menu = {
    "Turn Test",
    turn_test_items,
    (uint8_t)(sizeof(turn_test_items) / sizeof(turn_test_items[0])),
};

/* --- 根菜单 --- */
static const MENU_ITEM root_items[] = {
    { .name = "Line Follow",   .kind = MENU_ENTRY_TASK,    .u.task = &APP_LINE_FOLLOW_TEST },
    { .name = "Line Guided 80", .kind = MENU_ENTRY_TASK,   .u.task = &APP_LINE_GUIDED_TEST },
    { .name = "Vision Red",    .kind = MENU_ENTRY_TASK,   .u.task = &APP_VISION_LINE_TEST },
    { .name = "Straight Test", .kind = MENU_ENTRY_SUBMENU, .u.submenu = &straight_test_menu },
    { .name = "Turn Test",     .kind = MENU_ENTRY_SUBMENU, .u.submenu = &turn_test_menu },
    { .name = "Device Check",  .kind = MENU_ENTRY_SUBMENU, .u.submenu = &device_check_menu },
};

const MENU_NODE APP_ROOT_MENU = {
    "Main Menu",
    root_items,
    (uint8_t)(sizeof(root_items) / sizeof(root_items[0])),
};

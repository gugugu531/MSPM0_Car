#include "app_launcher.h"

#include "app_device_check.h"
#include "app_e_task.h"
#include "bsp_time.h"
#include "key.h"
#include "ui.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_MENU_DELAY_MS 20U

typedef enum {
    APP_MENU_E1_LINE_1 = 0,
    APP_MENU_E1_LINE_2,
    APP_MENU_E1_LINE_3,
    APP_MENU_E1_LINE_4,
    APP_MENU_E1_LINE_5,
    APP_MENU_E2_AIM_2S,
    APP_MENU_E3_AIM_4S,
    APP_MENU_DEVICE_CHECK,
    APP_MENU_COUNT
} APP_MENU_ITEM;

static const char *const s_app_menu_items[APP_MENU_COUNT] = {
    "E1 Line 1 lap",
    "E1 Line 2 laps",
    "E1 Line 3 laps",
    "E1 Line 4 laps",
    "E1 Line 5 laps",
    "E2 Aim 2s",
    "E3 Aim 4s",
    "Device check",
};

static uint8_t App_GetFirstVisibleIndex(APP_MENU_ITEM selected){
    uint8_t selected_index = (uint8_t)selected;

    if (selected_index < UI_LIST_VISIBLE_COUNT){
        return 0U;
    }

    return (uint8_t)(selected_index - UI_LIST_VISIBLE_COUNT + 1U);
}

static void App_RenderMenu(APP_MENU_ITEM selected){
    UI_LIST_PAGE page = {
        .title = "NUEDC 2025 E",
        .items = s_app_menu_items,
        .item_count = APP_MENU_COUNT,
        .selected_index = (uint8_t)selected,
        .first_visible_index = App_GetFirstVisibleIndex(selected),
    };

    Ui_RenderListPage(&page);
}

static bool App_IsNextMenuEvent(void){
    return Key_IsShortPress(KEY_ID_1) || Key_IsDoubleClick(KEY_ID_1);
}

static void App_RunMenuItem(APP_MENU_ITEM selected){
    if (selected <= APP_MENU_E1_LINE_5){
        AppE_RunLineFollow((uint8_t)selected + 1U);
        return;
    }

    if (selected == APP_MENU_E2_AIM_2S){
        AppE_RunAimCenter2s();
        return;
    }

    if (selected == APP_MENU_E3_AIM_4S){
        AppE_RunAimCenter4s();
        return;
    }

    if (selected == APP_MENU_DEVICE_CHECK){
        AppDeviceCheck_Run();
    }
}

void App_Launch(void){
    APP_MENU_ITEM selected = APP_MENU_E1_LINE_1;

    Key_ClearAllEvents();
    Ui_RenderStatusPage("NUEDC 2025 E", UI_STATUS_NORMAL, "Short:next", "Long:enter");
    BSP_DelayMs(800U);
    App_RenderMenu(selected);

    while (1){
        if (App_IsNextMenuEvent()){
            selected = (APP_MENU_ITEM)(((uint8_t)selected + 1U) % (uint8_t)APP_MENU_COUNT);
            Key_ClearAllEvents();
            App_RenderMenu(selected);
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            App_RunMenuItem(selected);
            App_RenderMenu(selected);
        }

        BSP_DelayMs(APP_MENU_DELAY_MS);
    }
}

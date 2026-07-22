/**
 * @file  app_mode.c
 * @brief 顶层应用状态机实现：菜单选择、任务执行委派、故障处理。
 */
#include "app_mode.h"
#include "app_tasks.h"

#include "ui.h"
#include "key.h"
#include "chassis.h"
#include "system_fault.h"

#include <stdbool.h>
#include <stddef.h>

/* 控制周期，单位 s，须与调度器注册 App_ControlTick 的周期一致（20ms）。 */
#define APP_CONTROL_DT 0.02f
/* 菜单项指针数组上限（≥ 任务数即可）。 */
#define APP_MENU_MAX 8U

static APP_MODE app_mode;
static uint8_t  menu_sel;
static const APP_TASK_DESC *current_task;
static bool menu_dirty;
static bool fault_dirty;
static const char *menu_items[APP_MENU_MAX];

/* ---------------- 状态转移入口（唯一改 app_mode 的地方） ---------------- */

static void App_EnterRun(uint8_t index){
    current_task = App_TaskAt(index);
    if (current_task == NULL){
        return;
    }
    Chassis_ResetDistance();
    if (current_task->on_enter != NULL){
        current_task->on_enter();
    }
    Key_ClearAllEvents();
    app_mode = APP_MODE_RUN;
}

static void App_ExitRun(APP_MODE next){
    if ((current_task != NULL) && (current_task->on_exit != NULL)){
        current_task->on_exit();
    }
    (void)Chassis_Brake();
    Key_ClearAllEvents();
    menu_dirty = true;
    app_mode = next;
}

static void App_RaiseFault(SYSTEM_FAULT_CODE code, const char *msg){
    (void)SystemFault_Set(code, msg);
    (void)Chassis_Brake();
    Key_ClearAllEvents();
    fault_dirty = true;
    app_mode = APP_MODE_FAULT;
}

/* ---------------- MENU ---------------- */

static void Menu_Render(void){
    uint8_t count = App_TaskCount();
    uint8_t first = 0U;
    if (menu_sel >= UI_LIST_VISIBLE_COUNT){
        first = (uint8_t)(menu_sel - (UI_LIST_VISIBLE_COUNT - 1U));
    }

    UI_LIST_PAGE page = {
        .title               = "Select Task",
        .items               = menu_items,
        .item_count          = count,
        .selected_index      = menu_sel,
        .first_visible_index = first,
    };
    Ui_RenderListPage(&page);
}

static void Menu_Tick(void){
    uint8_t count = App_TaskCount();
    if (count == 0U){
        return;
    }

    if (Key_GetEvent(KEY_ID_DOWN) == KEY_EVENT_SHORT_PRESS){
        menu_sel = (uint8_t)((menu_sel + 1U) % count);
        menu_dirty = true;
    }
    if (Key_GetEvent(KEY_ID_UP) == KEY_EVENT_SHORT_PRESS){
        menu_sel = (uint8_t)((menu_sel + count - 1U) % count);
        menu_dirty = true;
    }
    if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        App_EnterRun(menu_sel);
        return;
    }

    if (menu_dirty){
        Menu_Render();
        menu_dirty = false;
    }
}

/* ---------------- FAULT ---------------- */

static void Fault_Tick(void){
    if (fault_dirty){
        Ui_RenderStatusPage("FAULT", UI_STATUS_ERROR,
                            SystemFault_GetMessage(), "ENTER: reset");
        fault_dirty = false;
    }
    if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        SystemFault_Clear();
        Key_ClearAllEvents();
        menu_dirty = true;
        app_mode = APP_MODE_MENU;
    }
}

/* ---------------- 对外接口 ---------------- */

void App_Mode_Init(void){
    uint8_t count = App_TaskCount();
    if (count > APP_MENU_MAX){
        count = APP_MENU_MAX;
    }
    for (uint8_t i = 0U; i < count; i++){
        menu_items[i] = App_TaskName(i);
    }

    current_task = NULL;
    menu_sel     = 0U;
    menu_dirty   = true;
    fault_dirty  = false;
    app_mode     = APP_MODE_MENU;
}

APP_MODE App_Mode_Get(void){
    return app_mode;
}

void App_ControlTick(void){
    if (app_mode != APP_MODE_RUN){
        return;
    }

    /* BACK 短按中止优先。 */
    if (Key_GetEvent(KEY_ID_BACK) == KEY_EVENT_SHORT_PRESS){
        App_ExitRun(APP_MODE_MENU);
        return;
    }

    if ((current_task == NULL) || (current_task->on_tick == NULL)){
        App_ExitRun(APP_MODE_MENU);
        return;
    }

    APP_TASK_STATUS status = current_task->on_tick(APP_CONTROL_DT);
    if (status == APP_TASK_DONE){
        App_ExitRun(APP_MODE_MENU);
    } else if (status == APP_TASK_FAULT){
        App_RaiseFault(SYSTEM_FAULT_STATE_ERROR, current_task->name);
    } else {
        /* RUNNING：继续。 */
    }
}

void App_UiTick(void){
    switch (app_mode){
        case APP_MODE_MENU:
            Menu_Tick();
            break;
        case APP_MODE_FAULT:
            Fault_Tick();
            break;
        case APP_MODE_RUN:
        case APP_MODE_INIT:
        default:
            /* RUN 由任务自渲染；INIT 不渲染。 */
            break;
    }
}

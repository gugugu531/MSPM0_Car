/**
 * @file  app_tasks.c
 * @brief 任务注册表：3 个空占位任务 + 1 个测试任务（倒计时）。
 */
#include "app_tasks.h"

#include "ui.h"
#include "bsp_time.h"

#include <stddef.h>

/* ============================ 空占位任务 ============================ */
/* 进入后显示占位页, on_tick 什么都不做; 由框架的 BACK 短按退回菜单。 */

static void EmptyTask_Show(const char *title){
    Ui_RenderLines(title, "(empty task)", "BACK: exit",
                   NULL, NULL, NULL, NULL);
}

static void Task1_Enter(void){ EmptyTask_Show("Task 1"); }
static void Task2_Enter(void){ EmptyTask_Show("Task 2"); }
static void Task3_Enter(void){ EmptyTask_Show("Task 3"); }

static APP_TASK_STATUS EmptyTask_Tick(float dt){
    (void)dt;
    return APP_TASK_RUNNING;
}

/* ============================ 测试任务 ============================ */
/*
 * 用作框架自检入口: 5s 倒计时, 到时返回 DONE 自动退回菜单; 期间 BACK 短按可中止。
 * 演示 on_enter 复位、on_tick 非阻塞计时(用 BSP_Time_GetMs 差值)、按变化节流刷屏、
 * 以及 DONE/中止两条退出路径。不驱动任何执行器, 安全。
 */
#define TIMER_TASK_DURATION_MS 5000U

static uint32_t timer_start_ms;
static uint8_t  timer_last_remain;

static void TimerTask_Enter(void){
    timer_start_ms   = BSP_Time_GetMs();
    timer_last_remain = 0xFFU;
}

static APP_TASK_STATUS TimerTask_Tick(float dt){
    (void)dt;

    uint32_t elapsed = BSP_Time_GetMs() - timer_start_ms;
    if (elapsed >= TIMER_TASK_DURATION_MS){
        return APP_TASK_DONE;   /* 完成 → 框架退回菜单 */
    }

    uint8_t remain = (uint8_t)((TIMER_TASK_DURATION_MS - elapsed) / 1000U) + 1U;
    if (remain != timer_last_remain){   /* 仅在秒数变化时刷屏 */
        timer_last_remain = remain;

        char line[8];
        line[0] = 'T';
        line[1] = '-';
        line[2] = (char)('0' + (remain % 10U));
        line[3] = 's';
        line[4] = '\0';

        Ui_RenderLines("Timer Test", line, "counting down", "BACK: abort",
                       NULL, NULL, NULL);
    }

    return APP_TASK_RUNNING;
}

/* ============================ 注册表 ============================ */
/* 新增任务 = 在此加一行 {名字, on_enter, on_tick, on_exit}。 */

static const APP_TASK_DESC TASK_REGISTRY[] = {
    { "Timer Test", TimerTask_Enter, TimerTask_Tick, NULL },
    { "Task 1",     Task1_Enter,     EmptyTask_Tick, NULL },
    { "Task 2",     Task2_Enter,     EmptyTask_Tick, NULL },
    { "Task 3",     Task3_Enter,     EmptyTask_Tick, NULL },
};

#define TASK_REGISTRY_COUNT (sizeof(TASK_REGISTRY) / sizeof(TASK_REGISTRY[0]))

uint8_t App_TaskCount(void){
    return (uint8_t)TASK_REGISTRY_COUNT;
}

const char *App_TaskName(uint8_t index){
    return (index < TASK_REGISTRY_COUNT) ? TASK_REGISTRY[index].name : "";
}

const APP_TASK_DESC *App_TaskAt(uint8_t index){
    return (index < TASK_REGISTRY_COUNT) ? &TASK_REGISTRY[index] : NULL;
}

/**
 * @file  app_tasks.c
 * @brief 业务任务实现：Timer Test（倒计时自检入口）。
 */
#include "app_tasks.h"

#include "ui.h"
#include "bsp_time.h"

#include <stddef.h>
#include <stdint.h>

/*
 * 测试任务: 5s 倒计时, 到时返回 DONE 自动退回菜单; 期间 BACK 短按可中止。
 * 演示 on_enter 复位、on_tick 非阻塞计时(BSP_Time_GetMs 差值)、按变化节流刷屏、
 * 以及 DONE/中止两条退出路径。不驱动任何执行器, 安全。
 */
#define TIMER_TASK_DURATION_MS 5000U

static uint32_t timer_start_ms;
static uint8_t  timer_last_remain;

static void TimerTask_Enter(void){
    timer_start_ms    = BSP_Time_GetMs();
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

const APP_TASK_DESC APP_TASK_TIMER = {
    "Timer Test", TimerTask_Enter, TimerTask_Tick, NULL
};

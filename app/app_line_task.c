/**
 * @file  app_line_task.c
 * @brief 循迹测试任务实现。
 *
 * 用 middleware/line_follow 的完整闭环（读灰度 → 计算差速 → 驱动底盘）跑巡线，
 * 供上板验证循迹行为。灰度为 GPIO 数字量(SysConfig 已配), 陀螺 JY61P 挂 I2C0、
 * 由本任务 Init + 每拍 Poll, 使默认配置的陀螺增稳生效; 丢线时刹停(测试期安全)。
 */
#include "app_line_task.h"

#include "line_follow.h"
#include "chassis.h"
#include "wit_sdk.h"

#include "ui.h"
#include "app_fmt.h"
#include "bsp_time.h"
#include "bsp_common.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* 刷屏节流周期。 */
#define LT_UI_PERIOD_MS 150U

static uint32_t lt_last_ui;

/* 把字符串拷入 buf, 返回长度(不终止), 便于随后接 AppFmt_* 拼数字。 */
static uint8_t LtPutStr(char *buf, const char *s){
    uint8_t i = 0U;
    while (s[i] != '\0'){
        buf[i] = s[i];
        i++;
    }
    return i;
}

static void LineFollowTest_Enter(void){
    LineFollow_Init(NULL);            /* 默认配置（含陀螺增稳），并复位控制状态。 */
    JY61P_I2C_SetSuspended(false);    /* 确保 JY61P 占用 I2C0。 */
    JY61P_I2C_Init();
    lt_last_ui = 0U;
}

static APP_TASK_STATUS LineFollowTest_Tick(float dt){
    JY61P_I2C_Poll();                 /* 推进陀螺 I2C 状态机, 供增稳读 gz。 */

    /* 完整巡线闭环: 读灰度 → 计算 → 驱动底盘。丢线返回 NOT_READY 且不下发占空比。 */
    BSP_STATUS st = LineFollow_Update(dt);
    if (st != BSP_STATUS_OK){
        (void)Chassis_Brake();        /* 丢线/异常: 刹停(测试期安全, 不自行搜索)。 */
    }

    uint32_t now = BSP_Time_GetMs();
    if ((now - lt_last_ui) < LT_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    lt_last_ui = now;

    LINE_FOLLOW_OUTPUT out = LineFollow_GetOutput();

    /* 与 Device Check 一致：bit='1' 表示该路未检测到黑线，'0' 表示检测到黑线。 */
    char bits[LINE_FOLLOW_SENSOR_COUNT + 1U];
    for (uint8_t i = 0U; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        bits[i] = ((out.level_mask & (1U << i)) != 0U) ? '1' : '0';
    }
    bits[LINE_FOLLOW_SENSOR_COUNT] = '\0';

    char l2[20];
    char l3[20];
    char l4[20];
    uint8_t n;

    n = LtPutStr(l2, "err ");
    AppFmt_Fixed(&l2[n], out.error, 1);
    n = LtPutStr(l3, "dL ");
    AppFmt_Fixed(&l3[n], out.left_duty, 0);
    n = LtPutStr(l4, "dR ");
    AppFmt_Fixed(&l4[n], out.right_duty, 0);

    const char *status = out.line_lost ? "LINE LOST" : "track";

    Ui_RenderLines("Line Follow", bits, l2, l3, l4, status, "BACK: exit");
    return APP_TASK_RUNNING;
}

const APP_TASK_DESC APP_LINE_FOLLOW_TEST = {
    "Line Follow", LineFollowTest_Enter, LineFollowTest_Tick, NULL
};

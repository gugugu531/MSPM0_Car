/**
 * @file  app_bt_task.c
 * @brief 蓝牙串口接收测试任务实现。
 *
 * 用 bsp/bluetooth(BlueTooth/UART0, 9600 8N1)接收 ASCII 字符串: 每拍非阻塞取出收到的
 * 字节, 把最近若干可见字符滚动显示在 OLED, 并显示累计收到/丢弃字节数, 供上板验证蓝牙链路。
 */
#include "app_bt_task.h"

#include "bluetooth.h"
#include "key.h"
#include "ui.h"
#include "app_fmt.h"
#include "bsp_time.h"

#include <stddef.h>
#include <stdint.h>

#define BT_UI_PERIOD_MS 150U
#define BT_SHOW_LEN     16U   /* OLED 滚动显示最近 N 个字符 */

static char     bt_show[BT_SHOW_LEN + 1U];
static uint8_t  bt_show_len;
static uint32_t bt_tx_sends;   /* 累计发送 "hello" 次数 */
static uint32_t bt_last_ui;

/* 把字符串拷入 buf, 返回长度(不终止), 便于随后接 AppFmt_* 拼数字。 */
static uint8_t BtPutStr(char *buf, const char *s){
    uint8_t i = 0U;
    while (s[i] != '\0'){
        buf[i] = s[i];
        i++;
    }
    return i;
}

/* 追加一个字符到滚动显示串: 非可见字符显示为 '.'; 满则左移一位腾出末位。 */
static void BtShowAppend(char c){
    if ((c < 0x20) || (c > 0x7E)){
        c = '.';
    }
    if (bt_show_len >= BT_SHOW_LEN){
        for (uint8_t k = 0U; k < (BT_SHOW_LEN - 1U); k++){
            bt_show[k] = bt_show[k + 1U];
        }
        bt_show_len = BT_SHOW_LEN - 1U;
    }
    bt_show[bt_show_len] = c;
    bt_show_len++;
    bt_show[bt_show_len] = '\0';
}

static void ChkBt_Enter(void){
    BlueTooth_Init();
    bt_show[0] = '\0';
    bt_show_len = 0U;
    bt_tx_sends = 0U;
    bt_last_ui = 0U;
}

static APP_TASK_STATUS ChkBt_Tick(float dt){
    (void)dt;

    /* 每拍取出收到的字节, 滚动追加到显示串, 并原样回显(TX 端到端自测:
     * 手机发什么就应收到什么回来; 若能显示收到却收不到回显 → 问题在 PB0→模块RXD 接线/模块)。 */
    uint8_t buf[32];
    uint16_t n = BlueTooth_Read(buf, (uint16_t)sizeof(buf));
    if (n > 0U){
        BlueTooth_Write(buf, n);
        for (uint16_t i = 0U; i < n; i++){
            BtShowAppend((char)buf[i]);
        }
    }

    /* ENTER 短按发送 "hello"(带回车换行, 便于对端按行显示)。 */
    if (Key_GetEvent(KEY_ID_ENTER) == KEY_EVENT_SHORT_PRESS){
        BlueTooth_Puts("hello\r\n");
        bt_tx_sends++;
    }

    uint32_t now = BSP_Time_GetMs();
    if ((now - bt_last_ui) < BT_UI_PERIOD_MS){
        return APP_TASK_RUNNING;
    }
    bt_last_ui = now;

    char l2[20];
    char l3[20];
    uint8_t k;

    k = BtPutStr(l2, "rx ");
    AppFmt_I32(&l2[k], (int32_t)BlueTooth_GetRxCount());
    while (l2[k] != '\0'){ k++; }
    k += BtPutStr(&l2[k], " er ");
    AppFmt_I32(&l2[k], (int32_t)BlueTooth_GetRxErrors());   /* UART RX 错误数 */
    k = BtPutStr(l3, "tx ");
    AppFmt_I32(&l3[k], (int32_t)bt_tx_sends);

    Ui_RenderLines("Chk BlueTooth",
                   (bt_show[0] != '\0') ? bt_show : "(waiting)",
                   l2, l3, "EN:send hello", "BACK: exit", NULL);
    return APP_TASK_RUNNING;
}

static void ChkBt_Exit(void){
    BlueTooth_Deinit();
}

const APP_TASK_DESC APP_CHK_BLUETOOTH = {
    "BlueTooth", ChkBt_Enter, ChkBt_Tick, ChkBt_Exit
};

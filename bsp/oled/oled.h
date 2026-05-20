/**
 * @file  oled.h
 * @brief SSD1306 OLED 显示接口。
 */

#ifndef OLED_H
#define OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef OLED_GPIO_PORT
#ifdef OLED_PORT
#define OLED_GPIO_PORT OLED_PORT
#else
#define OLED_GPIO_PORT GPIOA
#endif
#endif

#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
#define OLED_PAGE_COUNT 8U

void OLED_Init(void);

void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_ColorTurn(uint8_t enable);

void OLED_Clear(void);
void OLED_ClearLine(uint8_t page);

void OLED_ShowChar(uint8_t x, uint8_t page, uint8_t chr, uint8_t sizey);
void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t page, const char *chr, uint8_t sizey);
void OLED_ShowStringClearLine(uint8_t x, uint8_t page, const char *chr, uint8_t sizey);
void OLED_DrawBMP(uint8_t x, uint8_t page, uint8_t sizex, uint8_t sizey, const uint8_t *bmp);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */

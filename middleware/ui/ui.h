/**
 * @file  ui.h
 * @brief Lightweight OLED UI renderer.
 */
#ifndef UI_H
#define UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UI_CONTENT_LINE_COUNT 6U
#define UI_LIST_VISIBLE_COUNT 6U
#define UI_FONT_SIZE 8U

typedef enum {
    UI_STATUS_NORMAL = 0,
    UI_STATUS_OK,
    UI_STATUS_WARN,
    UI_STATUS_ERROR
} UI_STATUS_LEVEL;

typedef struct {
    const char *title;
    const char *line[UI_CONTENT_LINE_COUNT];
} UI_TEXT_PAGE;

typedef struct {
    const char *title;
    const char *const *items;
    uint8_t item_count;
    uint8_t selected_index;
    uint8_t first_visible_index;
} UI_LIST_PAGE;

void Ui_Init(void);
void Ui_Clear(void);
void Ui_ClearLine(uint8_t page);

void Ui_DrawText(uint8_t x, uint8_t page, const char *text);
void Ui_UpdateText(uint8_t x, uint8_t page, const char *text);

void Ui_DrawTitle(const char *title);
void Ui_DrawContentLine(uint8_t line_index, const char *text);
void Ui_UpdateContentLine(uint8_t line_index, const char *text);

void Ui_RenderTextPage(const UI_TEXT_PAGE *page);
void Ui_RenderLines(const char *title,
                    const char *line0,
                    const char *line1,
                    const char *line2,
                    const char *line3,
                    const char *line4,
                    const char *line5);

void Ui_RenderStatusPage(const char *title,
                         UI_STATUS_LEVEL level,
                         const char *message,
                         const char *hint);

void Ui_RenderListPage(const UI_LIST_PAGE *page);

#ifdef __cplusplus
}
#endif

#endif /* UI_H */

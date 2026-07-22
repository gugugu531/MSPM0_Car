/**
 * @file  ui.h
 * @brief Middleware 层轻量 OLED UI 渲染接口。
 *
 * 刷屏契约 (帧缓冲模型): 底层 OLED 为帧缓冲, 绘制只写 RAM, 须调用 Flush 才真正显示。
 * 本层据此分两类接口:
 *   - 页级/整体接口 (Ui_Render*, Ui_UpdateContentLine) 内部已 Flush, 调用即显示;
 *   - 绘制原语 (Ui_Clear/Ui_ClearLine/Ui_DrawText/Ui_DrawTitle/Ui_DrawContentLine/
 *     Ui_UpdateText) 只写帧缓冲、不 Flush, 需批量绘制后自行调用 Ui_Flush() 显示。
 * 多行更新优先用 Ui_Render*(一次 Flush), 避免逐行 Ui_UpdateContentLine 触发多次整帧刷新。
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

/**
 * @brief 状态页等级。
 */
typedef enum {
    UI_STATUS_NORMAL = 0,
    UI_STATUS_OK,
    UI_STATUS_WARN,
    UI_STATUS_ERROR
} UI_STATUS_LEVEL;

/**
 * @brief 文本页描述。
 */
typedef struct {
    /** 页面标题。 */
    const char *title;
    /** 内容行文本，允许为 NULL。 */
    const char *line[UI_CONTENT_LINE_COUNT];
} UI_TEXT_PAGE;

/**
 * @brief 列表页描述。
 */
typedef struct {
    /** 页面标题。 */
    const char *title;
    /** 列表项字符串数组。 */
    const char *const *items;
    /** 列表项总数。 */
    uint8_t item_count;
    /** 当前选中项索引。 */
    uint8_t selected_index;
    /** 首个可见项索引。 */
    uint8_t first_visible_index;
} UI_LIST_PAGE;

/**
 * @brief 初始化 OLED 并清屏。
 */
void Ui_Init(void);

/**
 * @brief 把帧缓冲整帧刷新到屏幕。
 * @note 供批量调用绘制原语后手动显示；透传 bsp/oled 的 OLED_Flush()，使上层无需直接依赖 oled.h。
 */
void Ui_Flush(void);

/**
 * @brief 清空整屏。
 * @note 只写帧缓冲，不 Flush；须自行调用 Ui_Flush() 才显示。
 */
void Ui_Clear(void);

/**
 * @brief 清空指定页行。
 * @note 只写帧缓冲，不 Flush；须自行调用 Ui_Flush() 才显示。
 */
void Ui_ClearLine(uint8_t page);

/**
 * @brief 在指定位置绘制文本，不主动清行。
 * @note 只写帧缓冲，不 Flush；须自行调用 Ui_Flush() 才显示。
 */
void Ui_DrawText(uint8_t x, uint8_t page, const char *text);

/**
 * @brief 清空指定行后绘制文本。
 * @note 只写帧缓冲，不 Flush；须自行调用 Ui_Flush() 才显示。
 */
void Ui_UpdateText(uint8_t x, uint8_t page, const char *text);

/**
 * @brief 绘制页面标题。
 * @note 只写帧缓冲，不 Flush；须自行调用 Ui_Flush() 才显示。
 */
void Ui_DrawTitle(const char *title);

/**
 * @brief 绘制内容区指定行。
 * @note 只写帧缓冲，不 Flush；须自行调用 Ui_Flush() 才显示。
 */
void Ui_DrawContentLine(uint8_t line_index, const char *text);

/**
 * @brief 清空并更新内容区指定行。
 * @note 更新后自动 Flush；逐行调用会多次整帧刷新，多行更新宜改用 Ui_Render*。
 */
void Ui_UpdateContentLine(uint8_t line_index, const char *text);

/**
 * @brief 渲染完整文本页。
 * @note 内部已 Flush，调用即显示。
 */
void Ui_RenderTextPage(const UI_TEXT_PAGE *page);

/**
 * @brief 便捷渲染标题和最多 6 行文本。
 * @note 内部已 Flush，调用即显示。
 */
void Ui_RenderLines(const char *title,
                    const char *line0,
                    const char *line1,
                    const char *line2,
                    const char *line3,
                    const char *line4,
                    const char *line5);

/**
 * @brief 渲染状态页。
 * @note 内部已 Flush，调用即显示。
 */
void Ui_RenderStatusPage(const char *title,
                         UI_STATUS_LEVEL level,
                         const char *message,
                         const char *hint);

/**
 * @brief 渲染列表页。
 * @note 内部已 Flush，调用即显示。
 */
void Ui_RenderListPage(const UI_LIST_PAGE *page);

#ifdef __cplusplus
}
#endif

#endif /* UI_H */

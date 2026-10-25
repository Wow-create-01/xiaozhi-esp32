#ifndef ROUND_DISPLAY_H
#define ROUND_DISPLAY_H

#include "lcd_display.h"

/**
 * 240×240 圆屏显示。
 *
 * 抄自官方 boards/m5stack/stopwatch/m5stack_stopwatch.cc 里的 RoundLcdDisplay，
 * 只改了适配 1.28" 圆屏（240×240）的偏移量。
 *
 * 关键点是那个 LV_EVENT_INVALIDATE_AREA 事件回调：它把刷新的区域对齐到偶数像素，
 * 圆屏在边缘做抗锯齿时需要这个，缺了它会出现四角撕裂 / 花屏。
 *
 * 布局调整（父类是矩形布局，直接套到圆屏上四角会被切）：
 *  - 顶栏：下移，避开圆顶弧线
 *  - 状态文字：放上半圆，宽度收窄
 *  - 字幕：从底部弧线往上提
 */
class RoundDisplay : public SpiLcdDisplay {
public:
    static void rounder_event_cb(lv_event_t* e) {
        lv_area_t* area = static_cast<lv_area_t*>(lv_event_get_param(e));
        area->x1 = (area->x1 >> 1) << 1;
        area->y1 = (area->y1 >> 1) << 1;
        area->x2 = ((area->x2 >> 1) << 1) + 1;
        area->y2 = ((area->y2 >> 1) << 1) + 1;
    }

    RoundDisplay(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t panel_handle,
                 int width, int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                 bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle, width, height, offset_x, offset_y, mirror_x,
                        mirror_y, swap_xy) {}

    void SetupUI() override;
};

#endif  // ROUND_DISPLAY_H

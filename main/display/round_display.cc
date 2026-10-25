#include "round_display.h"
#include "lvgl_theme.h"

#include <esp_log.h>
#include <esp_lvgl_port.h>

#define TAG "RoundDisplay"

/**
 * 布局思路抄自官方 boards/m5stack/stopwatch/m5stack_stopwatch.cc 的 RoundLcdDisplay：
 * 先让父类建完整 UI（保证所有成员指针都存在、SetTheme 不会空指针崩），
 * 再只调整那几个在圆屏上会被切掉的位置。
 *
 * 偏移量从官方 466×466 的实现按屏宽比例换算，240×240 和 466×466 都适用。
 */
void RoundDisplay::SetupUI() {
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI 重复调用，跳过");
        return;
    }

    // 1) 父类建出完整结构
    SpiLcdDisplay::SetupUI();

    DisplayLockGuard lock(this);

    const int w = width_;
    const int edge_inset = w * 12 / 100;          // 圆顶/圆底安全区
    const int status_bar_top_off = w * 12 / 100;  // 状态文字：上半圆，在图标下面
    const int chat_bar_bottom_off = w * 14 / 100; // 字幕：从底部弧线往上提

    // 2) 状态栏：左右收边，避免文字压在圆弧上
    if (status_bar_ != nullptr) {
        lv_obj_set_style_pad_left(status_bar_, w * 20 / 100, 0);
        lv_obj_set_style_pad_right(status_bar_, w * 20 / 100, 0);
        lv_obj_align(status_bar_, LV_ALIGN_TOP_MID, 0, status_bar_top_off);
    }
    if (status_label_ != nullptr) {
        lv_obj_set_width(status_label_, w * 60 / 100);
        lv_obj_align(status_label_, LV_ALIGN_CENTER, 0, 0);
    }
    if (notification_label_ != nullptr) {
        lv_obj_set_width(notification_label_, w * 60 / 100);
        lv_obj_align(notification_label_, LV_ALIGN_CENTER, 0, 0);
    }

    // 3) 字幕区：收窄 + 上提。
    //    默认样式（CONFIG_USE_DEFAULT_MESSAGE_STYLE）没有 bottom_bar_，
    //    消息直接堆在 content_ 里，所以两种情况都要处理。
    if (bottom_bar_ != nullptr) {
        // 微信气泡样式
        lv_obj_align(bottom_bar_, LV_ALIGN_BOTTOM_MID, 0, -chat_bar_bottom_off);
        if (chat_message_label_ != nullptr) {
            lv_obj_set_width(chat_message_label_, w * 75 / 100);
        }
    }
    if (content_ != nullptr) {
        lv_obj_set_width(content_, w * 82 / 100);
        lv_obj_set_style_pad_left(content_, 0, 0);
        lv_obj_set_style_pad_right(content_, 0, 0);
    }

    // 4) 顶栏（网络/静音/电量图标）：整体往下挪进圆的安全区
    if (top_bar_ != nullptr) {
        lv_obj_align(top_bar_, LV_ALIGN_TOP_MID, 0, edge_inset / 2);
        lv_obj_set_style_pad_top(top_bar_, edge_inset / 4, 0);
        lv_obj_set_style_pad_left(top_bar_, w * 16 / 100, 0);
        lv_obj_set_style_pad_right(top_bar_, w * 16 / 100, 0);
    }

    // 5) 表情保持居中（父类已经设好，这里不动）

    // 6) 关键：让 LVGL 把刷新区域对齐到偶数像素。
    //    圆屏边缘抗锯齿需要这个，缺了会在四角出现撕裂/花屏。
    if (display_ != nullptr) {
        lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    }

    ESP_LOGI(TAG, "圆屏 UI 就绪（%dx%d，edge_inset=%d）", w, height_, edge_inset);
}

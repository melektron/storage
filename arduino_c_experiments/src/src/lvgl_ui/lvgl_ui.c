#include "lvgl_ui.h"


void lvgl_ui_init(void)
{
    // https://docs.lvgl.io/master/details/widgets/tabview.html

    lv_obj_t *tabview = lv_tabview_create(lv_screen_active());
    lv_tabview_set_tab_bar_position(tabview, LV_DIR_LEFT);
    lv_tabview_set_tab_bar_size(tabview, 80);
    // lv_obj_set_style_bg_color(tabview, lv_palette_lighten(LV_PALETTE_RED, 2), 0);

    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    lv_obj_t *tab = lv_tabview_add_tab(tabview, "System");
    system_tab_create(tab);
    tab = lv_tabview_add_tab(tabview, "QMI8658");
    qmi8658_tab_create(tab);
    tab = lv_tabview_add_tab(tabview, "Camera");
    camera_tab_create(tab);
    tab = lv_tabview_add_tab(tabview, "WiFi");
    wifi_tab_create(tab);
    tab = lv_tabview_add_tab(tabview, "Anim");
    animtest_tab_create(tab);

    // style the tab buttons
    lv_obj_t * tab_bar = lv_tabview_get_tab_bar(tabview);
    lv_obj_set_style_bg_color(tab_bar, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_text_color(tab_bar, lv_palette_lighten(LV_PALETTE_GREY, 5), 0);
    //lv_obj_set_style_border_side(tab_bar, LV_BORDER_SIDE_RIGHT, LV_PART_ITEMS | LV_STATE_CHECKED);   // no longer needed (or even breaking) after LVGL 9
    uint32_t tab_count = lv_tabview_get_tab_count(tabview);
    for(uint32_t i = 0; i < tab_count; i++) {
        lv_obj_t * button = lv_obj_get_child(tab_bar, i);
        lv_obj_set_style_border_side(button, LV_BORDER_SIDE_RIGHT, LV_PART_MAIN | LV_STATE_CHECKED);
    }

    lv_obj_remove_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);
}
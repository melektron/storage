#include "tabview.h"

static void anim_x_cb(void * var, int32_t v)
{
    lv_obj_set_x((lv_obj_t *) var, v);
}

static void anim_size_cb(void * var, int32_t v)
{
    lv_obj_set_size((lv_obj_t *) var, v, v);
}

void scrollflex_tab_create(lv_obj_t *tab)
{
    /*Create a container with ROW flex direction*/
    lv_obj_t * cont_row = lv_obj_create(tab);
    lv_obj_set_size(cont_row, 220, 60);
    lv_obj_align(cont_row, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);

    /*Create a container with COLUMN flex direction*/
    lv_obj_t * cont_col = lv_obj_create(tab);
    lv_obj_set_size(cont_col, 180, 150);
    lv_obj_align_to(cont_col, cont_row, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
    lv_obj_set_flex_flow(cont_col, LV_FLEX_FLOW_COLUMN);

    uint32_t i;
    for(i = 0; i < 10; i++) {
        lv_obj_t * obj;
        lv_obj_t * label;

        /*Add items to the row*/
        obj = lv_button_create(cont_row);
        lv_obj_set_size(obj, 100, LV_PCT(100));

        label = lv_label_create(obj);
        lv_label_set_text_fmt(label, "Item: %" LV_PRIu32"", i);
        lv_obj_center(label);

        /*Add items to the column*/
        obj = lv_button_create(cont_col);
        lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);

        label = lv_label_create(obj);
        lv_label_set_text_fmt(label, "Item: %" LV_PRIu32, i);
        lv_obj_center(label);
    }

    //lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN);
    //lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN);
    //lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN);
    //lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN);
}

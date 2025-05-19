#include "tabview.h"
lv_obj_t *img_camera;
void camera_tab_create(lv_obj_t *tab) {
  img_camera = lv_image_create(tab);
  lv_obj_set_size(img_camera, 240, 240);
  lv_image_set_rotation(img_camera, 2700);
  //lv_image_set_offset_x(img_camera, -10);
  //lv_image_set_offset_y(img_camera, -10);
  lv_obj_align(img_camera, LV_ALIGN_CENTER, 0, 0);  // 居中显示
  lv_obj_set_pos(img_camera, 0, -1);
  lv_obj_set_scroll_dir(tab, LV_DIR_NONE);

  lv_obj_set_style_pad_top(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(img_camera, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(img_camera, 0, LV_PART_MAIN);
}

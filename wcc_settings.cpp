/*
 * File: wcc_settings.cpp
 * Author: John
 * Date: 2025-11-14
 * Description:
 *   Implementation of settings management for the Watch Cleaner Controller (WCC).
 *   This file will contain functions to load, save, and validate configuration
 *   parameters for the ESP32-based WCC project.
 */
#include <Arduino.h>
#include <lvgl.h>

 /* Externs */
extern lv_obj_t * main_screen;
extern lv_obj_t * settings_screen;
extern lv_obj_t * return_to_main_button;
extern lv_style_t transparent_button_style;

static void return_to_main_button_event_cb(lv_event_t * event)
{
  lv_obj_t * button = lv_event_get_target_obj(event);
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t * label = lv_obj_get_child(button, 0);  // Label of button

  //Serial.printf("Button event is %d\n", code);
  // LV_EVENT_VALUE_CHANGED, LV_EVENT_VALUE_CLICKED
  Serial.println("Return button handler");
  if (code == LV_EVENT_CLICKED) {
    Serial.println("Return button clicked");
    lv_screen_load_anim(main_screen, LV_SCR_LOAD_ANIM_OVER_TOP, 500 /* time*/, 10 /* delay */, false /* auto_del */ );
  }
}

static lv_obj_t * create_duration_item(const char * desc)
{
    // A container for the desc label, buttons, and time label
    lv_obj_t * cont = lv_obj_create(settings_screen);

    // Descriptor label
    lv_obj_t * desc_label = lv_label_create(cont);
    lv_label_set_text(desc_label, desc);
    lv_obj_set_style_text_font(desc_label, &lv_font_montserrat_14, 0);
    lv_obj_align(cont,LV_ALIGN_TOP_MID, 0, 10);

    // Buttons
    lv_obj_t * up_button = lv_btn_create(cont);
    lv_obj_set_width(up_button, 30);
    lv_obj_set_height(up_button, 30);
    lv_obj_t * up_button_label = lv_label_create(up_button);
    lv_obj_center(up_button_label);
    lv_label_set_text(up_button_label, LV_SYMBOL_UP);
    lv_obj_set_style_text_font(up_button_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(up_button_label, lv_color_black(), 0);
    lv_obj_align_to(up_button, desc_label, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    lv_obj_t * down_button = lv_btn_create(cont);
    lv_obj_set_width(down_button, 30);
    lv_obj_set_height(down_button, 30);
    lv_obj_t * down_button_label = lv_label_create(down_button);
    lv_obj_center(down_button_label);
    lv_label_set_text(down_button_label, LV_SYMBOL_DOWN);
    lv_obj_set_style_text_font(down_button_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(down_button_label, lv_color_black(), 0);
    lv_obj_align_to(down_button, up_button, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    // Time display field
    lv_obj_t * time_label = lv_label_create(cont);
    lv_label_set_text(time_label, "00:00");
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_16, 0);
    lv_obj_align_to(time_label, down_button, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    return cont;
}

static lv_obj_t * create_return_to_main_button()
{
    lv_obj_t * button = lv_btn_create(settings_screen);
    lv_obj_remove_style_all(button);
    lv_obj_t * button_label = lv_label_create(button);
    lv_obj_add_style(button, &transparent_button_style, LV_PART_MAIN);
    lv_obj_set_style_text_font(button_label, &lv_font_montserrat_48, 0);
    lv_label_set_text(button_label, LV_SYMBOL_NEW_LINE);
    lv_obj_center(button_label);
    lv_obj_remove_flag(button, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(button, return_to_main_button_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_width(button, 60); 
    lv_obj_set_height(button,60);
    return button;
}

void wcc_create_settings(void)
{
    settings_screen = lv_obj_create(NULL);

    /* Create return to main button on settings screen */
    return_to_main_button = create_return_to_main_button();
    lv_obj_align(return_to_main_button, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    lv_obj_t * clean_duration_item_container = create_duration_item("Clean duration");
    lv_obj_set_width(clean_duration_item_container, 300);
    lv_obj_set_height(clean_duration_item_container,30);
    lv_obj_set_style_border_width(clean_duration_item_container, 2, 0);
    lv_obj_set_style_border_color(clean_duration_item_container,lv_color_black(), 0);
    lv_obj_align(clean_duration_item_container, LV_ALIGN_TOP_LEFT, 10, 25);

}
/*
 * File: wcc_main_screen.cpp
 * Author: John
 * Date: 2025-11-17
 * Description:
 *      GUI widgets for the main screen 
 */
#include <lvgl.h>
#include <Arduino.h>
#include "wcc_common.h"

/* Externs */
extern lv_style_t on_button_style;
extern lv_style_t off_button_style;
extern lv_style_t stop_button_style;
extern lv_style_t transparent_button_style;
extern lv_style_t style_radio_button_container;
extern lv_style_t style_radio;
extern lv_style_t style_radio_chk;

extern enum class OperatingMode g_operating_mode; 
extern lv_obj_t * settings_screen;
extern lv_obj_t * main_screen;

#define MAIN_CYCLE_REPEAT_COUNT 62 
static uint32_t g_periods_remaining = MAIN_CYCLE_REPEAT_COUNT; 

static char *Mach_Status_Text_Stopped = "Stopped ...";
static char *Mach_Status_Text_Running = "Running ...";
static char *Mach_Status_Text_Paused =  "Paused ...";
/* Statically allocate some room to paste in seconds remaining*/
static char Mach_Status_Text_Time_Remaining[7] = "      ";

static lv_obj_t * start_button; 
static lv_obj_t * stop_button;
static lv_obj_t * settings_button;
static lv_obj_t * mach_status_label;
static lv_obj_t * time_remaining_label;
static lv_timer_t * clean_rinse_timer;

/* Public functions */
void wcc_create_main_screen_widgets();
lv_obj_t * wcc_create_title_bar(lv_obj_t * scr, const char * title);


lv_obj_t * wcc_create_title_bar(lv_obj_t * scr, const char * title)
{
    // Pleasant blue screen title bar
    lv_color_t  tc = lv_color_make(WCC_TITLE_BLUE);
    lv_obj_t * title_cont = lv_obj_create(scr);
    lv_obj_t * title_label = lv_label_create(title_cont);
    // No border
    lv_obj_set_style_border_width(title_cont, 0, 0);
    // Squared off corners
    lv_obj_set_style_radius(title_cont, 1, 0);
    lv_label_set_text(title_label, title);
    lv_obj_center(title_label);
    LV_ASSERT(scr != NULL);
    lv_obj_set_width(title_cont, lv_obj_get_width(scr));
    lv_obj_set_height(title_cont, 35);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(title_label, lv_color_white(), 0);
    lv_obj_set_style_bg_color(title_cont, tc, 0);
    lv_obj_align( title_cont, LV_ALIGN_TOP_LEFT, 0, 0 );
    return title_cont;
}

static void format_and_publish_time_remaining(uint32_t periods_remaining)
{
  uint32_t minutes = periods_remaining/60;
  uint32_t seconds = periods_remaining%60;
  if (seconds == 0 && minutes == 0) {
    seconds = periods_remaining;
  }
  if (periods_remaining <= 0) {
    lv_label_set_text(time_remaining_label, "");
    return;
  }
  
  lv_snprintf(Mach_Status_Text_Time_Remaining,7,"%03d:%02d",minutes,seconds);
  lv_label_set_text(time_remaining_label, Mach_Status_Text_Time_Remaining);
  return;
}

static  void clear_time_remaining_label(void * user_data) {
  lv_label_set_text(time_remaining_label,"");
}

static void clean_rinse_timer_cb(lv_timer_t * timer) 
{
  uint32_t *periods_remaining = (uint32_t *)timer->user_data;
  Serial.println("clean_rinse timer callback");
  char * mach_status_text = lv_label_get_text(mach_status_label);

  *periods_remaining = *periods_remaining - 1;

  format_and_publish_time_remaining(*periods_remaining);
  lv_label_set_text(mach_status_label,Mach_Status_Text_Running);

  // The next lv_timer_handler() call in loop() will decrement the repeat count
  // to 0 and delete the timer. We want to nullify the timer pointer and reset the 
  // button state here before that happens 
  if (*periods_remaining == 0) {
    clean_rinse_timer = NULL;
    lv_obj_t * label = lv_obj_get_child(start_button, 0);  // Label of button

    // Timer should only be running in RUN mode, i.e. start button in checked state
    LV_ASSERT(lv_obj_has_state(start_button,LV_STATE_CHECKED));
    // Now manually change the start button back to unchecked state and update the label
    lv_obj_clear_state(start_button, LV_STATE_CHECKED); 
    lv_label_set_text(label, LV_SYMBOL_PLAY LV_SYMBOL_PAUSE);
    lv_label_set_text(mach_status_label,Mach_Status_Text_Stopped);
    clear_time_remaining_label(NULL);
  }
}
static void start_button_event_cb(lv_event_t * event)
{
  lv_obj_t * button = lv_event_get_target_obj(event);
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t * label = lv_obj_get_child(button, 0);  // Label of button

  //Serial.printf("Button event is %d\n", code);
  // This is a checked button. 
  // LV_EVENT_VALUE_CHANGED, LV_EVENT_VALUE_CLICKED
  Serial.println("start button handler");
  switch (g_operating_mode) {
    case OperatingMode::clean:
      Serial.println("Clean mode");
      break;
    case OperatingMode::rinse:
      Serial.println("Rinse mode");
      break;
    case OperatingMode::spin:
      Serial.println("Spin mode");
      break;
    default:
      LV_ASSERT(false);
  }

  // Change back to start mode, if checked already
  // I use strcmp instead of button state because timing of state change
  // button callback is not deterministic in my experiements
  if (code == LV_EVENT_VALUE_CHANGED) {
    Serial.printf("Button state: %d\n", lv_obj_get_state(start_button));
    // "play" button is clicked, changes to "pause" icon
    if (lv_obj_has_state(start_button, LV_STATE_CHECKED)) {
      // Set button to PAUSE and create or resume the timer
      lv_label_set_text(label,LV_SYMBOL_PAUSE);

    /* Timers are used to control the duration of the clean/rinse cycle.
      The timer repeats essentially once per second, acting like a countdown
      timers. The total duration is determined by the repeat count. */
    
      /* If timer doesn't exist, create and start it */
      if (clean_rinse_timer == NULL) {
        g_periods_remaining=MAIN_CYCLE_REPEAT_COUNT + 1; /* 1 second per period */
        clean_rinse_timer = lv_timer_create(clean_rinse_timer_cb, 1000 /* ms*/, &g_periods_remaining);
        lv_timer_set_repeat_count(clean_rinse_timer, g_periods_remaining);
        lv_label_set_text(mach_status_label, Mach_Status_Text_Running);
      }
      else { /* otherwise, resume an existing timer */ 
        lv_timer_resume(clean_rinse_timer);
      }

    } 
    else {  // "pause" button is clicked 
      lv_label_set_text(label,LV_SYMBOL_PLAY);
      lv_label_set_text(mach_status_label,Mach_Status_Text_Paused);
      format_and_publish_time_remaining(g_periods_remaining);
      // Manually stop the timer.
      if (clean_rinse_timer != NULL) {
        lv_timer_pause(clean_rinse_timer);
      }
    }
  }
}

static lv_obj_t * create_start_button(lv_obj_t * scr)
{
    /* Start Button*/
    start_button = lv_btn_create(scr);
    lv_obj_remove_style_all(start_button);
    lv_obj_t * start_button_label = lv_label_create(start_button);
    lv_label_set_text(start_button_label, LV_SYMBOL_PLAY LV_SYMBOL_PAUSE);
    lv_obj_center(start_button_label);
    lv_obj_set_style_text_font(start_button_label, &lv_font_montserrat_20, 0);
    lv_obj_add_flag(start_button, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_remove_flag(start_button, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(start_button, start_button_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_width(start_button, 60); 
    lv_obj_set_height(start_button, 60);
    lv_obj_align(start_button, LV_ALIGN_TOP_LEFT, 30, 45);
    /* Apply styles */
    lv_obj_add_style(start_button, &on_button_style, LV_STATE_DEFAULT);
    lv_obj_add_style(start_button, &off_button_style, LV_STATE_CHECKED);
    return start_button;

}

static void stop_button_event_cb(lv_event_t * event)
{
  lv_obj_t * button = lv_event_get_target_obj(event);
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t * label = lv_obj_get_child(button, 0);  // Label of button

  //Serial.printf("Button event is %d\n", code);
  // LV_EVENT_VALUE_CHANGED, LV_EVENT_VALUE_CLICKED
  Serial.println("Stop button handler");
  if (code == LV_EVENT_CLICKED) {
    // The duration timer is active (running or paused)
    if (clean_rinse_timer != NULL) {
      lv_timer_delete(clean_rinse_timer);
      clean_rinse_timer = NULL;
      // Now manually change the start button back to unchecked state and update the label
      lv_obj_clear_state(start_button, LV_STATE_CHECKED); 
      lv_obj_t * start_button_label = lv_obj_get_child(start_button, 0);
      lv_label_set_text(start_button_label, LV_SYMBOL_PLAY LV_SYMBOL_PAUSE);
      lv_label_set_text(mach_status_label,Mach_Status_Text_Stopped);
      clear_time_remaining_label(NULL);
    }
  }
}

static lv_obj_t * create_stop_button(lv_obj_t * scr)
{
    stop_button = lv_btn_create(scr);
    lv_obj_remove_style_all(stop_button);
    lv_obj_t * stop_button_label = lv_label_create(stop_button);
    lv_label_set_text(stop_button_label, LV_SYMBOL_STOP);
    lv_obj_center(stop_button_label);
    lv_obj_set_style_text_font(stop_button_label, &lv_font_montserrat_20, 0);
    // Stop button is not checkable, does not retain button state.
    // lv_obj_add_flag(stop_button, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_remove_flag(stop_button, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(stop_button, stop_button_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_width(stop_button, 60); 
    lv_obj_set_height(stop_button,60);
    lv_obj_align_to(stop_button, start_button, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* Apply styles */
    lv_obj_add_style(stop_button, &stop_button_style, LV_STATE_DEFAULT);
    return stop_button;

}

static void settings_button_event_cb(lv_event_t * event)
{
  lv_obj_t * button = lv_event_get_target_obj(event);
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t * label = lv_obj_get_child(button, 0);  // Label of button

  //Serial.printf("Button event is %d\n", code);
  // This is a checked button. 
  // LV_EVENT_VALUE_CHANGED, LV_EVENT_VALUE_CLICKED
  Serial.println("Settings button handler");
  // Change back to start mode, if checked already
  // I use strcmp instead of button state because timing of state change
  // button callback is not deterministic in my experiements
  if (code == LV_EVENT_CLICKED) {
    Serial.println("Settings button clicked");
    lv_screen_load_anim(settings_screen, LV_SCR_LOAD_ANIM_OVER_TOP, 500 /* time*/, 10 /* delay */, false /* auto_del */ );
  }
}

static lv_obj_t * create_settings_button(lv_obj_t * scr)
{
    settings_button= lv_btn_create(scr);
    lv_obj_remove_style_all(settings_button);
    lv_style_init(&transparent_button_style);
    lv_style_set_bg_opa(&transparent_button_style,  LV_OPA_TRANSP);
    lv_obj_add_style(settings_button, &transparent_button_style,LV_PART_MAIN);
    lv_obj_t * settings_button_label = lv_label_create(settings_button);
    // Custom sized gear "settings" icon button, see LVGL lv_font docs
    lv_obj_set_style_text_font(settings_button_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(settings_button_label, lv_color_black(), 0);
    lv_label_set_text(settings_button_label, LV_SYMBOL_SETTINGS);
    lv_obj_center(settings_button_label);
    lv_obj_remove_flag(settings_button, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(settings_button, settings_button_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_width(settings_button, 60); 
    lv_obj_set_height(settings_button, 60);
    lv_obj_align(settings_button, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    return settings_button;
}

static void radio_event_handler(lv_event_t * e)
{
    int32_t * active_id = (int32_t *)lv_event_get_user_data(e);
    lv_obj_t * container = (lv_obj_t *)lv_event_get_current_target(e);
    lv_obj_t * act_cb = lv_event_get_target_obj(e);
    lv_obj_t * old_cb = lv_obj_get_child(container, *active_id);

    /*Do nothing if the container was clicked*/
    if(act_cb == container) return;

    lv_obj_remove_state(old_cb, LV_STATE_CHECKED);   /*Uncheck the previous radio button*/
    lv_obj_add_state(act_cb, LV_STATE_CHECKED);     /*Check the current radio button*/

    *active_id = lv_obj_get_index(act_cb);
    /* Save the operating mode: clean, rinse or spin */
    /*  active id range is 1 (clean), 2(rinse), 3(spin) - correct for OperatingMode enum values*/
    int32_t opmode = *active_id - 1;
    g_operating_mode = static_cast<OperatingMode>(opmode);
}

static void radio_button_create(lv_obj_t * parent, const char * txt)
{
    lv_obj_t * obj = lv_checkbox_create(parent);
    lv_checkbox_set_text(obj, txt);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_style(obj, &style_radio, LV_PART_INDICATOR);
    lv_obj_add_style(obj, &style_radio_chk, LV_PART_INDICATOR | LV_STATE_CHECKED);
}

static lv_obj_t * create_mode_selector(lv_obj_t * scr)
{
    static int32_t active_index=1;
    lv_obj_t * radio_button_container = lv_obj_create(scr);
    lv_obj_set_size(radio_button_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(radio_button_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_t * rb_box_label = lv_label_create(radio_button_container);
    lv_label_set_text(rb_box_label, "Mode Select");
    lv_obj_set_style_text_font(rb_box_label, &lv_font_montserrat_16, 0);
    //lv_obj_align(rb_box_label, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_align_to(radio_button_container, start_button, LV_ALIGN_OUT_RIGHT_TOP, 50, 0);
    lv_obj_add_event_cb(radio_button_container, radio_event_handler, LV_EVENT_CLICKED, &active_index);
    lv_obj_add_style(radio_button_container, &style_radio_button_container, 0);

    radio_button_create(radio_button_container, "Clean");
    radio_button_create(radio_button_container, "Rinse");
    radio_button_create(radio_button_container, "Spin");
    lv_obj_add_state(lv_obj_get_child(radio_button_container, 1), LV_STATE_CHECKED); 
    return radio_button_container;
}

static void create_machine_status(lv_obj_t * scr)
{
    /* Machine status text label*/
    mach_status_label = lv_label_create(scr);
    lv_obj_set_width(mach_status_label, 100);
    lv_obj_set_height(mach_status_label, 25);
    lv_obj_align(mach_status_label, LV_ALIGN_TOP_LEFT, 15, 215);
    lv_label_set_text(mach_status_label, Mach_Status_Text_Stopped); // Keyed by start button 
    /* Time remaining label - aligned next to machine status label */
    time_remaining_label = lv_label_create(scr);
    lv_obj_set_width(time_remaining_label, 50);
    lv_obj_set_height(time_remaining_label, 25);
    lv_obj_align_to(time_remaining_label, mach_status_label, LV_ALIGN_OUT_RIGHT_MID, 0, 0); 
    lv_label_set_text(time_remaining_label, Mach_Status_Text_Time_Remaining); // Keyed by start button 
    return;
}

void wcc_create_main_screen_widgets()
{
    LV_ASSERT(main_screen != NULL);
    (void)wcc_create_title_bar(main_screen, "Watch Cleaner Controller");
    (void)create_start_button(main_screen);
    (void)create_stop_button(main_screen);
    (void)create_settings_button(main_screen);
    (void)create_mode_selector(main_screen);
    (void)create_machine_status(main_screen);
    return;
}
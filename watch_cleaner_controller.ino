/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/integration/framework/arduino.html  */

#include <lvgl.h>
#include "FS.h"
#if LV_USE_TFT_ESPI
#include <TFT_eSPI.h>
#endif

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 *Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 *as the examples and demos are now part of the main LVGL library. */

//#include <examples/lv_examples.h>
//#include <demos/lv_demos.h>
// Used in lv_indev.c in LVGL library


#define WCC_VER "2.0.0" 
#define FIX_LVINDEV_ROTATION
#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL false

#define SCREEN_PORTRAIT 0
#define SCREEN_LANDSCAPE 1
#define POR_HOR 240
#define POR_VER 320
#define LAND_HOR 320
#define LAND_VER 240


/*Set to your screen resolution and rotation*/
#define SCREEN_ORIENTATION SCREEN_LANDSCAPE


#if SCREEN_ORIENTATION == SCREEN_LANDSCAPE
#define TFT_ROTATION LV_DISPLAY_ROTATION_90
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define TFT_HOR_RES LAND_HOR
#define TFT_VER_RES LAND_VER
#else
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define TFT_ROTATION   LV_DISPLAY_ROTATION_0 //LV_DISPLAY_ROTATION_0
#define TFT_HOR_RES   POR_HOR 
#define TFT_VER_RES   POR_VER 
#endif

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

/* Create the TFT_eSPI object used for touch calibration.
   The TFT_eSPI object is always created with the canonical
   screen width and height (240x320 for ILI9341).
 */
TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT); 

#if 0
/* Global Widget Objects */
lv_obj_t * btn1; // button 1 (ON)
lv_obj_t * slider; // slider
lv_obj_t * sl_label; // slider label
#endif
lv_obj_t * mach_status_label;

/* Create reusable on/off button styles */
static lv_style_t on_button_style;  // button appearance when not clicked
static lv_style_t off_button_style; // button appearance when clicked

static void set_screen_bg_style(lv_obj_t * scr)
{
  static lv_style_t style;
  lv_style_init(&style);
  lv_style_set_radius(&style, 5);

  /*Make a gradient*/
  lv_style_set_bg_opa(&style, LV_OPA_COVER);
  lv_style_set_bg_color(&style, lv_palette_lighten(LV_PALETTE_RED, 1));
  lv_obj_add_style(scr, &style, 0);
}

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}
#endif

/* LVGL calls it when a rendered image needs to copied to the display.
    Note this is not needed for TFT_eSPI created via the LVGL interface.
    Bodmer supplies the rendering code. Left here for future reference.
 */
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
    /*Copy `px map` to the `area`*/

    /*For example ("my_..." functions needs to be implemented by you)
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);

    my_set_window(area->x1, area->y1, w, h);
    my_draw_bitmaps(px_map, w * h);
     */
    if (!disp) {
        Serial.println("my_disp_flush failed, no valid display returned");
        return;
    }
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    tft.pushImage(area->x1, area->y1, w, h, (uint16_t *) px_map);

    /*Call it to tell LVGL you are ready*/
    lv_display_flush_ready(disp);
}

/* Read the touchpad coordinates. It appears that the 
  TFT_eSPI touch coordinates are translated to the screen orientation.
  That is, in portrait and landscape modes (0 and 90 rotation) upper left
  is coordinate 0,0 (x,y). LVGL was modified in lv_indev.c to leave x,y
  as is from TFT_eSPI getTouch call.
 */
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
    uint16_t x, y;
    bool touched = tft.getTouch( &x, &y );

    if(!touched) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
#if 0
        Serial.printf("x actual: %d y actual: %d\n", x, y);
        Serial.println("Note: x and y are untranslated in lv_indev.c");
        Serial.printf("data.x = %d\n", x);
        Serial.printf("data.y = %d\n", y);
#endif
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x =x ;
        data->point.y = y ;
#if 0
        lv_area_t bc;
        lv_obj_get_coords(btn1, &bc);
        Serial.printf("Button coords: x1: %d, y1: %d, x2: %d, y2: %d\n",
            bc.x1,bc.y1,bc.x2,bc.y2);
#endif
    }
}


/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void)
{
    return millis();
}
static void main_button_event_cb(lv_event_t * event)
{
  lv_obj_t * button = lv_event_get_target_obj(event);
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t * label = lv_obj_get_child(button, 0);  // Label of button

  //Serial.printf("Button event is %d\n", code);
  // This is a checked button. 
  // LV_EVENT_VALUE_CHANGED, LV_EVENT_VALUE_CLICKED
  Serial.println("main button handler");
  // Change back to start mode, if checked already
  // I use strcmp instead of button state because timing of state change
  // button callback is not deterministic in my experiements
  if (code == LV_EVENT_VALUE_CHANGED) {
    if (lv_strcmp(lv_label_get_text(label),"STOP")==0) {
      lv_label_set_text(label,"START");
      lv_label_set_text(mach_status_label,"Stopped...");
    } 
    else {
      lv_label_set_text(label, "STOP");
      lv_label_set_text(mach_status_label,"Running...");
    }
  }
}

static void clean_spin_button_event_cb(lv_event_t * event)
{
  lv_obj_t * button = lv_event_get_target_obj(event);
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t * label = lv_obj_get_child(button, 0);  // Label of button

  //Serial.printf("Button event is %d\n", code);
  // This is a checked button. 
  // LV_EVENT_VALUE_CHANGED, LV_EVENT_VALUE_CLICKED
  Serial.println("Clean/spin button handler");
  // Change back to start mode, if checked already
  // I use strcmp instead of button state because timing of state change
  // button callback is not deterministic in my experiements
  if (code == LV_EVENT_VALUE_CHANGED) {
    if (lv_strcmp(lv_label_get_text(label),"CLEAN")==0) {
      lv_label_set_text(label,"SPIN");
    } 
    else {
      lv_label_set_text(label, "CLEAN");
    }
  }
}
#if 0
static void slider_event_cb(lv_event_t * e)
{
    //lv_obj_t * slider = lv_event_get_target_obj(e);
    Serial.println("slider callback");
    /*Refresh the text*/
    lv_label_set_text_fmt(sl_label, "%" LV_PRId32, lv_slider_get_value(slider));
    lv_obj_align_to(sl_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
}
#endif


void setup()
{
    String LVGL_Msg = "Watch Cleaner Controller";
    LVGL_Msg += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.begin( 115200 );
    Serial.println( LVGL_Msg );

    lv_init();

    /*Set a tick source so that LVGL will know how much time elapsed. */
    lv_tick_set_cb(my_tick);

    /* register print function for debugging */
#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print );
#endif

    lv_display_t * disp;
#if LV_USE_TFT_ESPI
    /* TFT_eSPI can be enabled lv_conf.h to initialize the display in a simple way
      Note modification of lv_tft_espi_create to accomodate TFT_ROTATION value. This 
      modification is in the LVGL library.
      */
    disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf), TFT_ROTATION);
    lv_display_set_default(disp);
    // This rotation is also required to be coherent with the TFT_eSPI rotation.
    lv_display_set_rotation(disp, TFT_ROTATION);
    // Add nice background to main screen
    set_screen_bg_style(lv_screen_active());
#if 0
    Serial.println("After lv_display_set_rotation");
    Serial.printf("disp->hor_res: %d  disp->ver_res: %d\n", 
      lv_display_get_horizontal_resolution(disp), lv_display_get_vertical_resolution(disp));
#endif
    
    /* Manual screen touch calibration */
    touch_calibrate();
#else
    /*Else create a display yourself*/
    disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif

    /*Initialize the (dummy) input device driver*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);

    /* Create some global styles */
    lv_style_init(&on_button_style);
    lv_style_init(&off_button_style);
    /* On button style */
    lv_style_set_radius(&on_button_style, 3);
    lv_style_set_bg_opa(&on_button_style, LV_OPA_100);
    lv_style_set_bg_color(&on_button_style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_border_opa(&on_button_style, LV_OPA_40);
    lv_style_set_border_width(&on_button_style, 4);
    lv_style_set_border_color(&on_button_style, lv_palette_main(LV_PALETTE_GREY));
    //lv_style_set_shadow_width(&on_button_style, 4);
    //lv_style_set_shadow_color(&on_button_style, lv_palette_main(LV_PALETTE_GREY));
    //lv_style_set_shadow_offset_x(&on_button_style, 8);
    //lv_style_set_shadow_offset_y(&on_button_style, 8);
    lv_style_set_outline_opa(&on_button_style, LV_OPA_COVER);
    lv_style_set_outline_color(&on_button_style, lv_color_black());
    lv_style_set_outline_width(&on_button_style, 2);
    lv_style_set_text_color(&on_button_style, lv_color_black());
    lv_style_set_pad_all(&on_button_style, 10);

    /* Off button style*/
    lv_style_set_radius(&off_button_style, 3);
    lv_style_set_bg_opa(&off_button_style, LV_OPA_100);
    lv_style_set_bg_color(&off_button_style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_border_opa(&off_button_style, LV_OPA_40);
    lv_style_set_border_width(&off_button_style, 4);
    lv_style_set_border_color(&off_button_style, lv_palette_darken(LV_PALETTE_GREY, 128));
    //lv_style_set_shadow_width(&off_button_style, 4);
    //lv_style_set_shadow_color(&off_button_style, lv_palette_main(LV_PALETTE_GREY));
    //lv_style_set_shadow_offset_x(&off_button_style, 8);
    //lv_style_set_shadow_offset_y(&off_button_style, 8);
    lv_style_set_outline_opa(&off_button_style, LV_OPA_COVER);
    lv_style_set_outline_color(&on_button_style, lv_color_black());
    lv_style_set_outline_width(&on_button_style, 2);
    lv_style_set_text_color(&off_button_style, lv_color_black());
    lv_style_set_pad_all(&off_button_style, 10);


    /* Header label */
    lv_obj_t *main_header_label = lv_label_create( lv_screen_active() );
    lv_label_set_text( main_header_label, "Watch Cleaner Controller, v" WCC_VER );
    lv_obj_set_style_text_font(main_header_label, &lv_font_montserrat_20, 0);
    lv_obj_align( main_header_label, LV_ALIGN_TOP_MID, 0, 0 );

    /* Start Button*/
    lv_obj_t * start_button; 
    start_button = lv_btn_create(lv_screen_active());
    lv_obj_remove_style_all(start_button);
    lv_obj_t * start_button_label = lv_label_create(start_button);
    lv_label_set_text(start_button_label, "START");
    lv_obj_center(start_button_label);
    lv_obj_set_style_text_font(start_button_label, &lv_font_montserrat_18, 0);
    lv_obj_add_flag(start_button, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_remove_flag(start_button, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(start_button, main_button_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_width(start_button, 75); 
    lv_obj_set_height(start_button,75);
    lv_obj_align(start_button, LV_ALIGN_TOP_LEFT, 30, 85);
    /* Apply styles */
    lv_obj_add_style(start_button, &on_button_style, LV_STATE_DEFAULT);
    lv_obj_add_style(start_button, &off_button_style, LV_STATE_CHECKED);

    /* Clean/Spin Button*/
    lv_obj_t * clean_spin_button; 
    clean_spin_button = lv_btn_create(lv_screen_active());
    lv_obj_remove_style_all(clean_spin_button);
    lv_obj_t * clean_spin_button_label = lv_label_create(clean_spin_button);
    lv_label_set_text(clean_spin_button_label, "CLEAN");
    lv_obj_center(clean_spin_button_label);
    lv_obj_set_style_text_font(clean_spin_button_label, &lv_font_montserrat_18, 0);
    lv_obj_add_flag(clean_spin_button, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_remove_flag(clean_spin_button, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(clean_spin_button, clean_spin_button_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_width(clean_spin_button, 75); 
    lv_obj_set_height(clean_spin_button,75);
    lv_obj_align_to(clean_spin_button, start_button, LV_ALIGN_OUT_RIGHT_MID, 15, 0);

    /* Apply styles */
    lv_obj_add_style(clean_spin_button, &on_button_style, LV_STATE_DEFAULT);
    lv_obj_add_style(clean_spin_button, &off_button_style, LV_STATE_CHECKED);

    /* Machine status text label*/
    mach_status_label = lv_label_create(lv_screen_active());
    lv_obj_set_width(mach_status_label, 100);
    lv_obj_set_height(mach_status_label, 25);
    lv_obj_align(mach_status_label, LV_ALIGN_TOP_LEFT, 15, 215);
    lv_label_set_text(mach_status_label, "Stopped..."); // Keyed by start button 

    #if 0
    /* Slider */
    slider = lv_slider_create(lv_screen_active());
    lv_obj_set_width(slider, 200);               
    lv_obj_align(slider, LV_ALIGN_TOP_LEFT, 10, 100);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);     /*Assign an event function*/

       /*Create a label above the slider*/
    sl_label = lv_label_create(lv_screen_active());
    lv_label_set_text(sl_label, "0");
    lv_obj_align_to(sl_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
    #endif

    Serial.println( "Setup done" );
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay(5); /* let this time pass */
}

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        Serial.println("Reading calibration data file");
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setRotation(SCREEN_ORIENTATION);
    tft.setTouch(calData);
  } else {
 
    // data not valid so recalibrate
  
    tft.setRotation(SCREEN_ORIENTATION);
    //Serial.printf("TFT width: %d\n", tft.width());
    //Serial.printf("TFT height: %d\n", tft.height());
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    Serial.println("Storing calibration data");
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

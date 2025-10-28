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

// Apparently the canonical screen width and height (always 240x320 for this TFT display)
TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT); 

// Global GUI Objects
lv_obj_t * btn1; // button 1 (ON)
lv_obj_t * on_off_label;
lv_obj_t * slider; // slider
lv_obj_t * sl_label; // slider label

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}
#endif

/* LVGL calls it when a rendered image needs to copied to the display*/
/* Not needed for TFT_eSPI, Bodmer already supplied a callback, this is not called */
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

/*Read the touchpad*/
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
    uint16_t x, y;
    bool touched = tft.getTouch( &x, &y );

    if(!touched) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
#if 0
        Serial.printf("x sent: %d y sent: %d\n", x, TFT_HOR_RES-y);
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x ;
        data->point.y = TFT_HOR_RES - y;
#endif
        Serial.printf("x actual: %d y actual: %d\n", x, y);
        Serial.println("Note: x and y are untranslated in lv_indev.c");
        Serial.printf("data.x = %d\n", x);
        Serial.printf("data.y = %d\n", y);
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
static void btn_event_cb(lv_event_t * event)
{
    lv_event_code_t code = lv_event_get_code(event);
    lv_obj_t * label = lv_obj_get_child(btn1, 0);  // Label of button

    //Serial.printf("Button event is %d\n", code);
    if(code == LV_EVENT_CLICKED) {
        Serial.printf("Clicked\n");
        if (strncmp(lv_label_get_text(label),"ON",2)==0) {
            lv_label_set_text(label, "OFF");
            lv_label_set_text(on_off_label, "OFF");
        }
        else {
            lv_label_set_text(label, "ON");
            lv_label_set_text(on_off_label, "ON");

        }
    }

}
static void slider_event_cb(lv_event_t * e)
{
    //lv_obj_t * slider = lv_event_get_target_obj(e);
    Serial.println("slider callback");
    /*Refresh the text*/
    lv_label_set_text_fmt(sl_label, "%" LV_PRId32, lv_slider_get_value(slider));
    lv_obj_align_to(sl_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
}


void setup()
{
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.begin( 115200 );
    Serial.println( LVGL_Arduino );

    lv_init();

    /*Set a tick source so that LVGL will know how much time elapsed. */
    lv_tick_set_cb(my_tick);

    /* register print function for debugging */
#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print );
#endif

    lv_display_t * disp;
#if LV_USE_TFT_ESPI
    /*TFT_eSPI can be enabled lv_conf.h to initialize the display in a simple way*/
    //disp = lv_tft_espi_create(TFT_VER_RES, TFT_HOR_RES, draw_buf, sizeof(draw_buf));
    // Note modification of lv_tft_espi_create to accomodate TFT_ROTATION value
    disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf), TFT_ROTATION);
    lv_display_set_default(disp);
    // This rotation is also required
    lv_display_set_rotation(disp, TFT_ROTATION);
    Serial.println("After lv_display_set_rotation");
    Serial.printf("disp->hor_res: %d  disp->ver_res: %d\n", 
      lv_display_get_horizontal_resolution(disp), lv_display_get_vertical_resolution(disp));
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

    /* Create a simple label
     * ---------------------
     lv_obj_t *label = lv_label_create( lv_screen_active() );
     lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
     lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );

     * Try an example. See all the examples
     *  - Online: https://docs.lvgl.io/master/examples.html
     *  - Source codes: https://github.com/lvgl/lvgl/tree/master/examples
     * ----------------------------------------------------------------

     lv_example_btn_1();

     * Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMO_WIDGETS
     * -------------------------------------------------------------------------------------------

     lv_demo_widgets();
     */

    lv_obj_t *label = lv_label_create( lv_screen_active() );
    lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
    lv_obj_align( label, LV_ALIGN_TOP_MID, 0, 0 );

    /* Button*/
    btn1 = lv_btn_create(lv_screen_active());
    lv_obj_t * btn1_label = lv_label_create(btn1);
    lv_label_set_text(btn1_label, "ON");

    lv_obj_add_event_cb(btn1, btn_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_width(btn1, 50); //50
    lv_obj_set_height(btn1,25);
    //lv_obj_align(btn1, LV_ALIGN_CENTER, -80, -30);
    //lv_obj_align(btn1, LV_ALIGN_CENTER, 80, -50);
    lv_obj_align(btn1, LV_ALIGN_TOP_LEFT, 50, 150);
    //lv_obj_align(btn1, LV_ALIGN_TOP_RIGHT, 0, 0);
    //lv_obj_align(btn1, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    //lv_obj_align(btn1, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

    /* On/Off label modified by button click */
    on_off_label = lv_label_create(lv_screen_active());
    lv_obj_set_width(on_off_label, 50);
    lv_obj_set_height(on_off_label, 25);
    lv_obj_align(on_off_label, LV_ALIGN_TOP_MID, 0, 25);
    lv_label_set_text(on_off_label, "ON"); // Start button value

    /* Slider */
    #if 1
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
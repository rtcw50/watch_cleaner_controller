/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/integration/framework/arduino.html  */

#include <lvgl.h>
#include "FS.h"
#if LV_USE_TFT_ESPI
#include <TFT_eSPI.h>
#endif
#include "wcc_common.h"

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 *Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 *as the examples and demos are now part of the main LVGL library. */

//#include <examples/lv_examples.h>
//#include <demos/lv_demos.h>
// Used in lv_indev.c in LVGL library

SET_LOOP_TASK_STACK_SIZE(16384);
//#define ESP_LOOP_TASK_STACK_SIZE (16384)


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

/* Externs */
extern void wcc_create_main_screen_widgets();
extern void wcc_init_and_define_styles(void);
extern void wcc_set_screen_bg_style(lv_obj_t * scr);
extern void wcc_create_settings(void);

/* Create the TFT_eSPI object used for touch calibration.
   The TFT_eSPI object is always created with the canonical
   screen width and height (240x320 for ILI9341).
 */
TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT); 

/* Global Widget Objects */
lv_obj_t * main_screen;
OperatingMode g_operating_mode;

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
//    Enable 10s delay for Serial output in setup() function
    delay(10000);
    String LVGL_Msg = "Watch Cleaner Controller";
    LVGL_Msg += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.begin( 115200 );
    Serial.println( LVGL_Msg );
    Serial.printf("Arduino Stack was set to %d bytes", getArduinoLoopTaskStackSize());

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
    main_screen = lv_screen_active();
    LV_ASSERT(main_screen != NULL);
    wcc_set_screen_bg_style(main_screen);

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
#endif // LV_USE_TFT_ESPI

    /*Initialize the (dummy) input device driver*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);

    /* Create some global styles for buttons and whatnot */
    wcc_init_and_define_styles();

    /* Main screen layout */
    wcc_create_main_screen_widgets();

    /* Settings Screen layout */
    wcc_create_settings();

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

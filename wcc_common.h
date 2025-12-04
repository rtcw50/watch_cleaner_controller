
/*
 * File: wcc_common.h
 * Author: John
 * Date: 2025-11-16
 * Description: Common defines and data declarations
 */
#ifndef WCC_COMMON_H
#define WCC_COMMON_H

/* Version */
#define WCC_VER "1.0"

/* Custom colors (lv_color_t) picked from: https://codepen.io/kevinli/pen/GRpXOvo */
#define WCC_BACKGROUND_GREY 0x99,0x99,0x99
#define WCC_TITLE_BLUE 0x0c,0x00,0xcc
#define WCC_BUTTON_GREEN 0x6f,0xe0,0x00
#define WCC_BUTTON_RED 0xff,0x28,0x28
#define WCC_BUTTON_YELLOW 0xff,0xff,0x78
#define TIME_FORMAT "%02d:%02d"

 #define CLEAN_DUR_DEFAULT (5*60)
 #define CLEAN_DUR_MAX (60*60)
 #define RINSE_DUR_DEFAULT (3*60)
 #define RINSE_DUR_MAX (60*60)
 #define SPIN_DUR_DEFAULT  (1*60) 
 #define SPIN_DUR_MAX (60*60)
 #define AGITATE_DUR_DEFAULT (10)
 #define AGITATE_DUR_MAX (60)
 #define MAX_RPM_DEFAULT (600)
 #define MAX_RPM_MAX (600)
 #define SPIN_UP_DEFAULT (3)
 #define SPIN_UP_MAX (10)

 #define RAMP_UPDATE_MS (250)
 #define RAMP_UPDATE_STEPS_PER_SECOND (4) // 1/RAMP_UPDATE_MS

 #define WCC_IN1 D4 
 #define WCC_IN2 D5

enum class OperatingMode {
  clean,
  rinse,
  spin
};

/* Motor state */
enum class OperatingState {
  running,
  stopped
};

typedef struct time_format {
  int32_t min;
  int32_t sec;
} time_format;

typedef struct pwm_info { 
int32_t pwm_rpm;       // Actual PWM setting based on the RPM setting, calculate when RPM changes
int32_t pwm_increment; // Actual PWM increment based on spin up time setting 
} pwm_info;

/* Generic WCC callback type */
typedef void (*wcc_cb_t)(void);

#endif
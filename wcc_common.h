
/*
 * File: wcc_common.h
 * Author: John
 * Date: 2025-11-16
 * Description: Common defines and data declarations
 */
#ifndef WCC_COMMON_H
#define WCC_COMMON_H

/* Custom colors (lv_color_t) picked from: https://codepen.io/kevinli/pen/GRpXOvo */
#define WCC_BACKGROUND_GREY 0x99,0x99,0x99
#define WCC_TITLE_BLUE 0x0c,0x00,0xcc
#define WCC_BUTTON_GREEN 0x6f,0xe0,0x00
#define WCC_BUTTON_RED 0xff,0x28,0x28
#define WCC_BUTTON_YELLOW 0xff,0xff,0x78

enum class OperatingMode {
  clean,
  rinse,
  spin
};

#endif
/*
 * cardputer_bsp — minimal ESP-IDF board support for M5Cardputer (ESP32-S3):
 * ST7789 240x135 SPI LCD, SD over SPI, and the 74HC138 GPIO keyboard matrix.
 * Pin numbers from the M5Cardputer library (MIT).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_lcd_panel_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CP_LCD_W 240
#define CP_LCD_H 135

/* Display */
esp_lcd_panel_handle_t cp_lcd_init(void);          /* returns panel handle      */
void cp_lcd_backlight(bool on);
/* draw rgb565 buffer to a rect (inclusive-exclusive) */
void cp_lcd_draw(int x0, int y0, int x1, int y1, const uint16_t *px);

/* SD card (SPI). Mounts at /sd. Returns true on success. */
bool cp_sd_init(void);

/* Keyboard matrix. Returns up to `max` pressed logical keys as (x,y) in [0..13]
 * x and [0..3] y (the M5Cardputer layout grid). Returns the count. */
typedef struct { uint8_t x, y; } cp_key_t;
void cp_kbd_init(void);
int  cp_kbd_scan(cp_key_t *out, int max);

#ifdef __cplusplus
}
#endif

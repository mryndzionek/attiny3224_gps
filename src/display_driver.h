#pragma once

#include <stdint.h>

#include "driver_st7789.h"

#define DISPLAY_COLUMNS (240)
#define DISPLAY_ROWS (240)

uint8_t display_driver_init(void);
uint8_t display_driver_deinit(void);

uint8_t display_driver_write_point(uint16_t x, uint16_t y, uint32_t color);
uint8_t display_driver_draw_picture_16bits(uint16_t left, uint16_t top,
                                           uint16_t right, uint16_t bottom,
                                           uint16_t *img);

uint8_t display_driver_clear(void);
uint8_t display_driver_string(uint16_t x, uint16_t y, char *str, uint16_t len,
                              uint32_t color, st7789_font_t font);
uint8_t display_driver_rect(uint16_t left, uint16_t top, uint16_t right,
                            uint16_t bottom, uint32_t color);
uint8_t display_driver_on(void);
uint8_t display_driver_off(void);

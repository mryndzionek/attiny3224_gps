#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "config.h"
#include "display_driver.h"
#include "pff.h"

#define LED_PIN (PIN2_bm)

#define TILE_SIZE (128)
#define LINES_IN_XFER (2)

#define CENTER_TILE_LEFT ((DISPLAY_COLUMNS - TILE_SIZE) / 2)
#define CENTER_TILE_TOP ((DISPLAY_ROWS - TILE_SIZE) / 2)

#define DISP_CENTER_X (DISPLAY_COLUMNS / 2)
#define DISP_CENTER_Y (DISPLAY_ROWS / 2)

// #define TEST_LAT (54.42065279135835)
// #define TEST_LON (18.575293700140403)

#define TEST_LON (18.548752)
#define TEST_LAT (54.422813)

#define COLOR_WHITE (0xFFFF)
#define COLOR_RED (0xF800)
#define COLOR_GREEN (0x7E0)

typedef struct {
  bool locked;
  bool redraw;
  double lat;
  double lon;
  uint32_t center_tx;
  uint32_t center_ty;
  uint32_t current_tx;
  uint32_t current_ty;
  uint16_t tile_dx;
  uint16_t tile_dy;
  int16_t disp_dx;
  int16_t disp_dy;
  uint8_t zoom;
  char map;
  uint8_t pos_size;
} map_view_t;

static void configure_pins(void) {
  PORTA.DIR = 0;
  PORTA.PIN0CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN1CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN2CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN3CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN4CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN5CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN6CTRL |= PORT_PULLUPEN_bm;
  PORTA.PIN7CTRL |= PORT_PULLUPEN_bm;

  PORTB.DIR = 0;
  PORTB.PIN0CTRL |= PORT_PULLUPEN_bm;
  PORTB.PIN1CTRL |= PORT_PULLUPEN_bm;
  PORTB.PIN2CTRL |= PORT_PULLUPEN_bm;
  PORTB.PIN3CTRL |= PORT_PULLUPEN_bm;
}

static double asinh(double x) { return log(x + sqrt((x * x) + 1.0)); }

static void debug_print(const char *const fmt, ...) {
  (void)fmt;
  while (true) {
    PORTB.OUTTGL = LED_PIN;
    _delay_ms(100);
  }
}

static uint32_t long2tilex(double lon, int z, uint16_t *dx) {
  const double x = (lon + 180.0) / 360.0 * (1UL << z);
  *dx = floor(TILE_SIZE * fmod(x, 1.0));
  return (uint32_t)floor(x);
}

static uint32_t lat2tiley(double lat, int z, uint16_t *dy) {
  const double latrad = lat * M_PI / 180.0;
  long long const n = 1UL << z;
  const double y = (1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * n;
  *dy = floor(TILE_SIZE * fmod(y, 1.0));
  return (uint32_t)floor(y);
}

static void draw_map_tile(char map, int16_t dx, int16_t dy, uint32_t tx,
                          uint32_t ty, uint8_t zoom) {
  char tmp[32];
  uint16_t line[LINES_IN_XFER * TILE_SIZE];
  FRESULT fr;
  UINT br;

  uint16_t ox = 0;
  uint16_t oy = 0;
  uint16_t lx = TILE_SIZE;
  uint16_t ly = TILE_SIZE;

  if (dx >= DISPLAY_COLUMNS) {
    return;
  }
  if (dx <= -TILE_SIZE) {
    return;
  }

  if (dy >= DISPLAY_ROWS) {
    return;
  }
  if (dy <= -TILE_SIZE) {
    return;
  }

  if (dx < 0) {
    ox = -dx;
    lx = (int16_t)TILE_SIZE + dx;
    dx = 0;
  } else if ((dx + TILE_SIZE) >= DISPLAY_COLUMNS) {
    lx = DISPLAY_COLUMNS - dx;
  }

  if (dy < 0) {
    oy = -dy;
    ly = (int16_t)TILE_SIZE + dy;
    dy = 0;
  } else if ((dy + TILE_SIZE) >= DISPLAY_ROWS) {
    ly = DISPLAY_ROWS - dy;
  }

  snprintf(tmp, sizeof(tmp), "%c/%d/%ld/%ld.BIN", map, zoom, tx, ty);
  fr = pf_open(tmp);
  if (fr != FR_OK) {
    return;
  }

  fr = pf_lseek(2 * (TILE_SIZE * ox));
  if (fr != FR_OK) {
    debug_print("Failed to seek from file");
    return;
  }

  for (uint8_t x = 0; x < lx; x += LINES_IN_XFER) {
    fr = pf_read(line, TILE_SIZE * LINES_IN_XFER * 2, &br);
    if (fr != FR_OK) {
      debug_print("Failed to read line from file");
      return;
    }
    if (!br) {
      return;
    }

    if ((br % (TILE_SIZE * 2)) != 0) {
      debug_print("Wrong line length");
      return;
    }
    const uint8_t ln = br / (TILE_SIZE * 2);

    if (ly != TILE_SIZE) {
      for (uint8_t k = 0; k < ln; k++) {
        memmove(&line[k * ly], &line[k * TILE_SIZE + oy], 2 * ly);
      }
    }

    display_driver_draw_picture_16bits(dx + x, dy, dx + x + (ln - 1),
                                       dy + ly - 1, line);
  }
}

static void draw_pos(uint16_t x, uint16_t y, uint8_t size, uint16_t color) {
  const uint8_t r = size / 2;
  display_driver_rect(x - r, y - r, x + r, y + r, color);
}

void map_view_get_xy_from_center(map_view_t *self, int16_t *dx, int16_t *dy) {
  if (self->tile_dx >= (TILE_SIZE / 2)) {
    *dx = self->tile_dx - (TILE_SIZE / 2);
  } else {
    *dx = -((TILE_SIZE / 2) - self->tile_dx);
  }

  if (self->current_tx > self->center_tx) {
    *dx += (TILE_SIZE * (self->current_tx - self->center_tx));
  } else if (self->current_tx < self->center_tx) {
    *dx -= (TILE_SIZE * (self->center_tx - self->current_tx));
  }

  if (self->tile_dy >= (TILE_SIZE / 2)) {
    *dy = self->tile_dy - (TILE_SIZE / 2);
  } else {
    *dy = -((TILE_SIZE / 2) - self->tile_dy);
  }

  if (self->current_ty > self->center_ty) {
    *dy += (TILE_SIZE * (self->current_ty - self->center_ty));
  } else if (self->current_ty < self->center_ty) {
    *dy -= (TILE_SIZE * (self->center_ty - self->current_ty));
  }

  *dx += self->disp_dx;
  *dy += self->disp_dy;
}

void map_view_set_pos(map_view_t *self, double lat, double lon) {
  self->lat = lat;
  self->lon = lon;
  self->current_ty = lat2tiley(self->lat, self->zoom, &self->tile_dy);
  self->current_tx = long2tilex(self->lon, self->zoom, &self->tile_dx);
}

void map_view_update_pos(map_view_t *self, double lat, double lon) {
  map_view_set_pos(self, lat, lon);
  if (!self->locked) {
    self->center_ty = self->current_ty;
    self->center_tx = self->current_tx;
    self->disp_dy = (TILE_SIZE / 2) - self->tile_dy;
    self->disp_dx = (TILE_SIZE / 2) - self->tile_dx;
    self->locked = true;
    self->redraw = true;
  } else {
    int16_t dx = 0;
    int16_t dy = 0;
    map_view_get_xy_from_center(self, &dx, &dy);

    if ((abs(dx) > ((DISPLAY_COLUMNS / 2) - 5)) ||
        (abs(dy) > ((DISPLAY_ROWS / 2) - 5))) {
      self->center_ty = self->current_ty;
      self->center_tx = self->current_tx;
      self->disp_dy = (TILE_SIZE / 2) - self->tile_dy;
      self->disp_dx = (TILE_SIZE / 2) - self->tile_dx;
      self->redraw = true;
    }
  }
}

void map_view_set_zoom(map_view_t *self, uint8_t zoom) {
  if (self->zoom != zoom) {
    self->zoom = zoom;
    self->current_ty = lat2tiley(self->lat, self->zoom, &self->tile_dy);
    self->current_tx = long2tilex(self->lon, self->zoom, &self->tile_dx);
    self->center_ty = self->current_ty;
    self->center_tx = self->current_tx;
    if (self->locked) {
      self->disp_dy = (TILE_SIZE / 2) - self->tile_dy;
      self->disp_dx = (TILE_SIZE / 2) - self->tile_dx;
      self->redraw = true;
    }
  }
}

void map_view_draw(map_view_t *self) {
  if (!self->locked) {
    return;
  }

  int16_t dx = 0;
  int16_t dy = 0;
  map_view_get_xy_from_center(self, &dx, &dy);

  if (self->redraw) {
    // display_driver_off();
    display_driver_clear();

    for (int8_t x = -1; x < 2; x++) {
      for (int8_t y = -1; y < 2; y++) {
        draw_map_tile(
            self->map,
            (int16_t)CENTER_TILE_LEFT - (TILE_SIZE * x) + self->disp_dx,
            (int16_t)CENTER_TILE_TOP - (TILE_SIZE * y) + self->disp_dy,
            self->center_tx - x, self->center_ty - y, self->zoom);
      }
    }
    // display_driver_on();
    self->redraw = false;
  }
  draw_pos(DISP_CENTER_X + dx, DISP_CENTER_Y + dy, self->pos_size, COLOR_RED);
}

int main(void) {
  static FATFS fs;
  // 20MHz / 2 = 10MHz clock
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
  _PROTECTED_WRITE(CLKCTRL.OSC20MCTRLA, CLKCTRL_RUNSTDBY_bp);

  power_all_disable();
  configure_pins();

  PORTB.DIRSET = LED_PIN;
  PORTB.OUTSET = LED_PIN;

  PORTA.DIRSET = PIN4_bm;
  PORTA.OUTCLR = PIN4_bm;
  PORTA.OUTSET = PIN4_bm;

  uint8_t res = display_driver_init();
  if (res != 0) {
    debug_print("Failed to initialize the display");
    return 0;
  }

  FRESULT fr;
  for (size_t i = 0; i < 10; i++) {
    fr = pf_mount(&fs);
    if (fr == FR_OK) {
      break;
    }
  }
  if (fr != FR_OK) {
    debug_print("Failed to mount SD card");
    return 0;
  }

  map_view_t main_view = {
      .zoom = 18,
      .locked = false,
      .redraw = false,
      .map = 'M',
      .pos_size = 8,
  };

  map_view_t helper_view = main_view;

  // Precompute track
  static double track[2 * 360 / 4];
  for (size_t i = 0; i < 360 / 4; i++) {
    const double radians = (double)(i * 4) * M_PI / 180.0;
    track[2 * i] = TEST_LAT + 0.002 * sin(radians);
    track[2 * i + 1] = TEST_LON + 0.004 * cos(radians);
  }

  for (uint8_t z = 12; z < 19; z++) {
    map_view_update_pos(&main_view, TEST_LAT, TEST_LON);
    map_view_set_zoom(&main_view, z);
    map_view_draw(&main_view);
    _delay_ms(1000);
  }

  while (true) {
    for (size_t i = 0; i < 360 / 4; i++) {
      bool red = false;
      const double lat = track[2 * i];
      const double lon = track[2 * i + 1];
      map_view_update_pos(&main_view, lat, lon);
      if (!main_view.redraw) {
        _delay_ms(500);
      } else {
        red = true;
      }
      map_view_draw(&main_view);
      if (red)
        for (size_t j = 0; j < i; j++) {
          const double lat = track[2 * j];
          const double lon = track[2 * j + 1];
          map_view_set_pos(&helper_view, lat, lon);
          helper_view.center_tx = main_view.center_tx;
          helper_view.center_ty = main_view.center_ty;
          helper_view.disp_dx = main_view.disp_dx;
          helper_view.disp_dy = main_view.disp_dy;
          int16_t dx, dy;
          map_view_get_xy_from_center(&helper_view, &dx, &dy);
          draw_pos(DISP_CENTER_X + dx, DISP_CENTER_Y + dy, 4, COLOR_RED);
        }
    }
  }
}

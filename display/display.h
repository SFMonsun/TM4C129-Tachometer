#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

// ======================
// Display configuration
// ======================
#define MAX_X 800
#define MAX_Y 480
#define NEEDLE_THICK 4

// ======================
// Display controller command codes
// ======================
#define RST                 0x10
#define INITIAL_STATE       0x1F
#define SOFTWARE_RESET      0x01
#define SET_PLL_MN          0xE2
#define START_PLL           0xE0
#define LOCK_PLL            0xE0  // same as START_PLL
#define SET_LSHIFT          0xE6
#define SET_LCD_MODE        0xB0
#define SET_HORI_PERIOD     0xB4
#define SET_VERT_PERIOD     0xB6
#define SET_ADRESS_MODE     0x36
#define SET_PIXEL_DATA_FORMAT 0xF0
#define SET_DISPLAY_ON      0x29
#define SET_DISPLAY_OFF     0x28

// ======================
// Utility macro
// ======================
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

// ======================
// Color definitions
// ======================
enum colors {
    BLACK         = 0x00000000,
    WHITE         = 0x00FFFFFF,
    GREY          = 0x00AAAAAA,
    RED           = 0x00FF0000,
    GREEN         = 0x0000FF00,
    BLUE          = 0x000000FF,
    YELLOW        = 0x00FFFF00,
    ORANGE        = 0x00FF7034,
    BURNT_ORANGE  = 0x00662D15
};

// ======================
// External variables
// ======================
extern uint32_t sysClock;
extern int colorarray[];

// ======================
// Basic display functions
// ======================
void init_ports_display(void);
void configure_display_controller_large(void);
void write_command(unsigned char command);
void write_data(unsigned char data);
void window_set(int min_x, int min_y, int max_x, int max_y);

// ======================
// Drawing primitives
// ======================
void drawBox(int x_min, int x_max, int y_min, int y_max, enum colors col);
void drawPixel(int x, int y, enum colors col);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, enum colors color);
void drawCircle(int16_t x0, int16_t y0, int16_t r, enum colors color);
void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, enum colors color);

// ======================
// Digit and number rendering
// ======================
void drawDigit16x24(int x, int y, int digit, enum colors col, enum colors bgcol);
void drawDigit32x50(int x, int y, uint8_t digit, enum colors col, enum colors bgcol);
void drawNumber16x24(int x, int y, int number, enum colors col, enum colors bgcol);
void drawNumber32x50(int x, int y, uint64_t number, uint8_t digAmount, int8_t dp, uint8_t id, enum colors col, enum colors bgcol);

// ======================
// Bitmap rendering
// ======================
void drawBitmap1BPP(int x0, int y0, const unsigned char *bmp, int width, int height, enum colors colorFG, enum colors colorBG);

// ======================
// High-level display functions
// ======================
void InitSpeedometerDisplay(void);
void UpdateSpeedBars(uint32_t rpm, uint8_t *shadowArray, uint8_t *pictureArray, uint8_t startUp);
void UpdateRPMDisplay(uint32_t rpm);
void UpdateKMHDisplay(uint32_t kmh);
void UpdateODODisplay(uint64_t odo_decimeters);
void UpdateDirectionGear(uint8_t isForward);
void UpdateWarningLights(uint8_t errorCode);

// ======================
// Utility functions
// ======================
int16_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax);
int16_t ABS(int16_t value);

#endif  // DISPLAY_H

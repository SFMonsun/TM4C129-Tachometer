#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

// ======================
// Display configuration
// ======================
#define MAX_X 800
#define MAX_Y 480

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
#define SET_DISPLAY_OFF     0x28  // likely typo fixed from your old code

// ======================
// Color definitions
// ======================
enum colors {
    BLACK   = 0x00000000,
    WHITE   = 0x00FFFFFF,
    GREY    = 0x00AAAAAA,
    RED     = 0x00FF0000,
    GREEN   = 0x0000FF00,
    BLUE    = 0x000000FF,
    YELLOW  = 0x00FFFF00
};

// ======================
// External variables
// ======================
extern uint32_t sysClock;
extern int colorarray[];

// ======================
// Function prototypes
// ======================
void init_ports_display(void);
void configure_display_controller_large(void);
void write_command(unsigned char command);
void write_data(unsigned char data);
void window_set(int min_x, int min_y, int max_x, int max_y);

#endif  // DISPLAY_H

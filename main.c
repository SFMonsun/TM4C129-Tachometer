#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "display/display.h"
#include <driverlib/sysctl.h>
#include <stdbool.h> 
#include "Sensor/Sensor.h"
#include <driverlib/timer.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>

#define SENSOR_TIMER_BASE    TIMER0_BASE
#define SENSOR_TIMER_INT     INT_TIMER0A
#define DISPLAY_TIMER_BASE   TIMER1_BASE
#define DISPLAY_TIMER_INT    INT_TIMER1A

// Volatile variables updated in ISR
volatile float latestSpeed_kmh = 0.0f;
volatile float latestDistance_m = 0.0f;
volatile uint8_t displayUpdate = 0;

// Function prototypes
void SensorTimer_Init(uint32_t sysClock);
void DisplayTimer_Init(uint32_t sysClock);
void DisplaySpeedometer(float speed_kmh, float distance_m);
void DrawRectangle(int x, int y, int width, int height, uint32_t color);
void DrawString(int x, int y, const char* str, uint32_t color);

// Timer ISR for sensor reading (10ms = 100Hz)
void Timer0A_Handler(void)
{
    TimerIntClear(SENSOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    latestSpeed_kmh = Sensor_ReadSpeed();
    latestDistance_m = Sensor_GetDistance();
}

// Timer ISR for display update (100ms = 10Hz)
void Timer1A_Handler(void)
{
    TimerIntClear(DISPLAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    displayUpdate = 1;  // Signal main loop to update display
}

// Interrupt handler registration (if not using startup file auto-registration)
void Timer0IntHandler(void)
{
    Timer0A_Handler();
}

void Timer1IntHandler(void)
{
    Timer1A_Handler();
}

void SensorTimer_Init(uint32_t sysClock)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    TimerConfigure(SENSOR_TIMER_BASE, TIMER_CFG_PERIODIC);
    uint32_t loadValue = (sysClock / 100) - 1; // 10ms period
    TimerLoadSet(SENSOR_TIMER_BASE, TIMER_A, loadValue);

    IntRegister(SENSOR_TIMER_INT, Timer0IntHandler);  // Register handler
    TimerIntEnable(SENSOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(SENSOR_TIMER_INT);

    TimerEnable(SENSOR_TIMER_BASE, TIMER_A);
}

void DisplayTimer_Init(uint32_t sysClock)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)) {}

    TimerConfigure(DISPLAY_TIMER_BASE, TIMER_CFG_PERIODIC);
    uint32_t loadValue = (sysClock / 10) - 1; // 100ms period
    TimerLoadSet(DISPLAY_TIMER_BASE, TIMER_A, loadValue);

    IntRegister(DISPLAY_TIMER_INT, Timer1IntHandler);  // Register handler
    TimerIntEnable(DISPLAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(DISPLAY_TIMER_INT);

    TimerEnable(DISPLAY_TIMER_BASE, TIMER_A);
}

void DrawRectangle(int x, int y, int width, int height, uint32_t color)
{
    int i;
    window_set(x, y, x + width - 1, y + height - 1);
    write_command(0x2C);
    for(i = 0; i < (width * height); i++) {
        write_data((color >> 16) & 0xFF);
        write_data((color >> 8) & 0xFF);
        write_data(color & 0xFF);
    }
}

// Simple function to draw a digit (0-9) using rectangles
void DrawDigit(int x, int y, int digit, uint32_t color)
{
    // Simple 7-segment style digit representation
    // Each segment is 10x40 pixels for horizontal, 40x10 for vertical
    int segWidth = 40;
    int segHeight = 10;
    int gap = 5;
    
    // Define which segments are on for each digit (top, top-right, bottom-right, bottom, bottom-left, top-left, middle)
    const uint8_t segments[10] = {
        0b1111110,  // 0
        0b0110000,  // 1
        0b1101101,  // 2
        0b1111001,  // 3
        0b0110011,  // 4
        0b1011011,  // 5
        0b1011111,  // 6
        0b1110000,  // 7
        0b1111111,  // 8
        0b1111011   // 9
    };
    
    uint8_t seg = segments[digit % 10];
    
    // Top
    if(seg & 0b1000000) DrawRectangle(x + gap, y, segWidth, segHeight, color);
    // Top-right
    if(seg & 0b0100000) DrawRectangle(x + gap + segWidth, y + gap, segHeight, segWidth, color);
    // Bottom-right
    if(seg & 0b0010000) DrawRectangle(x + gap + segWidth, y + gap + segWidth + gap, segHeight, segWidth, color);
    // Bottom
    if(seg & 0b0001000) DrawRectangle(x + gap, y + 2 * segWidth + gap, segWidth, segHeight, color);
    // Bottom-left
    if(seg & 0b0000100) DrawRectangle(x, y + gap + segWidth + gap, segHeight, segWidth, color);
    // Top-left
    if(seg & 0b0000010) DrawRectangle(x, y + gap, segHeight, segWidth, color);
    // Middle
    if(seg & 0b0000001) DrawRectangle(x + gap, y + segWidth, segWidth, segHeight, color);
}

void DisplaySpeedometer(float speed_kmh, float distance_m)
{
    char buffer[32];
    int digit, i;
    int speed_int = (int)speed_kmh;
    int distance_int = (int)distance_m;
    
    // Clear display area for speed (top section)
    DrawRectangle(50, 50, 700, 150, 0x000000);  // Black background
    
    // Draw "SPEED:" label
    DrawRectangle(60, 60, 150, 40, 0x00FFFFFF);  // White label area
    
    // Draw speed value with large digits
    int xpos = 250;
    int hundreds = speed_int / 100;
    int tens = (speed_int / 10) % 10;
    int ones = speed_int % 10;
    
    // Draw three digits for speed
    if(hundreds > 0) {
        DrawDigit(xpos, 70, hundreds, 0x00FF0000);  // Red
        xpos += 70;
    }
    DrawDigit(xpos, 70, tens, 0x00FF0000);
    xpos += 70;
    DrawDigit(xpos, 70, ones, 0x00FF0000);
    
    // Draw "km/h" unit
    DrawRectangle(xpos + 80, 100, 120, 40, 0x00AAAAAA);  // Gray unit label
    
    // Clear display area for distance (bottom section)
    DrawRectangle(50, 250, 700, 150, 0x000000);  // Black background
    
    // Draw "DISTANCE:" label
    DrawRectangle(60, 260, 200, 40, 0x00FFFFFF);  // White label area
    
    // Draw distance value (in meters)
    xpos = 300;
    int dist_thousands = distance_int / 1000;
    int dist_hundreds = (distance_int / 100) % 10;
    int dist_tens = (distance_int / 10) % 10;
    int dist_ones = distance_int % 10;
    
    // Draw up to 4 digits for distance
    if(dist_thousands > 0) {
        DrawDigit(xpos, 270, dist_thousands, 0x0000FF00);  // Green
        xpos += 70;
    }
    if(dist_hundreds > 0 || dist_thousands > 0) {
        DrawDigit(xpos, 270, dist_hundreds, 0x0000FF00);
        xpos += 70;
    }
    DrawDigit(xpos, 270, dist_tens, 0x0000FF00);
    xpos += 70;
    DrawDigit(xpos, 270, dist_ones, 0x0000FF00);
    
    // Draw "m" unit
    DrawRectangle(xpos + 80, 300, 80, 40, 0x00AAAAAA);  // Gray unit label
}

int main(void)
{
    int x, y;
    uint32_t color;

    // Initialize system clock to 120 MHz
    sysClock = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Initialize display
    init_ports_display();
    configure_display_controller_large();

    printf("Display initialized\n");

    // Set background to black
    color = 0x000000;  // Black
    window_set(0, 0, MAX_X-1, MAX_Y-1);
    write_command(0x2C);
    for(x = 0; x < MAX_X; x++) {
        for(y = 0; y < MAX_Y; y++) {
            write_data((color >> 16) & 0xFF);
            write_data((color >> 8) & 0xFF);
            write_data(color & 0xFF);
        }
    }

    printf("Background ready\n");

    // Initialize sensor and timers
    Sensor_Init();
    SensorTimer_Init(sysClock);   // 100Hz sensor sampling
    DisplayTimer_Init(sysClock);  // 10Hz display update
    IntMasterEnable();

    printf("System ready - measuring speed\n");

    // Main loop
    while(1)
    {
        // Wait for display update flag from timer ISR
        if(displayUpdate) {
            displayUpdate = 0;
            
            // Update speedometer display with latest values
            DisplaySpeedometer(latestSpeed_kmh, latestDistance_m);
            
            // Print to console for debugging
            printf("Speed: %.2f km/h, Distance: %.2f m\n", latestSpeed_kmh, latestDistance_m);
        }
        
        // Optional: Add reset button check here
        // if(button_pressed) Sensor_ResetDistance();
    }
}
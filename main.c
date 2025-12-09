/**
 * main.c - Motor Speed Display for TM4C1294NCPDT
 *
 * Uses KMZ60 sensor for speed/direction detection
 * Displays RPM on NX8048T050 800x480 racing speedometer display
 */

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

#define DISPLAY_TIMER_BASE   TIMER1_BASE
#define DISPLAY_TIMER_INT    INT_TIMER1A

/* Volatile variables for display update */
volatile uint8_t displayUpdate = 0;

/* Display state arrays for speed bars */
uint8_t shadowArray[110] = {0};
uint8_t pictureArray[110] = {0};
uint8_t startUp = 1;

/* Counter for RPM display update (slower than bars) */
uint32_t rpmDisplayCounter = 0;

/* Last direction for change detection */
uint8_t lastDirection = 1;  // Start with forward (D)

/* Function prototypes */
void DisplayTimer_Init(uint32_t sysClock);

/* Timer ISR for display update (10Hz = every 100ms) */
void Timer1IntHandler(void)
{
    TimerIntClear(DISPLAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    displayUpdate = 1;
}

void DisplayTimer_Init(uint32_t sysClock)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)) {}

    TimerConfigure(DISPLAY_TIMER_BASE, TIMER_CFG_PERIODIC);
    uint32_t loadValue = (sysClock / 10) - 1; /* 100ms period = 10Hz */
    TimerLoadSet(DISPLAY_TIMER_BASE, TIMER_A, loadValue);

    IntRegister(DISPLAY_TIMER_INT, Timer1IntHandler);
    TimerIntEnable(DISPLAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(DISPLAY_TIMER_INT);

    TimerEnable(DISPLAY_TIMER_BASE, TIMER_A);
}

int main(void)
{
    /* Initialize system clock to 120 MHz */
    sysClock = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    /* Initialize display */
    init_ports_display();
    configure_display_controller_large();
    printf("Display initialized\n");

    /* Initialize speedometer display with background and static elements */
    InitSpeedometerDisplay();
    printf("Speedometer display ready\n");

    /*
     * CRITICAL: Enable master interrupts BEFORE initializing sensor
     * The sensor uses IntRegister() which needs interrupts enabled
     */
    IntMasterEnable();

    /* Initialize sensor (sets up GPIO per-pin interrupts for Port P) */
    Sensor_Init();

    /* Initialize display update timer */
    DisplayTimer_Init(sysClock);

    printf("System ready - spin motor to measure speed\n");
    printf("=============================================\n");

    /* Main loop */
    while(1)
    {
        /* Update display periodically */
        if(displayUpdate) {
            displayUpdate = 0;

            /* Update speed calculation (only when display updates) */
            /* IMPORTANT: Call Sensor_GetSpeed() FIRST - it calculates both speed and RPM */
            float speed = Sensor_GetSpeed();
            float rpm = Sensor_GetRPM();
            float distance = Sensor_GetDistance();
            RotationDirection dir = Sensor_GetDirection();

            /* Convert RPM to integer for display */
            uint32_t rpm_int = (uint32_t)rpm;

            /* Update speed bars (always update for smooth animation) */
            UpdateSpeedBars(rpm_int, shadowArray, pictureArray, startUp);

            /* Clear startup flag after first update */
            if (startUp) {
                startUp = 0;
            }

            /* Update RPM digital display every ~200ms (every other display update) */
            rpmDisplayCounter++;
            if (rpmDisplayCounter >= 2) {
                rpmDisplayCounter = 0;
                UpdateRPMDisplay(rpm_int);
            }

            /* Update direction gear indicator when direction changes */
            uint8_t isForward = (dir == DIR_FORWARD) ? 1 : 0;
            if (isForward != lastDirection) {
                lastDirection = isForward;
                UpdateDirectionGear(isForward);
            }

            /* Print to console for debugging */
            const char* dir_str = (dir == DIR_FORWARD) ? "FWD" :
                                  (dir == DIR_REVERSE) ? "REV" : "STOP";
            printf("RPM: %6.1f, Speed: %6.2f km/h [%s], Distance: %7.2f m, Edges: %lu\n",
                   rpm, speed, dir_str, distance,
                   (unsigned long)Sensor_GetEdgeCount());
        }
    }
}

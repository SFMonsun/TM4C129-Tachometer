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
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>

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

/* Counter for odometer update (1 Hz = once per second) */
uint32_t odoDisplayCounter = 0;

/* Last direction for change detection */
uint8_t lastDirection = 1;  // Start with forward (D)

/* Warning light flash counter (for 1s interval flashing) */
uint32_t warningFlashCounter = 0;
uint8_t warningFlashState = 0;

/* Check engine light state - turns ON permanently after 14k RPM hit once */
uint8_t checkEngineTriggered = 0;

/* Button state for reset functionality */
volatile uint8_t buttonPressed = 0;

/* Function prototypes */
void DisplayTimer_Init(uint32_t sysClock);
void Button_Init(void);

/* Timer ISR for display update (10Hz = every 100ms) */
void Timer1IntHandler(void)
{
    TimerIntClear(DISPLAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    displayUpdate = 1;
}

/* Button ISR for reset functionality (PJ0) */
void GPIOPortJIntHandler(void)
{
    uint32_t status = GPIOIntStatus(GPIO_PORTJ_BASE, true);
    GPIOIntClear(GPIO_PORTJ_BASE, status);

    if (status & GPIO_PIN_0) {
        buttonPressed = 1;
    }
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

void Button_Init(void)
{
    /* Enable Port J */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)) {}

    /* Configure PJ0 as input with pull-up (button connects to ground when pressed) */
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* Configure interrupt on falling edge (button press) */
    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);

    /* Clear any pending interrupts */
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);

    /* Enable GPIO interrupt for the pin */
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0);

    /* Register ISR with NVIC */
    IntRegister(INT_GPIOJ, GPIOPortJIntHandler);

    /* Enable interrupt in NVIC */
    IntEnable(INT_GPIOJ);
}

int main(void)
{
    /* Initialize system clock to 120 MHz */
    sysClock = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    /* Initialize display */
    init_ports_display();
    configure_display_controller_large();
   // printf("Display initialized\n");

    /* Initialize speedometer display with background and static elements */
    InitSpeedometerDisplay();
    //printf("Speedometer display ready\n");

    /*
     * CRITICAL: Enable master interrupts BEFORE initializing sensor
     * The sensor uses IntRegister() which needs interrupts enabled
     */
    IntMasterEnable();

    /* Initialize sensor (sets up GPIO per-pin interrupts for Port P) */
    Sensor_Init();

    /* Initialize display update timer */
    DisplayTimer_Init(sysClock);

    /* Initialize reset button (PJ0) */
    Button_Init();

    //printf("System ready - spin motor to measure speed\n");
    //printf("=============================================\n");

    /* Main loop */
    while(1)
    {
        /* Handle button press for reset */
        if(buttonPressed) {
            buttonPressed = 0;

            /* Reset odometer */
            Sensor_ResetDistance();

            /* Reset check engine light */
            checkEngineTriggered = 0;

            /* Brief delay to debounce */
            SysCtlDelay(sysClock / 30); /* ~100ms debounce */
        }

        /* Update display periodically */
        if(displayUpdate) {
            displayUpdate = 0;

            /* Update speed calculation (only when display updates) */
            /* IMPORTANT: Call Sensor_GetSpeed() FIRST - it calculates both speed and RPM */
            float speed_kmh = Sensor_GetSpeed();
            float rpm = Sensor_GetRPM();
            float distance_m = Sensor_GetDistance();
            RotationDirection dir = Sensor_GetDirection();

            /* Convert to integers for display */
            uint32_t rpm_int = (uint32_t)rpm;
            uint32_t kmh_int = (uint32_t)(speed_kmh * 7.0f); /* Scale KMH by 7x for display */
            uint64_t odo_decimeters = (uint64_t)(distance_m * 10.0f); /* Convert meters to decimeters */

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

                /* Update KMH display at same rate as RPM */
                UpdateKMHDisplay(kmh_int);
            }

            /* Update ODO display every 1s (10 * 100ms = 1000ms) */
            odoDisplayCounter++;
            if (odoDisplayCounter >= 10) {
                odoDisplayCounter = 0;
                UpdateODODisplay(odo_decimeters);
            }

            /* Update direction gear indicator when direction changes */
            uint8_t isForward = (dir == DIR_FORWARD) ? 1 : 0;
            if (isForward != lastDirection) {
                lastDirection = isForward;
                UpdateDirectionGear(isForward);
            }

            /* Warning lights management */
            /* 100ms tick counter for 1s interval flashing (10 ticks = 1 second) */
            warningFlashCounter++;
            if (warningFlashCounter >= 10) {
                warningFlashCounter = 0;
                warningFlashState = !warningFlashState; /* Toggle every 1 second */
            }

            /* Build error code bitfield:
             * Bit 0 (0x01): Water temperature - flashes at 1s interval when RPM > 14000
             * Bit 1 (0x02): ABS - always ON
             * Bit 2 (0x04): Battery - flashes at 1s interval when RPM > 14000
             * Bit 3 (0x08): Check engine - OFF initially, turns ON permanently after 14k RPM hit once
             */
            uint8_t errorCode = 0x00;

            /* Check engine light - trigger once at 14k RPM, then stay ON permanently */
            if (rpm_int > 14000 && !checkEngineTriggered) {
                checkEngineTriggered = 1;
            }
            if (checkEngineTriggered) {
                errorCode |= 0x08;
            }

            /* ABS light - always ON */
            errorCode |= 0x02;

            /* Water temp and battery - flash at 1s interval when RPM > 14000 */
            if (rpm_int > 14000) {
                if (warningFlashState) {
                    errorCode |= 0x01; /* Water temp ON */
                    errorCode |= 0x04; /* Battery ON */
                }
                /* else: both OFF (flashing effect) */
            }

            /* Update warning lights */
            UpdateWarningLights(errorCode);

            /* Print to console for debugging */
            const char* dir_str = (dir == DIR_FORWARD) ? "FWD" :
                                  (dir == DIR_REVERSE) ? "REV" : "STOP";
            //printf("RPM: %6.1f, Speed: %6.2f km/h [%s], Distance: %7.2f m, Edges: %lu\n",
            //       rpm, speed_kmh, dir_str, distance_m,
            //       (unsigned long)Sensor_GetEdgeCount());
        }
    }
}

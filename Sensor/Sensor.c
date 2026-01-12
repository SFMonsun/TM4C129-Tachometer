/**
 * Sensor.c - KMZ60 Speed and Direction Detection for TM4C1294NCPDT
 * 
 * CRITICAL FIX: Port P on TM4C1294 uses per-pin interrupts!
 * - P0 has INT_GPIOP0
 * - P1 has INT_GPIOP1
 * 
 * Based on the signal diagram:
 * - S1 connected to Port P0 (comparator output)
 * - S2 connected to Port P1 (comparator output)
 * - Signals are 90° phase-shifted (quadrature)
 * 
 * Forward (Rechtslauf):  11 -> 01 -> 00 -> 10 -> 11
 * Backward (Linkslauf):  11 -> 10 -> 00 -> 01 -> 11
 */

#include "Sensor.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

/* ============== Pin Definitions ============== */
#define S1_PORT         GPIO_PORTP_BASE
#define S1_PIN          GPIO_PIN_0
#define S2_PORT         GPIO_PORTP_BASE
#define S2_PIN          GPIO_PIN_1

/* Timer for measuring time between edges */
#define EDGE_TIMER_BASE TIMER2_BASE

/* 
 * CRITICAL: TM4C1294 Port P uses PER-PIN interrupts!
 * These values come from hw_ints.h - use the actual defines
 */
#ifndef INT_GPIOP0
#define INT_GPIOP0      92   /* Actual value from TM4C1294 hw_ints.h */
#endif
#ifndef INT_GPIOP1
#define INT_GPIOP1      93
#endif

/* Wheel parameters */
#define WHEEL_RADIUS_M      0.005f      /* 0.5cm radius */
#define WHEEL_CIRCUMFERENCE (2.0f * M_PI * WHEEL_RADIUS_M)
#define TIMER_FREQ          120000000.0f   /* 120 MHz system clock as float */

/* 
 * Edges per rotation - using BOTH edges on S1 AND S2
 * With quadrature: 4 edges per full rotation (2 on S1 + 2 on S2)
 * If magnet has multiple poles, multiply accordingly
 */
#define EDGES_PER_ROTATION  4.0f

/* Timeout: if no edge for this many timer ticks, motor is stopped */
/* At 120MHz, 60000000 ticks = 0.5 second */
#define STOPPED_TIMEOUT     60000000UL

/* Minimum period to filter noise (10µs at 120MHz = 1200 ticks) */
#define MIN_PERIOD          1200UL

/* Minimum update interval for speed calculation (100ms = 12M ticks at 120MHz) */
/* This ensures consistent measurement windows for stable readings */
/* Using 100ms to match display update rate for responsive updates */
#define MIN_UPDATE_INTERVAL 12000000UL

/* ============== Volatile Variables (shared with ISR) ============== */
volatile static uint32_t last_edge_time = 0;
volatile static uint32_t edge_period = 0;
volatile static uint8_t  new_edge_detected = 0;
volatile static uint8_t  last_state = 0;        /* Combined state: (S1<<1)|S2 */
volatile static int32_t  direction_counter = 0; /* Accumulated direction votes */
volatile static uint32_t edge_count = 0;        /* Total edges for distance */
volatile static uint32_t interrupt_count = 0;   /* Debug counter */

/* ============== Non-Volatile State ============== */
static RotationDirection current_direction = DIR_STOPPED;
static float current_speed_kmh = 0.0f;
static float current_rpm = 0.0f;
static float accumulated_distance = 0.0f;
static uint32_t last_edge_count = 0;
static uint32_t last_display_edge_count = 0;
static uint32_t last_display_time = 0;

/* Moving average filter for speed and RPM smoothing
 * Using smaller filter since we now have 300ms measurement windows */
#define SPEED_FILTER_SIZE 3
#define RPM_FILTER_SIZE 3
static float speed_buffer[SPEED_FILTER_SIZE] = {0};
static uint8_t speed_filter_index = 0;
static uint8_t speed_filter_count = 0;

static float rpm_buffer[RPM_FILTER_SIZE] = {0};
static uint8_t rpm_filter_index = 0;
static uint8_t rpm_filter_count = 0;

/* Direction hysteresis - need multiple consistent readings */
#define DIRECTION_THRESHOLD 5

/* ============== Direction Lookup Table ============== */
/*
 * Quadrature state transitions:
 * State = (S1 << 1) | S2
 *   State 0 = 00 (both low)
 *   State 1 = 01 (S2 high)
 *   State 2 = 10 (S1 high)
 *   State 3 = 11 (both high)
 * 
 * Forward sequence:  3 -> 1 -> 0 -> 2 -> 3  (11->01->00->10->11)
 * Backward sequence: 3 -> 2 -> 0 -> 1 -> 3  (11->10->00->01->11)
 * 
 * Table[old_state][new_state] = direction (+1, -1, or 0 for invalid/same)
 */
static const int8_t DIRECTION_TABLE[4][4] = {
    /*          to: 00  01  10  11  */
    /* from 00 */  { 0, +1, -1,  0 },
    /* from 01 */  {-1,  0,  0, +1 },
    /* from 10 */  {+1,  0,  0, -1 },
    /* from 11 */  { 0, -1, +1,  0 }
};

/* ============== Common Edge Handler ============== */
static void HandleEdge(void)
{
    uint32_t current_time;
    uint8_t current_state;
    uint8_t s1, s2;
    int8_t dir;
    
    /* Read timer IMMEDIATELY for accurate timing */
    current_time = TimerValueGet(EDGE_TIMER_BASE, TIMER_A);
    
    /* Read both pin states */
    s1 = GPIOPinRead(S1_PORT, S1_PIN) ? 1 : 0;
    s2 = GPIOPinRead(S2_PORT, S2_PIN) ? 1 : 0;
    current_state = (s1 << 1) | s2;
    
    /* Decode direction from state transition */
    dir = DIRECTION_TABLE[last_state][current_state];
    
    /* Only process valid transitions (skip glitches where state didn't change) */
    if (current_state != last_state) {
        uint32_t period;
        
        /* Calculate period (timer counts DOWN on TM4C) */
        if (last_edge_time >= current_time) {
            period = last_edge_time - current_time;
        } else {
            /* Timer wrapped around */
            period = (last_edge_time) + (0xFFFFFFFFUL - current_time) + 1;
        }
        
        /* Sanity check: ignore very short periods (noise/bounce) */
        if (period > MIN_PERIOD && period < STOPPED_TIMEOUT) {
            edge_period = period;
            new_edge_detected = 1;
            edge_count++;
            
            /* Accumulate direction votes for hysteresis */
            if (dir > 0) {
                direction_counter++;
                if (direction_counter > 100) direction_counter = 100; /* Clamp */
            } else if (dir < 0) {
                direction_counter--;
                if (direction_counter < -100) direction_counter = -100;
            }
        }
        
        last_edge_time = current_time;
        last_state = current_state;
    }
    
    interrupt_count++;
}

/* ============== GPIO Port P Pin 0 ISR (S1) ============== */
void GPIOP0_IRQHandler(void)
{
    /* Clear the interrupt */
    GPIOIntClear(S1_PORT, S1_PIN);
    
    /* Process the edge */
    HandleEdge();
}

/* ============== GPIO Port P Pin 1 ISR (S2) ============== */
void GPIOP1_IRQHandler(void)
{
    /* Clear the interrupt */
    GPIOIntClear(S2_PORT, S2_PIN);
    
    /* Process the edge */
    HandleEdge();
}

/* ============== Initialization ============== */
void Sensor_Init(void)
{
    /* Enable GPIO Port P */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)) {}
    
    /* Configure P0 (S1) and P1 (S2) as inputs */
    GPIOPinTypeGPIOInput(S1_PORT, S1_PIN | S2_PIN);
    
    /* Configure with weak pull-up (comparator outputs might be open-drain) */
    GPIOPadConfigSet(S1_PORT, S1_PIN | S2_PIN, 
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    /* Configure interrupts on BOTH edges for both pins */
    GPIOIntTypeSet(S1_PORT, S1_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(S2_PORT, S2_PIN, GPIO_BOTH_EDGES);
    
    /* Clear any pending interrupts */
    GPIOIntClear(S1_PORT, S1_PIN | S2_PIN);
    
    /* Enable GPIO interrupts for both pins */
    GPIOIntEnable(S1_PORT, S1_PIN);
    GPIOIntEnable(S2_PORT, S2_PIN);
    
    /* Register ISRs with NVIC - CRITICAL for TM4C1294! */
    IntRegister(INT_GPIOP0, GPIOP0_IRQHandler);
    IntRegister(INT_GPIOP1, GPIOP1_IRQHandler);
    
    /* Set interrupt priorities to HIGHEST (0x00 = highest, 0xE0 = lowest) */
    /* CRITICAL: Sensor timing is most important - must not be delayed */
    IntPrioritySet(INT_GPIOP0, 0x00);
    IntPrioritySet(INT_GPIOP1, 0x00);
    
    /* Enable interrupts in NVIC */
    IntEnable(INT_GPIOP0);
    IntEnable(INT_GPIOP1);
    
    /* Enable Timer2 as free-running counter for time measurement */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) {}
    
    /* Configure as 32-bit periodic timer (counts down) */
    TimerConfigure(EDGE_TIMER_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(EDGE_TIMER_BASE, TIMER_A, 0xFFFFFFFF);
    TimerEnable(EDGE_TIMER_BASE, TIMER_A);
    
    /* Read initial state */
    uint8_t s1 = GPIOPinRead(S1_PORT, S1_PIN) ? 1 : 0;
    uint8_t s2 = GPIOPinRead(S2_PORT, S2_PIN) ? 1 : 0;
    last_state = (s1 << 1) | s2;
    last_edge_time = TimerValueGet(EDGE_TIMER_BASE, TIMER_A);
    
    /* Initialize variables */
    edge_period = 0;
    new_edge_detected = 0;
    direction_counter = 0;
    edge_count = 0;
    last_edge_count = 0;
    interrupt_count = 0;
    current_speed_kmh = 0.0f;
    accumulated_distance = 0.0f;
    current_direction = DIR_STOPPED;
    
    /* Initialize time-based calculation references */
    last_display_edge_count = 0;
    last_display_time = TimerValueGet(EDGE_TIMER_BASE, TIMER_A);
    
    /* Clear speed and RPM filters */
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        speed_buffer[i] = 0.0f;
    }
    speed_filter_index = 0;
    speed_filter_count = 0;

    for (int i = 0; i < RPM_FILTER_SIZE; i++) {
        rpm_buffer[i] = 0.0f;
    }
    rpm_filter_index = 0;
    rpm_filter_count = 0;
    
    //printf("Sensor_Init complete\n");
   // printf("  S1 on P0, S2 on P1\n");
   // printf("  Initial state: S1=%d, S2=%d (combined=%d)\n", s1, s2, last_state);
   // printf("  Using per-pin interrupts: INT_GPIOP0=%d, INT_GPIOP1=%d\n", 
        //   INT_GPIOP0, INT_GPIOP1);
   // printf("  Timer frequency: 120 MHz\n");
   // printf("  Edges per rotation: %.1f\n", EDGES_PER_ROTATION);
   // printf("  Wheel circumference: %.4f m\n", WHEEL_CIRCUMFERENCE);
}

/* ============== Get Speed (call from main loop) ============== */
float Sensor_GetSpeed(void)
{
    static uint32_t call_count = 0;
    static uint32_t last_int_count = 0;
    uint32_t int_copy;
    uint32_t edge_copy;
    int32_t dir_copy;
    uint32_t current_time;
    
    /* Get current timer value for time-based calculation */
    current_time = TimerValueGet(EDGE_TIMER_BASE, TIMER_A);
    
    /* Atomic read of volatile variables */
    IntMasterDisable();
    int_copy = interrupt_count;
    edge_copy = edge_count;
    dir_copy = direction_counter;
    IntMasterEnable();
    
    /* Debug output every ~500 calls */
    call_count++;
    if (call_count >= 500) {
        uint8_t s1 = GPIOPinRead(S1_PORT, S1_PIN) ? 1 : 0;
        uint8_t s2 = GPIOPinRead(S2_PORT, S2_PIN) ? 1 : 0;
       // printf("[DBG] Int:%lu Edges:%lu DirCnt:%ld S1=%d S2=%d\n",
       //        (unsigned long)int_copy, 
       //       (unsigned long)edge_copy,
       //        (long)dir_copy,
       //       s1, s2);
        last_int_count = int_copy;
        call_count = 0;
    }
    
    /*
     * NEW APPROACH: Calculate speed from edge count over time interval
     * This is more reliable than single-edge period measurement
     * 
     * Speed = (edges_delta / EDGES_PER_ROTATION) * WHEEL_CIRCUMFERENCE / time_delta
     */
    
    /* Calculate edges since last display update */
    uint32_t edges_delta = edge_copy - last_display_edge_count;
    
    /* Calculate time since last display (timer counts DOWN) */
    uint32_t time_delta;
    if (last_display_time >= current_time) {
        time_delta = last_display_time - current_time;
    } else {
        time_delta = last_display_time + (0xFFFFFFFFUL - current_time) + 1;
    }
    
    /* Only update if enough time has passed for accurate measurement (at least 100ms) */
    if (time_delta >= MIN_UPDATE_INTERVAL && edges_delta > 0) {
        /* Calculate time in seconds */
        float time_seconds = (float)time_delta / TIMER_FREQ;

        /* Calculate rotations */
        float rotations = (float)edges_delta / EDGES_PER_ROTATION;

        /* Calculate RPM: (rotations / time) * 60 */
        float rpm = (rotations / time_seconds) * 60.0f;

        /* Debug: Print calculation details occasionally */
        static uint32_t calc_count = 0;
        calc_count++;
        /*
        if (calc_count % 5 == 0) {
            printf("[CALC] Edges=%lu TimeMs=%.1f RPM=%.1f\n",
                   (unsigned long)edges_delta,
                   time_seconds * 1000.0f,
                   rpm);
        }
        */
        /* Calculate speed: rotations * circumference / time = m/s */
        float velocity_mps = (rotations * WHEEL_CIRCUMFERENCE) / time_seconds;

        /* Convert to km/h */
        float velocity_kmh = velocity_mps * 3.6f;
        
        /* Sanity check for speed */
        if (velocity_kmh > 200.0f) {
            velocity_kmh = current_speed_kmh; /* Keep previous value */
        } else if (velocity_kmh < 0.0f) {
            velocity_kmh = 0.0f;
        }

        /* Sanity check for RPM - allow values above 20k up to display max */
        if (rpm > 99999.0f) {
            rpm = 99999.0f; /* Clamp to display maximum (5 digits) */
        } else if (rpm < 0.0f) {
            rpm = 0.0f;
        }

        /* Apply moving average filter to speed */
        speed_buffer[speed_filter_index] = velocity_kmh;
        speed_filter_index = (speed_filter_index + 1) % SPEED_FILTER_SIZE;
        if (speed_filter_count < SPEED_FILTER_SIZE) {
            speed_filter_count++;
        }

        /* Calculate average speed */
        float speed_sum = 0.0f;
        for (int i = 0; i < speed_filter_count; i++) {
            speed_sum += speed_buffer[i];
        }
        current_speed_kmh = speed_sum / (float)speed_filter_count;

        /* Apply moving average filter to RPM */
        rpm_buffer[rpm_filter_index] = rpm;
        rpm_filter_index = (rpm_filter_index + 1) % RPM_FILTER_SIZE;
        if (rpm_filter_count < RPM_FILTER_SIZE) {
            rpm_filter_count++;
        }

        /* Calculate average RPM - this is the smoothed value shown in display */
        float rpm_sum = 0.0f;
        for (int i = 0; i < rpm_filter_count; i++) {
            rpm_sum += rpm_buffer[i];
        }
        current_rpm = rpm_sum / (float)rpm_filter_count;
        
        /* Update direction with hysteresis */
        if (dir_copy > DIRECTION_THRESHOLD) {
            current_direction = DIR_FORWARD;
        } else if (dir_copy < -DIRECTION_THRESHOLD) {
            current_direction = DIR_REVERSE;
        }
        /* Otherwise keep current direction (hysteresis) */
        
        /* Update distance */
        accumulated_distance += rotations * WHEEL_CIRCUMFERENCE;
        
        /* Reset counters for next interval */
        last_display_edge_count = edge_copy;
        last_display_time = current_time;
        
    } else if (time_delta > STOPPED_TIMEOUT) {
        /* No edges for too long - motor stopped */
        current_speed_kmh = 0.0f;
        current_rpm = 0.0f;
        current_direction = DIR_STOPPED;

        /* Clear speed and RPM filters */
        for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
            speed_buffer[i] = 0.0f;
        }
        speed_filter_count = 0;

        for (int i = 0; i < RPM_FILTER_SIZE; i++) {
            rpm_buffer[i] = 0.0f;
        }
        rpm_filter_count = 0;

        /* Reset reference point */
        last_display_edge_count = edge_copy;
        last_display_time = current_time;
    }
    
    return current_speed_kmh;
}

/* ============== Get Direction ============== */
RotationDirection Sensor_GetDirection(void)
{
    return current_direction;
}

/* ============== Get Distance ============== */
float Sensor_GetDistance(void)
{
    return accumulated_distance;
}

/* ============== Reset Distance ============== */
void Sensor_ResetDistance(void)
{
    accumulated_distance = 0.0f;
}

/* ============== Debug: Get Raw Interrupt Count ============== */
uint32_t Sensor_GetInterruptCount(void)
{
    return interrupt_count;
}

/* ============== Debug: Get Edge Count ============== */
uint32_t Sensor_GetEdgeCount(void)
{
    return edge_count;
}

/* ============== Get RPM ============== */
float Sensor_GetRPM(void)
{
    return current_rpm;
}
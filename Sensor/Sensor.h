/**
 * Sensor.h - KMZ60 Quadrature Encoder Interface
 * 
 * For TM4C1294NCPDT with KMZ60 magnetic sensor
 * S1 on Port P0, S2 on Port P1
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

/* Direction enumeration */
typedef enum {
    DIR_STOPPED = 0,
    DIR_FORWARD = 1,
    DIR_REVERSE = -1
} RotationDirection;

/**
 * Initialize the KMZ60 sensor using S1/S2 comparator outputs
 * Sets up GPIO, Timer, and per-pin interrupts for Port P
 */
void Sensor_Init(void);

/**
 * Get current speed in km/h
 * IMPORTANT: Call this FIRST - it calculates both speed and RPM internally
 * Call this regularly from main loop to update calculations
 * Returns 0 if motor is stopped
 */
float Sensor_GetSpeed(void);

/**
 * Get current speed in RPM
 * IMPORTANT: Call Sensor_GetSpeed() FIRST before calling this
 * Returns the RPM value calculated by the last Sensor_GetSpeed() call
 * Returns 0 if motor is stopped
 */
float Sensor_GetRPM(void);

/**
 * Get accumulated distance in meters
 */
float Sensor_GetDistance(void);

/**
 * Reset accumulated distance to zero
 */
void Sensor_ResetDistance(void);

/**
 * Get current rotation direction
 * Returns DIR_FORWARD, DIR_REVERSE, or DIR_STOPPED
 */
RotationDirection Sensor_GetDirection(void);

/**
 * Debug: Get total interrupt count
 */
uint32_t Sensor_GetInterruptCount(void);

/**
 * Debug: Get total edge count
 */
uint32_t Sensor_GetEdgeCount(void);

#endif /* SENSOR_H */
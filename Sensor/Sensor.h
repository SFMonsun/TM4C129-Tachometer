#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

// Initialize the KMZ60 sensor (setup ADC pins)
void Sensor_Init(void);

// Read speed from KMZ60 sensor (returns speed in km/h)
float Sensor_ReadSpeed(void);

// Reset accumulated distance
void Sensor_ResetDistance(void);

// Get accumulated distance in meters
float Sensor_GetDistance(void);

#endif // SENSOR_H
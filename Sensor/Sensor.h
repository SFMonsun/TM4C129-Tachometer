#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

// Initialize the KMZ60 sensor (setup ADC pins)
void Sensor_Init(void);

// Read speed from KMZ60 sensor
float Sensor_ReadSpeed(void);

#endif // SENSOR_H

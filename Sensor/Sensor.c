#include "Sensor.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

// ADC pin mapping for TM4C1294
#define ADC_BASE        ADC0_BASE
#define ADC_SEQUENCE    0  // Sequence 0 supports multiple steps

#define ADC_VCOS_CH     ADC_CTL_CH1   // PE2, AIN1
#define ADC_VSIN_CH     ADC_CTL_CH0   // PE3, AIN0
#define ADC_VREF_CH     ADC_CTL_CH2   // PE4, AIN2

#define ADC_MAX_VALUE   4095.0f
#define VREF            3.3f

// Wheel parameters
#define WHEEL_RADIUS_M  0.03f   // Example: 3 cm radius, adjust to your wheel
#define DT              0.01f   // Sampling interval in seconds, adjust to your loop

// Static variable to store previous rotor angle for speed calculation
static float theta_prev = 0.0f;

void Sensor_Init(void)
{
    // Enable ADC0 and GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){}

    // Configure pins PE2, PE3, PE4 as ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

    // Configure sequence 0 with 3 steps: Vcos, Vsin, Vref
    ADCSequenceConfigure(ADC_BASE, ADC_SEQUENCE, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCE, 0, ADC_VCOS_CH);
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCE, 1, ADC_VSIN_CH);
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCE, 2, ADC_VREF_CH | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC_BASE, ADC_SEQUENCE);
    ADCIntClear(ADC_BASE, ADC_SEQUENCE);
}

// Separate function to calculate speed in km/h from ADC voltages
float calculateSpeedKMH(float vcos, float vsin, float vref)
{
    // 1. Compute rotor angle
    float theta = atan2(vsin - vref/2.0f, vcos - vref/2.0f);

    // 2. Compute delta angle and handle wrapping
    float delta_theta = theta - theta_prev;
    if(delta_theta > M_PI) delta_theta -= 2.0f * M_PI;
    if(delta_theta < -M_PI) delta_theta += 2.0f * M_PI;

    theta_prev = theta;

    // 3. Angular speed [rad/s]
    float omega = delta_theta / DT;

    // 4. Linear speed [m/s]
    float v_mps = omega * WHEEL_RADIUS_M;

    // 5. Convert to km/h
    float v_kmh = v_mps * 3.6f;

    return v_kmh;
}

float Sensor_ReadSpeed(void)
{
    uint32_t adcValues[3];
    float vcos, vsin, vref;
    float speed_kmh;

    // Trigger ADC conversion
    ADCProcessorTrigger(ADC_BASE, ADC_SEQUENCE);

    // Wait for conversion to complete
    while(!ADCIntStatus(ADC_BASE, ADC_SEQUENCE, false)){}

    // Read all three ADC values
    ADCSequenceDataGet(ADC_BASE, ADC_SEQUENCE, adcValues);
    ADCIntClear(ADC_BASE, ADC_SEQUENCE);

    // Convert ADC to voltage
    vcos = ((float)adcValues[0] / ADC_MAX_VALUE) * VREF;
    vsin = ((float)adcValues[1] / ADC_MAX_VALUE) * VREF;
    vref = ((float)adcValues[2] / ADC_MAX_VALUE) * VREF;

    // Debug: print voltages
    printf("Vcos: %.3f V, Vsin: %.3f V, Vref: %.3f V\n", vcos, vsin, vref);

    // Calculate speed in km/h
    speed_kmh = calculateSpeedKMH(vcos, vsin, vref);
    printf("Speed: %.3f km/h", speed_kmh);
    return speed_kmh;
}

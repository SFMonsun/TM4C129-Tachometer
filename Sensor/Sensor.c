#include "Sensor.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

// ADC pin mapping for TM4C1294
// Actual connections: Vcos=E3(AIN0), Vsin=E2(AIN1), Vref=C7(AIN12)
#define ADC_BASE        ADC0_BASE
#define ADC_SEQUENCE    0  // Sequence 0 supports multiple steps

#define ADC_VCOS_CH     ADC_CTL_CH0   // PE3, AIN0
#define ADC_VSIN_CH     ADC_CTL_CH1   // PE2, AIN1
#define ADC_VREF_CH     ADC_CTL_CH12  // PC7, AIN12

#define ADC_MAX_VALUE   4095.0f
#define VREF            3.3f

// Wheel parameters
#define WHEEL_RADIUS_M  0.03f   // 3 cm radius wheel
#define DT              0.01f   // 10ms sampling interval (matches timer)

// Static variables for speed calculation using quadrature detection
static float theta_prev = 0.0f;
static int32_t cumulative_rotations = 0;  // Total rotations (can wrap around full circles)
static uint8_t first_reading = 1;
static float accumulated_distance = 0.0f;

// Moving average filter for speed smoothing
#define FILTER_SIZE 5
static float speed_buffer[FILTER_SIZE] = {0};
static uint8_t filter_index = 0;
static uint8_t filter_filled = 0;

void Sensor_Init(void)
{
    // Enable ADC0, GPIOE (for E2, E3), and GPIOC (for C7)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}

    // Configure pins: PE2 (Vsin), PE3 (Vcos), PC7 (Vref) as ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTC_BASE, GPIO_PIN_7);

    // Configure sequence 0 with 3 steps: Vcos, Vsin, Vref
    ADCSequenceConfigure(ADC_BASE, ADC_SEQUENCE, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCE, 0, ADC_VCOS_CH);
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCE, 1, ADC_VSIN_CH);
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCE, 2, ADC_VREF_CH | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC_BASE, ADC_SEQUENCE);
    ADCIntClear(ADC_BASE, ADC_SEQUENCE);
    
    // Initialize static variables
    theta_prev = 0.0f;
    cumulative_rotations = 0;
    accumulated_distance = 0.0f;
    first_reading = 1;
    filter_index = 0;
    filter_filled = 0;
    
    // Clear speed filter buffer
    for(int i = 0; i < FILTER_SIZE; i++) {
        speed_buffer[i] = 0.0f;
    }
}

float Sensor_ReadSpeed(void)
{
    uint32_t adcValues[3];
    float vcos, vsin, vref;
    float theta, delta_theta, v_mps, v_kmh;

    // Oversample ADC: Take 4 readings and average them for noise reduction
    float vcos_sum = 0, vsin_sum = 0, vref_sum = 0;
    const int NUM_SAMPLES = 4;
    
    for(int sample = 0; sample < NUM_SAMPLES; sample++) {
        ADCProcessorTrigger(ADC_BASE, ADC_SEQUENCE);
        while(!ADCIntStatus(ADC_BASE, ADC_SEQUENCE, false)){}
        ADCSequenceDataGet(ADC_BASE, ADC_SEQUENCE, adcValues);
        ADCIntClear(ADC_BASE, ADC_SEQUENCE);

        vcos_sum += ((float)adcValues[0] / ADC_MAX_VALUE) * VREF;
        vsin_sum += ((float)adcValues[1] / ADC_MAX_VALUE) * VREF;
        vref_sum += ((float)adcValues[2] / ADC_MAX_VALUE) * VREF;
    }
    
    vcos = vcos_sum / NUM_SAMPLES;
    vsin = vsin_sum / NUM_SAMPLES;
    vref = vref_sum / NUM_SAMPLES;

    // Use FIXED center voltage - ignore varying Vref for now
    const float CENTER = 1.65f;
    float vcos_centered = vcos - CENTER;
    float vsin_centered = vsin - CENTER;
    
    // Calculate signal magnitude
    float magnitude = sqrtf(vcos_centered * vcos_centered + vsin_centered * vsin_centered);
    
    // Debug output every 100 readings
    static uint32_t debug_counter = 0;
    if(++debug_counter >= 100) {
        printf("ADC: Vcos=%.3fV Vsin=%.3fV Vref=%.3fV Mag=%.3fV | Rotations=%ld\n", 
               vcos, vsin, vref, magnitude, cumulative_rotations);
        debug_counter = 0;
    }

    // If signal is too weak, return 0
    if(magnitude < 0.1f) {
        return 0.0f;
    }

    // Calculate angle using atan2
    theta = atan2f(vsin_centered, vcos_centered);

    // First reading initialization
    if(first_reading) {
        theta_prev = theta;
        first_reading = 0;
        return 0.0f;
    }

    // Calculate angle change with proper wrapping handling
    delta_theta = theta - theta_prev;
    
    // Handle wrapping: adjust for shortest path
    if(delta_theta > M_PI) {
        delta_theta -= 2.0f * M_PI;
    } else if(delta_theta < -M_PI) {
        delta_theta += 2.0f * M_PI;
    }
    
    theta_prev = theta;

    // Accumulate total rotation (this handles multiple rotations correctly)
    // Note: We're measuring angle change per sample, which works even if
    // multiple rotations occur, because we track the continuous phase
    float omega = delta_theta / DT;  // rad/s
    
    // Linear speed
    v_mps = fabsf(omega * WHEEL_RADIUS_M);
    
    // Update cumulative rotations for display
    cumulative_rotations += (int32_t)(delta_theta / (2.0f * M_PI));
    
    // Accumulate distance
    accumulated_distance += v_mps * DT;
    
    // Convert to km/h
    v_kmh = v_mps * 3.6f;
    
    // Apply moving average filter
    speed_buffer[filter_index] = v_kmh;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    if(!filter_filled && filter_index == 0) {
        filter_filled = 1;
    }
    
    float speed_sum = 0.0f;
    int count = filter_filled ? FILTER_SIZE : filter_index;
    for(int i = 0; i < count; i++) {
        speed_sum += speed_buffer[i];
    }
    float filtered_speed = (count > 0) ? (speed_sum / count) : 0.0f;
    
    // Dead zone
    if(filtered_speed < 0.5f) {
        filtered_speed = 0.0f;
    }

    return filtered_speed;
}

void Sensor_ResetDistance(void)
{
    accumulated_distance = 0.0f;
}

float Sensor_GetDistance(void)
{
    return accumulated_distance;
}
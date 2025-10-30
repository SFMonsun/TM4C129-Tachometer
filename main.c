#include <stdint.h>
#include <stdio.h>
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

volatile float latestSpeed_kmh = 0.0f; // updated in ISR

void SensorTimer_Init(uint32_t sysClock)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    TimerConfigure(SENSOR_TIMER_BASE, TIMER_CFG_PERIODIC);
    uint32_t loadValue = (sysClock / 100) - 1; // 10ms
    TimerLoadSet(SENSOR_TIMER_BASE, TIMER_A, loadValue);

    TimerIntEnable(SENSOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(SENSOR_TIMER_INT);

    TimerEnable(SENSOR_TIMER_BASE, TIMER_A);
}

void Timer0A_Handler(void)
{
    TimerIntClear(SENSOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    latestSpeed_kmh = Sensor_ReadSpeed(); // read sensor at fixed interval
}

int main(void)
{
    int i,j,x,y;
    enum colors color;

    sysClock = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    init_ports_display();
    configure_display_controller_large();

    Sensor_Init();               // <-- must be initialized first
    SensorTimer_Init(sysClock);  // <-- start ISR after sensor ready
    IntMasterEnable();

    printf("Start Background Pixel by Pixel set\n");
    color = YELLOW;
    window_set(0,0,MAX_X-1,MAX_Y-1);
    write_command(0x2C);
    for (x=0;x<=MAX_X-1;x++)
        for (y=0;y<=MAX_Y-1;y++)
            write_data((color>>16)&0xff),
            write_data((color>>8)&0xff),
            write_data((color)&0xff);

    printf("Background ready \n");
    j = 0;

    while(1)
    {
        // Use speed updated in ISR
        printf("Speed: %.2f km/h\n", latestSpeed_kmh);

        // Display rectangles (slow, does not affect sensor sampling)
        printf("Write rectangles\n");
        for (x=0;x<=MAX_X-1-20;x+=40)
            for (y=0;y<=MAX_Y-1-20;y+=40)
            {
                color = colorarray[(j++)%7];
                window_set(x,y,x+20,y+20);
                write_command(0x2C);
                for(i=0;i<(40*40);i++)
                    write_data((color>>16)&0xff),
                    write_data((color>>8)&0xff),
                    write_data((color)&0xff);
            }
    }
}

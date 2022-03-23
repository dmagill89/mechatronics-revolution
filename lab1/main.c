/******************************************************************************
 * The Mechatronics Revolution: Fundamentals and Core Concepts
 * Lab Assignment 1: MSP432 LaunchPad Example Project
 *
 * Description: An example project that uses DriverLib to toggle LEDs
 *
 * Revision: 1.0
 * Date: March 2020
*******************************************************************************/

/* DriverLib Include */
#include "driverlib.h"
/* mechrev Include */
#include "mechrev.h"

#define LEDS    GPIO_PIN2|GPIO_PIN1

const Timer_A_UpModeConfig upConfig_0 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_64,
    46875,
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
    TIMER_A_DO_CLEAR
};

int main(void)
{
    WDT_A_holdTimer();

    mechrev_setup();

    unsigned int dcoFrequency = 3E+6;
    MAP_CS_setDCOFrequency(dcoFrequency);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2,LEDS);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, LEDS);

    MAP_Timer_A_configureUpMode(TIMER_A0_BASE,&upConfig_0);

    MAP_Interrupt_enableInterrupt(INT_TA0_0);

    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    MAP_Interrupt_enableMaster();

    int i = 10;

    while(1)
    {
        i += 10;
        if(i > 65000)
        {
            i = 10;
        }
    }
}


void TA0_0_IRQHandler(void)
{
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, LEDS);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

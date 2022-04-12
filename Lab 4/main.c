/******************************************************************************
 * The Mechatronics Revolution: Fundamentals and Core Concepts
 * Code Template for Lab Assignment 4
 *
 * Note: Create a new project as described in Lab Assignment 1.
 * After including the DriverLib library in your project settings,
 * you can use this template to design your program for Lab Assignment 4.
 *
 * This template follows the steps described in the Software Architecture
 * section (Section 4.6) of the Lab Assignment 4 document.
*******************************************************************************/

/* Include header files */
#include "driverlib.h"
#include "mechrev.h"

/* Define macros and function prototypes if needed */

/* Define configuration structs if needed */
Timer_A_PWMConfig pwmConfigTA01 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_1,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

Timer_A_PWMConfig pwmConfigTA02 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_2,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

Timer_A_PWMConfig pwmConfigTA03 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_3,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

Timer_A_PWMConfig pwmConfigTA04 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_4,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};


/* Declare global variables if needed */


/* Main program */
void main(void)
{
    /* Stop Watchdog Timer */
    WDT_A_holdTimer();

    /* Call the mechrev_setup function included in the mechrev.h header file */
    mechrev_setup();

    /* Initialize GPIOs P1.1 and P1.4 for PushButtons (S1 and S2 switches) */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);

    /* Initialize GPIOs P1.6 and P1.7 for Motor Driver IC Enable Pins */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);

    /* Initialize GPIOs P2.4, P2.5, P2.6 and P2.7 for PWM functionality  */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Initialize Timer A0 to generate PWM signals */
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigTA01);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigTA02);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigTA03);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigTA04);

    /* Declare local variables if needed */

    /* Call the initialization grading macro */
    MACRO_LAB4_INIT();

    while(1)
    {
        /* Design a Polling process to detect PushButtons press and adjust the PWM duty cycles accordingly */

        uint8_t sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
        uint8_t sw2 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);

        if (sw1 == GPIO_INPUT_PIN_LOW) {

            TA0CCR2 = 0;
            TA0CCR4 = 0;

            if (TA0CCR1 == 3000 && TA0CCR3 == 3000) {
                TA0CCR1 = 1000;
                TA0CCR3 = 1000;
            } else {
                TA0CCR1 += 1000;
                TA0CCR3 += 1000;
            }

            /* Note: Call the event grading macro after changing PWMs */
            MACRO_LAB4_EVENT();
        }

        if (sw2 == GPIO_INPUT_PIN_LOW) {
            TA0CCR1 = 0;
            TA0CCR3 = 0;

            if (TA0CCR2 == 3000 && TA0CCR4 == 3000) {
                TA0CCR2 = 1000;
                TA0CCR4 = 1000;
            } else {
                TA0CCR2 += 1000;
                TA0CCR4 += 1000;
            }
            /* Note: Call the event grading macro after changing PWMs */
            MACRO_LAB4_EVENT();
        }
    }
}


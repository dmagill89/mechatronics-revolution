/******************************************************************************
 * The Mechatronics Revolution: Fundamentals and Core Concepts
 * Code Template for Lab Assignment 3
 *
 * Note: Create a new project as described in Lab Assignment 1.
 * After including the DriverLib library in your project settings,
 * you can use this template to design your program for Lab Assignment 3. 
 *
 * This template follows the steps described in the Software Architecture
 * section (Section 3.6) of the Lab Assignment 3 document.
*******************************************************************************/

/* Include header files */
#include "driverlib.h"
#include "mechrev.h"

/* Define macros and function prototypes if needed */
#define LED2        GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2
#define BUMPERS     GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7
#define SWITCHES    GPIO_PIN1 | GPIO_PIN4
/* Declare global and volatile variables if needed */


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
    /* Initialize GPIOs P1.0, P2.0, P2.1 and P2.2 for LED1 and LED2 */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, LED2);
    /* Initialize GPIOs P4.0, P4.2, P4.3, P4.5, P4.6 and P4.7 for Bump Sensors */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, BUMPERS);
    /* Enable interrupts for Bump Sensors' GPIOs */
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, BUMPERS);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, BUMPERS, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT4);
    /* Declare local variables if needed */
    uint8_t switch1_status;
    uint8_t switch2_status;
    /* Call the initialization grading macro */
    MACRO_LAB3_INIT();

    while(1)
    {
        switch1_status = MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
        switch2_status = MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);
        /* Design a Polling process to detect PushButtons press and turn on or off LED1 accordingly */
        if (switch1_status == 0 || switch2_status == 0) {
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            /* Note: Call the event grading macro after turning on LED1 */
            MACRO_LAB3_EVENT();
        }

    }
}

/* Interrupt Service Routine for PORT4 to handle Bump Sensors */
void PORT4_IRQHandler(void)
{
    /* Check the interrupt status */
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    /* Change the color of LED2 according to the interrupt status */

    // LED2 to Red
    if (status & GPIO_PIN0 || status & GPIO_PIN7) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
    }

    // LED2 to Green
    if (status & GPIO_PIN2 || status & GPIO_PIN6) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
    }

    // LED2 to Blue
    if (status & GPIO_PIN3 || status & GPIO_PIN5) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
    }

    /* Note: Call the event grading macro after changing the color of LED2 */
    MACRO_LAB3_EVENT();

    /* Clear the PORT4 interrupt flag */
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
}

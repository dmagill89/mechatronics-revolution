/******************************************************************************
 * The Mechatronics Revolution: Fundamentals and Core Concepts
 * Code Template for Lab Assignment 5
 *
 * Note: Create a new project as described in Lab Assignment 1.
 * After including the DriverLib library in your project settings,
 * you can use this template to design your program for Lab Assignment 5.
 *
 * This template follows the steps described in the Software Architecture
 * section (Section 5.6) of the Lab Assignment 5 document.
*******************************************************************************/

/* Include header files */
#include <math.h>
#include <stdio.h>
#include "driverlib.h"
#include "mechrev.h"

/* Define macros and function prototypes if needed */

/* Define configuration structs if needed */
const eUSCI_UART_Config UART_init = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,
    3,
    4,
    2,
    EUSCI_A_UART_NO_PARITY,
    EUSCI_A_UART_LSB_FIRST,
    EUSCI_A_UART_ONE_STOP_BIT,
    EUSCI_A_UART_MODE,
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

const Timer_A_UpModeConfig upConfig_1 = {
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_32,
    37500,
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
    TIMER_A_DO_CLEAR
};
/* Declare global and volatile variables if needed */
volatile float distance_var;


/* Main program */
void main(void)
{
    /* Stop Watchdog Timer */
    WDT_A_holdTimer();

    /* Call the mechrev_setup function included in the mechrev.h header file */
    mechrev_setup();

    /* Initialize GPIOs P1.2 and P1.3 for UART receive and transmit functionality */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Initialize and enable UART EUSCI_A0 module */
    MAP_UART_initModule(EUSCI_A0_BASE, &UART_init);
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Initialize UART RX Interrupt */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);

    /* Initialize GPIOs P6.1 for ADC functionality */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initialize ADC14 module */
    ADC14_enableModule();
    ADC14_setResolution(ADC_10BIT);
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_32, ADC_DIVIDER_8, 0);
    ADC14_configureSingleSampleMode(ADC_MEM0, ADC_MANUAL_ITERATION);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, 0);
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    ADC14_enableConversion();

    /* Initialize Timer A1 to generate periodic interrupts for for ADC readings */
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig_1);
    Interrupt_enableInterrupt(INT_TA1_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    MAP_Interrupt_enableMaster();

    /* Declare local variables if needed */

    /* Call the initialization grading macro */
    MACRO_LAB5_INIT();

    while(1)
    {
        // Empty while loop
    }
}


/* Interrupt Service Routine for Timer A1 */
void TA1_0_IRQHandler(void)
{
    /* Toggle an ADC conversion, wait for the results and read the ADC module */
    ADC14_toggleConversionTrigger();
    while( ADC14_isBusy());

    /* Convert the ADC result to the actual voltage, */
    /* and then convert the voltage to the actual distance value in cm. */
    /* Note: Store the result in the "distance_var" variable */
    uint16_t adc_result = (uint16_t) ADC14_getResult(ADC_MEM0);
    float voltage = adc_result * (3.3/1023.0);
    distance_var = 27.726 * pow(voltage, -1.2045);

    /* Call the ADC grading macro with the distance variable */
    MACRO_LAB5_ADC_EVENT(distance_var);

    /* Clear the Timer A1 Capture/Compare interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


/* Interrupt Service Routine for UART */
void EUSCIA0_IRQHandler(void)
{
    /* Read the received data from UART RX */
    uint8_t rxData = UART_receiveData(EUSCI_A0_BASE);
    /* Call the UART RX grading macro with the distance variable */
    MACRO_LAB5_UART_RX_EVENT(distance_var);

    /* Design a code so that when the byte recevied through UART RX is a
     * carriage return (HEX code 0x0D), convert the distance variable to
     * ASCII characters using "sprintf" function and
     * transmit those characters through UART TX.
     */
    if (rxData == 0x0D) {
        char tx_message[10];
        int numChars = sprintf(tx_message, "%.2f", distance_var);

        int i;
        for (i = 0; i < numChars; i++) {

            while((UCA0IFG & 0x02) == 0) {} // wait until transmit interrupt flag is high
            UART_transmitData(EUSCI_A0_BASE, tx_message[i]);

            /* Note: Call the UART TX grading macro after transmitting each character */
            MACRO_LAB5_UART_TX_EVENT();
        }
    }

    /* Append a carriage return character (0x0D) and a newline character (0x0A)
     * at the end and transmit those characters through UART TX.
     */
    while((UCA0IFG & 0x02) == 0) {}
    UART_transmitData(EUSCI_A0_BASE, 0x0D);
    while((UCA0IFG & 0x02) == 0) {}
    UART_transmitData(EUSCI_A0_BASE, 0x0A);

    /* Clear the UART RX interrupt flag */
    EUSCI_A_UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

}

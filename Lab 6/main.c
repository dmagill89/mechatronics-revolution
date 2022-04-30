/******************************************************************************
 * The Mechatronics Revolution: Fundamentals and Core Concepts
 * Code Template for Lab Assignment 6
 *
 * Note: Create a new project as described in Lab Assignment 1.
 * After including the DriverLib library in your project settings,
 * you can use this template to design your program for Lab Assignment 6.
 *
 * This template follows the steps described in the Software Architecture
 * section (Section 6.7) of the Lab Assignment 6 document.
*******************************************************************************/

/* Include header files */
#include <math.h>
#include "driverlib.h"
#include "mechrev.h"

/* Define macros and function prototypes if needed */
#define BUMPERS     GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7

/* Define configuration structs if needed */
const Timer_A_PWMConfig pwmConfigTA01 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_1,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

const Timer_A_PWMConfig pwmConfigTA02 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_2,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

const Timer_A_PWMConfig pwmConfigTA03 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_3,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

const Timer_A_PWMConfig pwmConfigTA04 =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  10000,
  TIMER_A_CAPTURECOMPARE_REGISTER_4,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

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

typedef enum {
    FWD,
    BKWD,
    RIGHT,
    LEFT,
    STOP
} drive_t;

drive_t drive_mode = STOP;


/* Main program */
void main(void)
{
    /* Stop Watchdog Timer */
    WDT_A_holdTimer();

    /* Call the mechrev_setup function included in the mechrev.h header file */
    mechrev_setup();

    /* Initialize GPIOs P1.6 and P1.7 for Motor Driver IC Enable Pins */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);

    /* Initialize GPIOs P4.0, P4.2, P4.3, P4.5, P4.6 and P4.7 for Bump Sensors */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, BUMPERS);

    /* Enable interrupts for Bump Sensors' GPIOs */
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, BUMPERS);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, BUMPERS, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT4);

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
    MACRO_LAB6_INIT();

    while(1)
    {
        /* Adjust the PWM duty cycles according to the drive mode of the robot (FWD, BKWD, RIGHT, LEFT, STOP) */
        switch (drive_mode) {
            case FWD:
                TA0CCR1 = 3000;
                TA0CCR2 = 0;
                TA0CCR3 = 3000;
                TA0CCR4 = 0;
                break;
            case RIGHT:
                TA0CCR1 = 3000;
                TA0CCR2 = 0;
                TA0CCR3 = 1500;
                TA0CCR4 = 0;
                break;
            case LEFT:
                TA0CCR1 = 1500;
                TA0CCR2 = 0;
                TA0CCR3 = 3000;
                TA0CCR4 = 0;
                break;
            case BKWD:
                TA0CCR1 = 0;
                TA0CCR2 = 3000;
                TA0CCR3 = 0;
                TA0CCR4 = 3000;
                break;
            case STOP:
                TA0CCR1 = 0;
                TA0CCR2 = 0;
                TA0CCR3 = 0;
                TA0CCR4 = 0;
                break;
        }
    }
}


/* Interrupt Service Routine for PORT4 to handle Bump Sensors */
void PORT4_IRQHandler(void)
{
    /* Check the interrupt status */
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);

    /* Change the drive mode of the robot to STOP according to the interrupt status */
    if (status & GPIO_PIN0 || status & GPIO_PIN7 || status & GPIO_PIN2 || status & GPIO_PIN6 || status & GPIO_PIN3 || status & GPIO_PIN5) {
        TA0CCR1 = 0;
        TA0CCR2 = 0;
        TA0CCR3 = 0;
        TA0CCR4 = 0;
    }

    /* Call the Switch grading macro */
    MACRO_LAB6_SWITCH_EVENT();

    /* Clear the PORT4 interrupt flag */
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
}


/* Interrupt Service Routine for Timer A1 */
void TA1_0_IRQHandler(void)
{
    /* Toggle an ADC conversion, wait for the results and read the ADC module */

    ADC14_toggleConversionTrigger();
    while( ADC14_isBusy());

    /* Convert the ADC result to the actual voltage, */
    /* and then convert the voltage to the actural distance value in cm. */
    /* Note: Store the result in the "distance_var" variable */

    uint16_t adc_result = (uint16_t) ADC14_getResult(ADC_MEM0);
    float voltage = adc_result * (3.3/1023.0);
    distance_var = 27.726 * pow(voltage, -1.2045);

    /* Call the ADC grading macro with the distance variable */
    MACRO_LAB6_ADC_EVENT(distance_var);

    /* If the distance is smaller that 20 cm, change the drive mode of the robot to STOP */
    if (distance_var < 20) {
        TA0CCR1 = 0;
        TA0CCR2 = 0;
        TA0CCR3 = 0;
        TA0CCR4 = 0;
    }

    /* Clear the Timer A1 Capture/Compare interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


/* Interrupt Service Routine for UART */
void EUSCIA0_IRQHandler(void)
{
    /* Read the received data from UART RX */
    uint8_t rx_data = UART_receiveData(EUSCI_A0_BASE);

    /* Call the UART RX grading macro with the distance variable */
    MACRO_LAB6_UART_RX_EVENT(distance_var);

    /* Change the drive mode of the robot based on the received data (FWD, BKWD, RIGHT, LEFT, STOP) */
    switch (rx_data) {
        case 'w':
            drive_mode = FWD;
            break;
        case 'a':
            drive_mode = LEFT;
            break;
        case 's':
            drive_mode = STOP;
            break;
        case 'd':
            drive_mode = RIGHT;
            break;
        case 'z':
            drive_mode = BKWD;
            break;
    }

    /* If the byte recevied through UART RX is a carriage return (HEX code 0x0D),
     * convert the distance variable to ASCII characters using "sprintf"
     * function and transmit those characters through UART TX.
     */
     if (rx_data == 0x0D) {
         char tx_message[10];
         int numChars = sprintf(tx_message, "%.2f", distance_var);

         int i;
         for (i = 0; i < numChars; i++) {

            while((UCA0IFG & 0x02) == 0) {} // wait until transmit interrupt flag is high
            UART_transmitData(EUSCI_A0_BASE, tx_message[i]);

            /* Note: Call the UART TX grading macro after transmitting each character */
            MACRO_LAB6_UART_TX_EVENT();
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

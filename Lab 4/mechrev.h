/*
 *  mechrev.h
 *  The Mechatronics Revolution: Fundamentals and Core Concepts
 *  Header file for Lab 1,3,4,5,6
 *  Revision: 1.1
 *  Date: March 2020
 *
 *  DO NOT MODIFY THIS FILE.
 */

#ifndef MECHREV_H_
#define MECHREV_H_

#include <stdio.h>

volatile int32_t encA_cnt = 0;
volatile int32_t encB_cnt = 0;
volatile bool encA_dir = 0;
volatile bool encB_dir = 0;


#define MACRO_LAB3_INIT()               printf("\n0xff"); printf(" 0x%02x",P1DIR); printf(" 0x%02x",P2DIR); printf(" 0x%02x",P4DIR); printf(" 0x%02x",P1REN); printf(" 0x%02x",P4REN); printf(" 0x%02x\n",P4IE);
#define MACRO_LAB3_EVENT()              printf("0xf1"); printf(" 0x%02x",P1IN); printf(" 0x%02x",P4IFG); printf(" 0x%02x",P1OUT); printf(" 0x%02x\n",P2OUT);

#define MACRO_LAB4_INIT()               printf("\n0xff"); printf(" 0x%02x",P1DIR); printf(" 0x%02x",P1REN); printf(" 0x%04x",TA0CCR0); printf(" 0x%04x",TA0CCTL1); printf(" 0x%04x",TA0CCTL2); printf(" 0x%04x",TA0CCTL3); printf(" 0x%04x",TA0CCTL4); printf(" 0x%04x\n",TA0CTL);;
#define MACRO_LAB4_EVENT()              printf("0xf2"); printf(" 0x%02x",P1IN); printf(" 0x%04x",TA0CCR1); printf(" 0x%04x",TA0CCR2); printf(" 0x%04x",TA0CCR3); printf(" 0x%04x",TA0CCR4); printf(" %d %d",encA_cnt,encB_cnt); printf(" %d %d\n",encA_dir,encB_dir)

#define MACRO_LAB5_INIT()               printf("\n0xff"); printf(" 0x%04x",UCA0CTLW0); printf(" 0x%04x",UCA0BRW); printf(" 0x%02x",UCA0IE); printf(" 0x%08x\n",ADC14->CTL0);
#define MACRO_LAB5_ADC_EVENT(x)         printf("0xf3"); printf(" %.2f", x); printf(" 0x%04x\n",ADC14->MEM[0]);
#define MACRO_LAB5_UART_RX_EVENT(x)     printf("0xf4"); printf(" 0x%02x ",UCA0RXBUF); printf("%.2f\n", x);
#define MACRO_LAB5_UART_TX_EVENT()      printf("0xf5"); printf(" 0x%02x\n",UCA0TXBUF);

#define MACRO_LAB6_INIT()               printf("\n0xff"); printf(" 0x%02x",P4DIR); printf(" 0x%02x",P4REN); printf(" 0x%02x",P4IE); printf(" 0x%04x",TA0CCR0); printf(" 0x%04x",TA0CCTL1); printf(" 0x%04x",TA0CCTL2); printf(" 0x%04x",TA0CCTL3); printf(" 0x%04x",TA0CCTL4); printf(" 0x%04x",TA0CTL); printf(" 0x%04x",TA1CTL); printf(" 0x%04x",UCA0CTLW0); printf(" 0x%04x",UCA0BRW); printf(" 0x%02x",UCA0IE); printf(" 0x%08x\n",ADC14->CTL0);
#define MACRO_LAB6_UART_RX_EVENT(x)     printf("0xf6"); printf(" 0x%02x ",UCA0RXBUF); printf("%.2f\n", x);
#define MACRO_LAB6_UART_TX_EVENT()      printf("0xf7"); printf(" 0x%02x\n",UCA0TXBUF);
#define MACRO_LAB6_ADC_EVENT(x)         printf("0xf8"); printf(" %.2f", x); printf(" 0x%04x\n",ADC14->MEM[0]); printf("0xfa"); printf(" 0x%04x",TA0CCR1); printf(" 0x%04x",TA0CCR2); printf(" 0x%04x",TA0CCR3); printf(" 0x%04x",TA0CCR4); printf(" %d %d",encA_cnt,encB_cnt); printf(" %d %d\n",encA_dir,encB_dir);
#define MACRO_LAB6_SWITCH_EVENT()       printf("0xf9"); printf(" 0x%02x\n",P4IFG);


void mechrev_setup(void)
{
    /* Sleep pins for DRV8825 on TI-RSLK MAX Chassis Board */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);

    /* GPIOs for encoders interrupt configuration */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN0);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN2);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P10,GPIO_PIN4);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P10,GPIO_PIN5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5,GPIO_PIN0);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5,GPIO_PIN2);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5,GPIO_PIN0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5,GPIO_PIN2);
    MAP_Interrupt_enableInterrupt(INT_PORT5);
}


/* Encoder GPIO ISR */
void PORT5_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);

    if(status & GPIO_PIN0)
    {
        encA_cnt++;

        if (GPIO_INPUT_PIN_HIGH == MAP_GPIO_getInputPinValue(GPIO_PORT_P10,GPIO_PIN4))
            encA_dir = 1;
        else
            encA_dir = 0;
    }

    if(status & GPIO_PIN2)
    {
        encB_cnt++;
        if (GPIO_INPUT_PIN_HIGH == MAP_GPIO_getInputPinValue(GPIO_PORT_P10,GPIO_PIN5))
            encB_dir = 1;
        else
            encB_dir = 0;
    }

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
}

#endif /* MECHREV_H_ */

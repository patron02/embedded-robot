//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include <string.h>
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
//Global variables
char command[4];
void Fwd_func(void);
void Bwd_func(void);
void Red_func(void);
void Blu_func(void);
void LED_PWM_func(void);

typedef struct{
    char cmd[4];
    void (*fp)(void);

} Cmd;

const Cmd cmdLoopUp[] ={
         {"fwd", Fwd_func},
         {"bwd", Bwd_func},
         {"red", Red_func},
         {"blu", Blu_func},
         {"led", LED_PWM_func}

};
uint32_t PWM_clock, PWM_freq, Load, i; // Declare these variables
uint32_t PWM_freq = 1000;  // Set frequency to 1000 Hz
uint32_t count = 5;

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************
// character counter
volatile uint32_t charCounter = 0;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
// Buffer to store the received characters
#define COMMAND_BUFFER_SIZE 5 // 3 characters + null terminator
#define pwm_frequency 1000  // Define the PWM frequency
char commandBuffer[COMMAND_BUFFER_SIZE];
uint32_t charIndex = 0;

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UARTIntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        // Read the next character from the UART
        char receivedChar = ROM_UARTCharGetNonBlocking(UART0_BASE);

        // Echo the received character back to the UART
        ROM_UARTCharPutNonBlocking(UART0_BASE, receivedChar);
        ROM_UARTCharPutNonBlocking(UART1_BASE, receivedChar);

        // Increment the character counter
        charCounter++;
   }
}


void UART1IntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        // Read the next character from the UART
        char receivedChar = ROM_UARTCharGetNonBlocking(UART1_BASE);

        // Echo the received character back to the UART
        ROM_UARTCharPutNonBlocking(UART0_BASE, receivedChar);
        ROM_UARTCharPutNonBlocking(UART1_BASE, receivedChar);

        // Store the received character into the buffer
        command[charIndex] = receivedChar;
        charIndex++;

        // When 3 characters are received, process the command
        if (charIndex == 3) {
            command[3] = '\0';  // Null-terminate the string
            charIndex = 0;      // Reset the index
        }

        // Increment the character counter
        charCounter++;
    }

}
//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(uint32_t base, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART (blocking).
        ROM_UARTCharPut(base, *pui8Buffer++);
    }
}

void PWM_init(void) {
    // Set the PWM clock divisor to divide the system clock by 64, 16MHz/64 = 250MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Enable the PWM1 peripheral so that the LED can be controlled
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {};

    // Enable the GPIO port F for the on-board LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {};

    // Configure PF1 for M1PWM6 to control brightness
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);  // Set PF1 as a PWM pin

    // Set the PWM frequency
    uint32_t PWM_clock = SysCtlClockGet() / 64;  // Adjust for the divisor set by SysCtlPWMClockSet()
    uint32_t PWM_freq = 50;  // Set frequency to 50 Hz for LED control
    Load = (PWM_clock / PWM_freq) - 1;


    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, Load);

    // Set to 50% duty cycle
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, (uint32_t)(Load * 0.5));

    // Configure PWM for countdown mode
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Enable PWM output and generator for the RED LED (M1PWM5)
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);  // Enable M1PWM5 for RED LED (PF1)
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable the GPIO port that is used for the on-board LED (Port F).
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    PWM_init();  // Initialize PWM


    // Set the direction and enable the GPIO pins for the LED (PF2 for Blue, PF3 for Green).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Prompt for text to be entered.
    //
    UARTSend(UART0_BASE, (uint8_t *)"Please enter characters and see the color change: ",50);
    UARTSend(UART1_BASE, (uint8_t *)"Enter 3-letter commands: ",25);
    // Loop forever echoing data through the UART.

while (1)
    {
        if (strlen(command) == 3)
        {
            UARTSend(UART0_BASE, (uint8_t *)"\nCOMMAND IS ", 13);
            UARTSend(UART0_BASE, (uint8_t *)command, 3);

            // Search the CmdLoopUp table for the matchi2ng command
            int i = 0; // Declare i before using it in a loop or condition.
            for (i = 0; i < sizeof(cmdLoopUp) / sizeof(cmdLoopUp[0]); i++)
            {
                if (!(strcmp(command, cmdLoopUp[i].cmd)))
                {
                    UARTSend(UART0_BASE, (uint8_t *)"\nMatched command \n", 17);
                    cmdLoopUp[i].fp();  // Call the function pointer
                    break;
                }
            }
            // Reset ready and command after processing
            memset(command, 0, sizeof(command));
        }
}

}
//Command Functions
void Fwd_func(void){
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);  // Turn off all LEDs
//    UARTSend(UART0_BASE, (uint8_t *)"\nMove Forward\n", 13);
//    UARTSend(UART1_BASE, (uint8_t *)"\nMove Forward\n", 13);

}

void Bwd_func(void){
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);  // Turn off all LEDs
//    UARTSend(UART0_BASE, (uint8_t *)"\nMove Backward\n", 14);
//    UARTSend(UART1_BASE, (uint8_t *)"\nMove Backward\n", 14);
}

void Red_func(void){
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // Set only GPIO_PIN_1
//
}

void Blu_func(void){
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0x0);
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // Set only GPIO_PIN_2

}
void LED_PWM_func(void) {

   // Set the PWM duty cycle to adjust the LED brightness
   while(1){
       float percent = count/10.0;
       SysCtlDelay(SysCtlClockGet()/3 *2); //delay by 2 sec
   if(count != 0  ){
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, (uint32_t)(Load * percent));  // Set 50% duty cycle for Red LED

   }
   else{
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, (uint32_t)(1));  // Set 50% duty cycle for Red LED
   }
    count++;
    if(count == 11){
        count = 0;
    }
   }
}


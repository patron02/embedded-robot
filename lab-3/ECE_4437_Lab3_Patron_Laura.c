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

//Global variables
int color = 1;
int ready;
char blue_in[4];
void Fwd_func(void);
void Bwd_func(void);
void Red_func(void);
void Blu_func(void);

typedef struct{
    char cmd[4];
    void (*fp)(void);

} Cmd;

const Cmd cmdLoopUp[] ={
         {"fwd", Fwd_func},
         {"bwd", Bwd_func},
         {"red", Red_func},
         {"blu", Blu_func}
};

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
#define COMMAND_BUFFER_SIZE 4 // 3 characters + null terminator
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
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        // Read the next character from the UART
        char receivedChar = ROM_UARTCharGetNonBlocking(UART0_BASE);

        // Echo the received character back to the UART
        ROM_UARTCharPutNonBlocking(UART0_BASE, receivedChar);
        ROM_UARTCharPutNonBlocking(UART1_BASE, receivedChar);

        // Increment the character counter
        charCounter++;
        //
        // Determine the LED color based on the number of characters entered
        uint32_t ledColor = 0;

        if (charCounter % 3 == 1)
        {
        // First character, set LED to Blue
        ledColor = GPIO_PIN_2; // PF2 (Blue)
                }
                else if (charCounter % 3 == 2)
                {
                    // Second character, set LED to Red
                    ledColor = GPIO_PIN_1; // PF1 (Red)
                }
                else
                {
                    // Third character (or any multiple of 3), set LED to Green
                    ledColor = GPIO_PIN_3; // PF3 (Green)
                }

        // Turn off all LEDs first
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

        // Turn on the selected LED
        GPIOPinWrite(GPIO_PORTF_BASE, ledColor, ledColor);
            }

}


void
UART1IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        // Read the next character from the UART
        char receivedChar = ROM_UARTCharGetNonBlocking(UART1_BASE);

        // Echo the received character back to the UART
        ROM_UARTCharPutNonBlocking(UART0_BASE, receivedChar);
        ROM_UARTCharPutNonBlocking(UART1_BASE, receivedChar);

        // Store the received character into the buffer
        blue_in[charIndex] = receivedChar;
        charIndex++;

        // When 3 characters are received, process the command
        if (charIndex == 3) {
            blue_in[3] = '\0';  // Null-terminate the string
            ready = 1;          // Signal that a command is ready
            charIndex = 0;      // Reset the index
        }


        // Increment the character counter
        charCounter++;
        //
        // Determine the LED color based on the number of characters entered
        uint32_t ledColor = 0;

        if (charCounter % 3 == 1)
        {
        // First character, set LED to Blue
        ledColor = GPIO_PIN_2; // PF2 (Blue)
                }
                else if (charCounter % 3 == 2)
                {
                    // Second character, set LED to Red
                    ledColor = GPIO_PIN_1; // PF1 (Red)
                }
                else
                {
                    // Third character (or any multiple of 3), set LED to Green
                    ledColor = GPIO_PIN_3; // PF3 (Green)
                }

        // Update the LED color
         GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, ledColor);
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
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART (blocking).
        //
        ROM_UARTCharPut(base, *pui8Buffer++);

    }

}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
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
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    // Enable the GPIO port that is used for the on-board LED (Port F).
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);


    // Set the direction and enable the GPIO pins for the LED (PF2 for Blue, PF3 for Green).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);


    // Turn on the green LED initially.
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

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
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                              UART_CONFIG_PAR_NONE));
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
    UARTSend(UART1_BASE, (uint8_t *)"Please enter 3-letter commands: ",31);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    //
    // Loop forever echoing data through the UART.
    //

while (1)
    {
        if (strlen(blue_in) == 3)
        {
            blue_in[3] = '\0';
            UARTSend(UART0_BASE, (uint8_t *)"\nCOMMAND IS ", 13);
            UARTSend(UART0_BASE, (uint8_t *)blue_in, 3);

            // Search the CmdLoopUp table for the matching command
            int i = 0; // Declare i before using it in a loop or condition.
            for (i = 0; i < sizeof(cmdLoopUp) / sizeof(cmdLoopUp[0]); i++)
            {
                if (!(strcmp(blue_in, cmdLoopUp[i].cmd)))
                {
                    UARTSend(UART0_BASE, (uint8_t *)"\nMatched command ", 16);
                    cmdLoopUp[i].fp();  // Call the function pointer
                    break;
                }
            }


            // Reset ready and blue_in after processing
            ready = 0;
            memset(blue_in, 0, sizeof(blue_in));
        }
    }

    }
//Command Functions
void Fwd_func(void){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);  // Turn off all LEDs
    UARTSend(UART0_BASE, (uint8_t *)"Move Forward\n", 13);
}

void Bwd_func(void){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);  // Turn off all LEDs
    UARTSend(UART0_BASE, (uint8_t *)"Move Backward\n", 14);
}

void Red_func(void){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // Set only GPIO_PIN_1

}

void Blu_func(void){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // Set only GPIO_PIN_2

}

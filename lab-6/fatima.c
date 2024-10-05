/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty_min.c ========
 */
// XDCtools Header files /
#include <xdc/std.h>
#include <xdc/runtime/System.h>

// BIOS Header files /
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

// TI-RTOS Header files
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/PWM.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "uartstdio.h"

extern ti_sysbios_knl_Semaphore_Handle Task0Semaphore;
//use volatile to modify value in interrupt handler
volatile uint64_t startCount;
volatile uint64_t endCount;

void EnablePeripherals();
void InitializeUART0LocalTerminal();
void InitializeTimer1();


void WTimer1IntHandler()
{
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    UARTprintf("Timer Interrupt Triggered\n");
    Semaphore_post(Task0Semaphore);
}
void GPIOAIntHandler()
{
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
    endCount = TimerValueGet64(WTIMER0_BASE);

    uint64_t elapsedCount = endCount - startCount;
    UARTprintf("Decay Count Ticks: %i\n", (int)elapsedCount);
    if(elapsedCount > 40000)
    {
        UARTprintf("On a dark surface\n");
    }
    else
    {
        UARTprintf("On a white surface\n");
    }
}
void Task0Func(){

    while(1)
    {
        //Create a task and linked it to Task0Func.
        //Semaphore locks tasks and allow to sleep while other tasks run
        //Semaphore is a counter to check to enable task
        Semaphore_pend(Task0Semaphore, BIOS_WAIT_FOREVER);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(SysCtlClockGet()/3 * 10 * 1.0/1000000);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);
        startCount = TimerValueGet64(WTIMER0_BASE);
    }
}
void EnablePeripherals()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
}

void InitializeUART0LocalTerminal(){

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    UARTEnable(UART0_BASE);

    UARTStdioConfig(0, 115200, SysCtlClockGet());
}
void InitializeTimer0()
{
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);
    // big timer increases count value. 85 second timer
    TimerLoadSet64(WTIMER0_BASE, 18446744073709551615); //TIMER 2^64 -1 =1.84467441 x 10^19
    TimerEnable(WTIMER0_BASE, TIMER_A);
}

void InitializeTimer1()
{
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_WTIMER1A);
    TimerLoadSet(WTIMER1_BASE, TIMER_A, SysCtlClockGet() * 0.5);
    TimerEnable(WTIMER1_BASE, TIMER_A);
}
void InitializeGPIOAInterrupt()
{
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
}

/*
 *  ======== main ========
 */
int main(void)
{
    //main clock running at 200/5 = 40MHZ
        SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

        IntMasterEnable();

        EnablePeripherals();
        InitializeUART0LocalTerminal();
        InitializeTimer0();
        InitializeTimer1();
        InitializeGPIOAInterrupt();


        uint64_t elapsedCount = endCount - startCount;



        UARTprintf("Main program is running\n");
    /* Start BIOS */
    BIOS_start();

    return (0);
}

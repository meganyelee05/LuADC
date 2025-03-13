/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//******************************************************************************
//   Write a Word to Port A (Port1+Port2)
//
//   Writes a Word(FFFFh) to Port A and stays in LPM4
//   ACLK = 32.768kHz, MCLK = SMCLK = default DCO
//
//  Tested On:   MSP430FR5969
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST          PA.x|-->HI
//            |                 |
//            |                 |
//
//
//   This example uses the following peripherals and I/O signals.  You must
//   review these and change as needed for your own board:
//   - GPIO Port peripheral
//
//   This example uses the following interrupt handlers.  To use this example
//   in your own application you must add these interrupt handlers to your
//   vector table.
//   - None.
//******************************************************************************
#include "cs.h"
#include "driverlib.h"
#include "gpio.h"
#include "intrinsics.h"
#include "msp430fr5969.h" //try changing to msp430fr6043.h (hopefully nothing breaks)
#include "timer_a.h"

void main (void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    //PA.x output
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN1);

    // //Set all PA pins HI
    // GPIO_setOutputHighOnPin(

    //     GPIO_PORT_PJ,
    //     GPIO_PIN1
    //     );

    //Set SMCLK = DCO = 16MHz
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_4); //rsel: 0 = low, 1 = hi; fsesl6: low = 5.33MHz, hi = 16MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //configure Timer0_A
    Timer_A_initUpModeParam upConfig = {0};
    upConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    upConfig.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_4; //set TimerA0 to SMCLK/4 = 4MHz
    upConfig.timerPeriod = 10000; //each # is 500ns
    upConfig.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE; //disable overflow interrupt
    upConfig.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //enable CCR0 interrupt
    upConfig.timerClear = TIMER_A_DO_CLEAR;

    //initialize Timer0_A
    Timer_A_initUpMode(TIMER_A0_BASE, &upConfig);

    //enable Timer0_A0 interrupt
    __enable_interrupt(); //do i need this???

    //start Timer0_A
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();
    while(1){
        //Enter LPM4 w/interrupts enabled
        __bis_SR_register(LPM4_bits + GIE);

        //For debugger
        __no_operation();
    }
    
}


// Timer0_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    GPIO_toggleOutputOnPin(GPIO_PORT_PJ, GPIO_PIN1);
}

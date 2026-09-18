/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430G2xx3 Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 by xor'ing P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  E. Chen
//  Texas Instruments, Inc
//  May 2018
//  Built with CCS Version 8.0 and IAR Embedded Workbench Version: 7.11
//******************************************************************************

#include <msp430.h>

// Actif à 1
// LED1: 1.6



// #pragma vector=PORT1_VECTOR
// __interrupt void port1_isr(void)
// {
//   if ((P1IFG & BIT3) == BIT3)
//   { 
//     if (TA0CCR0 == 62500){
//       TA0CCR0 = 31250;
//     }
//     else{
//       TA0CCR0 =62500;
//     }
//     __delay_cycles(50);
//     P1IFG &= ~(BIT3);
//   }
// }


int main(void) {
  volatile unsigned int i;
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

  BCSCTL1= CALBC1_1MHZ; //frequence d’horloge 1MHz
  DCOCTL= CALDCO_1MHZ; // "

  P2DIR |= (BIT1 | BIT5); // P2.5 en sortie
  P2SEL &= ~(BIT1 | BIT5); // selection fonction TA1.2
  P2SEL2 &= ~(BIT1 | BIT5); // selection fonction TA1.2
  P2OUT |= (BIT5);
  P2OUT &= ~(BIT1);

  // opto gauche
  P2DIR &= ~BIT0; // P2.5 en sortie
  P2SEL |= BIT0; // selection fonction TA1.2
  P2SEL2 &= ~BIT0; // selection fonction TA1.2

  // opto droit
  P2DIR &= ~BIT3; // P2.5 en sortie
  P2SEL |= BIT3; // selection fonction TA1.2
  P2SEL2 &= ~BIT3; // selection fonction TA1.

  // moteur gauche
  P2DIR |= BIT2; // P2.5 en sortie
  P2SEL |= BIT2; // selection fonction TA1.2
  P2SEL2 &= ~BIT2; // selection fonction TA1.2
  
  // moteur droit
  P2DIR |= BIT4; // P2.5 en sortie
  P2SEL |= BIT4; // selection fonction TA1.2
  P2SEL2 &= ~BIT4; // selection fonction TA1.2

  //CONFIG TIMER TA1
  TA1CTL = 0 |TASSEL_2 | MC_1 | ID_3 | TACLR; // source SMCLK pour TimerA , mode comptage Up
  TA1CCTL2 |= OUTMOD_7; // activation mode de sortie n°7
  TA1CCTL1 |= OUTMOD_7; // activation mode de sortie n°7

  TA1CCR0 = 8191; // determine la periode du signal
  TA1CCR2 = 8000; // determine le rapport cyclique du signal
  TA1CCR1 = 8000; // determine le rapport cyclique du signal

  while (1){

  };
}

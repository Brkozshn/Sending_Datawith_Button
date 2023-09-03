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
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,"""
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430FR267x Demo - eUSCI_A0 UART echo at 9600 baud using BRCLK = 8MHz.
//
//  Description: This demo echoes back characters received via a PC serial port.
//  SMCLK/ DCO is used as a clock source and the device is put in LPM3
//  The auto-clock enable feature is used by the eUSCI and SMCLK is turned off
//  when the UART is idle and turned on when a receive edge is detected.
//  Note that level shifter hardware is needed to shift between RS232 and MSP
//  voltage levels.
//
//  The example code shows proper initialization of registers
//  and interrupts to receive and transmit data.
//  To test code in LPM3, disconnect the debugger.
//
//  ACLK = REFO = 32768Hz, MCLK = DCODIV = SMCLK = 8MHz.
//
//                MSP430FR2676
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//            |                 |
//            |     P5.2/UCA0TXD|----> PC (echo)
//            |     P5.1/UCA0RXD|<---- PC
//            |                 |
//
//   Longyu Fang
//   Texas Instruments Inc.
//   August 2018
//   Built with IAR Embedded Workbench v7.12.1 & Code Composer Studio v8.1.0
//******************************************************************************
#include <msp430.h>

void Init_GPIO();
//void Software_Trim();                       // Software Trim to get the best DCOFTRIM value
//void Init_ADC();


#define MCLK_FREQ_MHZ 8                     // MCLK = 8MHz

unsigned int ADC_Result;

enum{
    m_PHASE0,
    m_PHASE1,
    m_PHASE2,
    m_PHASE3,


    m_NUM_OF_PHASES
};

int g_phase = m_PHASE1;

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                // Stop watchdog timer


    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    P1IFG &= ~BIT3;                         // P1.3 IFG cleared



  // Configure GPIO
  Init_GPIO();
  // Configure ADC
  //Init_ADC();

  PM5CTL0 &= ~LOCKLPM5;                    // Disable the GPIO power-on default high-impedance mode
                                           // to activate 1previously configured port settings


  __bis_SR_register(SCG0);                 // disable FLL
  CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
  CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
  CSCTL2 = FLLD_0 + 243;                  // DCODIV = 8MHz
  __delay_cycles(3);
  __bic_SR_register(SCG0);                // enable FLL
//  Software_Trim();                        // Software Trim to get the best DCOFTRIM value

  CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                           // default DCODIV as MCLK and SMCLK source

  // Configure UART pins
  P5SEL0 |= BIT1 | BIT2;                    // set 2-UART pin as second function
  SYSCFG3|=USCIA0RMP;                       //Set the remapping source
  // Configure UART
  UCA0CTLW0 |= UCSWRST;
  UCA0CTLW0 |= UCSSEL__SMCLK;

  // Baud Rate calculation
  // 8000000/(16*9600) = 52.083
  // Fractional portion = 0.083
  // User's Guide Table 17-4: UCBRSx = 0x49
  // UCBRFx = int ( (52.083-52)*16) = 1
  UCA0BR0 = 52;                             // 8000000/16/9600
  UCA0BR1 = 0x00;
  UCA0MCTLW = 0x4900 | UCOS16 | UCBRF_1;

  UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt


  __bis_SR_register(GIE);          //Global interrupt enable
  while(1)
  {

__no_operation();

    if ((g_phase == m_PHASE1)) {

            P1OUT |= BIT0;    // Set P1.0 LED on
            UCA0TXBUF = 'x';
            //__bis_SR_register(LPM3_bits|GIE);

           // g_phase = m_PHASE0;
       }

  }


 // __bis_SR_register(LPM3_bits|GIE);         // Enter LPM3, interrupts enabled
  //__no_operation();                         // For debugger

}


void Init_GPIO()
{
    P1DIR = 0xFF; P2DIR = 0xFF;
    P1REN = 0xFF; P2REN = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00;

    // Configure GPIO
      P1DIR |= BIT0;                                           // Set P1.0/LED to output direction
      P1OUT &= ~BIT0;                                          // P1.0 LED off

      // Configure ADC A1 pin
      P1SEL0 |= BIT1;
      P1SEL1 |= BIT1;


      // Configure GPIO
         P1OUT &= ~BIT0;                         // Clear P1.0 output latch for a defined power-on state
         P1DIR |= BIT0;                          // Set P1.0 to output direction

         P1OUT |= BIT3;                          // Configure P1.3 as pulled-up
         P1REN |= BIT3;                          // P1.3 pull-up register enable
         P1IES |= BIT3;                          // P1.3 Hi/Low edge
         P1IE |= BIT3;                           // P1.3 interrupt enabled



}



// Port 1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{

    switch(g_phase)
               {
                   case m_PHASE0:
                       g_phase = m_PHASE2;
                       while(!(UCA0IFG&UCTXIFG));
                       UCA0TXBUF = 'C';
                       break;
                   case m_PHASE1:
                       g_phase = m_PHASE0;
                       while(!(UCA0IFG&UCTXIFG));
                       UCA0TXBUF = 'A';
                       break;
                   case m_PHASE2:
                       g_phase = m_PHASE1;
                       break;

               }


    __no_operation();                                    // For debug only




    P1IFG &= ~BIT3;                         // Clear P1.3 IFG
    __bic_SR_register_on_exit(LPM3_bits);   // Exit LPM3
}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
        while(!(UCA0IFG&UCTXIFG));
        UCA0TXBUF = UCA0RXBUF;
      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}


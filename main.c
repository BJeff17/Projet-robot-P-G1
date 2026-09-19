

#include <msp430.h>

#define FREQ_MOT (992)

volatile unsigned int capt_opto_g = 0;
volatile unsigned int capt_opto_d = 0;

#pragma vector=TIMER0_A1_VECTOR
__interrupt void capture_opto(void)
{ 
  if((P2IN & BIT0)== BIT0){
    capt_opto_g++;
  }

  if((P2IN & BIT3)== BIT3){
    capt_opto_d++;
  }
}

void init_moteur(){

  // opto gauche
  P2DIR &= ~BIT0; // P2.5 en sortie
  P2SEL |= BIT0; // selection fonction TA1.2
  P2SEL2 &= ~BIT0; // selection fonction TA1.2

  // opto droit
  P2DIR &= ~BIT3; // P2.5 en sortie
  P2SEL |= BIT3; // selection fonction TA1.2
  P2SEL2 &= ~BIT3; // selection fonction TA1.

  //config sens gauche et droit 
  P2DIR |= (BIT1 | BIT5); // P2.5 en sortie
  P2SEL &= ~(BIT1 | BIT5); // selection fonction TA1.2
  P2SEL2 &= ~(BIT1 | BIT5); // selection fonction TA1.2
  // moteur gauche
  P2DIR |= BIT2; // P2.5 en sortie
  P2SEL |= BIT2; // selection fonction TA1.2
  P2SEL2 &= ~BIT2; // selection fonction TA1.2
  
  // moteur droit
  P2DIR |= BIT4; // P2.5 en sortie
  P2SEL |= BIT4; // selection fonction TA1.2
  P2SEL2 &= ~BIT4; // selection fonction TA1.2

  //CONFIG TIMER TA1
  TA1CTL = 0 |TASSEL_2 | MC_1 | ID_0 | TACLR; // source SMCLK pour TimerA , mode comptage Up
  TA1CCTL2 |= OUTMOD_7; // activation mode de sortie n°7
  TA1CCTL1 |= OUTMOD_7; // activation mode de sortie n°7

  TA1CCR0 = FREQ_MOT; // determine la periode du signal
  TA1CCR2 = 0; // determine le rapport cyclique du signal
  TA1CCR1 = 0; // determine le rapport cyclique du signal

  P2OUT |= (BIT5);// sens
  P2OUT &= ~(BIT1);// sens
}

void pilotage_moteur(int sens_g, int puissance_g, int sens_d, int puissance_d){
  if (sens_g == 1 ){
   P2OUT |= (BIT5);// sens
  }
  else
  {
    P2OUT &= ~(BIT5);// sens
  }
  if (sens_d == 1 ){
   P2OUT |= (BIT1);// sens
  }
  else
  {
    P2OUT &= ~(BIT1);// sens
  }
  TA1CCR2 = FREQ_MOT*((float)puissance_d/100.0); // determine le rapport cyclique du signal
  TA1CCR1 = FREQ_MOT*((float)puissance_g/100.0); // determine le rapport cyclique du signal
}
int main(void) {
  volatile unsigned int i;
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

  BCSCTL1= CALBC1_1MHZ; //frequence d’horloge 1MHz
  DCOCTL= CALDCO_1MHZ; // "


  P2DIR &= ~(BIT0 | BIT3); // P1.1 en entree
  P2SEL |= (BIT0 | BIT3); // fonction entree capture sur P1.1 (TA0.CCI0A)
  P2SEL2 &= ~(BIT0 | BIT3);

  TA1CCTL0 |= CM_0 | CCIS_0; // front montant + CCI0A
  TA1CCTL0 |= CAP | CCIE; // mode capture + autorisation interruption

  init_moteur();
  pilotage_moteur(1,25,0,25);

  __enable_interrupt();
  while (1);
}

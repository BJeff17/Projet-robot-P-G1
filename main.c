

#include <msp430.h>

#define FREQ_MOT (992)

volatile unsigned int capt_opto_g = 0;
volatile unsigned int capt_opto_d = 0;
volatile int diff_capt = 0 ;


#define correction_rg (0)
#define correction_rd (0)

#pragma vector=PORT2_VECTOR
__interrupt void capture_opto(void)
{ 
  if((P2IFG & BIT0) == BIT0){
      capt_opto_g++;
      P2IES ^= (BIT0); 
    P2IFG &= ~BIT0;
  }
  if((P2IFG & BIT3) == BIT3){
      capt_opto_d++;
      P2IES ^= (BIT3); 
    P2IFG &= ~BIT3;
  }

}

void init_moteur(){

  // opto gauche
  P2DIR &= ~BIT0; // P2.0 en entrée
  P2SEL &= ~BIT0; // selection fonction TA1.2
  P2SEL2 &= ~BIT0; // selection fonction TA1.

  // opto droit
  P2DIR &= ~BIT3; // P2.3 en entrée
  P2SEL &= ~BIT3; // selection fonction TA1.2
  P2SEL2 &= ~BIT3; // selection fonction TA1.
  
  P2IE |= (BIT0 | BIT3); 
  P2IES |= (BIT0 | BIT3); 

  //config sens gauche et droit 
  P2DIR |= (BIT1 | BIT5); // P2.5 et P2.1 en sortie
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
    P2IFG &= ~BIT0;
    P2IFG &= ~BIT3;

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
  
  TA1CCR2 = (FREQ_MOT*(puissance_d - correction_rg))/100; // determine le rapport cyclique du signal
  TA1CCR1 = (FREQ_MOT*(puissance_g - correction_rd))/100; // determine le rapport cyclique du signal
}
void distance(float target_distance){ //distance en cm
  float actual_distance =  0;
  
  if (capt_opto_g > capt_opto_g){
    actual_distance =13.5*((float)(capt_opto_g) /24.0);//1 tic = 0.5 cm
  }else{
    actual_distance =13.5*((float)(capt_opto_g) /24.0);//1 tic = 0.5 cm
  } 

  if(actual_distance >= target_distance ){
    pilotage_moteur(0,0,0,0);
  }
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
  pilotage_moteur(1,80,0,80);

  __enable_interrupt();
  while (1) {
    distance(130.0);
  }
}



#include <msp430.h>

//#define ROBOT_FAST
#define ROBOT_SLOW

#if defined(ROBOT_FAST)
  #define FREQ_MOT     (992)
  #define BASE_SPEED   (90)
  #define GAIN         (2)
  #define CORR_MAX     (15)
  #define OPTO_D_OK    (0)   // capteur droit mort
  #define CORR_STATIC_G       (0)  
  #define CORR_STATIC_D       (-6)  
#elif defined(ROBOT_SLOW)
  #define FREQ_MOT     (992)
  #define BASE_SPEED   (85)
  #define GAIN         (1)
  #define CORR_MAX     (15)
  #define OPTO_D_OK    (1)
  #define CORR_STATIC_G       (0)
  #define CORR_STATIC_D       (0)
#else
  #error "ROBOT_FAST ou ROBOT_SLOW seulement"
#endif

volatile unsigned int capt_opto_g = 0;
volatile unsigned int capt_opto_d = 0;
volatile int diff_capt = 0 ;

volatile int correction = 0;
volatile unsigned char flag_correction = 0;

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

#pragma vector=TIMER0_A0_VECTOR
__interrupt void timer_correction(void)
{
  flag_correction = 1 ;
}


void init_moteur(){

  // opto gauche
  P2DIR &= ~BIT0; // P2.0 en entrée
  P2SEL &= ~BIT0; 
  P2SEL2 &= ~BIT0; 

  //opto droit
  P2DIR &= ~BIT3; //P2.3 en entrée
  P2SEL &= ~BIT3; 
  P2SEL2 &= ~BIT3; 
  
#if OPTO_D_OK
  P2IE |= (BIT0 | BIT3);   // les deux capteurs opto fonctionnent
  P2IES |= (BIT0 | BIT3);
#else
  P2IE |= BIT0;// capteur opto mort
  P2IES |= BIT0;          
#endif

  //config sens gauche et droit 
  P2DIR |= (BIT1 | BIT5); // P2.5 et P2.1 en sortie
  P2SEL &= ~(BIT1 | BIT5); 
  P2SEL2 &= ~(BIT1 | BIT5); 
  // moteur gauche
  P2DIR |= BIT2; //P2.2 en sortie
  P2SEL |= BIT2; 
  P2SEL2 &= ~BIT2; 
  
  // moteur droit
  P2DIR |= BIT4;// P2.4 en sortie
  P2SEL |= BIT4; 
  P2SEL2 &= ~BIT4; 

  //CONFIG TIMER TA1
  TA1CTL = 0 |TASSEL_2 | MC_1 | ID_0 | TACLR; 
  TA1CCTL2 |= OUTMOD_7; 
  TA1CCTL1 |= OUTMOD_7; 

  // Config PWM
  TA1CCR0 = FREQ_MOT; 
  TA1CCR2 = 0; 
  TA1CCR1 = 0; 

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
  
TA1CCR2 = (unsigned int)(((unsigned long)FREQ_MOT * puissance_d) / 100);
TA1CCR1 = (unsigned int)(((unsigned long)FREQ_MOT * puissance_g) / 100);
}
void config_timer_correction(unsigned int selec_horloge, unsigned int prediv,unsigned int mode_comptage, unsigned int max_comptage){

  TA0CTL = 0|(selec_horloge | prediv); //source SMCLK, pas de predivision ID_0
  TA0CCR0 = max_comptage; //voir diapo précédente
  TA0CTL |= mode_comptage; //arrêt du comptage
}
void corriger_trajectoire(void){
#if OPTO_D_OK
  static unsigned int temp_capt_g = 0, temp_capt_d = 0;
  int dg = capt_opto_g - temp_capt_g; 
  int dd = capt_opto_d - temp_capt_d;
  temp_capt_g = capt_opto_g;
  temp_capt_d = capt_opto_d;

  int erreur = dg - dd; 
  correction += erreur * GAIN;

  if (capt_opto_d == 220){
    correction = correction;
  }
  if(correction > CORR_MAX)  correction = CORR_MAX;
  if(correction < -CORR_MAX) correction = -CORR_MAX;
#endif
  // OPTO_D_OK = 0 = capteur opto mort
  pilotage_moteur(1, BASE_SPEED - correction - CORR_STATIC_G, 0, BASE_SPEED + correction - CORR_STATIC_D);
}
void distance(float target_distance){ //distance en cm
  float actual_distance =  0;
  
  if (capt_opto_d > capt_opto_g){
    actual_distance =13.5*((float)(capt_opto_d) /24.0);//1 tic = 0.5 cm
  }else{
    actual_distance =13.5*((float)(capt_opto_g) /24.0);//1 tic = 0.5 cm
  } 
  if(actual_distance >= target_distance ){
    pilotage_moteur(0,0,0,0);
  }
}

int main(void) {
  volatile unsigned int i;
  WDTCTL = WDTPW + WDTHOLD; 

  BCSCTL1= CALBC1_1MHZ; 
  DCOCTL= CALDCO_1MHZ; 

  P2DIR &= ~(BIT0 | BIT3); 
  P2SEL |= (BIT0 | BIT3); 
  P2SEL2 &= ~(BIT0 | BIT3);

  TA1CCTL0 |= CM_0 | CCIS_0; // front montant + CCI0A
  TA1CCTL0 |= CAP | CCIE; // mode capture + autorisation interruption

  config_timer_correction(TASSEL_2, ID_1, MC_1, 49999);
  TA0CCTL0 = CCIE;         
  
  init_moteur();
  pilotage_moteur(1, BASE_SPEED + CORR_STATIC_G, 0, BASE_SPEED + CORR_STATIC_D);

  __enable_interrupt();
  while (1) {
    if(flag_correction = 1){
      flag_correction = 0;
      corriger_trajectoire(); 
    }
    distance(130.0);
  }
}

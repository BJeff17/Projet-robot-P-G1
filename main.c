#include "ADC.h"
#include "Afficheur.h"
#include <msp430.h>

#define FREQ_MOT (992)
// #define NB_ORI (4)
#define NB_OBST (2)
#define NB_ETAT (2)

volatile unsigned int capt_opto_g = 0;
volatile unsigned int capt_opto_d = 0;
volatile int diff_capt = 0 ;
static int valeur_capt = 0;


#define correction_rg (0)
#define correction_rd (0)

// typedef enum {ORI_DROITE, ORI_HAUT, ORI_GAUCHE, ORI_BAS} ORI_ROBOT;    //orientation robot
typedef enum {CAPT_OFF, CAPT_ON} CAPT_OBST; //capteur d'obstacle
typedef enum {ETAT_AVANCER, ETAT_TOURNER} ETAT;
typedef void (*Action)(void);  

typedef struct {
Etat etat_suivant
Action action;
} Transition;



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
  // lecture_capteur_obstacle();

}
void action_tourner(void) { /*TOURNER*/  }
void action_avancer(void) { pilotage_moteur(1,60,0,60); }

void distance(float target_distance){ //distance en cm
  float actual_distance =  0;
  
  if (capt_opto_d > capt_opto_g){
    actual_distance = capt_opto_d * 0.5;//1 tic = 0.5 cm
  }else{
    actual_distance = capt_opto_g * 0.5;//1 tic = 0.5 cm
  } 

  if(actual_distance >= target_distance ){
    pilotage_moteur(1,0,0,0);//Vitesse des moteurs à 0 
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

    // capteur infrarouge
  P1DIR &= ~BIT1; // P1.7 en entrée
  P1SEL &= ~BIT1; // selection fonction TA1.2
  P1SEL2 &= ~BIT1; // selection fonction TA1.
  
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

  P1DIR |= (BIT0 | BIT6); // P2.5 et P2. en sortie


  P2OUT |= (BIT5);// sens
  P2OUT &= ~(BIT1);// sens
    P2IFG &= ~BIT0;
    P2IFG &= ~BIT3;

}

void lecture_capteur_obstacle()
{
    
    ADC_Demarrer_conversion(1);
    valeur_capt = ADC_Lire_resultat();
    Aff_valeur(convert_Hex_Dec(valeur_capt));

}

void obstacle_capteur(){
  if (valeur_capt >= 600) {
    pilotage_moteur(1,0,0,0);
  }
  else {
    pilotage_moteur(1,60,0,60);
  }
}

Transition table_transition[NB_ETAT][NB_OBST] = {
        [ETAT_AVANCER] = {
            [CAPT_ON] = {ETAT_TOURNER, action_tourner},
            [CAPT_OFF] = {ETAT_AVANCER, action_avancer}
        },
        [ETAT_TOURNER] = {
            [CAPT_ON] = {ETAT_TOURNER, action_tourner},
            [CAPT_OFF] = {ETAT_AVANCER, action_avancer}
        }
};


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

int main(void) {
  volatile unsigned int i;

  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
  CAPT_OBST capt= CAPT_OFF;
  ETAT etat = ETAT_AVANCER;
  Transition trs;

  BCSCTL1= CALBC1_1MHZ; //frequence d’horloge 1MHz
  DCOCTL= CALDCO_1MHZ; // "

  P2DIR &= ~(BIT0 | BIT3); // P1.1 en entree
  P2SEL |= (BIT0 | BIT3); // fonction entree capture sur P1.1 (TA0.CCI0A)
  P2SEL2 &= ~(BIT0 | BIT3);

  TA1CCTL0 |= CM_0 | CCIS_0; // front montant + CCI0A
  TA1CCTL0 |= CAP | CCIE; // mode capture + autorisation interruption

  init_moteur();
  pilotage_moteur(1,60,0,60);
  ADC_init();
  Aff_Init();

  __enable_interrupt();
  while (1){
    lecture_capteur_obstacle();
    obstacle_capteur();
    trs = table_transition[etat][capt];
    trs.action();
    etat = trs.etat_suivant;
  }
}

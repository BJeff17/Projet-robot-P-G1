#include "ADC.h"
#include "Afficheur.h"
#include <msp430.h>

#define FREQ_MOT (992)
#define NB_OBST (2)
#define NB_ETAT (2)

volatile unsigned int capt_opto_g = 0;
volatile unsigned int capt_opto_d = 0;
volatile int diff_capt = 0 ;
static int valeur_capt = 0;



#define correction_rg (0)
#define correction_rd (0)

typedef enum {CAPT_OFF, CAPT_ON} CAPT_OBST; //capteur d'obstacle
typedef enum {ETAT_AVANCER, ETAT_TOURNER} ETAT;
typedef void (*Action)(void);  

typedef struct {
Etat etat_suivant;
CAPT_OBST capt;
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

}

void action_tourner(void) { /*TOURNER*/  }
void action_avancer(void) { pilotage_moteur(1,60,0,60); }

void distance(int target_distance){ //fonction qui arrete le robot après une certaine distance en cm passée en paramètre 
  int actual_distance =  0;
  
  if (capt_opto_d > capt_opto_g){ // On prend l'opto avec la plus grande valeur 
    actual_distance = capt_opto_d/12;//1 tic = 0.5 cm
  }else{
    actual_distance = capt_opto_g /12;//1 tic = 0.5 cm
  } 

  if(actual_distance >= target_distance ){// Si la distance cible est atteinte alors on coupe les moteurs
    arret_moteur();
  }
}

void arret_moteur(void){ 
    pilotage_moteur(0,0,0,0);
    TA0CTL &= ~TAIE;
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

void lecture_capteur_obstacle() // Lecture de la valeur du capteur infrarouge 
{
    ADC_Demarrer_conversion(1);
    valeur_capt = ADC_Lire_resultat();
    Aff_valeur(convert_Hex_Dec(valeur_capt)); // Affichage de la valeur sur l'afficheur 
}

void obstacle_capteur(CAPT_OBST *capt){ // Le robot s'arrete s'il rencontre un obstacle à X cm 
  if (valeur_capt >= 600) {
    arret_moteur();
    *capt = CAPT_ON;
  }
  else {
    *capt = CAPT_OFF;
    // pilotage_moteur(1,60,0,60); -> Pas obligé si machine d'état ? 
  }
}

Transition table_transition[NB_ETAT][NB_OBST] = {
        [ETAT_AVANCER] = {
            [CAPT_ON] = {ETAT_TOURNER, action_tourner}, // Si obstacle alors le robot tourne 
            [CAPT_OFF] = {ETAT_AVANCER, action_avancer} // Sinon il continue d'avancer 
        },
        [ETAT_TOURNER] = {
            [CAPT_ON] = {ETAT_TOURNER, action_tourner}, // Obstacle alors le robot tourne à nouveau
            [CAPT_OFF] = {ETAT_AVANCER, action_avancer} // Pas d'obstacle alors il peut avancer 
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
  ETAT etat = ETAT_AVANCER; //initialisation de l'état
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
    distance(100);
    lecture_capteur_obstacle(); //Lecture de la valeur 
    obstacle_capteur(&capt); //s'arrete si un obstacle est detecté 
    trs = table_transition[etat][capt];
    trs.action();
    etat = trs.etat_suivant;
  }
}


#include "ADC.h"
#include "Afficheur.h"
#include <msp430.h>

//#define ROBOT_FAST
#define ROBOT_SLOW

#if defined(ROBOT_FAST)
  #define FREQ_MOT (992)
  #define BASE_SPEED (30)
  #define GAIN (2)
  #define CORR_MAX (15)
  #define OPTO_D_OK (0)   // capteur droit mort
  // #define CORR_STATIC_G (3)
  // #define CORR_STATIC_D (0)
  #define DISTANCE (130)
  #define DIST_CAPT_INFRA (485)
  #define CAPT_INFRA_OK (1)
#elif defined(ROBOT_SLOW)
  #define FREQ_MOT (992)
  #define BASE_SPEED (80)
  #define GAIN (2)
  #define CORR_MAX (15)
  #define OPTO_D_OK (0)
  // #define CORR_STATIC_G (0)
  // #define CORR_STATIC_D (0)
  #define DISTANCE (130)
  #define DIST_CAPT_INFRA (485)
  #define CAPT_INFRA_OK (1)
#else
  #error "ROBOT_FAST ou ROBOT_SLOW seulement"
#endif
int CORR_STATIC_G = 0;
int CORR_STATIC_D = 0;
volatile unsigned int capt_opto_g = 0;
volatile unsigned int capt_opto_d = 0;
volatile int diff_capt = 0 ;

volatile int correction = 0;
volatile unsigned char flag_correction = 0;
volatile unsigned char correction_active = 1;
static int valeur_capt = 0;

/* Temps */
volatile unsigned int compteur_ms = 0;
volatile unsigned int secondes = 0;


typedef enum {CAPT_OFF, CAPT_ON, DIST_MAX, NB_OBST} CAPT_OBST; //capteur d'obstacle
typedef enum {ETAT_AVANCER, ETAT_TOURNER, ETAT_ARRET, NB_ETAT} ETAT;
typedef enum {MODE_HOMOL, MODE_DANSE,NB_MODE} MODE;
typedef void (*Action)(void);  

CAPT_OBST capt= CAPT_OFF;
ETAT etat = ETAT_AVANCER; //initialisation de l'état
MODE mode = MODE_HOMOL;

typedef struct {
    ETAT etat_suivant;
    Action action;
} Transition; 

void action_tourner(void) { /*TOURNER*/  }
void action_avancer(void) { correction_active = 1; pilotage_moteur(1, BASE_SPEED - correction - CORR_STATIC_G, 0, BASE_SPEED + correction - CORR_STATIC_D); }
void action_arret(void)   { arret_moteur();}


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
  if(capt != DIST_MAX){
    lecture_capteur_obstacle();
    capt = obstacle_capteur();
  }
  if (capt == CAPT_OFF || capt == DIST_MAX){
    compteur_ms+= 100;
  }
  else{
    arret_moteur();
  }
  if (compteur_ms == 1000) {
    secondes++;
    compteur_ms = 0;
  }
  if (secondes == 3 && secondes == 4){
    CORR_STATIC_D = 1;
  }
  else{
    CORR_STATIC_D = 0;
  }
  flag_correction = 1 ;
  
}
void affiche_second(){
  Aff_valeur(convert_Hex_Dec(secondes));
}

void arret_moteur(void){
    correction_active = 0;
    TA1CCR2 = 0;
    TA1CCR1 = 0;
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

  // capteur infrarouge
  P1DIR &= ~BIT1; // P1.7 en entrée
  P1SEL &= ~BIT1; // selection fonction TA1.2
  P1SEL2 &= ~BIT1; // selection fonction TA1.
  
  P2IE |= (BIT0 | BIT3); 
  P2IES |= (BIT0 | BIT3); 

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

  if(correction > CORR_MAX)  correction = CORR_MAX;
  if(correction < -CORR_MAX) correction = -CORR_MAX;
#endif
  // OPTO_D_OK = 0 = capteur opto mort
  pilotage_moteur(1, BASE_SPEED - correction - CORR_STATIC_G, 0, BASE_SPEED + correction - CORR_STATIC_D);
}
void distance(float target_distance){ //distance en cm
  float actual_distance =  0;
#if OPTO_D_OK
  if (capt_opto_d > capt_opto_g){
    actual_distance =13.4*((float)(capt_opto_d) /24.0);//1 tic = 0.5 cm
  }{
    actual_distance =13.4*((float)(capt_opto_g) /24.0);//1 tic = 0.5 cm
  }
#else
  actual_distance =13.5*((float)(capt_opto_g) /24.0);//1 tic = 0.5 cm
#endif
  if(actual_distance >= target_distance ){
     capt = DIST_MAX;
  }
}

void lecture_capteur_obstacle() // Lecture de la valeur du capteur infrarouge 
{
    {
    ADC_Demarrer_conversion(1);

    valeur_capt = ADC_Lire_resultat();
    }
}
/* DETERMINER ETAT CAPTEUR */
CAPT_OBST obstacle_capteur(void)
{
  if(CAPT_INFRA_OK){
    if (valeur_capt >= 485) //&& valeur_capt <= 900 )
    {
      if(capt != DIST_MAX) return CAPT_ON;
      else return DIST_MAX;
    }
    else
    {
      if(capt != DIST_MAX) return CAPT_OFF;
      else return DIST_MAX;
    }
  }
  else{
    if(capt != DIST_MAX) return CAPT_OFF;
  }
}

void phare_robot(){
    // led en fonction de la luminosité
    ADC_Demarrer_conversion(2);
    int lum_hex = ADC_Lire_resultat(); 
    if (lum_hex <= 600) {
      P1OUT |= (BIT0 | BIT6);
    }else {
      P1OUT &= ~(BIT0 | BIT6);
    }
}
Transition table_transition[NB_MODE][NB_ETAT][NB_OBST]= {
    [MODE_HOMOL] = {
        [ETAT_AVANCER] = {
        [DIST_MAX] = {ETAT_ARRET, action_arret}, // Si la distance max est atteinte arrêt
        [CAPT_ON] = {ETAT_ARRET, action_arret}, // Si obstacle alors le robot tourne 
        [CAPT_OFF] = {ETAT_AVANCER, action_avancer} // Sinon il continue d'avancer 
        
      },
      [ETAT_ARRET] = {
        [DIST_MAX] = {ETAT_ARRET, action_arret}, // Si la distance max est atteinte arrêt 
        [CAPT_ON] = {ETAT_ARRET, action_arret}, // Obstacle alors le robot tourne à nouveau
        [CAPT_OFF] = {ETAT_AVANCER, action_avancer} // Pas d'obstacle alors il peut avancer
      }
    },
    [MODE_DANSE] = {
      [ETAT_AVANCER] = {
        [DIST_MAX] = {ETAT_ARRET, action_arret}, // Si la distance max est atteinte arrêt 
        [CAPT_ON] = {ETAT_TOURNER, action_tourner}, // Si obstacle alors le robot tourne 
        [CAPT_OFF] = {ETAT_AVANCER, action_avancer} // Sinon il continue d'avancer 

      },
      [ETAT_TOURNER] = {
        [DIST_MAX] = {ETAT_ARRET, action_arret}, // Si la distance max est atteinte arrêt 
        [CAPT_ON] = {ETAT_TOURNER, action_tourner}, // Obstacle alors le robot tourne à nouveau
        [CAPT_OFF] = {ETAT_AVANCER, action_avancer} // Pas d'obstacle alors il peut avancer 
      }
    }     
};
int main(void) {
  volatile unsigned int i;
  
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
  
  CAPT_OBST capt= CAPT_OFF;
  ETAT etat = ETAT_AVANCER; //initialisation de l'état
  MODE mode = MODE_HOMOL;
  Transition trs;

  BCSCTL1= CALBC1_1MHZ; 
  DCOCTL= CALDCO_1MHZ; 

  P2DIR &= ~(BIT0 | BIT3); 
  P2SEL |= (BIT0 | BIT3); 
  P2SEL2 &= ~(BIT0 | BIT3);

  TA1CCTL0 |= CM_0 | CCIS_0; // front montant + CCI0A
  TA1CCTL0 |= CAP | CCIE; // mode capture + autorisation interruption

  /* Allumage des phares */
  unsigned int lum_hex = 0;
  P1SEL &= ~(BIT0 | BIT6);
  P1SEL2 &= ~(BIT0 | BIT6);

  P1DIR |= (BIT0 |BIT6);

  P1OUT &= ~(BIT0);
  P1OUT &= ~(BIT6);

  config_timer_correction(TASSEL_2, ID_1, MC_1, 49999);
  TA0CCTL0 = CCIE;         

  ADC_init();
  Aff_Init();
  
  init_moteur();
  pilotage_moteur(1, BASE_SPEED - CORR_STATIC_G, 0, BASE_SPEED - CORR_STATIC_D);

  __enable_interrupt();
  while (1) {

    // correction avance ligne droite
    if(flag_correction){
      flag_correction = 0;
      if(correction_active){
         corriger_trajectoire();
      }
    }
    // commande 
    //machine d'etat
    lecture_capteur_obstacle();
    capt = obstacle_capteur();
    distance(130.0);
    trs = table_transition[mode][etat][capt];
    trs.action();
    etat = trs.etat_suivant;

    phare_robot();
    affiche_second();
  }
}

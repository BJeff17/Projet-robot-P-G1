
#include "ADC.h"
#include "Afficheur.h"
#include <msp430.h>
//-------selection du robot------------- 
//#define ROBOT_FAST
#define ROBOT_SLOW
//----define different en fonction du robot
#if defined(ROBOT_FAST)
  #define FREQ_MOT (992) // fréquence à 15 khz pour la PWM
  #define BASE_SPEED (30) // vitesse des roues
  #define GAIN (2) // multiplie l'erreur entre les roues pour une plus grande corretion
  #define CORR_MAX (15) // maximum de correction active autorisé 
  #define OPTO_D_OK (0) // capteur opto droit fonctionne = 1 sinon 0 (change la prise en compte du capteur dans le programme)
  // #define CORR_STATIC_G (3) // correction static appliquer en moins sur la roue G en % (finalement enlevé pour réussir l'homologation :explication dans le readme.md)
  // #define CORR_STATIC_D (0) // correction static appliquer en moins sur la roue D en %
  #define DISTANCE (130) // distance que le robot doit parcourir
  #define DIST_CAPT_INFRA (485) // distance de captation du capteur infra
  #define CAPT_INFRA_OK (1) // capteur infra droit fonctionne = 1 sinon 0 (change la prise en compte du capteur dans le programme)
#elif defined(ROBOT_SLOW)
  // meme definition de define que pour le robot fast
  #define FREQ_MOT (992)
  #define BASE_SPEED (80) 
  #define GAIN (2)
  #define CORR_MAX (15)
  #define OPTO_D_OK (0)
  // #define CORR_STATIC_G (0)
  // #define CORR_STATIC_D (0)
  #define DISTANCE (130.0)
  #define DIST_CAPT_INFRA (485)
  #define CAPT_INFRA_OK (1)
#else 
  #error "ROBOT_FAST ou ROBOT_SLOW seulement" 
#endif
//-----------variable globale----------------------------------
int CORR_STATIC_G = 0; // correction static des moteurs mis en variable globale pour pouvoir être modifier dans le programme (changement de dernière minute expliquer dans le readme.md)
int CORR_STATIC_D = 0; // correction en % appliquer en moins sur le moteur D

volatile unsigned int capt_opto_g = 0; //comptage des activation/désactivation capteur opto
volatile unsigned int capt_opto_d = 0;
volatile int diff_capt = 0 ; // soustraction des deux capteur opto utiliser dans la correction active

volatile int correction = 0; // utiliser dans la correction active
volatile unsigned char flag_correction = 0; // variable utilisé pour savoir si le programme est rentré dans l'interruption du TIMER A0
volatile unsigned char correction_active = 1; // vaut 1 sauf quand la focntion arret_moteur est appelé, permet d'arrêté la correction action quand un arrêt est demandé
static int valeur_capt = 0; // valeur brut du capteur infra

/* Temps */
volatile unsigned int compteur_ms = 0; // de 0 à 9
volatile unsigned int secondes = 0;

// Pour la machine d'etat
typedef enum {CAPT_OFF, CAPT_ON, DIST_MAX, NB_OBST} CAPT_OBST; //capteur d'obstacle
typedef enum {ETAT_AVANCER, ETAT_TOURNER, ETAT_ARRET, NB_ETAT} ETAT;
typedef enum {MODE_HOMOL, MODE_DANSE,NB_MODE} MODE;
typedef void (*Action)(void);  

// initialisation des variable servant à la machine d'etat
CAPT_OBST capt= CAPT_OFF;
ETAT etat = ETAT_AVANCER; //initialisation de l'état
MODE mode = MODE_HOMOL;

typedef struct {
    ETAT etat_suivant;
    Action action;
} Transition; 

// action noramlement utilisé dans la machine d'etat (incomplet, expliquer dans le readme.md)
void action_tourner(void) { /*TOURNER*/  }
void action_avancer(void) { correction_active = 1; pilotage_moteur(1, BASE_SPEED - correction - CORR_STATIC_G, 0, BASE_SPEED + correction - CORR_STATIC_D); }
void action_arret(void)   { arret_moteur();}

// interruption en mode entré sur les capteurs opto
#pragma vector=PORT2_VECTOR
__interrupt void capture_opto(void)
{ 
  if((P2IFG & BIT0) == BIT0){ // lorsque le flag du capt opto G est activé
      capt_opto_g++; // incrementation de la variable globale G
      P2IES ^= (BIT0); // inversion front d'activation (Front montant / Front descendant)
    P2IFG &= ~BIT0; // abaissement du flag G
  }
  if((P2IFG & BIT3) == BIT3){ // lorsque le flag du capt opto D est activé
      capt_opto_d++;// incrementation de la variable globale D
      P2IES ^= (BIT3); // inversion front d'activation (Front montant / Front descendant)
    P2IFG &= ~BIT3; //abaissement du flag D
  }
}
// TIMER A0, s'active toute les 100ms
#pragma vector=TIMER0_A0_VECTOR
__interrupt void timer_correction(void)
{ 
  if(capt != DIST_MAX){ // si event n'est pas egal à l'evenement distance max
    lecture_capteur_obstacle(); // je continue à lire les valeurs du capteur infra
    capt = obstacle_capteur();
  }
  if (capt == CAPT_OFF || capt == DIST_MAX){ // si aucun obstacle n'est détecté je continue à incrémenter le temps
    compteur_ms+= 100;
  }
  else{ // sinon je m'arrête car un obstacle est détecté
    arret_moteur();
  }
  // gestion du temps (pour l'afficheur)
  if (compteur_ms == 1000) {
    secondes++;
    compteur_ms = 0;
  }

  // correction fait main de dernière minute (explication dans le readme.md)
  if (secondes == 3 || secondes == 4){ // correction pendant 2s
    CORR_STATIC_D = 1; // 1% en moins sur le moteur gauche
  }
  else{
    CORR_STATIC_D = 0;
  }
  flag_correction = 1; // variable utilisé pour savoir si le programme est rentré dans l'interruption du TIMER A0
  
}
//Fonction pour l'affichage des seconds
void affiche_second(){
  Aff_valeur(convert_Hex_Dec(secondes));
}
//Fonctin d'arrêt moteur 
void arret_moteur(void){
    correction_active = 0; // arrête la correction active
    TA1CCR2 = 0; // arrêt moteur D (utilisation de TA1CCR2 car moins gourmand en ressource que pilotage_moteur)
    TA1CCR1 = 0; // arrêt moteur G (utilisation de TA1CCR1 car moins gourmand en ressource que pilotage_moteur)
}
// initalisation des registres pour l'utilisation des moteurs
void init_moteur(){

  // opto gauche
  P2DIR &= ~BIT0; // P2.0 en entrée
  P2SEL &= ~BIT0; 
  P2SEL2 &= ~BIT0; 

  //opto droit
  P2DIR &= ~BIT3; //P2.3 en entrée
  P2SEL &= ~BIT3; 
  P2SEL2 &= ~BIT3; 
  
#if OPTO_D_OK // les deux capteurs opto fonctionnent
  P2IE |= (BIT0 | BIT3);   
  P2IES |= (BIT0 | BIT3);
#else //seulement le capteur opto G marche
  P2IE |= BIT0; 
  P2IES |= BIT0;          
#endif

  // Initialisation du  capteur infrarouge
  P1DIR &= ~BIT1; // P1.1 en entrée
  P1SEL &= ~BIT1; //
  P1SEL2 &= ~BIT1; // 

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
  TA1CTL = 0 |TASSEL_2 | MC_1 | ID_0 | TACLR;  // 1MHz, mode up, prediv à 1 , Timer A counter clear
  TA1CCTL2 |= OUTMOD_7; //PWM output mode: 7 - PWM reset/set
  TA1CCTL1 |= OUTMOD_7; //PWM output mode: 7 - PWM reset/set

  // Config PWM
  TA1CCR0 = FREQ_MOT; // Fréquence PWM
  TA1CCR2 = 0; // ratio PWM moteur D
  TA1CCR1 = 0; // ratio PWM moteur G

  P2OUT |= (BIT5);// sens avant 
  P2OUT &= ~(BIT1);// sens avant 

  P2IFG &= ~BIT0; //flag à 0
  P2IFG &= ~BIT3; //flag à 0

}

void pilotage_moteur(int sens_g, int puissance_g, int sens_d, int puissance_d){
  // Gestion du sens
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
// gestion de la puissance des moteurs
TA1CCR2 = (unsigned int)(((unsigned long)FREQ_MOT * puissance_d) / 100);
TA1CCR1 = (unsigned int)(((unsigned long)FREQ_MOT * puissance_g) / 100);
}
// Config TIMER A0
void config_timer_correction(unsigned int selec_horloge, unsigned int prediv,unsigned int mode_comptage, unsigned int max_comptage){

  TA0CTL = 0|(selec_horloge | prediv); //source SMCLK, pas de predivision ID_0
  TA0CCR0 = max_comptage; //voir diapo précédente
  TA0CTL |= mode_comptage; //arrêt du comptage
}
// Fonction de correction de trajectoire (mis à l'écart car elle ne focntionnait pas bien)
void corriger_trajectoire(void){
#if OPTO_D_OK // si le capteur opto droit marche
  static unsigned int temp_capt_g = 0, temp_capt_d = 0;
  int dg = capt_opto_g - temp_capt_g; // différence gauche entre l'ancienne valeur
  int dd = capt_opto_d - temp_capt_d; // différence droite entre l'ancienne valeur
  temp_capt_g = capt_opto_g; 
  temp_capt_d = capt_opto_d;

  int erreur = dg - dd; // différence entre les deux différences (si erreur(+) alors roue gauche à parcouru plus de disctance)
  correction += erreur * GAIN; 

  if(correction > CORR_MAX)  correction = CORR_MAX; // maximum de correction
  if(correction < -CORR_MAX) correction = -CORR_MAX;
#endif // sinon aucune correction effecté
  pilotage_moteur(1, BASE_SPEED - correction - CORR_STATIC_G, 0, BASE_SPEED + correction - CORR_STATIC_D); // correction appliqué
}

// Fonction de calcul de distance  
void distance(float target_distance){ //distance en cm
  float actual_distance =  0;
#if OPTO_D_OK
  if (capt_opto_d > capt_opto_g){
    // la valeur 13.4 à été réglé à la main pour une plus grand précision dans l'estimation de la distance
    actual_distance =13.4*((float)(capt_opto_d) /24.0);//1 tic = 0.5 cm  
  }{
    actual_distance =13.4*((float)(capt_opto_g) /24.0);//1 tic = 0.5 cm
  }
#else
  // la valeur 13.4 à été réglé à la main pour une plus grand précision dans l'estimation de la distance  
  actual_distance =13.5*((float)(capt_opto_g) /24.0);//1 tic = 0.5 cm
#endif
  if(actual_distance >= target_distance ){
     capt = DIST_MAX; // déclanche l'evennement Distance max parcourue 
  }
}

// Fonction de lecture de la valeur du capteur infrarouge
void lecture_capteur_obstacle()  
{
  ADC_Demarrer_conversion(1);
  valeur_capt = ADC_Lire_resultat();
}
// fonction renvoyant l'etat du capteur infrarouge ()
CAPT_OBST obstacle_capteur(void)
{
  if(CAPT_INFRA_OK){// si le capteur marche
    if (valeur_capt >= DIST_CAPT_INFRA)// si le capteur detecte un obstacle
    { 
      // sécurité sur l'evennement DIST_MAX prioritaire
      if(capt != DIST_MAX) return CAPT_ON; // event retourné CAPT_ON
      else return DIST_MAX;
    }
    else
    { 
      // sécurité sur l'evennement DIST_MAX prioritaire
      if(capt != DIST_MAX) return CAPT_OFF; // event retourné CAPT_OFF
    }
  }
  else{// si le capteur ne marche pas
    if(capt != DIST_MAX) return CAPT_OFF;
  }
}
// gestion des phares
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
// table de transition (inachevé pour le mode DANSE car ce programme fait seulement l'homologation)
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

  Transition trs;

  BCSCTL1= CALBC1_1MHZ; 
  DCOCTL= CALDCO_1MHZ; 

  // capture pour les capteurs opto
  TA1CCTL0 |= CM_0 | CCIS_0; // front montant + CCI0A
  TA1CCTL0 |= CAP | CCIE; // mode capture + autorisation interruption

  /* phares */
  P1SEL &= ~(BIT0 | BIT6);
  P1SEL2 &= ~(BIT0 | BIT6);

  P1DIR |= (BIT0 |BIT6);

  P1OUT &= ~(BIT0);
  P1OUT &= ~(BIT6);

  // INIT
  config_timer_correction(TASSEL_2, ID_1, MC_1, 49999); // config pour 100ms
  TA0CCTL0 = CCIE;         

  ADC_init();
  Aff_Init();
  
  init_moteur();
  pilotage_moteur(1, BASE_SPEED - CORR_STATIC_G, 0, BASE_SPEED - CORR_STATIC_D);

  __enable_interrupt();

  while (1) {

    //------- correction --------
    if(flag_correction){// à chaque intéruption 
      flag_correction = 0; //RAZ
      if(correction_active){ // sécurité (est à zéro seulement quand la fonction arrêter_moteur est appelé)
         corriger_trajectoire();
      }
    }
    //-------machine d'etat-----------
    lecture_capteur_obstacle();
    capt = obstacle_capteur(); // recup de l'event
    distance(DISTANCE); 
    trs = table_transition[mode][etat][capt];
    trs.action();
    etat = trs.etat_suivant;

    // phares et afficheur
    phare_robot();
    affiche_second();
  }
}

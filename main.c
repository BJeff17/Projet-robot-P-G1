#include "ADC.h"
#include "Afficheur.h"
#include <msp430.h>


//#define ROBOT_FAST
#define ROBOT_FAST


#if defined(ROBOT_FAST)

  #define FREQ_MOT (992)
  #define BASE_SPEED (30)
  #define GAIN (2)
  #define CORR_MAX (15)

  #define OPTO_D_OK (0)   // capteur droit mort

  #define CORR_STATIC_G (3)
  #define CORR_STATIC_D (0)

  #define DISTANCE (130)
  #define DIST_CAPT_INFRA (485)


#elif defined(ROBOT_SLOW)

  #define FREQ_MOT (992)
  #define BASE_SPEED (80)
  #define GAIN (1)
  #define CORR_MAX (15)

  #define OPTO_D_OK (0)

  #define CORR_STATIC_G (0)
  #define CORR_STATIC_D (1)

  #define DISTANCE (130)
  #define DIST_CAPT_INFRA (485)


#else

  #error "ROBOT_FAST ou ROBOT_SLOW seulement"

#endif



volatile unsigned int capt_opto_g = 0;
volatile unsigned int capt_opto_d = 0;

volatile int diff_capt = 0;

volatile int correction = 0;

volatile unsigned char flag_correction = 0;
volatile unsigned char correction_active = 1;

static int valeur_capt = 0;

volatile unsigned int compteur_ms = 0;
volatile unsigned int secondes = 0;



typedef enum {CAPT_OFF,CAPT_ON,DIST_MAX,NB_OBST} CAPT_OBST; //capteur d'obstacle
typedef enum {ETAT_AVANCER,ETAT_TOURNER,ETAT_ARRET,NB_ETAT} ETAT;
typedef enum {MODE_HOMOL,MODE_DANSE,NB_MODE} MODE;
typedef void (*Action)(void);



CAPT_OBST capt = CAPT_OFF;
ETAT etat = ETAT_AVANCER; //initialisation de l'état
MODE mode = MODE_DANSE;


typedef struct
{
  ETAT etat_suivant;
  Action action;

} Transition;



void pilotage_moteur(int sens_g,
                     int puissance_g,
                     int sens_d,
                     int puissance_d);

void arret_moteur(void);

void lecture_capteur_obstacle(void);

CAPT_OBST obstacle_capteur(void);

void corriger_trajectoire(void);

void phare_robot(void);

void affiche_second(void);

void attendre_ms(unsigned int duree);



void action_tourner(void)
{
  correction_active = 0;

  pilotage_moteur(1, 60,
                  1, 60);
}


void action_avancer(void)
{
  correction_active = 1;

  pilotage_moteur(1,
                  BASE_SPEED - CORR_STATIC_G,
                  0,
                  BASE_SPEED - CORR_STATIC_D);
}


void action_arret(void)
{
  arret_moteur();
}


#pragma vector=PORT2_VECTOR
__interrupt void capture_opto(void)
{
  if ((P2IFG & BIT0) == BIT0)
  {
    capt_opto_g++;

    P2IES ^= BIT0;

    P2IFG &= ~BIT0;
  }


#if OPTO_D_OK

  if ((P2IFG & BIT3) == BIT3)
  {
    capt_opto_d++;

    P2IES ^= BIT3;

    P2IFG &= ~BIT3;
  }

#endif
}

//Timer
#pragma vector=TIMER0_A0_VECTOR
__interrupt void timer_correction(void)
{
  if (capt != CAPT_ON)
  {
    compteur_ms += 100;
  }

  if (compteur_ms >= 1000)
  {
    secondes++;
    compteur_ms = 0;
  }
  flag_correction = 1;
}



void attendre_ms(unsigned int duree)
{
  unsigned int i;

  for (i = 0; i < duree; i++)
  {
    __delay_cycles(1000);
  }
}


void affiche_second(void)
{
  Aff_valeur(convert_Hex_Dec(secondes));
}



void arret_moteur(void)
{
  correction_active = 0;
  TA1CCR2 = 0;
  TA1CCR1 = 0;
}



void init_moteur(void)
{
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

  P2IE |= BIT0; // capteur opto droit mort

  P2IES |= BIT0;

#endif


  // capteur infrarouge

  P1DIR &= ~BIT1;

  P1SEL &= ~BIT1;
  P1SEL2 &= ~BIT1;


  //config sens gauche et droit

  P2DIR |= (BIT1 | BIT5);

  P2SEL &= ~(BIT1 | BIT5);
  P2SEL2 &= ~(BIT1 | BIT5);


  // moteur gauche

  P2DIR |= BIT2;

  P2SEL |= BIT2;
  P2SEL2 &= ~BIT2;


  // moteur droit

  P2DIR |= BIT4;

  P2SEL |= BIT4;
  P2SEL2 &= ~BIT4;


  //CONFIG TIMER TA1

  TA1CTL = TASSEL_2 | MC_1 | ID_0 | TACLR;

  TA1CCTL2 = OUTMOD_7;
  TA1CCTL1 = OUTMOD_7;


  // Config PWM

  TA1CCR0 = FREQ_MOT;

  TA1CCR2 = 0;
  TA1CCR1 = 0;


  P2OUT |= BIT5;   // sens
  P2OUT &= ~BIT1;  // sens


  P2IFG &= ~BIT0;
  P2IFG &= ~BIT3;
}



void pilotage_moteur(int sens_g,int puissance_g,int sens_d,int puissance_d)
{
  if (sens_g == 1)
  {
    P2OUT |= BIT5;
  }
  else
  {
    P2OUT &= ~BIT5;
  }


  if (sens_d == 1)
  {
    P2OUT |= BIT1;
  }
  else
  {
    P2OUT &= ~BIT1;
  }


  TA1CCR2 =
      (unsigned int)(((unsigned long)FREQ_MOT * puissance_d) / 100);

  TA1CCR1 =
      (unsigned int)(((unsigned long)FREQ_MOT * puissance_g) / 100);
}

config_timer_correction(unsigned int selec_horloge,unsigned int prediv,unsigned int mode_comptage,unsigned int max_comptage)
{
  TA0CTL = selec_horloge | prediv;
  TA0CCR0 = max_comptage;
  TA0CTL |= mode_comptage;
}


void corriger_trajectoire(void)
{

#if OPTO_D_OK

  static unsigned int temp_capt_g = 0;
  static unsigned int temp_capt_d = 0;

  int dg = capt_opto_g - temp_capt_g;
  int dd = capt_opto_d - temp_capt_d;


  temp_capt_g = capt_opto_g;
  temp_capt_d = capt_opto_d;


  int erreur = dg - dd;

  correction += erreur * GAIN;


  if (correction > CORR_MAX)
  {
    correction = CORR_MAX;
  }


  if (correction < -CORR_MAX)
  {
    correction = -CORR_MAX;
  }

#endif


  // OPTO_D_OK = 0 = capteur opto mort

  pilotage_moteur(
      1,
      BASE_SPEED - correction - CORR_STATIC_G,

      0,
      BASE_SPEED + correction - CORR_STATIC_D
  );
}


void distance(float target_distance)
{
  float actual_distance = 0;


#if OPTO_D_OK

  float distance_g =
      13.5 * ((float)capt_opto_g / 24.0);

  float distance_d =
      13.5 * ((float)capt_opto_d / 24.0);


  actual_distance =
      (distance_g + distance_d) / 2.0;

#else

  actual_distance =
      13.5 * ((float)capt_opto_g / 24.0);

#endif


  if (actual_distance >= target_distance)
  {
    capt = DIST_MAX;
  }
}



void lecture_capteur_obstacle(void)
{
  ADC_Demarrer_conversion(1);

  valeur_capt = ADC_Lire_resultat();
}



CAPT_OBST obstacle_capteur(void)
{
  if (capt == DIST_MAX)
  {
    return DIST_MAX;
  }


  if (valeur_capt >= DIST_CAPT_INFRA)
  {
    return CAPT_ON;
  }


  return CAPT_OFF;
}



//Allumage des phares
void phare_robot(void)
{
  int lum_hex;


  // led en fonction de la luminosité

  ADC_Demarrer_conversion(2);

  lum_hex = ADC_Lire_resultat();


  if (lum_hex <= 600)
  {
    P1OUT |= (BIT0 | BIT6);
  }
  else
  {
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


//Gestion d'obstacle pendant la danse

void attendre_mouvement(unsigned int duree,Action action_reprise)
{
  unsigned int temps = 0;
  while (temps < duree)
  {
    lecture_capteur_obstacle();
    capt = obstacle_capteur();
    phare_robot();
    affiche_second();

    if (capt == CAPT_ON)
    {
      arret_moteur();
      while (capt == CAPT_ON)
      {
        attendre_ms(20);
        lecture_capteur_obstacle();
        capt = obstacle_capteur();
        phare_robot();
        affiche_second();
      }

      if (action_reprise != 0)
      {
        action_reprise();
      }
    }

    else
    {
      attendre_ms(20);

      temps += 20;
    }
  }
}


//distance parcouru depuis un point
float distance_depuis(unsigned int debut_g,unsigned int debut_d)
{

#if OPTO_D_OK

  float distance_g =13.5 *((float)(capt_opto_g - debut_g) / 24.0);
  float distance_d =13.5 *((float)(capt_opto_d - debut_d) / 24.0);
  return (distance_g + distance_d) / 2.0;

#else
  return 13.5 *
         ((float)(capt_opto_g - debut_g) / 24.0);
#endif
}


void avancer_distance(float target_distance)
{
  unsigned int debut_g = capt_opto_g;
  unsigned int debut_d = capt_opto_d;
  correction_active = 1;
  action_avancer();
  while (distance_depuis(debut_g, debut_d) < target_distance)
  {
    lecture_capteur_obstacle();
    capt = obstacle_capteur();
    phare_robot();
    affiche_second();

    if (capt == CAPT_ON)
    {
      arret_moteur();

      while (capt == CAPT_ON)
      {
        attendre_ms(20);
        lecture_capteur_obstacle();
        capt = obstacle_capteur();
        phare_robot();
        affiche_second();
      }

      action_avancer();
    }

    else
    {
      corriger_trajectoire();
      attendre_ms(20);
    }
  }
  arret_moteur();

  attendre_ms(300);
}


void avancer(void)
{
  correction_active = 1;
  action_avancer();
}


void reculer(void)
{
  correction_active = 0;
  pilotage_moteur(0, 65, 1, 65);
}


void tourner_droite(void)
{
  correction_active = 0;
  pilotage_moteur(1, 65, 1, 65);
}


void tourner_gauche(void)
{
  correction_active = 0; 
  pilotage_moteur(0, 65, 0, 65);
}


void rotation_sur_place(void)
{
  tourner_droite();
  attendre_mouvement(4200,tourner_droite);
  arret_moteur();
  attendre_ms(300);
}


void demi_tour(void)
{
  tourner_droite();
  attendre_mouvement(1500,tourner_droite);
  arret_moteur();
  attendre_ms(300);
}


//courbe du couer 
void courbe_gauche(void)
{
  correction_active = 0;
  pilotage_moteur(1, 35, 0, 65);
}

void courbe_droite(void)
{
  correction_active = 0;
  pilotage_moteur(1, 65, 0, 35);
}


void coeur(void)
{
 
  courbe_gauche();
  attendre_mouvement(650, courbe_gauche);
  arret_moteur();
  attendre_ms(100);
  

  //le creux du coeur
  tourner_droite();
  attendre_mouvement(250, tourner_droite);

  arret_moteur();
  attendre_ms(100);


  courbe_droite();
  attendre_mouvement(650,courbe_droite);

  arret_moteur();
  attendre_ms(100);

  //la pointe
  tourner_gauche();

  attendre_mouvement( 250, tourner_gauche);


  arret_moteur();
  attendre_ms(100);
  avancer();

  attendre_mouvement( 350, avancer);


  arret_moteur();
  attendre_ms(300);
}


//Les variables pour le twerking 
void recul_gauche(void)
{
  correction_active = 0;
  pilotage_moteur( 0, 70, 1, 10);
}


void recul_droite(void)
{
  correction_active = 0;
  pilotage_moteur(0, 10, 1, 70);
}


void twerk_recul(void)
{
  unsigned int i;

  for (i = 0; i < 12; i++)
  {
    recul_gauche();

    attendre_mouvement( 50, recul_gauche);


    recul_droite();
    attendre_mouvement( 50, recul_droite);
  }

  arret_moteur();
  attendre_ms(300);
}


//Dernier pose (on fait clignoter les phares)
void pose_finale(void)
{
  unsigned int i;
  arret_moteur();

  for (i = 0; i < 3; i++)
  {
    P1OUT |= (BIT0 | BIT6);

    attendre_ms(300);


    P1OUT &= ~(BIT0 | BIT6);

    attendre_ms(300);
  }

  P1OUT |= (BIT0 | BIT6);
}



//La choregraphie
void choregraphie(void)
{
  //Départ attandre 0.5secondes
  P1OUT |= (BIT0 | BIT6);
  attendre_ms(500);


    //aLLER VERS LE CENTRE
  avancer_distance(80);

    //On tourne sur place
  rotation_sur_place();

    //on fait le couer 
  coeur();


  //petit twerk en reculant
  twerk_recul();


    //on va de lautre coté
  avancer_distance(55);

  //deuxième twerk
  twerk_recul();


  demi_tour();

    //revenir vers le centre
  avancer_distance(55);


    //on fait le couer 
  coeur();


    //petite rotation final
  tourner_gauche();


  attendre_mouvement( 500, tourner_gauche);

  //On tourne sur place
  rotation_sur_place();

  twerk_recul();

  rotation_sur_place();
    //bye bye 
  arret_moteur();
  attendre_ms(300);

  pose_finale();
}




int main(void)
{
  Transition trs;


  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer


  BCSCTL1 = CALBC1_1MHZ;

  DCOCTL = CALDCO_1MHZ;



  /* Allumage des phares */

  P1SEL &= ~(BIT0 | BIT6);
  P1SEL2 &= ~(BIT0 | BIT6);


  P1DIR |= (BIT0 | BIT6);


  P1OUT &= ~BIT0;
  P1OUT &= ~BIT6;



  /* Timer */

  config_timer_correction( TASSEL_2, ID_1, MC_1, 49999);


  TA0CCTL0 = CCIE;

  ADC_init();

  Aff_Init();

  init_moteur();



  __enable_interrupt();



 //mode homologation
  if (mode == MODE_HOMOL)
  {
    action_avancer();


    while (1)
    {
      // correction avance ligne droite

      if (flag_correction)
      {
        flag_correction = 0;


        if (correction_active)
        {
          corriger_trajectoire();
        }
      }


      //machine d'etat

      lecture_capteur_obstacle();
      capt = obstacle_capteur();

      distance(DISTANCE);


      trs = table_transition [mode] [etat] [capt];
      trs.action();
      etat = trs.etat_suivant;

      phare_robot();
      affiche_second();
    }
  }



//mode choregraphie 
  else
  {
    secondes = 0;
    compteur_ms = 0;

    choregraphie();
    arret_moteur();

    while (1)
    {
      phare_robot();
      affiche_second();
    }
  }
}
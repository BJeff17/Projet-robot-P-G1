# Chorégraphie

## 1. Objectif

L’objectif de cette partie du projet est de faire réaliser une chorégraphie au robot.

Pour construire cette chorégraphie, nous avons choisi de ne pas mettre tous les mouvements directement dans le `main`.

Nous avons d’abord créé plusieurs fonctions simples pour contrôler les moteurs, puis nous avons créé des fonctions plus complexes à partir de ces mouvements. Enfin, toutes les étapes ont été regroupées dans une seule fonction appelée :

```c
choregraphie();
```

Cette organisation permet de tester les mouvements séparément et de modifier facilement une étape sans devoir changer toute la chorégraphie.


## 2. Fonction de base : pilotage des moteurs

La fonction principale utilisée pour commander les moteurs est :

```c
pilotage_moteur(sens_g, puissance_g, sens_d, puissance_d);
```

Elle permet de choisir :
- le sens du moteur gauche ;
- la puissance du moteur gauche ;
- le sens du moteur droit ;
- la puissance du moteur droit.

Exemple :

```c
pilotage_moteur(1, 65, 0, 65);
```

Les valeurs de puissance sont ensuite transformées en rapport cyclique PWM grâce aux registres `TA1CCR1` et `TA1CCR2`.

Cette fonction sert donc de base pour créer tous les mouvements du robot.



## 3. Mouvements simples

À partir de `pilotage_moteur()`, nous avons créé plusieurs fonctions simples.

### Avancer

```c
void avancer(void)
{
    correction_active = 1;
    action_avancer();
}
```

Cette fonction fait avancer le robot.

La correction de trajectoire est activée afin d’essayer de garder une trajectoire droite.

### Reculer

```c
void reculer(void)
{
    correction_active = 0;
    pilotage_moteur(0, 65, 1, 65);
}
```

Cette fonction fait tourner les deux roues dans le sens inverse afin de faire reculer le robot.

### Tourner à droite

```c
void tourner_droite(void)
{
    correction_active = 0;
    pilotage_moteur(1, 65, 1, 65);
}
```

Cette fonction permet au robot de tourner sur place vers la droite.

### Tourner à gauche

```c
void tourner_gauche(void)
{
    correction_active = 0;
    pilotage_moteur(0, 65, 0, 65);
}
```

Cette fonction permet au robot de tourner sur place vers la gauche.



## 4. Déplacements avec une distance

Pour les grands déplacements, nous avons créé :

```c
avancer_distance(float target_distance);
```

L’objectif est de ne pas avancer uniquement avec un temps fixe, mais d’utiliser les optocoupleurs pour estimer la distance parcourue.

La distance est calculée à partir du nombre d’impulsions des optocoupleurs.

Dans notre code, la conversion utilisée est :

```c
13.5 * ((float)capt_opto_g / 24.0);
```

Cela permet d’obtenir une estimation de la distance parcourue.

Exemple :

```c
avancer_distance(80);
```

Le robot essaye alors d’avancer d’environ 80 cm.

Dans la chorégraphie, cette méthode est utilisée pour aller vers le centre puis vers l’autre côté du terrain.



## 5. Gestion des obstacles pendant un mouvement

Pendant la chorégraphie, il faut continuer à vérifier si un obstacle est présent.

Pour cela, nous avons créé la fonction :

```c
attendre_mouvement(unsigned int duree, Action action_reprise);
```

Cette fonction permet de faire durer un mouvement pendant un certain temps tout en vérifiant régulièrement le capteur d’obstacle.

Exemple :

```c
attendre_mouvement(650, courbe_gauche);
```

Cela signifie que le robot doit continuer le mouvement `courbe_gauche()` pendant environ 650 ms.

Si un obstacle apparaît :
1. le robot s’arrête ;
2. il attend que l’obstacle soit retiré ;
3. il reprend le mouvement qu’il faisait ;
4. le temps restant du mouvement continue ensuite.

Cela permet d’éviter de recommencer toute la chorégraphie lorsqu’un obstacle est détecté.



## 6. Rotation sur place

La rotation complète est réalisée avec :

```c
void rotation_sur_place(void)
{
    tourner_droite();
    attendre_mouvement(4200, tourner_droite);
    arret_moteur();
    attendre_ms(300);
}
```

Le robot commence à tourner à droite, puis il garde ce mouvement pendant une durée déterminée.

La valeur `4200 ms` a été obtenue par réglage sur le robot réel.

À la fin de la rotation, les moteurs sont arrêtés.



## 7. Demi-tour

Le demi-tour utilise le même principe :

```c
void demi_tour(void)
{
    tourner_droite();
    attendre_mouvement(1500, tourner_droite);
    arret_moteur();
    attendre_ms(300);
}
```

La différence est que la durée est plus courte afin d’obtenir environ 180° au lieu d’une rotation complète.



## 8. Création du cœur

Pour créer le cœur, nous avons d’abord créé deux mouvements de courbe.

### Courbe à gauche

```c
void courbe_gauche(void)
{
    correction_active = 0;
    pilotage_moteur(1, 35, 0, 65);
}
```

Le moteur droit tourne plus vite que le moteur gauche. Le robot décrit donc une courbe.

### Courbe à droite

```c
void courbe_droite(void)
{
    correction_active = 0;
    pilotage_moteur(1, 65, 0, 35);
}
```

Cette fois, le moteur gauche tourne plus vite que le moteur droit.

Les deux courbes sont ensuite assemblées dans la fonction :

```c
void coeur(void)
{
    courbe_gauche();
    attendre_mouvement(650, courbe_gauche);
    arret_moteur();
    attendre_ms(100);

    tourner_droite();
    attendre_mouvement(250, tourner_droite);
    arret_moteur();
    attendre_ms(100);

    courbe_droite();
    attendre_mouvement(650, courbe_droite);
    arret_moteur();
    attendre_ms(100);

    tourner_gauche();
    attendre_mouvement(250, tourner_gauche);
    arret_moteur();
    attendre_ms(100);

    avancer();
    attendre_mouvement(350, avancer);

    arret_moteur();
    attendre_ms(300);
}
```

La fonction `coeur()` est donc un assemblage de plusieurs mouvements simples :
- une courbe à gauche ;
- une petite rotation ;
- une courbe à droite ;
- une nouvelle rotation ;
- un petit déplacement en avant.



## 9. Mouvement de twerk

Pour créer un mouvement rapide gauche/droite en reculant, nous avons créé deux fonctions.

### Recul vers la gauche

```c
void recul_gauche(void)
{
    correction_active = 0;
    pilotage_moteur(0, 70, 1, 10);
}
```

### Recul vers la droite

```c
void recul_droite(void)
{
    correction_active = 0;
    pilotage_moteur(0, 10, 1, 70);
}
```

Ensuite, les deux mouvements sont répétés rapidement dans :

```c
void twerk_recul(void)
{
    unsigned int i;

    for (i = 0; i < 12; i++)
    {
        recul_gauche();
        attendre_mouvement(50, recul_gauche);

        recul_droite();
        attendre_mouvement(50, recul_droite);
    }

    arret_moteur();
    attendre_ms(300);
}
```

Le robot alterne donc rapidement entre les deux côtés tout en reculant.



## 10. Phares

Le robot utilise aussi les LED de la MSP430.

La fonction :

```c
phare_robot();
```

lit la luminosité avec l’ADC.

Si la luminosité mesurée est suffisamment faible, les deux LED sont allumées :

```c
P1OUT |= (BIT0 | BIT6);
```

Sinon elles sont éteintes :

```c
P1OUT &= ~(BIT0 | BIT6);
```



## 11. Pose finale

La pose finale sert à marquer la fin de la chorégraphie.

```c
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
```

Le robot s’arrête, puis les deux phares clignotent trois fois avant de rester allumés.



## 12. Assemblage de la chorégraphie

Une fois les fonctions simples et les mouvements plus complexes testés séparément, nous les avons regroupés dans une seule fonction :

```c
void choregraphie(void)
```

La fonction actuelle est organisée de cette manière :

```c
void choregraphie(void)
{
    // Départ
    P1OUT |= (BIT0 | BIT6);
    attendre_ms(500);

    // Aller vers le centre
    avancer_distance(80);

    // Rotation sur place
    rotation_sur_place();

    // Faire un coeur
    coeur();

    // Twerk en reculant
    twerk_recul();

    // Aller vers l'autre côté
    avancer_distance(55);

    // Deuxième twerk
    twerk_recul();

    // Demi-tour
    demi_tour();

    // Revenir vers le centre
    avancer_distance(55);

    // Deuxième coeur
    coeur();

    // Petite rotation
    tourner_gauche();
    attendre_mouvement(500, tourner_gauche);

    // Rotation complète
    rotation_sur_place();

    // Twerk
    twerk_recul();

    // Dernière rotation
    rotation_sur_place();

    // Arrêt
    arret_moteur();
    attendre_ms(300);

    // Pose finale
    pose_finale();
}
```

L’intérêt de cette méthode est que la fonction `choregraphie()` ne contient presque que l’ordre des mouvements.

Les détails de chaque mouvement sont placés dans les fonctions correspondantes.

Par exemple, si nous voulons supprimer le twerk, il suffit d’enlever :

```c
twerk_recul();
```

 dans la fonction `choregraphie()`.

Le reste du programme continue de fonctionner.



## 13. Déroulement général

La chorégraphie suit cette ordre :

1. Allumage des phares et attente au départ.
2. Avance d’environ 80 cm vers le centre.
3. Rotation sur place.
4. Réalisation d’un cœur.
5. Mouvement de twerk en reculant.
6. Avance d’environ 55 cm vers l’autre côté.
7. Deuxième twerk.
8. Demi-tour.
9. Retour d’environ 55 cm.
10. Deuxième cœur.
11. Petite rotation.
12. Rotation sur place.
13. Nouveau twerk.
14. Dernière rotation.
15. Arrêt du robot.
16. Clignotement des phares pour la pose finale.


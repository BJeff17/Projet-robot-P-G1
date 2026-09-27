# Projet Robot Programme d'homologation seulement

Le programme prévoit un asservissement (`corriger_trajectoire()`) qui compare les tics des deux capteurs opto toutes les 100 ms pour corriger la trajectoire.

En pratique, le capteur opto droit ne fonctionne pas correctement. Sans mesure fiable des deux côtés, la différence calculée entre les roues n'a aucun sens, donc l'asservissement est désactivé (`OPTO_D_OK = 0`).

À la place, on utilise une correction statique : un décalage fixe de puissance entre les deux moteurs (`CORR_STATIC_G` / `CORR_STATIC_D`), réglé à la main en testant le robot, pour compenser sa dérive naturelle.

Par manque de temps, et en raison du manque de fiabilité de la correction de trajectoire (batterie, capteurs opto, + fonction sans doute à retravailler), nous avons modifié le programme avant le contrôle afin qu'une correction s'applique sur le moteur gauche pendant 2 s, entre la 3e et la 4e seconde. Nous somme conscient que cette méthode est beaucoups moins élégante mais c'est la seul que nous avons trouvé dans le temps imparti.

Le programme de chorégraphie a lui aussi été réalisé en très peu de temps, c'est pourquoi il n'est pas inclus ici.Par manque de temps, et en raison du manque de fiabilité de la correction de trajectoire (batterie, capteurs opto, fonction probablement à retravailler), nous avons modifié le programme avant le contrôle afin qu'une correction s'applique sur le moteur droit pendant 2 s, entre la 3e et la 4e seconde(ligne 109). Nous sommes conscients que cette méthode est beaucoup moins élégante, mais c'est la seule que nous avons trouvée dans le temps imparti.

Le programme de chorégraphie a lui aussi été réalisé en très peu de temps, c'est pourquoi il n'est pas inclus ici.
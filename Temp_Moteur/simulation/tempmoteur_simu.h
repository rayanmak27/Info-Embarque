#ifndef TEMPMOTEUR_SIMU_H
#define TEMPMOTEUR_SIMU_H

#include "../tempmoteur.h"

// Seulement avec simulation/tempmoteur_simu.c : donne le régime mesuré (tr/min)
// pour que la température simulée suive le moteur. 0 = moteur coupé, il refroidit.
// Sans appel, le moteur est supposé au ralenti.
void tempmoteur_simu_regime(float rpm);

#endif

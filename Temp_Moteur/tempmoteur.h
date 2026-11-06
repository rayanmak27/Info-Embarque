#ifndef TEMPMOTEUR_H
#define TEMPMOTEUR_H

// GPIO à définir
// Capteur à confirmer

int tempmoteur_init(void);
int tempmoteur_read(float *temperature);
void tempmoteur_close(void);

#endif
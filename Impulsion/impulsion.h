#ifndef IMPULSION_H
#define IMPULSION_H

// GPIO27 : impulsions moteur (broche 13)

int impulsion_init(void);
int impulsion_configurer(int impulsions_par_tour);
int impulsion_read(float *rpm);
void impulsion_reset(void);
void impulsion_close(void);

#endif
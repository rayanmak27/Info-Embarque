#ifndef ECRAN_H
#define ECRAN_H

#include "../DataLog/datalog.h"

// GPIO14 : TXD (broche 8)
// UART : 9600 bauds

int ecran_init(void);
int ecran_display(const DonneesVehicule *donnees);
int ecran_text(const char *texte);
int ecran_clear(void);
void ecran_close(void);

#endif
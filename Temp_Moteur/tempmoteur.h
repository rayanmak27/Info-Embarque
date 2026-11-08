#ifndef TEMPMOTEUR_H
#define TEMPMOTEUR_H

// GPIO8  : CS  (broche 24)
// GPIO9  : DO  (broche 21)
// GPIO11 : CLK (broche 23)
// GPIO10 : MOSI (broche 19), non relié : le module n'a pas d'entrée
// VCC : 3,3 V (broche 1 ou 17), jamais 5 V
// Capteur : thermocouple K sur module MAX6675 ou MAX31855
// Lecture au créneau 8 (800 ms) : la puce relance seule sa conversion
// Communication : SPI0 en lecture seule, /dev/spidev0.0 (dtparam=spi=on)

// Codes de retour (0 = OK)
#define TEMPMOTEUR_ERR_SPI -1     // SPI absent ou module pas initialisé
#define TEMPMOTEUR_ERR_SONDE -2   // thermocouple ouvert ou en court-circuit
#define TEMPMOTEUR_ERR_TRAME -3   // module absent ou mauvaise puce choisie
#define TEMPMOTEUR_ERR_ATTENTE -4 // première conversion pas encore finie

int tempmoteur_init(void);
int tempmoteur_read(float *temperature); // en °C
void tempmoteur_close(void);

#endif

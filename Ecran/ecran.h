#ifndef ECRAN_H
#define ECRAN_H
 
#include "../DataLog/datalog.h"
 
// GPIO14 : TXD (broche 8)  -> RX de l'écran
// GPIO15 : RXD (broche 10) <- TX de l'écran (boutons)
// UART : 9600 bauds
 
/* numéros de pages dans Nextion : adaptez-les */
#define PAGE_PRET    0
#define PAGE_GYRO    1     /* page des gX..aZ : À CONFIRMER */
#define PAGE_MESURE  2
#define PAGE_PAUSE   3
 
/* événements envoyés par l'écran (printh A1 FF FF FF, etc.) */
#define EVT_AUCUN  0
#define EVT_START  0xA1
#define EVT_PAUSE  0xA2
#define EVT_FIN    0xA3
 
int ecran_init(void);
int ecran_display(const DonneesVehicule *donnees);  /* 0 si tout est parti, -1 sinon */
int ecran_text(const char *texte);
int ecran_clear(void);
void ecran_close(void);
int ecran_page(int n);
int ecran_read_event(void);        /* EVT_START / EVT_PAUSE / EVT_FIN, 0 si rien */
int ecran_erreurs(void);           /* nombre de réponses 0x1A reçues (composant invalide) */
 
#endif
 
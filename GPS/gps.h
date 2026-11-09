#ifndef __GPS_H__
#define __GPS_H__

typedef struct {
    int    fix;            /* 1 = position valide (RMC statut 'A'), 0 sinon */
    int    qualite;        /* GGA : 0 = pas de fix, 1 = GPS, 2 = DGPS */
    int    satellites;     /* nombre de satellites utilisés */
    double latitude;       /* degrés décimaux, négatif = Sud */
    double longitude;      /* degrés décimaux, négatif = Ouest */
    double altitude_m;     /* altitude au-dessus du niveau de la mer */
    double vitesse_kmh;
    double cap_deg;        /* cap vrai par rapport au Nord */
    double hdop;           /* précision horizontale, plus petit = meilleur */
    char   heure_utc[9];   /* "hh:mm:ss" */
    char   date[11];       /* "jj/mm/aaaa" */
} GpsData;

/* Associe le module au port série déjà ouvert par uart_ouvrir(). */
void gps_init(int uart);

/* Lit tous les octets disponibles sans bloquer, assemble les lignes
 * et décode les trames complètes dans *d.
 * Retourne le nombre de trames valides décodées (0 si rien de nouveau),
 * ou -1 en cas d'erreur de lecture. */
int gps_Read(GpsData *d);

#endif
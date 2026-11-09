/*
 * Tests unitaires du module GPS (décodage NMEA du NEO-6M).
 *
 * Pas besoin du module ni du Raspberry Pi : le port série est remplacé
 * par un "pipe" non bloquant. On écrit des trames d'un côté, gps_Read()
 * les lit de l'autre, exactement comme avec l'UART (VMIN = 0, VTIME = 0).
 *
 * Compilation et lancement (depuis la racine du projet) :
 *     gcc -Wall -Wextra -o test_gps GPS/test_gps.c GPS/gps.c -lm
 *     ./test_gps
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "gps.h"

/* ---------- Mini framework de test ---------- */

static int nb_verifs  = 0;
static int nb_echecs  = 0;

#define VERIFIER(cond) do {                                              \
        nb_verifs++;                                                     \
        if (!(cond)) {                                                   \
            nb_echecs++;                                                 \
            printf("  ECHEC ligne %d : %s\n", __LINE__, #cond);          \
        }                                                                \
    } while (0)

#define VERIFIER_INT(obtenu, attendu) do {                               \
        nb_verifs++;                                                     \
        long _o = (long)(obtenu), _a = (long)(attendu);                  \
        if (_o != _a) {                                                  \
            nb_echecs++;                                                 \
            printf("  ECHEC ligne %d : %s = %ld, attendu %ld\n",         \
                   __LINE__, #obtenu, _o, _a);                           \
        }                                                                \
    } while (0)

#define VERIFIER_DBL(obtenu, attendu) do {                               \
        nb_verifs++;                                                     \
        double _o = (obtenu), _a = (attendu);                            \
        if (fabs(_o - _a) > 1e-6) {                                      \
            nb_echecs++;                                                 \
            printf("  ECHEC ligne %d : %s = %.7f, attendu %.7f\n",       \
                   __LINE__, #obtenu, _o, _a);                           \
        }                                                                \
    } while (0)

#define VERIFIER_STR(obtenu, attendu) do {                               \
        nb_verifs++;                                                     \
        if (strcmp((obtenu), (attendu)) != 0) {                          \
            nb_echecs++;                                                 \
            printf("  ECHEC ligne %d : %s = \"%s\", attendu \"%s\"\n",   \
                   __LINE__, #obtenu, (obtenu), (attendu));              \
        }                                                                \
    } while (0)

#define LANCER(test) do { printf("- %s\n", #test); test(); } while (0)

/* ---------- Faux port série ---------- */

static int pipe_lecture  = -1;   /* donné à gps_init(), joue le rôle de l'UART */
static int pipe_ecriture = -1;   /* le test y écrit, joue le rôle du GPS */

static void faux_uart_ouvrir(void)
{
    int p[2];
    if (pipe(p) < 0) { perror("pipe"); _exit(2); }
    pipe_lecture  = p[0];
    pipe_ecriture = p[1];
    /* Non bloquant : read() renvoie EAGAIN quand il n'y a rien, comme l'UART */
    fcntl(pipe_lecture, F_SETFL, fcntl(pipe_lecture, F_GETFL) | O_NONBLOCK);
    gps_init(pipe_lecture);
}

static void faux_uart_fermer(void)
{
    close(pipe_lecture);
    close(pipe_ecriture);
}

/* Envoie du texte brut tel quel */
static void envoyer_brut(const char *texte)
{
    size_t len = strlen(texte);
    if (write(pipe_ecriture, texte, len) != (ssize_t)len) {
        perror("write"); _exit(2);
    }
}

/* Envoie "$<corps>*HH\r\n" avec le bon checksum calculé automatiquement */
static void envoyer_trame(const char *corps)
{
    unsigned char somme = 0;
    for (const char *p = corps; *p; p++) somme ^= (unsigned char)*p;

    char trame[256];
    snprintf(trame, sizeof(trame), "$%s*%02X\r\n", corps, somme);
    envoyer_brut(trame);
}

/* ---------- Tests ---------- */

/* Doit passer en premier : gps_init() n'a encore jamais été appelé */
static void test_sans_init(void)
{
    GpsData d = {0};
    VERIFIER_INT(gps_Read(&d), -1);
}

static void test_pointeur_null(void)
{
    faux_uart_ouvrir();
    VERIFIER_INT(gps_Read(NULL), -1);
    faux_uart_fermer();
}

static void test_rien_a_lire(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    VERIFIER_INT(gps_Read(&d), 0);       /* ne bloque pas, renvoie 0 */
    faux_uart_fermer();
}

static void test_erreur_lecture(void)
{
    faux_uart_ouvrir();
    faux_uart_fermer();                  /* descripteur fermé -> read() = EBADF */
    GpsData d = {0};
    VERIFIER_INT(gps_Read(&d), -1);
}

/* Exemple classique de la norme NMEA, avec son vrai checksum */
static void test_rmc_reference(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_brut("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");

    VERIFIER_INT(gps_Read(&d), 1);
    VERIFIER_INT(d.fix, 1);
    VERIFIER_DBL(d.latitude,  48.0 + 7.038 / 60.0);
    VERIFIER_DBL(d.longitude, 11.0 + 31.0 / 60.0);
    VERIFIER_DBL(d.vitesse_kmh, 22.4 * 1.852);
    VERIFIER_DBL(d.cap_deg, 84.4);
    VERIFIER_STR(d.heure_utc, "12:35:19");
    VERIFIER_STR(d.date, "23/03/2094");  /* le code suppose les années 20xx */
    faux_uart_fermer();
}

static void test_gga_reference(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    VERIFIER_INT(gps_Read(&d), 1);
    VERIFIER_INT(d.qualite, 1);
    VERIFIER_INT(d.satellites, 8);
    VERIFIER_DBL(d.hdop, 0.9);
    VERIFIER_DBL(d.altitude_m, 545.4);
    VERIFIER_DBL(d.latitude,  48.0 + 7.038 / 60.0);
    VERIFIER_DBL(d.longitude, 11.0 + 31.0 / 60.0);
    VERIFIER_STR(d.heure_utc, "12:35:19");
    faux_uart_fermer();
}

/* Trame réaliste d'un NEO-6M en Belgique (Mons) */
static void test_rmc_et_gga_ensemble(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_trame("GPRMC,094512.00,A,5027.12345,N,00357.54321,E,0.250,152.30,091126,,,A");
    envoyer_trame("GPGGA,094512.00,5027.12345,N,00357.54321,E,1,07,1.25,62.3,M,46.1,M,,");

    VERIFIER_INT(gps_Read(&d), 2);
    VERIFIER_INT(d.fix, 1);
    VERIFIER_INT(d.qualite, 1);
    VERIFIER_INT(d.satellites, 7);
    VERIFIER_DBL(d.latitude,  50.0 + 27.12345 / 60.0);
    VERIFIER_DBL(d.longitude,  3.0 + 57.54321 / 60.0);
    VERIFIER_DBL(d.altitude_m, 62.3);
    VERIFIER_DBL(d.hdop, 1.25);
    VERIFIER_DBL(d.vitesse_kmh, 0.25 * 1.852);
    VERIFIER_DBL(d.cap_deg, 152.3);
    VERIFIER_STR(d.heure_utc, "09:45:12");
    VERIFIER_STR(d.date, "09/11/2026");
    faux_uart_fermer();
}

static void test_hemispheres_sud_ouest(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_trame("GPRMC,000000.00,A,3352.12800,S,15112.55800,W,0.0,0.0,010125,,,A");

    VERIFIER_INT(gps_Read(&d), 1);
    VERIFIER_DBL(d.latitude,  -(33.0 + 52.128 / 60.0));
    VERIFIER_DBL(d.longitude, -(151.0 + 12.558 / 60.0));
    faux_uart_fermer();
}

/* Au démarrage, le NEO-6M envoie des trames avec des champs vides */
static void test_pas_de_fix(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    d.latitude  = 12.5;                  /* ancienne position, ne doit pas bouger */
    d.longitude = 4.5;
    envoyer_trame("GPRMC,101010.00,V,,,,,,,091126,,,N");
    envoyer_trame("GPGGA,101010.00,,,,,0,00,99.99,,,,,,");

    VERIFIER_INT(gps_Read(&d), 2);
    VERIFIER_INT(d.fix, 0);
    VERIFIER_INT(d.qualite, 0);
    VERIFIER_INT(d.satellites, 0);
    VERIFIER_DBL(d.hdop, 99.99);
    VERIFIER_DBL(d.latitude, 12.5);
    VERIFIER_DBL(d.longitude, 4.5);
    VERIFIER_STR(d.heure_utc, "10:10:10");  /* l'heure arrive avant le fix */
    VERIFIER_STR(d.date, "09/11/2026");
    faux_uart_fermer();
}

static void test_mauvais_checksum(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    /* même trame que test_gga_reference mais *48 au lieu de *47 */
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n");

    VERIFIER_INT(gps_Read(&d), 0);
    VERIFIER_INT(d.satellites, 0);
    VERIFIER_DBL(d.latitude, 0.0);
    faux_uart_fermer();
}

static void test_sans_checksum(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,\r\n");
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*\r\n");
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*4G\r\n");

    VERIFIER_INT(gps_Read(&d), 0);
    faux_uart_fermer();
}

static void test_checksum_minuscules(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_brut("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6a\r\n");

    VERIFIER_INT(gps_Read(&d), 1);
    faux_uart_fermer();
}

/* GSV, GSA, VTG, GLL... sont valides mais pas utilisées */
static void test_trames_ignorees(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_trame("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00");
    envoyer_trame("GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1");
    envoyer_trame("GPVTG,77.52,T,,M,0.004,N,0.008,K,A");
    envoyer_trame("GPGLL,4916.45,N,12311.12,W,225444,A,");
    envoyer_trame("PUBX,00,123519");    /* trame propriétaire u-blox */

    VERIFIER_INT(gps_Read(&d), 0);
    faux_uart_fermer();
}

/* Récepteurs multi-constellations : préfixe GN au lieu de GP */
static void test_prefixe_gn(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_trame("GNRMC,120000.00,A,5000.00000,N,00400.00000,E,0.0,0.0,091126,,,A");
    envoyer_trame("GNGGA,120000.00,5000.00000,N,00400.00000,E,1,10,0.8,100.0,M,46.0,M,,");

    VERIFIER_INT(gps_Read(&d), 2);
    VERIFIER_DBL(d.latitude, 50.0);
    VERIFIER_DBL(d.longitude, 4.0);
    VERIFIER_INT(d.satellites, 10);
    faux_uart_fermer();
}

/* Une trame coupée en deux lectures : le morceau doit être gardé */
static void test_trame_coupee(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};

    envoyer_brut("$GPGGA,123519,4807.038,N,0113");
    VERIFIER_INT(gps_Read(&d), 0);       /* pas encore de fin de ligne */

    envoyer_brut("1.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
    VERIFIER_INT(gps_Read(&d), 0);       /* toujours pas de '\n' */

    envoyer_brut("\r\n");
    VERIFIER_INT(gps_Read(&d), 1);
    VERIFIER_INT(d.satellites, 8);
    VERIFIER_DBL(d.altitude_m, 545.4);
    faux_uart_fermer();
}

/* Envoi caractère par caractère, comme un UART très lent */
static void test_octet_par_octet(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    const char *t = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
    int total = 0;

    for (const char *p = t; *p; p++) {
        char c[2] = { *p, '\0' };
        envoyer_brut(c);
        total += gps_Read(&d);
    }
    VERIFIER_INT(total, 1);
    VERIFIER_INT(d.fix, 1);
    faux_uart_fermer();
}

/* Au branchement, on peut arriver au milieu d'une trame */
static void test_dechets_au_debut(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_brut("8.4,230394,003.1,W*6A\r\n");          /* fin d'une trame ratée */
    envoyer_brut("\xff\x00\x12 bruit");                 /* parasites */
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    VERIFIER_INT(gps_Read(&d), 1);
    VERIFIER_INT(d.satellites, 8);
    faux_uart_fermer();
}

/* Un '$' au milieu d'une trame abîmée doit redémarrer une nouvelle trame */
static void test_trame_interrompue(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_brut("$GPRMC,123519,A,48");                 /* coupée net */
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    VERIFIER_INT(gps_Read(&d), 1);
    VERIFIER_INT(d.fix, 0);                             /* le RMC n'a pas été pris */
    VERIFIER_INT(d.satellites, 8);
    faux_uart_fermer();
}

/* Fins de ligne \n seul, \r seul ou lignes vides : tout doit marcher */
static void test_fins_de_ligne(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_brut("\r\n\r\n");
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n");
    envoyer_brut("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r");

    VERIFIER_INT(gps_Read(&d), 2);
    faux_uart_fermer();
}

/* Ligne plus longue que le buffer : jetée, sans débordement, la suite marche */
static void test_ligne_trop_longue(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    char longue[400];
    memset(longue, 'A', sizeof(longue));
    longue[0] = '$';
    longue[sizeof(longue) - 1] = '\0';
    envoyer_brut(longue);
    envoyer_brut("\r\n");
    envoyer_brut("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    VERIFIER_INT(gps_Read(&d), 1);
    VERIFIER_INT(d.satellites, 8);
    faux_uart_fermer();
}

/* Trame avec un bon checksum mais trop peu de champs */
static void test_trame_trop_courte(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    envoyer_trame("GPRMC,123519,A,4807.038,N");
    envoyer_trame("GPGGA,123519,4807.038");

    VERIFIER_INT(gps_Read(&d), 0);
    VERIFIER_INT(d.fix, 0);
    faux_uart_fermer();
}

/* Beaucoup de données d'un coup (> buffer interne de 256 octets) */
static void test_rafale(void)
{
    faux_uart_ouvrir();
    GpsData d = {0};
    for (int i = 0; i < 20; i++) {
        envoyer_trame("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
        envoyer_trame("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        envoyer_trame("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00");
    }

    VERIFIER_INT(gps_Read(&d), 40);
    faux_uart_fermer();
}

/* ---------- Programme principal ---------- */

int main(void)
{
    printf("Tests du module GPS\n");

    LANCER(test_sans_init);             /* doit rester le premier */
    LANCER(test_pointeur_null);
    LANCER(test_rien_a_lire);
    LANCER(test_erreur_lecture);
    LANCER(test_rmc_reference);
    LANCER(test_gga_reference);
    LANCER(test_rmc_et_gga_ensemble);
    LANCER(test_hemispheres_sud_ouest);
    LANCER(test_pas_de_fix);
    LANCER(test_mauvais_checksum);
    LANCER(test_sans_checksum);
    LANCER(test_checksum_minuscules);
    LANCER(test_trames_ignorees);
    LANCER(test_prefixe_gn);
    LANCER(test_trame_coupee);
    LANCER(test_octet_par_octet);
    LANCER(test_dechets_au_debut);
    LANCER(test_trame_interrompue);
    LANCER(test_fins_de_ligne);
    LANCER(test_ligne_trop_longue);
    LANCER(test_trame_trop_courte);
    LANCER(test_rafale);

    printf("\n%d verifications, %d echec(s)\n", nb_verifs, nb_echecs);
    return nb_echecs == 0 ? 0 : 1;
}

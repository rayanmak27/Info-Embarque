#include <stdio.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

#include "Ecran/ecran.h"

/* numéros de pages dans Nextion : adaptez-les */
#define PAGE_PRET    0
#define PAGE_MESURE  2
#define PAGE_PAUSE   3

#define EVT_START 0xA1
#define EVT_PAUSE 0xA2
#define EVT_FIN   0xA3

typedef enum { ETAT_PRET, ETAT_MESURE, ETAT_PAUSE } Etat;

static volatile sig_atomic_t stop = 0;
static void on_sigint(int s) { (void)s; stop = 1; }

static double maintenant(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

/* valeurs inventées, qui varient avec le temps t (en secondes) */
static void simuler(DonneesVehicule *d, double t)
{
    d->vitesse_gps   = 60.0f + 60.0f * sinf(t * 0.3);   /* 0 à 120 km/h */
    d->rpm           = 3000.0f + 2500.0f * sinf(t * 0.5);
    d->temp_ambiante = 22.5f + 1.0f * sinf(t * 0.1);
    d->temp_moteur   = 85.0f + 10.0f * sinf(t * 0.05);
    d->accel_x = 0.5f * sinf(t);
    d->accel_y = 0.5f * cosf(t);
    d->accel_z = 1.0f;
    d->gyro_x  = 30.0f * sinf(t * 0.7);
    d->gyro_y  = 30.0f * cosf(t * 0.7);
    d->gyro_z  = 10.0f * sinf(t * 0.2);
}

int main(void)
{
    signal(SIGINT, on_sigint);

    if (ecran_init() != 0) {
        fprintf(stderr, "Erreur : impossible d'ouvrir l'UART\n");
        return 1;
    }

    DonneesVehicule d = {0};
    Etat   etat = ETAT_PRET;
    double debut = 0, pause_debut = 0, temps_pause = 0;
    double chrono = 0;

    ecran_page(PAGE_PRET);
    printf("Test écran : Ctrl+C pour quitter. Boutons de l'écran actifs.\n");

    while (!stop) {
        double now = maintenant();

        /* ---- boutons de l'écran ---- */
        int evt;
        while ((evt = ecran_read_event()) > 0) {
            if (evt == EVT_START && etat == ETAT_PRET) {
                debut = now; temps_pause = 0; chrono = 0;
                etat = ETAT_MESURE;
                ecran_page(PAGE_MESURE);
            } else if (evt == EVT_PAUSE && etat == ETAT_MESURE) {
                pause_debut = now;
                etat = ETAT_PAUSE;
                ecran_page(PAGE_PAUSE);
            } else if (evt == EVT_PAUSE && etat == ETAT_PAUSE) {
                temps_pause += now - pause_debut;
                etat = ETAT_MESURE;
                ecran_page(PAGE_MESURE);
            } else if (evt == EVT_FIN && etat != ETAT_PRET) {
                etat = ETAT_PRET;
                ecran_page(PAGE_PRET);
            }
            printf("Evénement reçu : 0x%02X\n", evt);
        }

        /* ---- chrono : s'arrête pendant la pause ---- */
        if (etat == ETAT_MESURE) chrono = now - debut - temps_pause;
        d.timestamp = chrono;

        /* ---- valeurs inventées + affichage ---- */
        simuler(&d, now);
        ecran_display(&d);

        usleep(250000);   /* 4 rafraîchissements par seconde */
    }

    ecran_page(PAGE_PRET);
    ecran_close();
    return 0;
}
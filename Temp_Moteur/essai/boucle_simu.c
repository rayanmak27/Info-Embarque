// Author Ali CAN
// Essai d'intégration de T moteur dans l'exécutif du groupe : 10 créneaux de 100 ms, réveil à date absolue,
// régime simulé au créneau 0, T moteur lue au créneau 8, une ligne de journal au créneau 9.
//
// Avec la simulation (sans module), température qui suit le régime, dynamique accélérée 10 fois :
//   gcc -Wall -Wextra -DAVEC_SIMU -DSIMU_TAU_S=12 -o essai/boucle essai/boucle_simu.c simulation/tempmoteur_simu.c
// Avec le vrai module (ou sans SPI, pour voir les « nan ») :
//   gcc -Wall -Wextra -o essai/boucle-reel essai/boucle_simu.c tempmoteur.c
// Lancer : ./essai/boucle [secondes] > journal.csv   (150 s par défaut ; le résumé sort sur stderr)

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../tempmoteur.h"
#ifdef AVEC_SIMU
#include "../simulation/tempmoteur_simu.h"
#endif

// régime du banc d'essai : coupé, ralenti, accélération, coupé (en tr/min)
static float regime_banc(double t)
{
    if (t < 10) return 0.0f;     // moteur coupé
    if (t < 30) return 800.0f;   // ralenti
    if (t < 50) return 3200.0f;  // accélération
    return 0.0f;                 // coupé : refroidit
}

static double secondes(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) + (double)(b->tv_nsec - a->tv_nsec) / 1e9;
}

int main(int argc, char **argv)
{
    int duree = argc > 1 ? atoi(argv[1]) : 150;
    float rpm = 0.0f, temp_moteur = NAN;
    long lectures = 0, erreurs = 0;
    double retard_max = 0.0, lecture_max = 0.0;
    struct timespec debut, prochain, maintenant, avant, apres;

    int r = tempmoteur_init();
    fprintf(stderr, "tempmoteur_init : %d\n", r);

    clock_gettime(CLOCK_MONOTONIC, &debut);
    prochain = debut;
    printf("timestamp_s;rpm;temp_moteur_c;code;lecture_us\n");

    for (long n = 0; n < (long)duree * 10; n++)
    {
        int creneau = (int)(n % 10);
        prochain.tv_nsec += 100000000L; // réveil suivant, 100 ms plus tard
        if (prochain.tv_nsec >= 1000000000L)
        {
            prochain.tv_sec++;
            prochain.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &prochain, NULL);
        clock_gettime(CLOCK_MONOTONIC, &maintenant);
        double retard = secondes(&prochain, &maintenant);
        if (retard > retard_max)
            retard_max = retard;
        double t = secondes(&debut, &maintenant);

        static int code = 0;
        static double lecture_us = 0.0;
        switch (creneau)
        {
        case 0: // régime : ici le banc, dans le projet impulsion_read()
            rpm = regime_banc(t);
#ifdef AVEC_SIMU
            tempmoteur_simu_regime(rpm);
#endif
            break;
        case 8: // T moteur
            clock_gettime(CLOCK_MONOTONIC, &avant);
            code = tempmoteur_read(&temp_moteur);
            clock_gettime(CLOCK_MONOTONIC, &apres);
            lecture_us = secondes(&avant, &apres) * 1e6;
            if (lecture_us > lecture_max)
                lecture_max = lecture_us;
            lectures++;
            if (code < 0)
            {
                temp_moteur = NAN; // le journal écrit nan, personne n'invente de valeur
                erreurs++;
            }
            break;
        case 9: // journal
            printf("%.1f;%.0f;%.2f;%d;%.0f\n", t, (double)rpm, (double)temp_moteur, code, lecture_us);
            fflush(stdout);
            break;
        default:
            break;
        }
    }
    tempmoteur_close();
    fprintf(stderr, "%ld lectures, %ld en erreur, lecture la plus longue %.0f µs, retard de réveil maximal %.2f ms\n",
            lectures, erreurs, lecture_max, retard_max * 1e3);
    return 0;
}

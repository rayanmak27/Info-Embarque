//Ali
//Essai du module T moteur sur la Pi : une mesure par seconde et durée de lecture
//Depuis Temp_Moteur : gcc -Wall -Wextra -o essai/essai essai/essai_tempmoteur.c tempmoteur.c
//Avec un MAX31855 : ajouter -DTEMPMOTEUR_PUCE=31855

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "../tempmoteur.h"

static long duree_us(const struct timespec *debut, const struct timespec *fin)
{
    return (long)(fin->tv_sec - debut->tv_sec) * 1000000L
         + (fin->tv_nsec - debut->tv_nsec) / 1000L;
}

int main(void)
{
    long pire_us = 0;

    if (tempmoteur_init() < 0)
    {
        printf("SPI indisponible : dtparam=spi=on dans /boot/firmware/config.txt ?\n");
        return 1;
    }

    sleep(1);

    for (int i = 1; i <= 60; i++)
    {
        struct timespec debut, fin;
        float temperature = 0.0f;
        long us;
        int r;

        clock_gettime(CLOCK_MONOTONIC, &debut);
        r = tempmoteur_read(&temperature);
        clock_gettime(CLOCK_MONOTONIC, &fin);

        us = duree_us(&debut, &fin);
        if (us > pire_us)
            pire_us = us;

        if (r == 0)
            printf("%2d  %7.2f °C   lecture %ld µs, pire %ld µs\n", i, (double)temperature, us, pire_us);
        else if (r == TEMPMOTEUR_ERR_SONDE)
            printf("%2d  sonde ouverte ou en court-circuit\n", i);
        else if (r == TEMPMOTEUR_ERR_TRAME)
            printf("%2d  trame invalide : module absent ou mauvaise puce choisie\n", i);
        else
            printf("%2d  erreur %d\n", i, r);

        fflush(stdout);
        sleep(1);
    }

    tempmoteur_close();
    return 0;
}

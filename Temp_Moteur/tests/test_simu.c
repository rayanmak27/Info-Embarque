// Essais de simulation/tempmoteur_simu.c, horloge remplacée.
// Depuis Temp_Moteur/tests : cc test_simu.c && ./a.out
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <stdio.h>
#include <time.h>

static long long horloge_ns = 5000000000LL;
static int faux_clock(int c, struct timespec *t)
{
    (void)c;
    t->tv_sec = (time_t)(horloge_ns / 1000000000LL);
    t->tv_nsec = (long)(horloge_ns % 1000000000LL);
    return 0;
}
#define clock_gettime(...) faux_clock(__VA_ARGS__)
#include "../simulation/tempmoteur_simu.c"

static int controles = 0;
#define OK(c) do { assert(c); controles++; } while (0)

int main(void)
{
    float v = -1.0f, avant = 0.0f;

    OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_SPI);
    OK(tempmoteur_init() == 0);
    OK(tempmoteur_read(NULL) == TEMPMOTEUR_ERR_SPI);
    OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_ATTENTE);

    // une lecture par seconde pendant 30 minutes
    for (int s = 1; s <= 1800; s++)
    {
        horloge_ns += 1000000000LL;
        OK(tempmoteur_read(&v) == 0);
        OK(v * 4.0f == (float)(int)(v * 4.0f)); // pas de 0,25 °C
        OK(v >= avant && v <= 90.0f);           // monte sans dépasser le thermostat
        avant = v;
        if (s == 1) OK(v >= 20.0f && v < 21.0f);
        if (s == 120) OK(v > 63.0f && v < 66.0f); // 20 + 70 (1 - 1/e) = 64,2 °C après tau
    }
    OK(v == 90.0f);

    // une lecture très en retard ne fait pas déborder le modèle
    horloge_ns += 3600LL * 1000000000LL;
    OK(tempmoteur_read(&v) == 0); OK(v == 90.0f);

    // moteur coupé : refroidit vers 20 °C, 4 fois plus lentement (tau = 480 s)
    tempmoteur_simu_regime(0.0f);
    for (int s = 1; s <= 480; s++) { horloge_ns += 1000000000LL; OK(tempmoteur_read(&v) == 0); OK(v <= avant); avant = v; }
    OK(v > 44.0f && v < 47.0f); // 20 + 70 / e = 45,8 °C
    for (int s = 1; s <= 7200; s++) { horloge_ns += 1000000000LL; OK(tempmoteur_read(&v) == 0); }
    OK(v == 20.0f);

    // 3200 tr/min : 4 fois plus vite qu'au ralenti, 64 °C en 30 s au lieu de 120 s
    tempmoteur_simu_regime(3200.0f);
    for (int s = 1; s <= 30; s++) { horloge_ns += 1000000000LL; OK(tempmoteur_read(&v) == 0); }
    OK(v > 63.0f && v < 66.0f);
    // sous le ralenti, même vitesse qu'au ralenti ; régime négatif = moteur coupé
    tempmoteur_simu_regime(-5.0f);
    horloge_ns += 1000000000LL; avant = v; OK(tempmoteur_read(&v) == 0); OK(v <= avant);

    tempmoteur_close();
    OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_SPI);
    OK(tempmoteur_init() == 0);
    horloge_ns += 1000000000LL;
    OK(tempmoteur_read(&v) == 0); OK(v < 21.0f); // repart à froid, au ralenti
    printf("simulation : %d contrôles OK\n", controles);
    return 0;
}

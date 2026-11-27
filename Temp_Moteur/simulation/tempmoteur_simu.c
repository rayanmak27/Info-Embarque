//Ali
//T moteur simulée, sans module : même interface que tempmoteur.c, à compiler à sa place
//Sert à essayer la boucle, l'écran et le journal avant que le thermocouple soit branché
//Modèle du premier ordre, arrondi à 0,25 °C comme la vraie puce :
//  T += (T_cible - T) * dt / tau
//  moteur tournant : T_cible = thermostat, tau plus court quand le régime monte
//  moteur coupé (régime 0) : T_cible = départ, refroidissement 4 fois plus lent

#define _POSIX_C_SOURCE 200809L

#include "tempmoteur_simu.h"

#include <stddef.h>
#include <stdint.h>
#include <time.h>

#ifndef SIMU_DEPART
#define SIMU_DEPART 20.0     // °C, moteur froid
#endif
#ifndef SIMU_THERMOSTAT
#define SIMU_THERMOSTAT 90.0 // °C, ouverture du thermostat
#endif
#ifndef SIMU_TAU_S
#define SIMU_TAU_S 120.0     // s au ralenti, accéléré pour la démo (un vrai moteur : environ 10 min)
#endif
#define SIMU_RALENTI 800.0   // tr/min

#define ATTENTE_NS 250000000LL

static int ouvert = 0;
static double temperature_simu = SIMU_DEPART;
static double regime = SIMU_RALENTI;
static int64_t debut_ns = 0;
static int64_t precedent_ns = 0;

static int64_t temps_ns(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + (int64_t)t.tv_nsec;
}

void tempmoteur_simu_regime(float rpm)
{
    regime = rpm > 0.0f ? (double)rpm : 0.0;
}

int tempmoteur_init(void)
{
    if (ouvert)
        return 0;
    ouvert = 1;
    temperature_simu = SIMU_DEPART;
    regime = SIMU_RALENTI;
    debut_ns = temps_ns();
    precedent_ns = debut_ns;
    return 0;
}

int tempmoteur_read(float *temperature)
{
    int64_t maintenant;
    double dt, cible, tau;

    if (!ouvert || temperature == NULL)
        return TEMPMOTEUR_ERR_SPI;

    maintenant = temps_ns();
    if (maintenant - debut_ns < ATTENTE_NS)
        return TEMPMOTEUR_ERR_ATTENTE; // comme la vraie puce juste après l'init

    if (regime > 0.0)
    {
        cible = SIMU_THERMOSTAT;
        tau = SIMU_TAU_S * SIMU_RALENTI / (regime > SIMU_RALENTI ? regime : SIMU_RALENTI);
    }
    else
    {
        cible = SIMU_DEPART;
        tau = SIMU_TAU_S * 4.0;
    }

    dt = (double)(maintenant - precedent_ns) / 1e9;
    precedent_ns = maintenant;
    if (dt > tau)
        dt = tau;
    temperature_simu += (cible - temperature_simu) * dt / tau;

    *temperature = (float)((double)(int64_t)(temperature_simu * 4.0 + 0.5) / 4.0);
    return 0;
}

void tempmoteur_close(void)
{
    ouvert = 0;
}

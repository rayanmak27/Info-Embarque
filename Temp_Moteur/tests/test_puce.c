// Essais de essai/identifier_puce.c sans Pi : seul le choix de la puce est testé.
// Depuis Temp_Moteur/tests : cc -Ifake test_puce.c && ./a.out
#define main identifier_main
#include "../essai/identifier_puce.c"
#undef main

#include <assert.h>

static int controles = 0;
#define OK(c) do { assert(c); controles++; } while (0)

// Trames d'un MAX31855 : thermocouple sur 14 bits (0,25 °C), puce sur 12 bits (0,0625 °C)
static uint32_t t31855(int tc_quarts, int jf_seiziemes, uint32_t defauts)
{
    uint32_t t = ((uint32_t)tc_quarts & 0x3FFFu) << 18 | ((uint32_t)jf_seiziemes & 0xFFFu) << 4 | defauts;
    if (defauts != 0)
        t |= 0x00010000u;
    return t;
}

// Trame d'un MAX6675 : 12 bits à 0,25 °C
static uint16_t t6675(int quarts, int ouverte)
{
    return (uint16_t)(((unsigned)quarts << 3) | (ouverte ? 4u : 0u));
}

int main(void)
{
    float a, b;

    // vrai MAX31855 à 22 °C, puce à 23,5 °C ; lu en 16 bits mode 1, il passerait pour un MAX6675
    uint32_t m = t31855(88, 376, 0);
    OK(est_31855(m, &a, &b)); OK(a == 22.0f); OK(b == 23.5f);
    OK(verdict(m, (uint16_t)(m >> 16)) == PUCE_31855);
    // exemple de la fiche : 100,75 °C, puce à 25 °C
    m = t31855(403, 400, 0);
    OK(verdict(m, 0) == PUCE_31855); est_31855(m, &a, &b); OK(a == 100.75f); OK(b == 25.0f);
    // températures négatives, thermocouple et puce
    m = t31855(-40, -160, 0);
    OK(verdict(m, 0) == PUCE_31855); est_31855(m, &a, &b); OK(a == -10.0f); OK(b == -10.0f);
    // MAX31855 sonde ouverte : D16 et D0 à 1, toujours reconnu
    m = t31855(0, 376, 1u);
    OK(verdict(m, 0) == PUCE_31855);
    // D16 à 1 sans aucun défaut D2-D0 : incohérent, pas un MAX31855
    OK(!est_31855(t31855(88, 376, 0) | 0x00010000u, &a, &b));

    // vrai MAX6675 à 22 °C : sur 32 bits en mode 0, les 16 derniers bits restent à 0
    uint16_t s = t6675(88, 0);
    OK(verdict((uint32_t)s << 16, s) == PUCE_6675);
    OK(verdict((uint32_t)s << 15, s) == PUCE_6675);   // décalé d'un bit en mode 0
    OK(verdict((uint32_t)s << 16 | 0xFFFFu, s) == PUCE_6675); // MISO tiré vers le haut
    est_6675(s, &a); OK(a == 22.0f);
    // MAX6675 sonde ouverte : D2 à 1, toujours reconnu
    OK(verdict((uint32_t)t6675(88, 1) << 16, t6675(88, 1)) == PUCE_6675);

    // rien de branché : MISO à 0 ou à 1
    OK(verdict(0, 0) == AUCUNE);
    OK(verdict(0xFFFFFFFFu, 0xFFFFu) == AUCUNE);
    // bits toujours à 0 mis à 1 : pas un MAX6675
    OK(!est_6675(0x8000u | t6675(88, 0), &a)); OK(!est_6675(0x0002u | t6675(88, 0), &a));
    // D17 ou D3 à 1 : pas un MAX31855
    OK(!est_31855(t31855(88, 376, 0) | 0x00020000u, &a, &b));
    OK(!est_31855(t31855(88, 376, 0) | 0x00000008u, &a, &b));

    printf("identifier_puce : %d contrôles OK\n", controles);
    return 0;
}

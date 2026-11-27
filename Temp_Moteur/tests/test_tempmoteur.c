// Essais du module sans Pi : open, ioctl, close et l'horloge sont remplacés.
// Depuis Temp_Moteur/tests : cc -Ifake -I.. -DTEMPMOTEUR_PUCE=6675 test_tempmoteur.c -lm && ./a.out
// puis la même chose avec -DTEMPMOTEUR_PUCE=31855
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

static int64_t horloge_ns = 1000000000LL;
static int ouverture_ok = 1, ioctl_ko = 0, transferts = 0, fermetures = 0;
static uint32_t mode_vu, vitesse_vue, bits_vus, len_vue, vitesse_tr, bits_tr;
static uint8_t reponse[4];

static int faux_open(const char *p, int f) { (void)p; (void)f; return ouverture_ok ? 7 : -1; }
static int faux_close(int fd) { (void)fd; fermetures++; return 0; }
static int faux_clock(int c, struct timespec *t)
{
    (void)c;
    t->tv_sec = (time_t)(horloge_ns / 1000000000LL);
    t->tv_nsec = (long)(horloge_ns % 1000000000LL);
    return 0;
}
static int faux_ioctl(int fd, unsigned long r, void *a)
{
    (void)fd;
    if (r == SPI_IOC_WR_MODE) { mode_vu = *(uint8_t *)a; return 0; }
    if (r == SPI_IOC_WR_BITS_PER_WORD) { bits_vus = *(uint8_t *)a; return 0; }
    if (r == SPI_IOC_WR_MAX_SPEED_HZ) { vitesse_vue = *(uint32_t *)a; return 0; }
    if (r == (unsigned long)SPI_IOC_MESSAGE(1)) {
        struct spi_ioc_transfer *t = a;
        transferts++;
        if (ioctl_ko) return -1;
        len_vue = t->len; vitesse_tr = t->speed_hz; bits_tr = t->bits_per_word;
        memcpy((void *)(uintptr_t)t->rx_buf, reponse, t->len);
        return (int)t->len;
    }
    return -1;
}
#define open(...) faux_open(__VA_ARGS__)
#define close(...) faux_close(__VA_ARGS__)
#define ioctl(...) faux_ioctl(__VA_ARGS__)
#define clock_gettime(...) faux_clock(__VA_ARGS__)
#include "tempmoteur.c"

static int controles = 0;
#define OK(c) do { assert(c); controles++; } while (0)
static void avance(int ms) { horloge_ns += (int64_t)ms * 1000000LL; }
static void arme(uint32_t t) { reponse[0] = (uint8_t)(t >> 24); reponse[1] = (uint8_t)(t >> 16); reponse[2] = (uint8_t)(t >> 8); reponse[3] = (uint8_t)t; }
static void lit(float attendu) { float v = -999.0f; avance(300); OK(tempmoteur_read(&v) == 0); OK(fabsf(v - attendu) < 0.001f); }
static void err(int code) { float v = -999.0f; avance(300); OK(tempmoteur_read(&v) == code); OK(v == -999.0f); }

int main(void)
{
    float v = -999.0f;
    int n;

    // codes de retour en -errno, comme Temp_Ambiante
    OK(TEMPMOTEUR_ERR_SPI == -ENODEV); OK(TEMPMOTEUR_ERR_SONDE == -EIO);
    OK(TEMPMOTEUR_ERR_TRAME == -EPROTO); OK(TEMPMOTEUR_ERR_ATTENTE == -EAGAIN);

    OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_SPI);
    ouverture_ok = 0; OK(tempmoteur_init() == TEMPMOTEUR_ERR_SPI); ouverture_ok = 1;
    OK(tempmoteur_init() == 0);
    OK(tempmoteur_read(NULL) == TEMPMOTEUR_ERR_SPI);
    OK(bits_vus == 8); OK(vitesse_vue == 1000000u);
    OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_ATTENTE); OK(transferts == 0);

#if TEMPMOTEUR_PUCE == 6675
    // fiche MAX6675 : D15 = 0, D14-D3 = 12 bits à 0,25 °C, D2 = sonde ouverte, D1 = 0
    OK(mode_vu == SPI_MODE_1);
    arme(0x0C880000u); lit(100.25f); OK(len_vue == 2); OK(vitesse_tr == 1000000u); OK(bits_tr == 8);
    arme(0x00080000u); lit(0.25f);
    arme(0x7FF80000u); lit(1023.75f);
    arme(0x00000000u); err(TEMPMOTEUR_ERR_TRAME);
    arme(0xFFFF0000u); err(TEMPMOTEUR_ERR_TRAME);
    arme(0x0C8A0000u); err(TEMPMOTEUR_ERR_TRAME);
    arme(0x0C8C0000u); err(TEMPMOTEUR_ERR_SONDE);
    // MAX31855 lu par erreur comme un MAX6675 : 25 °C donne 12,5 °C
    arme((uint32_t)100 << 18); lit(12.5f);
    arme(0x0C880000u); lit(100.25f);
#else
    // fiche MAX31855, table 4 : D31-D18 = 14 bits signés à 0,25 °C
    OK(mode_vu == SPI_MODE_0);
    static const struct { uint32_t brut; float c; } t4[] = {
        {0x1900, 1600.0f}, {0x0FA0, 1000.0f}, {0x0193, 100.75f}, {0x0064, 25.0f},
        {0x0000, 0.0f}, {0x3FFF, -0.25f}, {0x3FFC, -1.0f}, {0x3C18, -250.0f}
    };
    for (unsigned i = 0; i < sizeof t4 / sizeof t4[0]; i++) { arme((t4[i].brut << 18) | (0x190u << 4)); lit(t4[i].c); }
    OK(len_vue == 4);
    arme(0x01900000u | 0x00010001u); err(TEMPMOTEUR_ERR_SONDE);
    arme(0x01900000u | 0x00010002u); err(TEMPMOTEUR_ERR_SONDE);
    arme(0x01900000u | 0x00010004u); err(TEMPMOTEUR_ERR_SONDE);
    arme(0x7FFC0000u | 0x00010001u); err(TEMPMOTEUR_ERR_SONDE);
    arme(0x00000000u); err(TEMPMOTEUR_ERR_TRAME);
    arme(0xFFFFFFFFu); err(TEMPMOTEUR_ERR_TRAME);
    arme(0x01900000u | 0x00020000u); err(TEMPMOTEUR_ERR_TRAME);
    arme(0x01900000u | 0x00000008u); err(TEMPMOTEUR_ERR_TRAME);
    arme(0x01900000u); lit(25.0f);
    arme(0x06400000u); lit(100.0f);
#endif

    // lectures rapprochées : la puce n'est pas touchée, la dernière valeur revient
    n = transferts;
    avance(100); v = 0; OK(tempmoteur_read(&v) == 0); OK(v > 99.0f); OK(transferts == n);
    avance(100); OK(tempmoteur_read(&v) == 0); OK(transferts == n);
    avance(100); OK(tempmoteur_read(&v) == 0); OK(transferts == n + 1);

    // après une erreur, aucune valeur périmée rendue comme bonne
    arme(0xFFFFFFFFu); avance(300); OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_TRAME);
    avance(100); OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_ATTENTE);
    ioctl_ko = 1; avance(300); OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_SPI); ioctl_ko = 0;

    tempmoteur_close(); OK(fermetures == 1);
    tempmoteur_close(); OK(fermetures == 1);
    OK(tempmoteur_read(&v) == TEMPMOTEUR_ERR_SPI);
    OK(tempmoteur_init() == 0); tempmoteur_close();
    printf("puce %d : %d contrôles OK\n", TEMPMOTEUR_PUCE, controles);
    return 0;
}

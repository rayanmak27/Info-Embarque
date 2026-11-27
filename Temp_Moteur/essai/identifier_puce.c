//Ali
//Dit quelle puce porte le module thermocouple, sans lire le marquage
//Depuis Temp_Moteur : gcc -Wall -Wextra -o essai/puce essai/identifier_puce.c && ./essai/puce
//Pointe du thermocouple à la température de la pièce pendant l'essai

#define _POSIX_C_SOURCE 200809L

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define SPI_PERIPH "/dev/spidev0.0"
#define SPI_VITESSE_HZ 1000000u

enum { AUCUNE = 0, PUCE_31855 = 31855, PUCE_6675 = 6675 };

// Lit n octets dans le mode SPI donné, 0 = OK
static int lire(int fd, uint8_t mode, uint8_t *rx, uint32_t n)
{
    struct spi_ioc_transfer tr;
    uint8_t bits = 8;
    uint32_t vitesse = SPI_VITESSE_HZ;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0
        || ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0
        || ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &vitesse) < 0)
        return -1;

    memset(&tr, 0, sizeof tr);
    memset(rx, 0, n);
    tr.rx_buf = (uint64_t)(uintptr_t)rx;
    tr.len = n;
    tr.speed_hz = SPI_VITESSE_HZ;
    tr.bits_per_word = 8;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0 ? -1 : 0;
}

// MAX31855 : D17 et D3 à 0, D16 = OU de D2 à D0, jonction froide plausible.
// Un MAX6675 lu sur 32 bits laisse les 16 derniers bits à 0 (MISO tiré vers le bas)
static int est_31855(uint32_t t, float *tc, float *jf)
{
    int d16 = (int)((t >> 16) & 1u);
    int defauts = (t & 7u) != 0;

    *tc = (float)((int32_t)t >> 18) * 0.25f;
    *jf = (float)((int32_t)(t << 16) >> 20) * 0.0625f;

    if (t == 0 || t == 0xFFFFFFFFu || (t & 0xFFFFu) == 0 || (t & 0x00020008u) != 0)
        return 0;
    if (d16 != defauts)
        return 0;
    return *jf > -40.0f && *jf < 125.0f;
}

// MAX6675 : D15 et D1 à 0, trame ni toute à 0 ni toute à 1
static int est_6675(uint16_t t, float *temp)
{
    *temp = (float)(t >> 3) * 0.25f;
    return t != 0 && t != 0xFFFFu && (t & 0x8002u) == 0;
}

// Choisit la puce d'après les deux lectures. Le MAX31855 passe en premier :
// lu comme un MAX6675, il donnerait une trame valide mais la moitié de la température
static int verdict(uint32_t t32, uint16_t t16)
{
    float a, b;

    if (est_31855(t32, &a, &b))
        return PUCE_31855;
    if (est_6675(t16, &a))
        return PUCE_6675;
    return AUCUNE;
}

static void attendre_ms(long ms)
{
    struct timespec d = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&d, NULL);
}

int main(void)
{
    uint8_t rx[4];
    uint32_t t32;
    uint16_t t16;
    float tc, jf, t6675;
    int fd, puce;

    fd = open(SPI_PERIPH, O_RDWR);
    if (fd < 0)
    {
        printf("%s absent : dtparam=spi=on dans /boot/firmware/config.txt, puis redémarrer\n", SPI_PERIPH);
        return 1;
    }

    // Lecture MAX31855 : mode 0, 32 bits
    attendre_ms(300);
    if (lire(fd, SPI_MODE_0, rx, 4) < 0)
    {
        printf("transfert SPI refusé\n");
        close(fd);
        return 1;
    }
    t32 = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];

    // Lecture MAX6675 : mode 1, 16 bits, après une conversion complète (0,22 s au plus)
    attendre_ms(300);
    if (lire(fd, SPI_MODE_1, rx, 2) < 0)
    {
        printf("transfert SPI refusé\n");
        close(fd);
        return 1;
    }
    t16 = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    close(fd);

    est_31855(t32, &tc, &jf);
    est_6675(t16, &t6675);
    printf("Mode 0, 32 bits : 0x%08X  lu comme MAX31855 : thermocouple %.2f °C, puce %.4f °C, défaut %u (D2-D0 = %u%u%u)\n",
           (unsigned)t32, (double)tc, (double)jf, (unsigned)((t32 >> 16) & 1u),
           (unsigned)((t32 >> 2) & 1u), (unsigned)((t32 >> 1) & 1u), (unsigned)(t32 & 1u));
    printf("Mode 1, 16 bits : 0x%04X      lu comme MAX6675  : %.2f °C, sonde ouverte %u\n",
           (unsigned)t16, (double)t6675, (unsigned)((t16 >> 2) & 1u));

    puce = verdict(t32, t16);
    if (puce == PUCE_31855)
    {
        if ((t32 >> 16) & 1u)
            printf("=> MAX31855, sonde en défaut : vérifier le thermocouple\n");
        else if (tc - jf > 15.0f || jf - tc > 15.0f)
            printf("=> MAX31855 probable, mais pointe et puce diffèrent de plus de 15 °C : relancer pointe à l'air libre\n");
        else
            printf("=> MAX31855 : compiler avec -DTEMPMOTEUR_PUCE=31855\n");
    }
    else if (puce == PUCE_6675)
    {
        if ((t16 >> 2) & 1u)
            printf("=> MAX6675, sonde ouverte : vérifier le thermocouple\n");
        else
            printf("=> MAX6675 : réglage par défaut, rien à ajouter\n");
    }
    else
        printf("=> aucun module reconnu : câblage (CS 24, DO 21, CLK 23), 3,3 V sur VCC, masse commune ?\n");
    return 0;
}

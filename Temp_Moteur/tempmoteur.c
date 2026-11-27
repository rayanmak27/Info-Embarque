//Ali
//GPIO : SPI

#define _POSIX_C_SOURCE 200809L

#include "tempmoteur.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

// Puce du module, à lire sur le composant : 6675 ou 31855
#ifndef TEMPMOTEUR_PUCE
#define TEMPMOTEUR_PUCE 6675
#endif

#if TEMPMOTEUR_PUCE == 6675
#define SPI_MODE_PUCE SPI_MODE_1
#define TRAME_OCTETS 2
#elif TEMPMOTEUR_PUCE == 31855
#define SPI_MODE_PUCE SPI_MODE_0
#define TRAME_OCTETS 4
#else
#error "TEMPMOTEUR_PUCE doit valoir 6675 ou 31855"
#endif

#define SPI_PERIPH "/dev/spidev0.0"
#define SPI_VITESSE_HZ 1000000u
#define ATTENTE_NS 250000000LL

static int spi_fd = -1;
static int64_t derniere_lecture_ns = 0;
static float derniere_valeur = 0.0f;
static int valeur_connue = 0;

static int64_t temps_ns(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + (int64_t)t.tv_nsec;
}

#if TEMPMOTEUR_PUCE == 6675

// D15 et D1 valent toujours 0, D2 signale une sonde ouverte
static int decoder(const uint8_t *rx, float *valeur)
{
    uint16_t trame = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);

    if (trame == 0 || (trame & 0x8002u) != 0)
        return TEMPMOTEUR_ERR_TRAME;
    if ((trame & 0x0004u) != 0)
        return TEMPMOTEUR_ERR_SONDE;

    *valeur = (float)(trame >> 3) * 0.25f;
    return 0;
}

#else

// D17 et D3 valent toujours 0, D16 signale un défaut de sonde
static int decoder(const uint8_t *rx, float *valeur)
{
    uint32_t trame = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16)
                   | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];

    if (trame == 0 || (trame & 0x00020008u) != 0)
        return TEMPMOTEUR_ERR_TRAME;
    if ((trame & 0x00010000u) != 0)
        return TEMPMOTEUR_ERR_SONDE;

    *valeur = (float)((int32_t)trame >> 18) * 0.25f;
    return 0;
}

#endif

int tempmoteur_init(void)
{
    uint8_t mode = SPI_MODE_PUCE;
    uint8_t bits = 8;
    uint32_t vitesse = SPI_VITESSE_HZ;

    if (spi_fd >= 0)
        return 0;

    spi_fd = open(SPI_PERIPH, O_RDWR);
    if (spi_fd < 0)
        return TEMPMOTEUR_ERR_SPI;

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0
        || ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0
        || ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &vitesse) < 0)
    {
        tempmoteur_close();
        return TEMPMOTEUR_ERR_SPI;
    }

    derniere_lecture_ns = temps_ns();
    valeur_connue = 0;
    return 0;
}

int tempmoteur_read(float *temperature)
{
    uint8_t rx[TRAME_OCTETS] = {0};
    struct spi_ioc_transfer tr;
    int64_t maintenant;
    float valeur = 0.0f;
    int erreur;

    if (spi_fd < 0 || temperature == NULL)
        return TEMPMOTEUR_ERR_SPI;

    // CS à l'état bas coupe la conversion du MAX6675 : laisser la puce finir
    maintenant = temps_ns();
    if (maintenant - derniere_lecture_ns < ATTENTE_NS)
    {
        if (!valeur_connue)
            return TEMPMOTEUR_ERR_ATTENTE;
        *temperature = derniere_valeur;
        return 0;
    }

    memset(&tr, 0, sizeof tr);
    tr.rx_buf = (uint64_t)(uintptr_t)rx;
    tr.len = TRAME_OCTETS;
    tr.speed_hz = SPI_VITESSE_HZ;
    tr.bits_per_word = 8;

    derniere_lecture_ns = maintenant;
    valeur_connue = 0;

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
        return TEMPMOTEUR_ERR_SPI;

    erreur = decoder(rx, &valeur);
    if (erreur < 0)
        return erreur;

    derniere_valeur = valeur;
    valeur_connue = 1;
    *temperature = valeur;
    return 0;
}

void tempmoteur_close(void)
{
    if (spi_fd >= 0)
    {
        close(spi_fd);
        spi_fd = -1;
    }
}

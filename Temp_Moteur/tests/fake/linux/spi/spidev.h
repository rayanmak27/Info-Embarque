#ifndef FAUX_SPIDEV_H
#define FAUX_SPIDEV_H
// Faux en-tête pour tester sur Mac : mêmes noms que <linux/spi/spidev.h>
#include <stdint.h>
#define SPI_MODE_0 0
#define SPI_MODE_1 1
struct spi_ioc_transfer {
    uint64_t tx_buf, rx_buf;
    uint32_t len, speed_hz;
    uint16_t delay_usecs;
    uint8_t bits_per_word, cs_change, tx_nbits, rx_nbits, word_delay_usecs, pad;
};
#define SPI_IOC_WR_MODE 101
#define SPI_IOC_WR_BITS_PER_WORD 102
#define SPI_IOC_WR_MAX_SPEED_HZ 103
#define SPI_IOC_MESSAGE(n) (200 + (n))
#endif

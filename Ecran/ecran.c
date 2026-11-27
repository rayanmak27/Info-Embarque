// Matthias et Alexandre
// Nextion : GPIO14 TX, GPIO15 RX

#include "ecran.h"
#include "../Uart/uart.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define VITESSE_MAX 200.0f

static int fd_ecran = -1;
static int page_courante = PAGE_PRET;
static int nb_erreurs = 0;

static int envoyer(const char *cmd)
{
    const uint8_t fin[3] = {0xFF, 0xFF, 0xFF};

    if (fd_ecran < 0) return -1;

    const uint8_t *p = (const uint8_t *)cmd;
    size_t n = strlen(cmd);

    while (n > 0) {
        ssize_t r = write(fd_ecran, p, n);

        if (r > 0) {
            p += r;
            n -= (size_t)r;
        } else if (r < 0 && (errno == EAGAIN || errno == EINTR)) {
            usleep(1000);
        } else {
            return -1;
        }
    }

    size_t i = 0;

    while (i < 3) {
        ssize_t r = write(fd_ecran, fin + i, 3 - i);

        if (r > 0) {
            i += (size_t)r;
        } else if (r < 0 && (errno == EAGAIN || errno == EINTR)) {
            usleep(1000);
        } else {
            return -1;
        }
    }

    return 0;
}

static int texte(const char *nom, const char *format, double valeur)
{
    char contenu[32];
    char cmd[80];

    snprintf(contenu, sizeof(contenu), format, valeur);
    snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", nom, contenu);

    return envoyer(cmd);
}

int ecran_init(void)
{
    fd_ecran = UART_Init(RASPI_4, 9600);

    if (fd_ecran < 0) {
        printf("[NEXTION] ERREUR ouverture UART\n");
        return -1;
    }

    printf("[NEXTION] UART ouvert : fd=%d\n", fd_ecran);

    return 0;
}

int ecran_page(int page)
{
    char cmd[24];

    snprintf(cmd, sizeof(cmd), "page %d", page);

    printf("[NEXTION TX] Commande : %s\n", cmd);
    printf("[NEXTION TX] Octets   : ");

    for (size_t i = 0; i < strlen(cmd); i++)
        printf("%02X ", (unsigned char)cmd[i]);

    printf("FF FF FF\n");

    if (envoyer(cmd) < 0) {
        printf("[NEXTION] ERREUR envoi page %d\n", page);
        return -1;
    }

    page_courante = page;

    usleep(100000);

    printf("[NEXTION] Page %d demandée\n", page);

    return 0;
}

int ecran_display(const DonneesVehicule *d)
{
    if (!d) return -1;

    int erreur = 0;
    char cmd[48];

    if (page_courante == PAGE_GYRO) {

        erreur |= texte("gX", "%.1f", d->tilt_x);
        erreur |= texte("gY", "%.1f", d->tilt_y);
        erreur |= texte("gZ", "%.1f", d->tilt_z);

        erreur |= texte("aX", "%.2f", d->accel_x);
        erreur |= texte("aY", "%.2f", d->accel_y);
        erreur |= texte("aZ", "%.2f", d->accel_z);

    } else {

        float v = d->vitesse_gps;

        if (v < 0) v = 0;
        if (v > VITESSE_MAX) v = VITESSE_MAX;

        int angle = (45 + (int)(v / VITESSE_MAX * 270)) % 360;

        snprintf(cmd, sizeof(cmd), "jVit.val=%d", angle);
        erreur |= envoyer(cmd);

        erreur |= texte("vit", "%.0f", d->vitesse_gps);
        erreur |= texte("rpm", "%.0f", d->rpm);
        erreur |= texte("tamb", "%.1f", d->temp_ambiante);
        erreur |= texte("tmot", "%.1f", d->temp_moteur);

        int secondes = (int)d->timestamp;

        if (secondes < 0) secondes = 0;

        snprintf(cmd, sizeof(cmd),
                 "chrono.txt=\"%02d:%02d\"",
                 secondes / 60,
                 secondes % 60);

        erreur |= envoyer(cmd);
    }

    return erreur ? -1 : 0;
}

int ecran_read_event(void)
{
    uint8_t c;

    static int ff = 0;
    static int debut = -1;

    if (fd_ecran < 0) return -1;

    while (read(fd_ecran, &c, 1) == 1) {

        // Affichage des octets reçus
        printf("[NEXTION RX] %02X\n", c);

        // Détection immédiate des boutons
        if (c == EVT_START ||
            c == EVT_PAUSE ||
            c == EVT_FIN) {

            printf("[NEXTION] Bouton détecté : 0x%02X\n", c);

            ff = 0;
            debut = -1;

            return c;
        }

        // Détection des réponses Nextion
        if (debut < 0)
            debut = c;

        if (c == 0xFF)
            ff++;
        else
            ff = 0;

        if (ff == 3) {

            if (debut == 0x1A) {
                nb_erreurs++;
                printf("[NEXTION] ERREUR : composant invalide (0x1A)\n");
            }

            ff = 0;
            debut = -1;
        }
    }

    return EVT_AUCUN;
}

int ecran_erreurs(void)
{
    return nb_erreurs;
}

int ecran_text(const char *cmd)
{
    return envoyer(cmd);
}

int ecran_clear(void)
{
    return ecran_page(PAGE_PRET);
}

void ecran_close(void)
{
    if (fd_ecran >= 0)
        UART_Close(fd_ecran);

    fd_ecran = -1;
}
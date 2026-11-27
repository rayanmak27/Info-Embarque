// Matthias et Alexandre
// Nextion : GPIO14 TX, GPIO15 RX, 9600 bauds

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
    char valeur_txt[32];
    char cmd[80];

    snprintf(valeur_txt, sizeof(valeur_txt), format, valeur);
    snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", nom, valeur_txt);

    return envoyer(cmd);
}

int ecran_init(void)
{
    fd_ecran = UART_Init(RASPI_4, 9600);

    if (fd_ecran < 0) {
        printf("[NEXTION] ERREUR ouverture UART (code=%d)\n", fd_ecran);
        return -1;
    }

    printf("[NEXTION] UART ouvert : fd=%d\n", fd_ecran);
    return 0;
}

int ecran_page(int page)
{
    char cmd[24];

    snprintf(cmd, sizeof(cmd), "page %d", page);

    if (envoyer(cmd) < 0) {
        printf("[NEXTION] ERREUR changement de page\n");
        return -1;
    }

    page_courante = page;
    usleep(100000);

    printf("[NEXTION TX] page %d\n", page);

    return 0;
}

int ecran_display(const DonneesVehicule *d)
{
    if (!d) return -1;

    int erreur = 0;
    char cmd[48];

    // Page d'accueil : pas de mise à jour
    if (page_courante == PAGE_PRET)
        return 0;

    // Page gyroscope
    if (page_courante == PAGE_GYRO) {

        erreur |= texte("gX", "%.1f", d->tilt_x);
        erreur |= texte("gY", "%.1f", d->tilt_y);
        erreur |= texte("gZ", "%.1f", d->tilt_z);

        erreur |= texte("aX", "%.2f", d->accel_x);
        erreur |= texte("aY", "%.2f", d->accel_y);
        erreur |= texte("aZ", "%.2f", d->accel_z);

        return erreur ? -1 : 0;
    }

    // Pages mesure et pause
    if (page_courante == PAGE_MESURE ||
        page_courante == PAGE_PAUSE) {

        float vitesse = d->vitesse_gps;

        if (vitesse < 0) vitesse = 0;
        if (vitesse > VITESSE_MAX) vitesse = VITESSE_MAX;

        int angle = (45 + (int)(vitesse / VITESSE_MAX * 270)) % 360;

        // Nom corrigé : jvit en minuscules
        snprintf(cmd, sizeof(cmd), "jvit.val=%d", angle);
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

/*
 * Réception Nextion :
 * A1 = démarrer / continuer
 * A2 = pause
 * A3 = terminer
 * 66 XX FF FF FF = numéro de page (réponse à sendme)
 * 1A FF FF FF = commande invalide
 */
int ecran_read_event(void)
{
    static uint8_t paquet[16];
    static int n = 0;
    static int ff = 0;

    uint8_t c;

    if (fd_ecran < 0) return -1;

    while (read(fd_ecran, &c, 1) == 1) {

        // Boutons : un seul octet
        if (n == 0 &&
            (c == EVT_START ||
             c == EVT_PAUSE ||
             c == EVT_FIN)) {

            printf("[NEXTION RX] Bouton : 0x%02X\n", c);
            return c;
        }

        // Reconstitution des réponses Nextion
        if (n < (int)sizeof(paquet)) {
            paquet[n++] = c;
        } else {
            n = 0;
            ff = 0;
            continue;
        }

        if (c == 0xFF)
            ff++;
        else
            ff = 0;

        if (ff >= 3) {

            if (paquet[0] == 0x66 && n >= 5) {

                int nouvelle_page = paquet[1];

                if (nouvelle_page != page_courante) {
                    page_courante = nouvelle_page;

                    printf("[NEXTION RX] Page active : %d\n",
                           page_courante);
                }
            }

            else if (paquet[0] == 0x1A) {
                nb_erreurs++;
                printf("[NEXTION] ERREUR composant (0x1A)\n");
            }

            n = 0;
            ff = 0;
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
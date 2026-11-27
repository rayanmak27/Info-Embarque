/*
 * Test du GPS NEO-6M sur le Raspberry Pi, avec le vrai module branché.
 *
 * Le programme lit l'UART en continu et affiche, selon la commande tapée :
 *     b + Entrée  ->  trames BRUTES (le texte NMEA tel qu'il sort du GPS)
 *     t + Entrée  ->  trames TRAITÉES (GpsData décodé par gps_Read)
 *     d + Entrée  ->  les DEUX
 *     p + Entrée  ->  pause (les données continuent d'être lues mais pas affichées)
 *     h + Entrée  ->  aide
 *     q + Entrée  ->  quitter
 *
 * Comment ça marche : le programme lit lui-même l'UART, affiche les octets
 * bruts si demandé, puis les recopie dans un pipe interne. C'est ce pipe qui
 * est donné à gps_init() : le module GPS décode donc exactement les mêmes
 * octets que ceux affichés en brut, sans que les deux se volent les données.
 *
 * Compilation (sur le Pi, depuis la racine du projet) :
 *     gcc -Wall -Wextra -std=gnu2x -I Raspi -I Uart -o test_gps_raspi \
 *         GPS/test_gps_raspi.c GPS/gps.c Uart/uart.c Raspi/raspi.c
 *
 * Lancement :
 *     ./test_gps_raspi          modèle de Pi détecté automatiquement
 *     ./test_gps_raspi 4        forcer le modèle (valeur de l'enum RASPI)
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "gps.h"
#include "../Raspi/raspi.h"
#include "../Uart/uart.h"

#define GPS_BAUD          9600   /* vitesse d'usine du NEO-6M */
#define SILENCE_ALERTE_S  3      /* alerte si rien reçu pendant ce temps */

typedef enum { MODE_BRUT, MODE_TRAITE, MODE_DEUX, MODE_PAUSE } Mode;

static const char *nom_mode(Mode m)
{
    switch (m) {
        case MODE_BRUT:   return "BRUT";
        case MODE_TRAITE: return "TRAITE";
        case MODE_DEUX:   return "BRUT + TRAITE";
        default:          return "PAUSE";
    }
}

static void afficher_aide(void)
{
    printf("\nCommandes (lettre puis Entree) :\n"
           "  b  trames brutes (NMEA)\n"
           "  t  trames traitees (GpsData)\n"
           "  d  les deux\n"
           "  p  pause\n"
           "  h  cette aide\n"
           "  q  quitter\n\n");
}

/* Lit /proc/device-tree/model pour deviner le modèle de Raspberry Pi */
static int detecter_raspi(RASPI *r)
{
    char modele[128] = {0};
    FILE *f = fopen("/proc/device-tree/model", "r");
    if (!f) return 0;
    size_t n = fread(modele, 1, sizeof(modele) - 1, f);
    fclose(f);
    modele[n] = '\0';

    printf("Carte detectee : %s\n", modele);

    /* Les tests les plus précis d'abord ("Zero 2 W" contient aussi "Zero") */
    if      (strstr(modele, "Zero 2 W"))         *r = RASPI_ZERO_2W;
    else if (strstr(modele, "Zero W"))           *r = RASPI_ZERO_1W;
    else if (strstr(modele, "Zero"))             *r = RASPI_ZERO_1;
    else if (strstr(modele, "Raspberry Pi 400")) *r = RASPI_400;
    else if (strstr(modele, "Raspberry Pi 5"))   *r = RASPI_5;
    else if (strstr(modele, "Raspberry Pi 4"))   *r = RASPI_4;
    else if (strstr(modele, "Raspberry Pi 3"))   *r = RASPI_3;
    else if (strstr(modele, "Raspberry Pi 2"))   *r = RASPI_2;
    else if (strstr(modele, "Raspberry Pi"))     *r = RASPI_1;
    else return 0;
    return 1;
}

static const char *texte_erreur_uart(int code)
{
    switch (code) {
        case UART_ERR_BAUDRATE_INVALID:          return "vitesse invalide";
        case UART_ERR_RASPI_INVALID:             return "modele de Raspberry Pi invalide";
        case UART_ERR_IMPOSSIBLE_TO_READ_UART:   return "impossible d'ouvrir le port "
                                                        "(UART active ? groupe dialout ?)";
        case UART_ERR_IMPOSSIBLE_TO_INIT_STRUCT: return "tcgetattr a echoue";
        case UART_ERR_IMPOSSIBLE_TO_PARAM_UART:  return "tcsetattr a echoue";
        default:                                 return "erreur inconnue";
    }
}

/* Affiche les octets bruts ligne par ligne : on garde les morceaux jusqu'au
 * '\n' pour qu'une ligne [TRAITE] ne vienne jamais se glisser au milieu.
 * Texte NMEA tel quel, octets bizarres affichés en <xx>. */
static void afficher_brut(const char *buf, ssize_t n)
{
    static char ligne[512];
    static size_t len = 0;

    for (ssize_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)buf[i];
        if (c == '\r') continue;

        if (c == '\n') {
            if (len > 0) {
                ligne[len] = '\0';
                printf("[BRUT]   %s\n", ligne);
                len = 0;
            }
        } else if (len < sizeof(ligne) - 5) {
            if (c >= 32 && c < 127)
                ligne[len++] = (char)c;
            else
                len += (size_t)snprintf(ligne + len, 5, "<%02X>", c);
        }
    }
    fflush(stdout);
}

static void afficher_traite(const GpsData *d, int nb_trames)
{
    printf("[TRAITE] %s %s | %d trame(s) | fix=%s qualite=%d sat=%d hdop=%.2f\n",
           d->heure_utc[0] ? d->heure_utc : "--:--:--",
           d->date[0]      ? d->date      : "--/--/----",
           nb_trames,
           d->fix ? "OUI" : "non",
           d->qualite, d->satellites, d->hdop);

    if (d->fix)
        printf("         lat=%.6f lon=%.6f alt=%.1f m  vitesse=%.2f km/h  cap=%.1f deg\n",
               d->latitude, d->longitude, d->altitude_m, d->vitesse_kmh, d->cap_deg);
    else
        printf("         pas encore de position (mettre l'antenne face au ciel)\n");
    fflush(stdout);
}

int main(int argc, char *argv[])
{
    RASPI raspi;

    if (argc > 1) {
        raspi = (RASPI)atoi(argv[1]);
    } else if (!detecter_raspi(&raspi)) {
        fprintf(stderr, "Modele non reconnu, passe-le en argument (ex : %s 4)\n", argv[0]);
        return 1;
    }

    if (!Rapsi_IsValid(raspi)) {
        fprintf(stderr, "Modele %d invalide\n", (int)raspi);
        return 1;
    }

    const char *port = Raspi_GetUArt_Folder(raspi);
    int uart = UART_Init(raspi, GPS_BAUD);
    if (uart < 0) {
        fprintf(stderr, "UART_Init(%s, %d) : %s\n", port, GPS_BAUD, texte_erreur_uart(uart));
        return 1;
    }
    printf("Port %s ouvert a %d bauds\n", port, GPS_BAUD);

    /* Pipe interne : nous écrivons dedans, le module GPS lit dedans */
    int p[2];
    if (pipe(p) < 0) { perror("pipe"); UART_Close(uart); return 1; }
    fcntl(p[0], F_SETFL, fcntl(p[0], F_GETFL) | O_NONBLOCK);
    gps_init(p[0]);

    GpsData gps = {0};
    Mode mode = MODE_TRAITE;
    time_t derniere_donnee = time(NULL);
    int alerte_affichee = 0;

    afficher_aide();
    printf("Mode : %s\n", nom_mode(mode));

    for (;;) {
        fd_set lecture;
        FD_ZERO(&lecture);
        FD_SET(STDIN_FILENO, &lecture);
        FD_SET(uart, &lecture);

        struct timeval attente = { .tv_sec = 1, .tv_usec = 0 };
        int max_fd = uart > STDIN_FILENO ? uart : STDIN_FILENO;

        if (select(max_fd + 1, &lecture, NULL, NULL, &attente) < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        /* ---- Commande de l'utilisateur ---- */
        if (FD_ISSET(STDIN_FILENO, &lecture)) {
            char cmd[32];
            if (!fgets(cmd, sizeof(cmd), stdin)) break;     /* Ctrl+D */

            Mode ancien = mode;
            switch (cmd[0]) {
                case 'b': mode = MODE_BRUT;   break;
                case 't': mode = MODE_TRAITE; break;
                case 'd': mode = MODE_DEUX;   break;
                case 'p': mode = MODE_PAUSE;  break;
                case 'h': afficher_aide();    break;
                case 'q': goto fin;
                case '\n': break;
                default:  printf("Commande inconnue, tape h pour l'aide\n"); break;
            }
            if (mode != ancien) printf("--- Mode : %s ---\n", nom_mode(mode));
        }

        /* ---- Données du GPS ---- */
        if (FD_ISSET(uart, &lecture)) {
            char buf[256];
            ssize_t n = read(uart, buf, sizeof(buf));
            if (n < 0 && errno != EAGAIN && errno != EINTR) {
                perror("read uart");
                break;
            }
            if (n > 0) {
                derniere_donnee = time(NULL);
                alerte_affichee = 0;

                if (mode == MODE_BRUT || mode == MODE_DEUX)
                    afficher_brut(buf, n);

                if (write(p[1], buf, (size_t)n) != n) {
                    perror("write pipe");
                    break;
                }

                int nb = gps_Read(&gps);
                if (nb < 0) {
                    fprintf(stderr, "gps_Read : erreur\n");
                    break;
                }
                if (nb > 0 && (mode == MODE_TRAITE || mode == MODE_DEUX))
                    afficher_traite(&gps, nb);
            }
        }

        /* ---- Rien ne vient du GPS ? ---- */
        if (!alerte_affichee && time(NULL) - derniere_donnee >= SILENCE_ALERTE_S) {
            printf("!! Aucune donnee depuis %d s : verifier le cablage (TX GPS -> GPIO15), "
                   "l'alimentation, enable_uart=1 et la console serie.\n", SILENCE_ALERTE_S);
            fflush(stdout);
            alerte_affichee = 1;
        }
    }

fin:
    printf("Fin du test.\n");
    close(p[0]);
    close(p[1]);
    UART_Close(uart);
    return 0;
}

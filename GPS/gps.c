#include "gps.h"

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define GPS_LIGNE_MAX 128      /* une trame NMEA fait au maximum 82 caractères */
#define GPS_CHAMPS_MAX 20

static int  g_uart = -1;
static char g_ligne[GPS_LIGNE_MAX];
static int  g_len  = 0;

void gps_init(int uart)
{
    g_uart = uart;
    g_len  = 0;
}

/* ---------- Outils ---------- */

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Checksum NMEA = XOR de tous les caractères entre '$' et '*' */
static int checksum_ok(const char *ligne)
{
    unsigned char somme = 0;
    const char *p = ligne + 1;              /* on saute le '$' */

    while (*p && *p != '*')
        somme ^= (unsigned char)*p++;

    if (*p != '*') return 0;                /* pas de checksum */
    int h = hex_val(p[1]), l = hex_val(p[2]);
    if (h < 0 || l < 0) return 0;
    return somme == (unsigned char)(h * 16 + l);
}

/* Découpe la ligne sur les ',' (garde les champs vides, contrairement à strtok) */
static int decouper(char *s, char *champs[], int max)
{
    int n = 0;
    champs[n++] = s;
    for (; *s; s++) {
        if (*s == '*') { *s = '\0'; break; }
        if (*s == ',') {
            *s = '\0';
            if (n < max) champs[n++] = s + 1;
        }
    }
    return n;
}

/* "ddmm.mmmm" + 'N'/'S'  ->  degrés décimaux. Retourne 0 si champ vide. */
static int convertir_coord(const char *val, const char *hemi, double *res)
{
    if (val[0] == '\0' || hemi[0] == '\0') return 0;
    double v    = atof(val);
    int    deg  = (int)(v / 100);
    double min  = v - deg * 100;
    double dec  = deg + min / 60.0;
    if (hemi[0] == 'S' || hemi[0] == 'W') dec = -dec;
    *res = dec;
    return 1;
}

/* "hhmmss.ss" -> "hh:mm:ss" */
static void copier_heure(const char *src, char *dst)
{
    if (strlen(src) < 6) return;
    dst[0] = src[0]; dst[1] = src[1]; dst[2] = ':';
    dst[3] = src[2]; dst[4] = src[3]; dst[5] = ':';
    dst[6] = src[4]; dst[7] = src[5]; dst[8] = '\0';
}

/* "ddmmyy" -> "jj/mm/20aa" */
static void copier_date(const char *src, char *dst)
{
    if (strlen(src) < 6) return;
    dst[0] = src[0]; dst[1] = src[1]; dst[2] = '/';
    dst[3] = src[2]; dst[4] = src[3]; dst[5] = '/';
    dst[6] = '2';    dst[7] = '0';
    dst[8] = src[4]; dst[9] = src[5]; dst[10] = '\0';
}

/* ---------- Décodage des trames ---------- */

/* $GPRMC,heure,statut,lat,N/S,lon,E/W,vitesse_noeuds,cap,date,...  */
static int decoder_rmc(char *c[], int n, GpsData *d)
{
    if (n < 10) return 0;

    copier_heure(c[1], d->heure_utc);
    copier_date(c[9], d->date);

    d->fix = (c[2][0] == 'A');
    if (!d->fix) return 1;                  /* trame valide, mais pas de position */

    convertir_coord(c[3], c[4], &d->latitude);
    convertir_coord(c[5], c[6], &d->longitude);
    if (c[7][0]) d->vitesse_kmh = atof(c[7]) * 1.852;   /* noeuds -> km/h */
    if (c[8][0]) d->cap_deg     = atof(c[8]);
    return 1;
}

/* $GPGGA,heure,lat,N/S,lon,E/W,qualite,nb_sat,hdop,altitude,M,...  */
static int decoder_gga(char *c[], int n, GpsData *d)
{
    if (n < 10) return 0;

    copier_heure(c[1], d->heure_utc);
    d->qualite    = atoi(c[6]);
    d->satellites = atoi(c[7]);
    if (c[8][0]) d->hdop = atof(c[8]);

    if (d->qualite > 0) {
        convertir_coord(c[2], c[3], &d->latitude);
        convertir_coord(c[4], c[5], &d->longitude);
        if (c[9][0]) d->altitude_m = atof(c[9]);
    }
    return 1;
}

static int traiter_ligne(char *ligne, GpsData *d)
{
    if (ligne[0] != '$' || !checksum_ok(ligne)) return 0;

    char *champs[GPS_CHAMPS_MAX];
    int n = decouper(ligne + 1, champs, GPS_CHAMPS_MAX);

    /* champs[0] = "GPRMC" : on ignore les 2 lettres de l'émetteur (GP, GN...) */
    if (strlen(champs[0]) != 5) return 0;
    const char *type = champs[0] + 2;

    if (strcmp(type, "RMC") == 0) return decoder_rmc(champs, n, d);
    if (strcmp(type, "GGA") == 0) return decoder_gga(champs, n, d);
    return 0;                               /* GSV, GSA, VTG... ignorées */
}

/* ---------- Lecture ---------- */

int gps_Read(GpsData *d)
{
    if (g_uart < 0 || d == NULL) return -1;

    char buf[256];
    int  nb_trames = 0;

    for (;;) {
        ssize_t n = read(g_uart, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
            return -1;
        }
        if (n == 0) break;                  /* plus rien à lire pour l'instant */

        for (ssize_t i = 0; i < n; i++) {
            char ch = buf[i];

            if (ch == '$') {                /* début de trame : on repart à zéro */
                g_len = 0;
                g_ligne[g_len++] = ch;
            } else if (ch == '\r' || ch == '\n') {
                if (g_len > 0) {            /* fin de trame */
                    g_ligne[g_len] = '\0';
                    nb_trames += traiter_ligne(g_ligne, d);
                    g_len = 0;
                }
            } else if (g_len > 0) {         /* on n'accumule qu'après un '$' */
                if (g_len < GPS_LIGNE_MAX - 1)
                    g_ligne[g_len++] = ch;
                else
                    g_len = 0;              /* trop long : trame corrompue */
            }
        }
    }
    return nb_trames;
}
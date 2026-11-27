//Matthias et Alexandre
//GPIO14 : UART TX
#include "ecran.h"
#include "../Uart/uart.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
 
/* ---- noms des composants Nextion : à vérifier dans Nextion Editor (champ objname) ---- */
#define C_VITESSE   "vit"     /* le GRAND nombre (pas l'unité uVit) */
#define C_AIGUILLE  "jVit"    /* composant Gauge */
#define C_RPM       "rpm"
#define C_TAMB      "tamb"
#define C_TMOT      "tmot"
#define C_CHRONO    "chrono"
 
#define VITESSE_MAX     200.0f   /* fin d'échelle du cadran (km/h) */
#define ANGLE_ZERO      45       /* angle Nextion pour 0 km/h : À CALIBRER */
#define ANGLE_BALAYAGE  270      /* amplitude totale de l'aiguille en degrés */
 
static int fd_ecran = -1;
static int page_courante = PAGE_PRET;
static int nb_erreurs = 0;
 
/* écrit TOUS les octets (le port est non bloquant) */
static int write_all(const void *buf, size_t n)
{
    const uint8_t *p = buf;
    int essais = 0;
    while (n > 0) {
        ssize_t w = write(fd_ecran, p, n);
        if (w < 0) {
            if ((errno == EAGAIN || errno == EINTR) && ++essais < 1000) {
                usleep(1000);
                continue;
            }
            return -1;
        }
        p += w;
        n -= (size_t)w;
    }
    return 0;
}
 
static int nextion_send(const char *cmd)
{
    static const uint8_t fin[3] = {0xFF, 0xFF, 0xFF};
    if (fd_ecran < 0) return -1;
    if (write_all(cmd, strlen(cmd)) < 0) return -1;
    return write_all(fin, 3);
}
 
static int set_txt(const char *composant, const char *fmt, double valeur)
{
    char valeur_txt[24], cmd[64];
    snprintf(valeur_txt, sizeof valeur_txt, fmt, valeur);
    snprintf(cmd, sizeof cmd, "%s.txt=\"%s\"", composant, valeur_txt);
    return nextion_send(cmd);
}
 
static int vitesse_vers_angle(float v)
{
    if (v < 0)           v = 0;
    if (v > VITESSE_MAX) v = VITESSE_MAX;
    return (ANGLE_ZERO + (int)(v / VITESSE_MAX * ANGLE_BALAYAGE)) % 360;
}
 
static int set_chrono(const char *composant, double secondes)
{
    if (secondes < 0) secondes = 0;
    int total = (int)secondes;
 
    char cmd[48];
    snprintf(cmd, sizeof cmd, "%s.txt=\"%02d:%02d\"", composant, total / 60, total % 60);
    return nextion_send(cmd);
}
 
int ecran_init(void)
{
    fd_ecran = UART_Init(RASPI_4, 9600);   /* adaptez le modèle de Pi */
    return (fd_ecran < 0) ? -1 : 0;
}
 
/* N'envoie que les composants de la page affichée : un composant absent
 * de la page courante fait répondre l'écran « 0x1A FF FF FF ». */
int ecran_display(const DonneesVehicule *d)
{
    char cmd[24];
    int err = 0;
 
    if (d == NULL) return -1;
 
    switch (page_courante) {
    case PAGE_PRET:
    case PAGE_MESURE:
    case PAGE_PAUSE:
        snprintf(cmd, sizeof cmd, C_AIGUILLE ".val=%d", vitesse_vers_angle(d->vitesse_gps));
        err |= nextion_send(cmd);
        err |= set_txt(C_VITESSE, "%.0f", d->vitesse_gps);
        err |= set_txt(C_RPM,     "%.0f", d->rpm);
        err |= set_txt(C_TAMB,    "%.1f", d->temp_ambiante);
        err |= set_txt(C_TMOT,    "%.1f", d->temp_moteur);
        err |= set_chrono(C_CHRONO, d->timestamp);
        break;
 
    case PAGE_GYRO:
        err |= set_txt("gX", "%.1f", d->tilt_x);
        err |= set_txt("gY", "%.1f", d->tilt_y);
        err |= set_txt("gZ", "%.1f", d->tilt_z);
        err |= set_txt("aX", "%.2f", d->accel_x);
        err |= set_txt("aY", "%.2f", d->accel_y);
        err |= set_txt("aZ", "%.2f", d->accel_z);
        break;
    }
    return err ? -1 : 0;
}
 
int ecran_page(int n)
{
    char cmd[16];
    snprintf(cmd, sizeof cmd, "page %d", n);
    if (nextion_send(cmd) < 0) return -1;
    page_courante = n;
    usleep(100000);   /* l'écran met un moment à charger la page */
    return 0;
}
 
/* ---- réception : l'écran envoie des messages terminés par FF FF FF ----
 *   0x1A FF FF FF : nom de composant / attribut invalide
 *   0x01 FF FF FF : commande exécutée (selon bkcmd)
 *   A1/A2/A3      : boutons (printh A1, ou printh A1 FF FF FF)
 * Un octet A1/A2/A3 en début de message est pris tout de suite comme événement. */
static int ecran_feed(uint8_t c)
{
    static uint8_t buf[16];
    static int n = 0, ff = 0;
 
    if (n == 0 && ff == 0 && (c == EVT_START || c == EVT_PAUSE || c == EVT_FIN))
        return c;
 
    if (n < (int)sizeof buf) buf[n++] = c;
    ff = (c == 0xFF) ? ff + 1 : 0;
 
    if (ff >= 3) {                       /* message complet */
        if (buf[0] == 0x1A) nb_erreurs++;
        n = 0; ff = 0;
    }
    return EVT_AUCUN;
}
 
int ecran_read_event(void)
{
    uint8_t c;
    if (fd_ecran < 0) return -1;
    while (read(fd_ecran, &c, 1) == 1) {
        int e = ecran_feed(c);
        if (e != EVT_AUCUN) return e;
    }
    return EVT_AUCUN;
}
 
int ecran_erreurs(void)           { return nb_erreurs; }
int ecran_text(const char *texte) { return nextion_send(texte); }  /* commande brute */
int ecran_clear(void)             { return nextion_send("page 0"); }
void ecran_close(void)            { UART_Close(fd_ecran); fd_ecran = -1; }
 
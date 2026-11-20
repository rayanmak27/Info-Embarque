//Matthias et Alexandre
//GPIO14 : UART TX
#include "ecran.h"
#include "../Uart/uart.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#define VITESSE_MAX     200.0f   /* fin d'échelle du cadran (km/h) */
#define ANGLE_ZERO      45       /* angle Nextion pour 0 km/h : À CALIBRER */
#define ANGLE_BALAYAGE  270      /* amplitude totale de l'aiguille en degrés */

#define EVT_AUCUN  0
#define EVT_START  0xA1
#define EVT_PAUSE  0xA2
#define EVT_FIN    0xA3

static int fd_ecran = -1;

static int nextion_send(const char *cmd)
{
    static const uint8_t fin[3] = {0xFF, 0xFF, 0xFF};
    if (fd_ecran < 0) return -1;
    if (write(fd_ecran, cmd, strlen(cmd)) < 0) return -1;
    if (write(fd_ecran, fin, 3) < 0) return -1;
    return 0;
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
    snprintf(cmd, sizeof cmd, "%s.txt=\"%02d:%02d:%02d\"",
             composant, total / 3600, (total % 3600) / 60, total % 60);
    return nextion_send(cmd);
}

int ecran_init(void)
{
    fd_ecran = UART_Init(RASPI_4, 9600);   /* adaptez le modèle de Pi */
    return (fd_ecran < 0) ? -1 : 0;
}

int ecran_display(const DonneesVehicule *d)
{
    char cmd[24];

    /* aiguille du compteur */
    snprintf(cmd, sizeof cmd, "jVit.val=%d", vitesse_vers_angle(d->vitesse_gps));
    nextion_send(cmd);

    /* valeurs principales */
    set_txt("rpm",  "%.0f", d->rpm);
    set_txt("vit",  "%.0f", d->vitesse_gps);
    set_txt("tamb", "%.1f", d->temp_ambiante);
    set_txt("tmot", "%.1f", d->temp_moteur);

    /* gyroscope et accélération */
    set_txt("gX", "%.1f", d->gyro_x);
    set_txt("gY", "%.1f", d->gyro_y);
    set_txt("gZ", "%.1f", d->gyro_z);
    set_txt("aX", "%.2f", d->accel_x);
    set_txt("aY", "%.2f", d->accel_y);
    set_txt("aZ", "%.2f", d->accel_z);

    set_chrono("chrono", d->timestamp);
    return 0;
}

int ecran_read_event(void)
{
    uint8_t c;
    if (fd_ecran < 0) return -1;
    return (read(fd_ecran, &c, 1) == 1) ? c : EVT_AUCUN;
}

int ecran_text(const char *texte) { return nextion_send(texte); }  /* commande brute */
int ecran_clear(void)             { return nextion_send("page 0"); }
void ecran_close(void)            { UART_Close(fd_ecran); fd_ecran = -1; }
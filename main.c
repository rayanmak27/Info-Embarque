#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <gpiod.h>

#include "DataLog/datalog.h"
#include "Ecran/ecran.h"
#include "GPS/gps.h"
#include "Gyro/gyro.h"
#include "Temp_Ambiante/tempamb.h"
#include "Temp_Moteur/tempmoteur.h"

#define GPIO_IMPULSION 27

static volatile sig_atomic_t actif = 1;

static struct gpiod_chip *chip;
static struct gpiod_line_request *request;
static struct gpiod_edge_event_buffer *buffer;

static uint64_t derniere_impulsion = 0;

static void arreter(int sig)
{
    (void)sig;
    actif = 0;
}

static double temps(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);

    return t.tv_sec + t.tv_nsec / 1e9;
}

static int gps_ouvrir(void)
{
    int fd = open("/dev/ttyAMA5", O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (fd < 0) return -1;

    struct termios t;

    if (tcgetattr(fd, &t) < 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&t);
    cfsetispeed(&t, B9600);
    cfsetospeed(&t, B9600);

    t.c_cflag |= CLOCAL | CREAD;

    if (tcsetattr(fd, TCSANOW, &t) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

static int impulsions_init(void)
{
    chip = gpiod_chip_open("/dev/gpiochip0");

    if (!chip) return -1;

    struct gpiod_line_settings *s = gpiod_line_settings_new();
    struct gpiod_line_config *c = gpiod_line_config_new();

    if (!s || !c) {
        gpiod_line_settings_free(s);
        gpiod_line_config_free(c);
        return -1;
    }

    unsigned int gpio = GPIO_IMPULSION;

    gpiod_line_settings_set_direction(
        s, GPIOD_LINE_DIRECTION_INPUT
    );

    gpiod_line_settings_set_edge_detection(
        s, GPIOD_LINE_EDGE_RISING
    );

    gpiod_line_settings_set_event_clock(
        s, GPIOD_LINE_CLOCK_MONOTONIC
    );

    gpiod_line_config_add_line_settings(c, &gpio, 1, s);

    request = gpiod_chip_request_lines(chip, NULL, c);
    buffer = gpiod_edge_event_buffer_new(16);

    gpiod_line_settings_free(s);
    gpiod_line_config_free(c);

    return (request && buffer) ? 0 : -1;
}

static void impulsions_lire(float *rpm)
{
    if (!request || !buffer) return;

    if (gpiod_line_request_wait_edge_events(request, 0) > 0) {

        int n = gpiod_line_request_read_edge_events(
            request, buffer, 16
        );

        for (int i = 0; i < n; i++) {

            struct gpiod_edge_event *e =
                gpiod_edge_event_buffer_get_event(buffer, i);

            uint64_t instant =
                gpiod_edge_event_get_timestamp_ns(e);

            if (derniere_impulsion && instant > derniere_impulsion)
                *rpm = 60e9f / (float)(instant - derniere_impulsion);

            derniere_impulsion = instant;
        }
    }

    if (derniere_impulsion &&
        (uint64_t)(temps() * 1e9) - derniere_impulsion > 6000000000ULL) {

        *rpm = 0;
        derniere_impulsion = 0;
    }
}

int main(void)
{
    signal(SIGINT, arreter);
    signal(SIGTERM, arreter);

    setvbuf(stdout, NULL, _IONBF, 0);

    DonneesVehicule d = {0};
    GpsData gps = {0};

    int mesure = 0;
    int pause = 0;
    int cycle = 0;

    double debut = 0;
    double debut_pause = 0;
    double duree_pause = 0;

    printf("\n=== PROJET INFO EMBARQUEE ===\n");

    int amb_ok = tempamb_init() == 0;
    int mot_ok = tempmoteur_init() == 0;
    int gyro_ok = gyro_init() == 0;
    int imp_ok = impulsions_init() == 0;

    int gps_fd = gps_ouvrir();
    int ecran_ok = ecran_init() == 0;

    printf("[TEMP AMBIANTE] %s\n", amb_ok ? "OK" : "ERREUR");
    printf("[TEMP MOTEUR]  %s\n", mot_ok ? "OK" : "ERREUR");
    printf("[GYRO]         %s\n", gyro_ok ? "OK" : "ERREUR");
    printf("[IMPULSIONS]   %s\n", imp_ok ? "OK" : "ERREUR");
    printf("[GPS]          %s\n", gps_fd >= 0 ? "OK" : "ERREUR");
    printf("[NEXTION]      %s\n", ecran_ok ? "OK" : "ERREUR");

    if (gps_fd >= 0)
        gps_init(gps_fd);

    if (ecran_ok)
        ecran_page(PAGE_PRET);

    printf("\n[SYSTEME] En attente des boutons Nextion...\n\n");

    while (actif) {

        double maintenant = temps();

        // Lecture des boutons Nextion
        int evt = ecran_ok ? ecran_read_event() : EVT_AUCUN;

        if (evt == EVT_START) {

            if (!mesure) {

                mesure = 1;
                pause = 0;

                debut = maintenant;
                duree_pause = 0;

                d.timestamp = 0;

                printf("\n[CHRONO] DEMARRER\n");

                if (ecran_ok)
                    ecran_page(PAGE_MESURE);

            } else if (pause) {

                duree_pause += maintenant - debut_pause;
                pause = 0;

                printf("\n[CHRONO] CONTINUER\n");

                if (ecran_ok)
                    ecran_page(PAGE_MESURE);
            }
        }

        else if (evt == EVT_PAUSE) {

            if (mesure && !pause) {

                pause = 1;
                debut_pause = maintenant;

                printf("\n[CHRONO] PAUSE\n");

                if (ecran_ok)
                    ecran_page(PAGE_PAUSE);
            }
        }

        else if (evt == EVT_FIN) {

            if (mesure) {

                mesure = 0;
                pause = 0;

                d.timestamp = 0;

                printf("\n[CHRONO] FIN\n");

                if (ecran_ok)
                    ecran_page(PAGE_PRET);
            }
        }

        // Calcul du chronomètre
        if (mesure) {

            double reference = pause ? debut_pause : maintenant;

            d.timestamp = reference - debut - duree_pause;
        }

        // Lecture des capteurs
        switch (cycle) {

        case 0:
            if (imp_ok)
                impulsions_lire(&d.rpm);
            break;

        case 1:
            if (gyro_ok) {

                gyro_get_accel(
                    &d.accel_x,
                    &d.accel_y,
                    &d.accel_z
                );

                gyro_get_angles(
                    &d.tilt_x,
                    &d.tilt_y,
                    &d.tilt_z
                );
            }
            break;

        case 2:
            if (gps_fd >= 0 && gps_Read(&gps) > 0 && gps.fix) {

                d.latitude = gps.latitude;
                d.longitude = gps.longitude;
                d.altitude = gps.altitude_m;
                d.vitesse_gps = gps.vitesse_kmh;
            }
            break;

        case 7:
            if (amb_ok)
                tempamb_read(&d.temp_ambiante);
            break;

        case 8:
            if (mot_ok)
                tempmoteur_read(&d.temp_moteur);
            break;

        case 9:
            if (ecran_ok && ecran_display(&d) < 0)
                printf("[NEXTION] ERREUR d'envoi des valeurs\n");

            if (mesure && !pause)
                datalog_write(d);

            break;
        }

        cycle = (cycle + 1) % 10;

        usleep(100000);
    }

    printf("\n[SYSTEME] Arrêt...\n");

    if (ecran_ok)
        ecran_close();

    if (gps_fd >= 0)
        close(gps_fd);

    if (buffer)
        gpiod_edge_event_buffer_free(buffer);

    if (request)
        gpiod_line_request_release(request);

    if (chip)
        gpiod_chip_close(chip);

    tempamb_close();
    tempmoteur_close();

    printf("[SYSTEME] Programme terminé\n");

    return 0;
}
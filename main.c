#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <gpiod.h>

#include "DataLog/datalog.h"
#include "Ecran/ecran.h"
#include "GPS/gps.h"
#include "Gyro/gyro.h"
#include "Temp_Ambiante/tempamb.h"
#include "Temp_Moteur/tempmoteur.h"

#define GPS_PORT "/dev/ttyAMA5"
#define PERIODE_NS 100000000L
#define GPIO_IMPULSION 27
#define IMPULSIONS_PAR_TOUR 1
#define TIMEOUT_IMPULSIONS_NS 6000000000ULL

static volatile sig_atomic_t actif = 1;

static struct gpiod_chip *chip = NULL;
static struct gpiod_line_request *request = NULL;
static struct gpiod_edge_event_buffer *buffer = NULL;

static uint64_t derniere_impulsion = 0;

static void arreter(int sig)
{
    (void)sig;
    actif = 0;
}

static double temps_actuel(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);

    return t.tv_sec + t.tv_nsec / 1000000000.0;
}

static uint64_t temps_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);

    return (uint64_t)t.tv_sec * 1000000000ULL + t.tv_nsec;
}

static void attendre_100ms(struct timespec *prochain)
{
    prochain->tv_nsec += PERIODE_NS;

    if (prochain->tv_nsec >= 1000000000L)
    {
        prochain->tv_sec++;
        prochain->tv_nsec -= 1000000000L;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, prochain, NULL);
}

static int gps_ouvrir(void)
{
    int fd = open(GPS_PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (fd < 0)
    {
        perror("GPS");
        return -1;
    }

    struct termios t;

    if (tcgetattr(fd, &t) < 0)
    {
        close(fd);
        return -1;
    }

    cfmakeraw(&t);

    t.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    t.c_cflag |= CS8 | CLOCAL | CREAD;

    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;

    cfsetispeed(&t, B9600);
    cfsetospeed(&t, B9600);

    if (tcsetattr(fd, TCSANOW, &t) < 0)
    {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIFLUSH);

    return fd;
}

static int impulsions_ouvrir(void)
{
    struct gpiod_line_settings *settings = NULL;
    struct gpiod_line_config *config = NULL;
    struct gpiod_request_config *req_config = NULL;

    unsigned int offset = GPIO_IMPULSION;
    int resultat = -1;

    chip = gpiod_chip_open("/dev/gpiochip0");

    if (!chip)
        return -1;

    settings = gpiod_line_settings_new();
    config = gpiod_line_config_new();
    req_config = gpiod_request_config_new();

    if (!settings || !config || !req_config)
        goto fin;

    gpiod_line_settings_set_direction(
        settings, GPIOD_LINE_DIRECTION_INPUT
    );

    gpiod_line_settings_set_edge_detection(
        settings, GPIOD_LINE_EDGE_RISING
    );

    gpiod_line_settings_set_event_clock(
        settings, GPIOD_LINE_CLOCK_MONOTONIC
    );

    if (gpiod_line_config_add_line_settings(
            config, &offset, 1, settings) < 0)
        goto fin;

    gpiod_request_config_set_consumer(
        req_config, "Info-Embarque"
    );

    request = gpiod_chip_request_lines(
        chip, req_config, config
    );

    if (!request)
        goto fin;

    buffer = gpiod_edge_event_buffer_new(16);

    if (!buffer)
        goto fin;

    resultat = 0;

fin:
    if (settings)
        gpiod_line_settings_free(settings);

    if (config)
        gpiod_line_config_free(config);

    if (req_config)
        gpiod_request_config_free(req_config);

    return resultat;
}

static void impulsions_lire(float *rpm)
{
    if (!request || !buffer)
        return;

    // Attente nulle : ne bloque jamais la boucle principale.
    int disponible = gpiod_line_request_wait_edge_events(request, 0);

    if (disponible > 0)
    {
        int n = gpiod_line_request_read_edge_events(
            request, buffer, 16
        );

        for (int i = 0; i < n; i++)
        {
            struct gpiod_edge_event *event =
                gpiod_edge_event_buffer_get_event(buffer, i);

            uint64_t instant =
                gpiod_edge_event_get_timestamp_ns(event);

            if (derniere_impulsion > 0 &&
                instant > derniere_impulsion)
            {
                double dt =
                    (instant - derniere_impulsion) / 1e9;

                *rpm = (float)(
                    60.0 / (dt * IMPULSIONS_PAR_TOUR)
                );
            }

            derniere_impulsion = instant;
        }
    }

    if (derniere_impulsion > 0 &&
        temps_ns() - derniere_impulsion > TIMEOUT_IMPULSIONS_NS)
    {
        *rpm = 0.0f;
        derniere_impulsion = 0;
    }
}

static void impulsions_fermer(void)
{
    if (buffer)
        gpiod_edge_event_buffer_free(buffer);

    if (request)
        gpiod_line_request_release(request);

    if (chip)
        gpiod_chip_close(chip);
}

int main(void)
{
    DonneesVehicule donnees = {0};
    GpsData gps = {0};

    int gps_fd = -1;
    int ecran_ok = 0;

    int mesure = 0;
    int pause = 0;
    int creneau = 0;

    double debut_chrono = 0;
    double debut_pause = 0;
    double temps_pause = 0;

    struct timespec prochain;

    signal(SIGINT, arreter);
    signal(SIGTERM, arreter);

    printf("Initialisation...\n");

    if (tempamb_init() < 0)
        fprintf(stderr, "Température ambiante indisponible\n");

    if (tempmoteur_init() < 0)
        fprintf(stderr, "Température moteur indisponible\n");

    if (gyro_init() < 0)
        fprintf(stderr, "Gyroscope indisponible\n");

    if (impulsions_ouvrir() < 0)
        fprintf(stderr, "Capteur d'impulsions indisponible\n");

    gps_fd = gps_ouvrir();

    if (gps_fd >= 0)
        gps_init(gps_fd);

    if (ecran_init() == 0)
    {
        ecran_ok = 1;
        ecran_page(PAGE_PRET);
    }
    else
    {
        fprintf(stderr, "Écran Nextion indisponible\n");
    }

    printf("Programme démarré\n");

    clock_gettime(CLOCK_MONOTONIC, &prochain);

    while (actif)
    {
        double maintenant = temps_actuel();

        // Événements envoyés par les boutons Nextion.
        int evenement = ecran_ok ? ecran_read_event() : EVT_AUCUN;

        if (evenement == EVT_START)
        {
            if (!mesure)
            {
                mesure = 1;
                pause = 0;

                debut_chrono = maintenant;
                temps_pause = 0;
                donnees.timestamp = 0;

                derniere_impulsion = 0;
                donnees.rpm = 0;

                ecran_page(PAGE_MESURE);

                printf("DÉMARRER\n");
            }
            else if (pause)
            {
                temps_pause += maintenant - debut_pause;
                pause = 0;

                ecran_page(PAGE_MESURE);

                printf("CONTINUER\n");
            }
        }
        else if (evenement == EVT_PAUSE)
        {
            if (mesure && !pause)
            {
                pause = 1;
                debut_pause = maintenant;

                ecran_page(PAGE_PAUSE);

                printf("PAUSE\n");
            }
        }
        else if (evenement == EVT_FIN)
        {
            if (mesure)
            {
                mesure = 0;
                pause = 0;

                donnees.timestamp = 0;

                ecran_page(PAGE_PRET);

                printf("FIN\n");
            }
        }

        if (mesure)
        {
            double reference = pause ? debut_pause : maintenant;

            donnees.timestamp =
                reference - debut_chrono - temps_pause;
        }

        // Créneau 0 : impulsions moteur
        if (creneau == 0)
        {
            impulsions_lire(&donnees.rpm);
        }

        // Créneau 1 : MPU6050
        if (creneau == 1)
        {
            gyro_get_accel(
                &donnees.accel_x,
                &donnees.accel_y,
                &donnees.accel_z
            );

            gyro_get_angles(
                &donnees.tilt_x,
                &donnees.tilt_y,
                &donnees.tilt_z
            );
        }

        // Créneau 2 : GPS
        if (creneau == 2 && gps_fd >= 0)
        {
            if (gps_Read(&gps) > 0)
            {
                donnees.latitude = gps.latitude;
                donnees.longitude = gps.longitude;
                donnees.vitesse_gps = (float)gps.vitesse_kmh;
                donnees.altitude = (float)gps.altitude_m;
            }
        }

        // Créneau 7 : température ambiante
        if (creneau == 7)
        {
            float temperature;

            if (tempamb_read(&temperature) == 0)
                donnees.temp_ambiante = temperature;
        }

        // Créneau 8 : température moteur
        if (creneau == 8)
        {
            float temperature;

            if (tempmoteur_read(&temperature) == 0)
                donnees.temp_moteur = temperature;
        }

        // Créneau 9 : écran + CSV
        if (creneau == 9)
        {
            if (ecran_ok)
                ecran_display(&donnees);

            if (mesure && !pause)
                datalog_write(donnees);
        }

        creneau = (creneau + 1) % 10;

        attendre_100ms(&prochain);
    }

    printf("Arrêt du programme...\n");

    if (ecran_ok)
        ecran_close();

    if (gps_fd >= 0)
        close(gps_fd);

    impulsions_fermer();

    tempamb_close();
    tempmoteur_close();

    return 0;
}
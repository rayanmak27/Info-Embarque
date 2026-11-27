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

#define DEBUG 1

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

    return (double)t.tv_sec +
           (double)t.tv_nsec / 1000000000.0;
}

static uint64_t temps_ns(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);

    return (uint64_t)t.tv_sec * 1000000000ULL +
           (uint64_t)t.tv_nsec;
}

static void attendre_100ms(struct timespec *prochain)
{
    prochain->tv_nsec += PERIODE_NS;

    if (prochain->tv_nsec >= 1000000000L)
    {
        prochain->tv_sec++;
        prochain->tv_nsec -= 1000000000L;
    }

    clock_nanosleep(
        CLOCK_MONOTONIC,
        TIMER_ABSTIME,
        prochain,
        NULL
    );
}

static int gps_ouvrir(void)
{
    int fd = open(
        GPS_PORT,
        O_RDONLY | O_NOCTTY | O_NONBLOCK
    );

    if (fd < 0)
    {
        perror("[GPS] Ouverture");
        return -1;
    }

    struct termios t;

    if (tcgetattr(fd, &t) < 0)
    {
        perror("[GPS] tcgetattr");
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
        perror("[GPS] tcsetattr");
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
    {
        perror("[IMPULSIONS] gpiochip0");
        return -1;
    }

    settings = gpiod_line_settings_new();
    config = gpiod_line_config_new();
    req_config = gpiod_request_config_new();

    if (!settings || !config || !req_config)
        goto fin;

    gpiod_line_settings_set_direction(
        settings,
        GPIOD_LINE_DIRECTION_INPUT
    );

    gpiod_line_settings_set_edge_detection(
        settings,
        GPIOD_LINE_EDGE_RISING
    );

    gpiod_line_settings_set_event_clock(
        settings,
        GPIOD_LINE_CLOCK_MONOTONIC
    );

    gpiod_line_settings_set_debounce_period_us(
        settings,
        500
    );

    if (gpiod_line_config_add_line_settings(
            config,
            &offset,
            1,
            settings) < 0)
        goto fin;

    gpiod_request_config_set_consumer(
        req_config,
        "Info-Embarque"
    );

    request = gpiod_chip_request_lines(
        chip,
        req_config,
        config
    );

    if (!request)
    {
        perror("[IMPULSIONS] Réservation GPIO27");
        goto fin;
    }

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
    if (!request || !buffer || !rpm)
        return;

    int disponible =
        gpiod_line_request_wait_edge_events(request, 0);

    if (disponible < 0)
    {
#if DEBUG
        printf("[IMPULSIONS] Erreur de lecture des événements\n");
#endif
        return;
    }

    if (disponible > 0)
    {
        int n = gpiod_line_request_read_edge_events(
            request,
            buffer,
            16
        );

        if (n < 0)
        {
#if DEBUG
            printf("[IMPULSIONS] Impossible de lire les fronts\n");
#endif
            return;
        }

        for (int i = 0; i < n; i++)
        {
            struct gpiod_edge_event *event =
                gpiod_edge_event_buffer_get_event(
                    buffer,
                    i
                );

            uint64_t instant =
                gpiod_edge_event_get_timestamp_ns(event);

            if (derniere_impulsion > 0 &&
                instant > derniere_impulsion)
            {
                double dt =
                    (double)(instant - derniere_impulsion) / 1e9;

                *rpm = (float)(
                    60.0 /
                    (dt * IMPULSIONS_PAR_TOUR)
                );
            }

            derniere_impulsion = instant;
        }
    }

    if (derniere_impulsion > 0 &&
        temps_ns() - derniere_impulsion >
            TIMEOUT_IMPULSIONS_NS)
    {
        *rpm = 0.0f;
        derniere_impulsion = 0;
    }
}

static void impulsions_fermer(void)
{
    if (buffer)
    {
        gpiod_edge_event_buffer_free(buffer);
        buffer = NULL;
    }

    if (request)
    {
        gpiod_line_request_release(request);
        request = NULL;
    }

    if (chip)
    {
        gpiod_chip_close(chip);
        chip = NULL;
    }
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

    double debut_chrono = 0.0;
    double debut_pause = 0.0;
    double temps_pause = 0.0;

    struct timespec prochain;

    signal(SIGINT, arreter);
    signal(SIGTERM, arreter);

    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\n");
    printf("=====================================\n");
    printf("      PROJET INFO EMBARQUEE\n");
    printf("         MODE DIAGNOSTIC\n");
    printf("=====================================\n\n");

    // Initialisation des modules

    printf("=== INITIALISATION DES MODULES ===\n");

    int rc = tempamb_init();

    printf("[TEMP AMBIANTE] %s (code=%d)\n",
           rc == 0 ? "OK" : "ERREUR",
           rc);

    rc = tempmoteur_init();

    printf("[TEMP MOTEUR]  %s (code=%d)\n",
           rc == 0 ? "OK" : "ERREUR",
           rc);

    rc = gyro_init();

    printf("[GYROSCOPE]    %s (code=%d)\n",
           rc == 0 ? "OK" : "ERREUR",
           rc);

    rc = impulsions_ouvrir();

    printf("[IMPULSIONS]   %s (code=%d)\n",
           rc == 0 ? "OK" : "ERREUR",
           rc);

    gps_fd = gps_ouvrir();

    printf("[GPS UART5]    %s (fd=%d)\n",
           gps_fd >= 0 ? "OK" : "ERREUR",
           gps_fd);

    if (gps_fd >= 0)
        gps_init(gps_fd);

    rc = ecran_init();

    printf("[NEXTION UART] %s (code=%d)\n",
           rc == 0 ? "OK" : "ERREUR",
           rc);

    if (rc == 0)
    {
        ecran_ok = 1;

        rc = ecran_page(PAGE_PRET);

        printf("[NEXTION PAGE] %s (code=%d)\n",
               rc == 0 ? "OK" : "ERREUR",
               rc);
    }

    printf("==================================\n\n");

    printf("[SYSTEME] Programme démarré\n");
    printf("[SYSTEME] Cycle de 100 ms activé\n");
    printf("[SYSTEME] Ctrl+C pour arrêter\n\n");

    clock_gettime(CLOCK_MONOTONIC, &prochain);

    while (actif)
    {
        double maintenant = temps_actuel();

        // Vérification des boutons Nextion

        int evenement =
            ecran_ok ? ecran_read_event() : EVT_AUCUN;

        if (evenement != EVT_AUCUN)
        {
#if DEBUG
            printf("[NEXTION] Événement reçu : 0x%02X\n",
                   (unsigned int)(evenement & 0xFF));
#endif
        }

        if (evenement == EVT_START)
        {
            if (!mesure)
            {
                mesure = 1;
                pause = 0;

                debut_chrono = maintenant;
                temps_pause = 0.0;

                donnees.timestamp = 0.0;
                donnees.rpm = 0.0f;
                derniere_impulsion = 0;

                if (ecran_ok)
                    ecran_page(PAGE_MESURE);

                printf("[CHRONO] DÉMARRER\n");
            }
            else if (pause)
            {
                temps_pause +=
                    maintenant - debut_pause;

                pause = 0;

                if (ecran_ok)
                    ecran_page(PAGE_MESURE);

                printf("[CHRONO] CONTINUER\n");
            }
        }

        else if (evenement == EVT_PAUSE)
        {
            if (mesure && !pause)
            {
                pause = 1;
                debut_pause = maintenant;

                if (ecran_ok)
                    ecran_page(PAGE_PAUSE);

                printf("[CHRONO] PAUSE\n");
            }
        }

        else if (evenement == EVT_FIN)
        {
            if (mesure)
            {
                mesure = 0;
                pause = 0;

                donnees.timestamp = 0.0;

                if (ecran_ok)
                    ecran_page(PAGE_PRET);

                printf("[CHRONO] FIN\n");
            }
        }

        // Calcul du chronomètre

        if (mesure)
        {
            double reference =
                pause ? debut_pause : maintenant;

            donnees.timestamp =
                reference - debut_chrono - temps_pause;
        }

        // Créneau 0 : impulsions

        if (creneau == 0)
        {
            impulsions_lire(&donnees.rpm);

#if DEBUG
            printf("[IMPULSIONS] RPM = %.2f\n",
                   donnees.rpm);
#endif
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

#if DEBUG
            printf(
                "[GYRO] ACCEL : X=%.2f Y=%.2f Z=%.2f | "
                "ANGLES : X=%.2f Y=%.2f Z=%.2f\n",
                donnees.accel_x,
                donnees.accel_y,
                donnees.accel_z,
                donnees.tilt_x,
                donnees.tilt_y,
                donnees.tilt_z
            );
#endif
        }

        // Créneau 2 : GPS

        if (creneau == 2 && gps_fd >= 0)
        {
            int resultat = gps_Read(&gps);

            if (resultat > 0)
            {
                donnees.latitude = gps.latitude;
                donnees.longitude = gps.longitude;

                donnees.vitesse_gps =
                    (float)gps.vitesse_kmh;

                donnees.altitude =
                    (float)gps.altitude_m;

#if DEBUG
                printf(
                    "[GPS] Fix=%d | Satellites=%d | "
                    "Lat=%.6f Lon=%.6f | V=%.2f km/h\n",
                    gps.fix,
                    gps.satellites,
                    donnees.latitude,
                    donnees.longitude,
                    donnees.vitesse_gps
                );
#endif
            }
            else if (resultat < 0)
            {
#if DEBUG
                printf("[GPS] Erreur de lecture UART\n");
#endif
            }
        }

        // Créneau 7 : température ambiante

        if (creneau == 7)
        {
            float temperature;

            int resultat =
                tempamb_read(&temperature);

            if (resultat == 0)
            {
                donnees.temp_ambiante =
                    temperature;

#if DEBUG
                printf("[TEMP AMBIANTE] %.2f °C\n",
                       donnees.temp_ambiante);
#endif
            }
            else
            {
#if DEBUG
                printf(
                    "[TEMP AMBIANTE] Erreur de lecture (%d)\n",
                    resultat
                );
#endif
            }
        }

        // Créneau 8 : température moteur

        if (creneau == 8)
        {
            float temperature;

            int resultat =
                tempmoteur_read(&temperature);

            if (resultat == 0)
            {
                donnees.temp_moteur =
                    temperature;

#if DEBUG
                printf("[TEMP MOTEUR] %.2f °C\n",
                       donnees.temp_moteur);
#endif
            }
            else
            {
#if DEBUG
                printf(
                    "[TEMP MOTEUR] Erreur de lecture (%d)\n",
                    resultat
                );
#endif
            }
        }

        // Créneau 9 : écran, chrono et CSV

        if (creneau == 9)
        {
#if DEBUG
            printf(
                "[CHRONO] mesure=%d | pause=%d | temps=%.2f s\n",
                mesure,
                pause,
                donnees.timestamp
            );
#endif

            if (ecran_ok)
            {
                int resultat =
                    ecran_display(&donnees);

#if DEBUG
                printf(
                    "[ECRAN] Envoi=%s | code=%d | erreurs=%d\n",
                    resultat == 0 ? "OK" : "ERREUR",
                    resultat,
                    ecran_erreurs()
                );
#endif
            }
            else
            {
#if DEBUG
                printf("[ECRAN] UART non initialisé\n");
#endif
            }

            if (mesure && !pause)
            {
                datalog_write(donnees);

#if DEBUG
                printf("[DATALOG] Ligne enregistrée\n");
#endif
            }

#if DEBUG
            printf("-------------------------------------\n");
#endif
        }

        creneau = (creneau + 1) % 10;

        attendre_100ms(&prochain);
    }

    // Libération des ressources

    printf("\n[SYSTEME] Arrêt du programme...\n");

    if (ecran_ok)
        ecran_close();

    if (gps_fd >= 0)
        close(gps_fd);

    impulsions_fermer();

    tempamb_close();
    tempmoteur_close();

    printf("[SYSTEME] Ressources libérées\n");
    printf("[SYSTEME] Programme terminé\n");

    return 0;
}
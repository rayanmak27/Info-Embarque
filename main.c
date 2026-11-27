#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "DataLog/datalog.h"
#include "Ecran/ecran.h"
#include "GPS/gps.h"
#include "Gyro/gyro.h"
#include "Impulsion/impulsion.h"
#include "Temp_Ambiante/tempamb.h"
#include "Temp_Moteur/tempmoteur.h"

#define GPS_PORT "/dev/ttyAMA5"
#define PERIODE_NS 100000000L
#define IMPULSIONS_PAR_TOUR 1

static volatile sig_atomic_t programme_actif = 1;

static double temps_actuel(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);

    return (double)t.tv_sec + (double)t.tv_nsec / 1000000000.0;
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

static void arreter(int signal_recu)
{
    (void)signal_recu;
    programme_actif = 0;
}

static int gps_ouvrir(void)
{
    int fd = open(GPS_PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (fd < 0)
    {
        perror("Ouverture GPS");
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

int main(void)
{
    DonneesVehicule donnees = {0};
    GpsData gps = {0};

    int gps_fd = -1;
    int mesure = 0;
    int pause = 0;
    int creneau = 0;

    double debut_chrono = 0.0;
    double debut_pause = 0.0;
    double temps_pause = 0.0;

    struct timespec prochain;

    signal(SIGINT, arreter);
    signal(SIGTERM, arreter);

    printf("Initialisation du système embarqué...\n");

    if (tempamb_init() < 0)
        fprintf(stderr, "Erreur : température ambiante\n");

    if (tempmoteur_init() < 0)
        fprintf(stderr, "Erreur : température moteur\n");

    if (gyro_init() < 0)
        fprintf(stderr, "Erreur : gyroscope\n");

    if (impulsion_init() < 0)
    {
        fprintf(stderr, "Erreur : capteur d'impulsions\n");
    }
    else
    {
        if (impulsion_configurer(IMPULSIONS_PAR_TOUR) < 0)
            fprintf(stderr, "Erreur : configuration des impulsions\n");
    }

    gps_fd = gps_ouvrir();

    if (gps_fd >= 0)
        gps_init(gps_fd);

    if (ecran_init() < 0)
    {
        fprintf(stderr, "Erreur : écran Nextion\n");
    }
    else
    {
        ecran_page(PAGE_PRET);
    }

    printf("Initialisation terminée.\n");

    clock_gettime(CLOCK_MONOTONIC, &prochain);

    while (programme_actif)
    {
        double maintenant = temps_actuel();

        // Boutons tactiles de l'écran Nextion
        int evenement = ecran_read_event();

        if (evenement == EVT_START)
        {
            if (!mesure)
            {
                mesure = 1;
                pause = 0;

                debut_chrono = maintenant;
                temps_pause = 0.0;
                donnees.timestamp = 0.0;

                impulsion_reset();
                ecran_page(PAGE_MESURE);

                printf("Mesure démarrée\n");
            }
            else if (pause)
            {
                temps_pause += maintenant - debut_pause;
                pause = 0;

                ecran_page(PAGE_MESURE);

                printf("Mesure reprise\n");
            }
        }

        else if (evenement == EVT_PAUSE)
        {
            if (mesure && !pause)
            {
                pause = 1;
                debut_pause = maintenant;

                ecran_page(PAGE_PAUSE);

                printf("Mesure en pause\n");
            }
        }

        else if (evenement == EVT_FIN)
        {
            if (mesure)
            {
                mesure = 0;
                pause = 0;

                donnees.timestamp = 0.0;

                ecran_page(PAGE_PRET);

                printf("Mesure terminée\n");
            }
        }

        // Chronomètre
        if (mesure)
        {
            double temps_reference = pause ? debut_pause : maintenant;

            donnees.timestamp =
                temps_reference - debut_chrono - temps_pause;
        }

        // Créneau 0 : impulsions moteur
        if (creneau == 0)
        {
            float rpm;

            if (impulsion_read(&rpm) == 0)
                donnees.rpm = rpm;
        }

        // Créneau 1 : gyroscope
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

        // Créneau 9 : écran et enregistrement
        if (creneau == 9)
        {
            ecran_display(&donnees);

            if (mesure && !pause)
                datalog_write(donnees);
        }

        creneau = (creneau + 1) % 10;

        attendre_100ms(&prochain);
    }

    printf("\nArrêt du programme...\n");

    ecran_close();

    if (gps_fd >= 0)
        close(gps_fd);

    impulsion_close();
    tempamb_close();
    tempmoteur_close();

    printf("Programme terminé.\n");

    return 0;
}
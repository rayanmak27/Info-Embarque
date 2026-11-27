//Donald
//GPIO27 : mesure
#define _GNU_SOURCE
#include "gpio.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
/* Buffer d'événements partagé par v_moteur() et gpio_cleanup() */
static struct gpiod_edge_event_buffer *buf = NULL;

/* 1. Initialisation du contrôleur */
struct gpiod_chip *gpio_init(void)
{
    struct gpiod_chip *chip = gpiod_chip_open(CHIP_PATH);

    if (chip == NULL)
    {
        perror("Erreur ouverture GPIO");
    }

    return chip;
}

/* 2. Configuration et réservation des broches */
struct gpiod_line_request *gpio_configure(
    struct gpiod_chip *chip)
{
    struct gpiod_line_settings *speed_s_settings = NULL;
    struct gpiod_line_settings *gbf_s_settings = NULL;

    struct gpiod_line_config *line_cfg = NULL;
    struct gpiod_request_config *req_cfg = NULL;

    struct gpiod_line_request *request = NULL;

    unsigned int speed_s_offset = SENSOR_OFFSET;
    unsigned int gbf_s_offset = ACTUATOR_OFFSET;

    /* Création des paramètres du capteur (speed_s) */
    speed_s_settings = gpiod_line_settings_new();

    if (speed_s_settings == NULL)
        goto cleanup;

    gpiod_line_settings_set_direction(
        speed_s_settings,
        GPIOD_LINE_DIRECTION_INPUT);

    /* Activation de la détection des fronts montants (impulsions) */
    gpiod_line_settings_set_edge_detection(
        speed_s_settings,
        GPIOD_LINE_EDGE_RISING);

    /* Anti-rebond (500 µs) et horloge monotone pour les timestamps */
    gpiod_line_settings_set_debounce_period_us(
        speed_s_settings,
        500);

    gpiod_line_settings_set_event_clock(
        speed_s_settings,
        GPIOD_LINE_CLOCK_MONOTONIC);

    /* Création des paramètres de l'actionneur (gbf_s) */
    gbf_s_settings = gpiod_line_settings_new();

    if (gbf_s_settings == NULL)
        goto cleanup;

    gpiod_line_settings_set_direction(
        gbf_s_settings,
        GPIOD_LINE_DIRECTION_OUTPUT);

    gpiod_line_settings_set_output_value(
        gbf_s_settings,
        GPIOD_LINE_VALUE_INACTIVE);

    /* Création de la configuration des lignes */
    line_cfg = gpiod_line_config_new();

    if (line_cfg == NULL)
        goto cleanup;

    /* Association du capteur (speed_s) à GPIO 27 */
    if (gpiod_line_config_add_line_settings(
            line_cfg,
            &speed_s_offset,
            1,
            speed_s_settings) < 0)
        goto cleanup;

    /* Association de l'actionneur (gbf_s) à GPIO 17 */
    if (gpiod_line_config_add_line_settings(
            line_cfg,
            &gbf_s_offset,
            1,
            gbf_s_settings) < 0)
        goto cleanup;

    /* Configuration de la demande */
    req_cfg = gpiod_request_config_new();

    if (req_cfg == NULL)
        goto cleanup;

    gpiod_request_config_set_consumer(
        req_cfg,
        "Mesure_vitesses");

    /* Réservation des deux broches */
    request = gpiod_chip_request_lines(
        chip,
        req_cfg,
        line_cfg);

    if (request == NULL)
        perror("Erreur demande GPIO");

cleanup:

    if (req_cfg != NULL)
        gpiod_request_config_free(req_cfg);

    if (line_cfg != NULL)
        gpiod_line_config_free(line_cfg);

    if (speed_s_settings != NULL)
        gpiod_line_settings_free(speed_s_settings);

    if (gbf_s_settings != NULL)
        gpiod_line_settings_free(gbf_s_settings);

    return request;
}

/* 3. Lecture du capteur */
enum gpiod_line_value gpio_read_sensor(
    struct gpiod_line_request *request)
{
    return gpiod_line_request_get_value(
        request,
        SENSOR_OFFSET);
}

/* 4. Commande de l'actionneur */
int gpio_control_actuator(
    struct gpiod_line_request *request,
    enum gpiod_line_value value)
{
    return gpiod_line_request_set_value(
        request,
        ACTUATOR_OFFSET,
        value);
}

/* 5. Vitesse du moteur (km/h) à partir des impulsions du capteur */
double v_moteur(struct gpiod_line_request *request)
{
    static uint64_t t_prec = 0;

    if (buf == NULL)
    {
        buf = gpiod_edge_event_buffer_new(1);
        if (buf == NULL)
            return -1.0;
    }

    while (1)
    {
        int r = gpiod_line_request_wait_edge_events(
            request,
            PULSE_TIMEOUT_NS);

        if (r < 0)
            return -1.0; /* erreur ou interruption */

        if (r == 0)
        {
            t_prec = 0; /* timeout : moteur à l'arrêt */
            return 0.0;
        }

        if (gpiod_line_request_read_edge_events(request, buf, 1) < 1)
            return -1.0;

        struct gpiod_edge_event *ev =
            gpiod_edge_event_buffer_get_event(buf, 0);
        uint64_t t = gpiod_edge_event_get_timestamp_ns(ev);

        if (t_prec == 0)
        {
            t_prec = t; /* première impulsion : on attend la suivante */
            continue;
        }

        double dt = (t - t_prec) / 1e9;
        t_prec = t;

        if (dt <= 0.0)
            continue;

        double v = (2.0 * M_PI * R / N) / dt; /* m/s */
        return v * 3.6;                       /* km/h */
    }
}

/* 6. Libération des ressources */
void gpio_cleanup(
    struct gpiod_chip *chip,
    struct gpiod_line_request *request)
{
    if (buf != NULL)
    {
        gpiod_edge_event_buffer_free(buf);
        buf = NULL;
    }

    if (request != NULL)
        gpiod_line_request_release(request);

    if (chip != NULL)
        gpiod_chip_close(chip);
}
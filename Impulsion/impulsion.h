#ifndef GPIO_H
#define GPIO_H

#include <gpiod.h>
#include <stdint.h>

/* Définition des constantes */
#define CHIP_PATH "/dev/gpiochip0"
#define SPEED_S_OFFSET 27
#define GBF_S_OFFSET 17

/* Noms utilisés dans gpio.c */
#define SENSOR_OFFSET SPEED_S_OFFSET
#define ACTUATOR_OFFSET GBF_S_OFFSET

/* Paramètres mécaniques (à adapter) */
#define R 0.30 /* rayon de la roue en mètres */
#define N 1    /* nombre d'impulsions par tour */

/* Temps max sans impulsion avant de considérer le moteur à l'arrêt (ns) */
#define PULSE_TIMEOUT_NS 6000000000LL

/* Déclaration des fonctions */

/* Initialisation du contrôleur GPIO */
struct gpiod_chip *gpio_init(void);

/* Configuration et réservation des broches */
struct gpiod_line_request *gpio_configure(
    struct gpiod_chip *chip);

/* Lecture du capteur (speed_s) */
enum gpiod_line_value gpio_read_sensor(
    struct gpiod_line_request *request);

/* Commande de l'actionneur (gbf_s) */
int gpio_control_actuator(
    struct gpiod_line_request *request,
    enum gpiod_line_value value);

/* Vitesse du moteur en km/h, calculée à partir des impulsions du capteur.
 * Bloque jusqu'à la prochaine mesure complète (deux impulsions successives).
 * Retour : vitesse en km/h
 *          0.0 si aucune impulsion pendant PULSE_TIMEOUT_NS (arrêt)
 *         -1.0 en cas d'erreur */
double v_moteur(struct gpiod_line_request *request);

/* Libération des ressources */
void gpio_cleanup(
    struct gpiod_chip *chip,
    struct gpiod_line_request *request);

#endif
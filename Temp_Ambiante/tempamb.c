/* tempamb.c - DS18B20 (GPIO17, 1-Wire) via le pilote noyau w1_therm.
 *
 * Fonctionnement non bloquant : chaque appel à tempamb_read()
 *   1) lit le résultat de la conversion lancée à l'appel précédent,
 *   2) lance une nouvelle conversion (écriture "trigger", retour immédiat).
 * Appelé 1 fois par seconde, la conversion (~750 ms) est toujours finie.
 *
 * Codes de retour : 0 = OK, valeur négative = -errno
 *   -EAGAIN : premier appel, aucune conversion encore disponible
 *   -EBUSY  : conversion pas finie (appels trop rapprochés)
 *   -EIO    : valeur invalide / erreur CRC
 *   -ENODEV : sonde absente ; -ENOTSUP : noyau trop ancien (< 4.19)
 * En cas d'erreur, *temperature n'est pas modifié.
 */
#define _POSIX_C_SOURCE 200809L
#include "tempamb.h"

#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define W1_DIR            "/sys/bus/w1/devices/"
#define PATH_LEN          192
#define TEMP_MIN_MC       (-55000)  /* plage du DS18B20, en milli-°C */
#define TEMP_MAX_MC       125000
#define TEMP_POWER_ON_MC  85000     /* valeur de reset = conversion non faite */

static char s_bulk_path[PATH_LEN];  /* .../w1_bus_masterN/therm_bulk_read */
static char s_temp_path[PATH_LEN];  /* .../28-xxxxxxxxxxxx/temperature    */
static int  s_ready   = 0;
static int  s_pending = 0;          /* une conversion a été lancée */

static int find_path(const char *pattern, const char *suffix,
                     char *out, size_t len)
{
    glob_t g;
    if (glob(pattern, 0, NULL, &g) != 0)
        return -ENODEV;
    int n = snprintf(out, len, "%s%s", g.gl_pathv[0], suffix);
    globfree(&g);
    return (n < 0 || (size_t)n >= len) ? -ENAMETOOLONG : 0;
}

static int read_text(const char *path, char *buf, size_t len)
{
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return -errno;
    ssize_t n = read(fd, buf, len - 1);
    int e = errno;
    close(fd);
    if (n < 0)
        return -e;
    buf[n] = '\0';
    return 0;
}

static int trigger(void)
{
    int fd = open(s_bulk_path, O_WRONLY | O_CLOEXEC);
    if (fd < 0)
        return -errno;
    ssize_t n = write(fd, "trigger", 7);
    int e = errno;
    close(fd);
    if (n != 7)
        return (n < 0) ? -e : -EIO;
    return 0;
}

int tempamb_init(void)
{
    int rc;

    s_ready = 0;
    s_pending = 0;

    rc = find_path(W1_DIR "w1_bus_master*", "/therm_bulk_read",
                   s_bulk_path, sizeof s_bulk_path);
    if (rc < 0)
        return rc;
    rc = find_path(W1_DIR "28-*", "/temperature",
                   s_temp_path, sizeof s_temp_path);
    if (rc < 0)
        return rc;
    if (access(s_bulk_path, W_OK) != 0)
        return -ENOTSUP;

    s_ready = 1;
    return trigger() == 0 ? (s_pending = 1, 0) : 0; /* 1re conversion lancée */
}

int tempamb_read(float *temperature)
{
    char buf[32];
    int rc, result;

    if (!s_ready || temperature == NULL)
        return -ENODEV;

    if (!s_pending) {                 /* rien à lire : on (re)lance */
        rc = trigger();
        if (rc == 0)
            s_pending = 1;
        return rc < 0 ? rc : -EAGAIN;
    }

    /* État : "1" en cours, "0" terminée, "-1" erreur */
    rc = read_text(s_bulk_path, buf, sizeof buf);
    if (rc < 0)
        return rc;
    long st = strtol(buf, NULL, 10);
    if (st == 1)
        return -EBUSY;                /* on garde s_pending : on réessaiera */
    if (st != 0) {
        s_pending = 0;
        return -EIO;
    }

    /* Température en milli-°C (CRC déjà vérifié par le pilote) */
    result = 0;
    rc = read_text(s_temp_path, buf, sizeof buf);
    if (rc < 0) {
        result = rc;
    } else {
        char *end;
        long mc = strtol(buf, &end, 10);
        if (end == buf || mc < TEMP_MIN_MC || mc > TEMP_MAX_MC
            || mc == TEMP_POWER_ON_MC)
            result = -EIO;
        else
            *temperature = (float)mc / 1000.0f;
    }

    /* Relance immédiatement la conversion suivante */
    s_pending = (trigger() == 0);
    return result;
}

void tempamb_close(void)
{
    s_ready = 0;
    s_pending = 0;
}
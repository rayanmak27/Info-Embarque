#include "gps.h"

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

int gps_Read(GpsData *d)
{
    
}

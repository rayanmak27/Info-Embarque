#ifndef GPS_H
#define GPS_H

// GPIO15 : RXD (broche 10)
// UART : 9600 bauds

typedef struct
{
    double latitude;
    double longitude;

    float vitesse;
    float altitude;

    int fix;

} GPSData;

int GPS_init(void);
int GPS_read(GPSData *donnees);
void GPS_close(void);

#endif
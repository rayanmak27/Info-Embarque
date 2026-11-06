#include <stdio.h>
#include <unistd.h>

typedef struct Mesures{
    double vitesse_tr_min;
    float gyro[3];
    double lat, lon;
    float temperature;
};

void logger(const Mesures m)
{
    FILE *fptr;
    fptr = fopen("/~/log.csv", "a");
    fprintf(fptr, "%f,%f,%f,%f,%f,%f,%f\n", m.vitesse_tr_min, m.gyro[0], m.gyro[1], m.gyro[2], m.lat, m.lon, m.temperature);
    fclose(fptr);



}
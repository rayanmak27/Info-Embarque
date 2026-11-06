//Rayan

#include <stdio.h>
#include <unistd.h>


void logger(const DonneesVehicule m)
{
    FILE *fptr;
    fptr = fopen("/~/log.csv", "a");
    fprintf(fptr, "%f,%f,%f,%f,%f,%f,%f\n", m.vitesse_tr_min, m.gyro[0], m.gyro[1], m.gyro[2], m.lat, m.lon, m.temperature);
    fclose(fptr);

}
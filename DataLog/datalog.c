//Rayan
#include "datalog.h"
#include <stdio.h>
#include <stdlib.h>


void datalog_write(const DonneesVehicule m)
{
    FILE *fptr;
    fptr = fopen("/home/rayan/log.csv", "a");
    fprintf(fptr, "%f,%f,%f,%f,%f,%f,%f,%f\n", m.timestamp, m.rpm, m.tilt_x, m.tilt_y, m.tilt_z, m.latitude, m.longitude, m.temp_ambiante);
    fclose(fptr);

}

void transfer_log(const char win_or_linux)
{
    //fonction qui envoie d'un linux sur soit un windows soit un linux
    if (win_or_linux == 'l')
    {
        system("scp -o BatchMode=yes '/home/rayan/log.csv' 'rayanmak@172.20.10.11:/home/rayanmak/Documents/Ingé/'");
    }
    else if (win_or_linux == 'w')
    {
        system("scp -o BatchMode=yes '/home/rayan/log.csv' 'matthias delcroix@172.20.10.4:C:/Users/Matthias Delcroix/Desktop/'");
    }
}


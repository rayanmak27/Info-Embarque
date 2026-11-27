#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>



typedef struct
{
    //date et heure de la prise de mesure depuis le début du "chrono"
    double timestamp;

    float temp_ambiante;
    float temp_moteur;
    float rpm;

    double latitude;
    double longitude;
    float vitesse_gps;
    float altitude;

    float accel_x;
    float accel_y;
    float accel_z;

    float rotation_x;
    float rotation_y;
    float rotation_z;

} DonneesVehicule;

void datalog_write(const DonneesVehicule m)
{
    FILE *fptr;
    fptr = fopen("/home/rayan/log.csv", "a");
    fprintf(fptr, "%f,%f,%f,%f,%f,%f,%f,%f\n", m.timestamp, m.rpm, m.rotation_x, m.rotation_y, m.rotation_z, m.latitude, m.longitude, m.temp_ambiante);
    fclose(fptr);

}
void transfer_log(const char win_or_linux)
{
    //fonction qui envoie d'un linux sur soit un windows soit un linux
    if (win_or_linux == 'l')
    {
        system("scp -o BatchMode=yes /home/rayan/log.csv 'rayanmak@172.20.10.11:/home/rayanmak/Documents/Ingé/Master 1/InfoEmbarque/DataLog'");
    }
    else if (win_or_linux == 'w')
    {
        system("scp -o BatchMode=yes /home/rayan/log.csv 'matthias delcroix@172.20.10.4:C:/Users/Mathias Delcroix/Desktop'");
    }
}

int main()
{
    DonneesVehicule m;
    m.timestamp = 0.0;
    m.temp_ambiante = 25.0;
    m.temp_moteur = 90.0;
    m.rpm = 3000.0;
    m.latitude = 48.8566;
    m.longitude = 2.3522;
    m.vitesse_gps = 50.0;
    m.altitude = 100.0;
    m.accel_x = 0.0;
    m.accel_y = 0.0;
    m.accel_z = 9.81;
    m.rotation_x = 0.0;
    m.rotation_y = 0.0;
    m.rotation_z = 0.0;
    

    datalog_write(m);
    printf("Data logged successfully.\n");

    transfer_log('w'); 
    printf("Log transferred successfully.\n");

    return 0;
}
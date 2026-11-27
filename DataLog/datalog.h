#ifndef DATALOG_H
#define DATALOG_H

// Aucun GPIO : stockage sur carte microSD

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
    float tilt_x;
    float tilt_y;
    float tilt_z;

} DonneesVehicule;

void transfer_log(const char win_or_linux); //'w' pour Windows, 'l' pour Linux
void datalog_write(const DonneesVehicule m);


#endif
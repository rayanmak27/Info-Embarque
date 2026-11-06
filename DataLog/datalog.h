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

    float gyro_x;
    float gyro_y;
    float gyro_z;

} DonneesVehicule;

int datalog_init(const char *chemin);
int datalog_write(const DonneesVehicule *donnees);
void datalog_close(void);

#endif
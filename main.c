#include <stdio.h>

#include "Bouton/bouton.h"
#include "DataLog/datalog.h"
#include "Ecran/ecran.h"
#include "GPS/gps.h"
#include "Gyro/gyro.h"
#include "Impulsion/impulsion.h"
#include "Temp_Ambiante/tempamb.h"
#include "Temp_Moteur/tempmoteur.h"

int main()
{
    // Initialisation des modules
    tempamb_init();
    tempmoteur_init();
    impulsion_init();
    gps_init();
    gyroscope_init();
    bouton_init();
    ecran_init();
}
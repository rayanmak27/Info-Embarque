//Petit programme de test du gyroscope
#define _DEFAULT_SOURCE //Pour usleep()
#include <stdio.h>
#include <unistd.h>
#include "gyro.h"

int main(void)
{
	if (gyro_init() != 0) {
		printf("Erreur d'initialisation du MPU6050\n");
		return 1;
	}

	sleep(1); //Laisse le thread faire quelques mesures

	while (1) {
		float ax, ay, az, roll, pitch, yaw;
		gyro_get_accel(&ax, &ay, &az);
		gyro_get_angles(&roll, &pitch, &yaw);
		printf("Accel (g) : %6.3f %6.3f %6.3f | Angles (deg) : roll %7.2f  pitch %7.2f  yaw %7.2f\n",
		       ax, ay, az, roll, pitch, yaw);
		usleep(200000); //5 affichages par seconde
	}
	return 0;
}

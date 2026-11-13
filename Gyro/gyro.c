//Nicolas et Mathias Cros
//GPIO 2 et GPIO 3 : I2C
#include "gyro.h"
#include "mpu6050.h"

#define MPU_ADDR 0x68

static MPU6050 mpu;          //Plus besoin de new : variable statique
static int initialise = 0;

int gyro_init(void)
{
	if (initialise) return 0;
	if (mpu6050_init(&mpu, MPU_ADDR) != 0) return -1; //Vraie erreur maintenant (capteur absent, I2C desactive...)
	mpu.calc_yaw = true;
	initialise = 1;
	return 0;
}

void gyro_get_accel(float *x, float *y, float *z)
{
	if (!initialise) { *x = *y = *z = 0; return; }
	mpu6050_get_accel(&mpu, x, y, z);
}

void gyro_get_angles(float *roll, float *pitch, float *yaw)
{
	if (!initialise) { *roll = *pitch = *yaw = 0; return; }
	mpu6050_get_angle(&mpu, 0, roll);
	mpu6050_get_angle(&mpu, 1, pitch);
	mpu6050_get_angle(&mpu, 2, yaw);
}

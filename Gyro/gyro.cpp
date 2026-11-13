//Nicolas et Mathias Cros
//GPIO 2 et GPIO 3 : I2C
#include "gyro.h"
#include "MPU6050.h"

#define MPU_ADDR 0x68

static MPU6050 *mpu = nullptr;

int gyro_init(void)
{
	if (mpu) return 0;
	mpu = new MPU6050(MPU_ADDR);
	mpu->calc_yaw = true;
	return mpu ? 0 : -1;
}

void gyro_get_accel(float *x, float *y, float *z)
{
	if (!mpu) { *x = *y = *z = 0; return; }
	mpu->getAccel(x, y, z);
}

void gyro_get_angles(float *roll, float *pitch, float *yaw)
{
	if (!mpu) { *roll = *pitch = *yaw = 0; return; }
	mpu->getAngle(0, roll);
	mpu->getAngle(1, pitch);
	mpu->getAngle(2, yaw);
}

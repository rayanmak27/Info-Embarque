//MPU6050 - adaptation en C de la lib C++ d'Alex Mous (Copyright (c) 2019, CC BY-NC-SA 4.0)
//Nicolas et Mathias Cros
#define _POSIX_C_SOURCE 200809L //Doit etre AVANT tous les #include (pour clock_gettime)

#include "mpu6050.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#define REG_WHO_AM_I 0x75

static void *mpu6050_update(void *arg);

//Lit un registre 16 bits (octet de poids fort a reg, poids faible a reg+1)
static int16_t read_word(int fd, uint8_t reg)
{
	int hi = i2c_smbus_read_byte_data(fd, reg);
	int lo = i2c_smbus_read_byte_data(fd, reg + 1);
	return (int16_t)((hi << 8) | lo);
}

int mpu6050_init(MPU6050 *mpu, int8_t addr)
{
	memset(mpu, 0, sizeof(*mpu));
	mpu->addr = addr;
	mpu->dt = 0.009f;
	mpu->first_run = true;
	mpu->calc_yaw = false;

	mpu->f_dev = open("/dev/i2c-1", O_RDWR);
	if (mpu->f_dev < 0) {
		fprintf(stderr, "ERR (mpu6050_init): impossible d'ouvrir /dev/i2c-1. Verifier que l'I2C est active (raspi-config)\n");
		return -1;
	}

	if (ioctl(mpu->f_dev, I2C_SLAVE, mpu->addr) < 0) {
		fprintf(stderr, "ERR (mpu6050_init): impossible d'acceder au bus I2C a l'adresse 0x%02x\n", mpu->addr);
		close(mpu->f_dev);
		return -1;
	}

	//Ajout : on verifie que le capteur repond vraiment (WHO_AM_I doit valoir 0x68)
	int id = i2c_smbus_read_byte_data(mpu->f_dev, REG_WHO_AM_I);
	if (id != 0x68) {
		fprintf(stderr, "ERR (mpu6050_init): pas de MPU6050 detecte (WHO_AM_I = %d)\n", id);
		close(mpu->f_dev);
		return -1;
	}

	i2c_smbus_write_byte_data(mpu->f_dev, 0x6b, 0x00);         //Sortie du mode veille
	i2c_smbus_write_byte_data(mpu->f_dev, 0x1a, 0x03);         //Filtre passe-bas (DLPF) a 44 Hz
	i2c_smbus_write_byte_data(mpu->f_dev, 0x19, 0x04);         //Diviseur de frequence (200 Hz)
	i2c_smbus_write_byte_data(mpu->f_dev, 0x1b, GYRO_CONFIG);  //Config gyroscope
	i2c_smbus_write_byte_data(mpu->f_dev, 0x1c, ACCEL_CONFIG); //Config accelerometre

	//Offsets a zero (meme sequence que la lib d'origine)
	i2c_smbus_write_byte_data(mpu->f_dev, 0x06, 0x00);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x07, 0x00);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x08, 0x00);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x09, 0x00);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x0A, 0x00);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x0B, 0x00);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x00, 0x81);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x01, 0x01);
	i2c_smbus_write_byte_data(mpu->f_dev, 0x02, 0x81);

	//Equivalent de std::thread(...).detach()
	if (pthread_create(&mpu->thread, NULL, mpu6050_update, mpu) != 0) {
		fprintf(stderr, "ERR (mpu6050_init): impossible de creer le thread de mise a jour\n");
		close(mpu->f_dev);
		return -1;
	}
	pthread_detach(mpu->thread);

	return 0;
}

void mpu6050_get_gyro_raw(MPU6050 *mpu, float *roll, float *pitch, float *yaw)
{
	*roll  = (float)read_word(mpu->f_dev, 0x43); //Axe X
	*pitch = (float)read_word(mpu->f_dev, 0x45); //Axe Y
	*yaw   = (float)read_word(mpu->f_dev, 0x47); //Axe Z
}

void mpu6050_get_gyro(MPU6050 *mpu, float *roll, float *pitch, float *yaw)
{
	mpu6050_get_gyro_raw(mpu, roll, pitch, yaw);
	//On retire l'offset, on divise par la sensibilite, arrondi a 3 decimales
	*roll  = round((*roll  - G_OFF_X) * 1000.0 / GYRO_SENS) / 1000.0;
	*pitch = round((*pitch - G_OFF_Y) * 1000.0 / GYRO_SENS) / 1000.0;
	*yaw   = round((*yaw   - G_OFF_Z) * 1000.0 / GYRO_SENS) / 1000.0;
}

void mpu6050_get_accel_raw(MPU6050 *mpu, float *x, float *y, float *z)
{
	*x = (float)read_word(mpu->f_dev, 0x3b);
	*y = (float)read_word(mpu->f_dev, 0x3d);
	*z = (float)read_word(mpu->f_dev, 0x3f);
}

void mpu6050_get_accel(MPU6050 *mpu, float *x, float *y, float *z)
{
	mpu6050_get_accel_raw(mpu, x, y, z);
	*x = round((*x - A_OFF_X) * 1000.0 / ACCEL_SENS) / 1000.0;
	*y = round((*y - A_OFF_Y) * 1000.0 / ACCEL_SENS) / 1000.0;
	*z = round((*z - A_OFF_Z) * 1000.0 / ACCEL_SENS) / 1000.0;
}

void mpu6050_get_offsets(MPU6050 *mpu, float *ax_off, float *ay_off, float *az_off,
                         float *gr_off, float *gp_off, float *gy_off)
{
	float g[3], a[3];

	*gr_off = *gp_off = *gy_off = 0;
	*ax_off = *ay_off = *az_off = 0;

	for (int i = 0; i < 10000; i++) {
		mpu6050_get_gyro_raw(mpu, &g[0], &g[1], &g[2]);
		*gr_off += g[0]; *gp_off += g[1]; *gy_off += g[2];

		mpu6050_get_accel_raw(mpu, &a[0], &a[1], &a[2]);
		*ax_off += a[0]; *ay_off += a[1]; *az_off += a[2];
	}

	*gr_off /= 10000; *gp_off /= 10000; *gy_off /= 10000;
	*ax_off /= 10000; *ay_off /= 10000; *az_off /= 10000;

	*az_off -= ACCEL_SENS; //On retire 1g (gravite)
}

int mpu6050_get_angle(MPU6050 *mpu, int axis, float *result)
{
	if (axis >= 0 && axis <= 2) {
		*result = mpu->angle[axis];
		return 0;
	}
	fprintf(stderr, "ERR (mpu6050_get_angle): 'axis' doit etre entre 0 et 2 (roll, pitch, yaw)\n");
	*result = 0;
	return 1;
}

//Boucle de mise a jour, tourne en continu dans un thread separe
static void *mpu6050_update(void *arg)
{
	MPU6050 *mpu = (MPU6050 *)arg;
	float ax, ay, az, gr, gp, gy; //Anciennes variables membres, maintenant locales

	clock_gettime(CLOCK_REALTIME, &mpu->start);

	while (1) {
		mpu6050_get_gyro(mpu, &gr, &gp, &gy);
		mpu6050_get_accel(mpu, &ax, &ay, &az);

		//Axe X (roll)
		mpu->accel_angle[0] = atan2(az, ay) * RAD_T_DEG - 90.0;
		mpu->gyro_angle[0] = mpu->angle[0] + gr * mpu->dt;

		//Axe Y (pitch)
		mpu->accel_angle[1] = atan2(az, ax) * RAD_T_DEG - 90.0;
		mpu->gyro_angle[1] = mpu->angle[1] + gp * mpu->dt;

		//Axe Z (yaw)
		if (mpu->calc_yaw)
			mpu->gyro_angle[2] = mpu->angle[2] + gy * mpu->dt;

		if (mpu->first_run) { //Le gyro part de l'angle absolu donne par l'accelerometre
			for (int i = 0; i <= 1; i++)
				mpu->gyro_angle[i] = mpu->accel_angle[i];
			mpu->gyro_angle[2] = 0;
			mpu->first_run = false;
		}

		//Attention : en C, abs() est pour les int -> fabsf() pour les float
		float asum = fabsf(ax) + fabsf(ay) + fabsf(az);
		float gsum = fabsf(gr) + fabsf(gp) + fabsf(gy);

		for (int i = 0; i <= 1; i++) {
			if (fabsf(mpu->gyro_angle[i] - mpu->accel_angle[i]) > 5) //Correction d'une grosse derive
				mpu->gyro_angle[i] = mpu->accel_angle[i];

			if (asum > 0.1 && asum < 3 && gsum > 0.3)  //Mouvement modere : filtre complementaire
				mpu->angle[i] = (1 - TAU) * mpu->gyro_angle[i] + TAU * mpu->accel_angle[i];
			else if (gsum > 0.3)                      //Forte acceleration : gyro seul
				mpu->angle[i] = mpu->gyro_angle[i];
			else                                      //Peu de mouvement : accelerometre seul
				mpu->angle[i] = mpu->accel_angle[i];
		}

		//Le yaw ne peut pas etre calcule avec l'accelerometre : gyro uniquement
		if (mpu->calc_yaw) {
			mpu->angle[2] = mpu->gyro_angle[2];
		} else {
			mpu->angle[2] = 0;
			mpu->gyro_angle[2] = 0;
		}

		clock_gettime(CLOCK_REALTIME, &mpu->end);
		mpu->dt = (mpu->end.tv_sec - mpu->start.tv_sec) + (mpu->end.tv_nsec - mpu->start.tv_nsec) / 1e9;
		clock_gettime(CLOCK_REALTIME, &mpu->start);
	}

	return NULL;
}

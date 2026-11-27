//MPU6050 - adaptation en C de la lib C++ d'Alex Mous (Copyright (c) 2019, CC BY-NC-SA 4.0)
//Nicolas et Mathias Cros
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

//-----------------------MODIFY THESE PARAMETERS-----------------------

#define GYRO_RANGE 0 //Plage du gyroscope (defaut 0)
//	0	+/- 250 degres/seconde
//	1	+/- 500 degres/seconde
//	2	+/- 1000 degres/seconde
//	3	+/- 2000 degres/seconde

#define ACCEL_RANGE 0 //Plage de l'accelerometre (defaut 0)
//	0	+/- 2g
//	1	+/- 4g
//	2	+/- 8g
//	3	+/- 16g

//Offsets - a calculer avec mpu6050_get_offsets()
//	Accelerometre
#define A_OFF_X 29466.4
#define A_OFF_Y 5329.9
#define A_OFF_Z -13404.7
//	Gyroscope
#define G_OFF_X -340.8
#define G_OFF_Y 129.8
#define G_OFF_Z 154.3
//-----------------------END MODIFY THESE PARAMETERS-----------------------

#define TAU 0.05                  //Coefficient du filtre complementaire
#define RAD_T_DEG 57.29577951308  //Radians -> degres (180/PI)

#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0x08
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0x10
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0x18
#else
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0x00
#endif

//(corrige : la lib d'origine testait GYRO_RANGE ici au lieu de ACCEL_RANGE)
#if ACCEL_RANGE == 1
	#define ACCEL_SENS 8192.0
	#define ACCEL_CONFIG 0x08
#elif ACCEL_RANGE == 2
	#define ACCEL_SENS 4096.0
	#define ACCEL_CONFIG 0x10
#elif ACCEL_RANGE == 3
	#define ACCEL_SENS 2048.0
	#define ACCEL_CONFIG 0x18
#else
	#define ACCEL_SENS 16384.0
	#define ACCEL_CONFIG 0x00
#endif

//Equivalent C de la classe MPU6050
typedef struct {
	float accel_angle[3];
	float gyro_angle[3];
	float angle[3];          //Angles finaux : roll, pitch, yaw

	int addr;
	int f_dev;               //Descripteur de /dev/i2c-1

	float dt;                //Temps de boucle (recalcule a chaque tour)
	struct timespec start, end;

	bool first_run;
	bool calc_yaw;

	pthread_t thread;
} MPU6050;

int  mpu6050_init(MPU6050 *mpu, int8_t addr);   //0 = OK, -1 = erreur
void mpu6050_get_accel_raw(MPU6050 *mpu, float *x, float *y, float *z);
void mpu6050_get_gyro_raw(MPU6050 *mpu, float *roll, float *pitch, float *yaw);
void mpu6050_get_accel(MPU6050 *mpu, float *x, float *y, float *z);       //en g
void mpu6050_get_gyro(MPU6050 *mpu, float *roll, float *pitch, float *yaw); //en deg/s
void mpu6050_get_offsets(MPU6050 *mpu, float *ax_off, float *ay_off, float *az_off,
                         float *gr_off, float *gp_off, float *gy_off);
int  mpu6050_get_angle(MPU6050 *mpu, int axis, float *result);           //0 = OK

#endif

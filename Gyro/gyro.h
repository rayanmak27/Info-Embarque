//Nicolas et Mathias Cros
//GPIO 2 et GPIO 3 : I2C
#ifndef GYRO_H
#define GYRO_H

#ifdef __cplusplus
extern "C" {
#endif

int gyro_init(void);                                         // 0 = OK
void gyro_get_accel(float *x, float *y, float *z);           // en g
void gyro_get_angles(float *roll, float *pitch, float *yaw); // en degres

#ifdef __cplusplus
}
#endif

#endif

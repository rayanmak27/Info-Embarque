#ifndef GYRO_H
#define GYRO_H

// GPIO2 : SDA (broche 3)
// GPIO3 : SCL (broche 5)
// I2C : adresse 0x68

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

} GyroscopeData;

int gyroscope_init(void);
int gyroscope_read(GyroscopeData *donnees);
int gyroscope_calibrer(void);
void gyroscope_close(void);

#endif
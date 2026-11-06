#ifndef TEMPAMB_H
#define TEMPAMB_H

// GPIO17 : DS18B20 (broche 11)
// Communication : 1-Wire

int tempamb_init(void);
int tempamb_read(float *temperature);
void tempamb_close(void);

#endif
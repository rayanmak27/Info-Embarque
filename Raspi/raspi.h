#ifndef __RASPI_H__
#define __RASPI_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum RASPI : uint8_t
{
    RASPI_1         = 1,
    RASPI_2         = 2,
    RASPI_3         = 3,
    RASPI_4         = 4,
    RASPI_5         = 5,

    RASPI_ZERO_1    = 0,
    RASPI_ZERO_1W   = 6,
    RASPI_ZERO_2W   = 7,
    
    RASPI_400       = 8

} RASPI;

bool Rapsi_IsValid(RASPI raspi);

const char* Raspi_GetUArt_Folder(RASPI raspi);

#endif
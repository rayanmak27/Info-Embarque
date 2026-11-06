#include "raspi.h"

static const char* UART_FOLDER[] = {"/dev/ttyAMA0",
                                    "/dev/ttyS0",
                                    "/dev/ttyAMA10"
                                };


bool Rapsi_IsValid(RASPI raspi)
{
    switch ((uint8_t)raspi) 
    {
        case RASPI_ZERO_1:
        case RASPI_1:
        case RASPI_2:
        case RASPI_ZERO_1W:
        case RASPI_ZERO_2W:
        case RASPI_3:
        case RASPI_4:
        case RASPI_400:
        case RASPI_5:
            return 1;

        default:
            return 0;
    };
}


const char* Raspi_GetUArt_Folder(RASPI raspi)
{
    switch ((uint8_t)raspi) 
    {
        case RASPI_ZERO_1:
        case RASPI_1:
        case RASPI_2:
            return UART_FOLDER[0];

        case RASPI_ZERO_1W:
        case RASPI_ZERO_2W:
        case RASPI_3:
        case RASPI_4:
        case RASPI_400:
            return UART_FOLDER[1];

        case RASPI_5:
            return UART_FOLDER[2];

        default:
            return "";
    };
}
#include "uart.h"

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

/* termios attend une constante (B9600...), pas le nombre 9600 */
static speed_t UART_BaudToSpeed(uint32_t baud)
{
    switch (baud) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B0;
    }
}

int32_t UART_Init(RASPI raspi, uint32_t baud)
{
    if(!UART_IsValidBaudRate(baud))
        return UART_ERR_BAUDRATE_INVALID;

    if(!Rapsi_IsValid(raspi))
        return UART_ERR_RASPI_INVALID;

    int fd = open(Raspi_GetUArt_Folder(raspi), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) 
        return UART_ERR_IMPOSSIBLE_TO_READ_UART;
    

    struct termios t;
    if (tcgetattr(fd, &t) < 0) {
        close(fd);
        return UART_ERR_IMPOSSIBLE_TO_INIT_STRUCT;
    }

    cfmakeraw(&t);                  /* pas d'écho, pas de traitement des caractères */
    t.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);  // étient tous les interrupteur
    t.c_cflag |= CS8 | CLOCAL | CREAD;   /* 8 bits, sans parité, 1 bit de stop */
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;
    cfsetispeed(&t, UART_BaudToSpeed(baud));   // vitesse d'écoute
    cfsetospeed(&t, UART_BaudToSpeed(baud));   // vitesse d'écriture

    if (tcsetattr(fd, TCSANOW, &t) < 0) {
        close(fd);
        return UART_ERR_IMPOSSIBLE_TO_PARAM_UART;
    }

    tcflush(fd, TCIOFLUSH);         /* vide les anciennes données */
    return fd;

}


void UART_Close(int32_t uart)
{
    if(uart >= 0)
        close(uart);   
}


bool UART_IsValidBaudRate(uint32_t baud)
{
    switch (baud) {
        case 4800:   
        case 9600:   
        case 19200:  
        case 38400:  
        case 57600:  
        case 115200: 
            return 1;

        default:     
            return 0;
    }
}
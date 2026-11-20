#include "uart.h"

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

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
    cfsetispeed(&t, baud);          // vitesse d'écoute
    cfsetospeed(&t, baud);          // vitesse d'écriture

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
    speed_t speed;
    switch (baud) {
        case 4800:   speed = B4800;   break;
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        default:     speed = B115200; break;
    }
}
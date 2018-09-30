
#ifdef __cplusplus
extern "C" {
#endif

#include "ftd2xx.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <assert.h>
#include <inttypes.h>
#include <time.h>

#ifdef __cplusplus
}
#endif

#ifndef LogError
#define	LogError(fmt, ...)	fprintf(stderr, fmt, ##__VA_ARGS__)
#endif

#ifndef bool
    #define bool int
    #define false ((bool)0)
    #define true  ((bool)1)
#endif

//#define sleep_ms(x) usleep(x * 1000)

static inline void sleep_ms(uint32_t ms){
    //printf("sleep 2ms\n");
    usleep(ms * 1000);
}

FTD2XX_API FT_STATUS FT_SetVIDPID( DWORD dwVID, DWORD dwPID){
    return 0;
}

#if defined(__APPLE__)
int setNonblock(int fd, bool nonblock)
{
	int flag = fcntl(fd, F_GETFD, 0);

	if (nonblock)
		flag |= O_NONBLOCK;
	else
		flag &= ~O_NONBLOCK;

	return fcntl(fd, F_SETFL, flag);
}
#endif


#ifndef B460800
#define B460800 460800
#endif

FTD2XX_API FT_STATUS WINAPI FT_OpenEx( PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle){
    assert(pArg1 != NULL);
    assert(pHandle != NULL);

    char device[64];
    #ifdef __APPLE__
    snprintf(device,64,"/dev/tty.usbserial-%s",(char*)pArg1);
    #else
    int i = atol((const char*)pArg1);
    if (i == 0){
        return FT_DEVICE_NOT_OPENED;
    }
    snprintf(device,64,"/dev/ttyUSB%d",i - 1);
    #endif
    //char *device = (char*)pArg1; /*--> /dev/ttyUSB0 */


    *(int*)pHandle = -1;
    int fd = 0;

#if defined(__APPLE__)
    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK); /*open in block mode under OSX*/
#else
    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY, 0);
#endif
    if (fd < 0) {
        LogError("Cannot open device - %s\n", device);
        return FT_DEVICE_NOT_OPENED;
    }

    if (isatty(fd) == 0) {
        LogError("%s is not a TTY device", device);
        close(fd);
        return FT_DEVICE_NOT_OPENED;
    }

    struct termios termios;
    if (tcgetattr(fd, &termios) < 0) {
        LogError("Cannot get the attributes for %s", device);
        close(fd);
        return FT_DEVICE_NOT_OPENED;
    }

#if defined(__APPLE__)
    termios.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    termios.c_cflag &= ~CSIZE;
    termios.c_cflag |= CS8;         /* 8-bit characters */
    termios.c_cflag &= ~PARENB;     /* no parity bit */
    termios.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    termios.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    termios.c_cc[VMIN] = 1;
    termios.c_cc[VTIME] = 1;
#else
    termios.c_lflag    &= ~(ECHO | ECHOE | ICANON | IEXTEN | ISIG);
    termios.c_iflag    &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | IXOFF | IXANY);
    termios.c_cflag    &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
    termios.c_cflag    |= CS8;
    termios.c_oflag    &= ~(OPOST);
    termios.c_cc[VMIN]  = 0;
    termios.c_cc[VTIME] = 10;
#endif

    // setup speed to 460800
    cfsetospeed(&termios, B460800);
    cfsetispeed(&termios, B460800);
    printf("Baudrate: %u\n",B460800);

    if (tcsetattr(fd, TCSANOW, &termios) < 0) {
        LogError("Cannot set the attributes for %s", device);
        close(fd);
        return FT_DEVICE_NOT_OPENED;
    }

#if defined(__APPLE__)
    setNonblock(fd, false);
#endif

    *(int*)pHandle = fd;
    printf("Opened serial port %s, fd=%d\n",device,fd);
    return 0;
}

FTD2XX_API
    FT_STATUS WINAPI FT_Purge(
    FT_HANDLE ftHandle,
    ULONG Mask
    ){
        //TODO - reset the serial port buffer
        //Flush cache.
        tcflush((int)ftHandle, TCIFLUSH);
        //sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_SetDataCharacteristics(
    FT_HANDLE ftHandle,
    UCHAR WordLength,
    UCHAR StopBits,
    UCHAR Parity
    ){
        sleep_ms(2);
        return 0;
    }

// FTD2XX_API 
//     FT_STATUS WINAPI FT_ListDevices(
//     PVOID pArg1,
//     PVOID pArg2,
//     DWORD Flags
//     );

// FTD2XX_API
//     FT_STATUS WINAPI FT_Close(
//     FT_HANDLE ftHandle
//     );

FTD2XX_API FT_STATUS WINAPI FT_Read( FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToRead, LPDWORD lpBytesReturned){
    char* buffer = (char*)lpBuffer;
    int m_fd = (int)ftHandle;
    ssize_t length = (ssize_t)dwBytesToRead;

    assert(buffer != NULL);
	assert(m_fd != -1);

    assert(lpBytesReturned != NULL);
    *lpBytesReturned = 0;

	if (length == 0U){
		return 0;
    }

	unsigned int offset = 0U;

	while (offset < length) {
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);
		int n;
		if (offset == 0U) {
			struct timeval tv;
			tv.tv_sec  = 0;
			tv.tv_usec = 0;
			n = select(m_fd + 1, &fds, NULL, NULL, &tv);
			if (n == 0){
				return 0;
            }
		} else {
			n = select(m_fd + 1, &fds, NULL, NULL, NULL);
		}

		if (n < 0) {
			LogError("Error from select(), errno=%d", errno);
			return FT_IO_ERROR;
		}

		if (n > 0) {
			ssize_t len = read(m_fd, buffer + offset, length - offset);
			if (len < 0) {
				if (errno != EAGAIN) {
					LogError("Error from read(), errno=%d", errno);
					return FT_IO_ERROR;
				}
			}

			if (len > 0)
				offset += len;
		}
	}

    *lpBytesReturned = length;
    return 0;    
}

bool canRead(int fd){
    fd_set rset;
	FD_ZERO(&rset);
	FD_SET(fd, &rset);

	struct timeval timeo;
	timeo.tv_sec  = 0;
	timeo.tv_usec = 0;

	int rc = select(fd + 1, &rset, NULL, NULL, &timeo);
	if (rc >0 && FD_ISSET(fd, &rset))
		return true;

	return false;
}

bool canWrite(int fd){
#if defined(__APPLE__)
	fd_set wset;
	FD_ZERO(&wset);
	FD_SET(fd, &wset);

	struct timeval timeo;
	timeo.tv_sec  = 0;
	timeo.tv_usec = 0;

	int rc = select(fd + 1, NULL, &wset, NULL, &timeo);
	if (rc >0 && FD_ISSET(fd, &wset))
		return true;
	return false;
#else
	return true;
#endif
}

FTD2XX_API FT_STATUS WINAPI FT_Write( FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToWrite, LPDWORD lpBytesWritten ){
    char* buffer = (char*)lpBuffer;
    int fd = (int)ftHandle;
    ssize_t length = (ssize_t)dwBytesToWrite;

    assert(buffer != NULL);
	assert(fd != -1);

    assert(lpBytesWritten != NULL);
    *lpBytesWritten = 0;

    unsigned int offset = 0U;
	while (offset < length) {
		ssize_t n = 0U;
		if (canWrite(fd)){
            n = ::write(fd, buffer + offset, length - offset);
        }
		if (n < 0) {
			if (errno != EAGAIN) {
				LogError("Error returned from write(), errno=%d", errno);
				return FT_IO_ERROR;
			}
		}

		if (n > 0){
			offset += n;
        }
	}

	*lpBytesWritten = length;

    return 0;
}

FTD2XX_API FT_STATUS WINAPI FT_CreateDeviceInfoList(LPDWORD lpdwNumDevs){
    assert (lpdwNumDevs != NULL);
    *lpdwNumDevs = 2; /*FIXME - determin by configuration */
    sleep_ms(2);
    return 0;
}


FTD2XX_API FT_STATUS WINAPI FT_GetDeviceInfoList( FT_DEVICE_LIST_INFO_NODE *pDest, LPDWORD lpdwNumDevs){
    assert (pDest != NULL);
    int size = *(int*)lpdwNumDevs;
    for (unsigned int i = 0;i < size;i++){
        FT_DEVICE_LIST_INFO_NODE *dev = pDest + i;
        memset(dev,0,sizeof(FT_DEVICE_LIST_INFO_NODE));
        dev->ID = (ULONG)(i + 1);
        dev->LocId = (ULONG)(i + 1);
        snprintf(dev->Description,64,"Nano-3080");
        #ifdef __APPLE__
        if(i == 0){
            snprintf(dev->SerialNumber,16,"DJ38AFGE");
        }else{
            snprintf(dev->SerialNumber,16,"DJ38C1GY");
        }
        #else
        snprintf(dev->SerialNumber,16,"%06d",i+1);
        #endif
        //printf("GetDeviceInfoList on device %d, addr: 0x%x", i, (dev & 0xffff) );
        #ifdef DEBUG
        printf("GetDeviceInfoList on device %d\n", i);
        #endif
    }
    *(int*)lpdwNumDevs = 2;
    return 0;
}

FTD2XX_API
    FT_STATUS WINAPI FT_SetBaudRate(
    FT_HANDLE ftHandle,
    ULONG BaudRate
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_SetFlowControl(
    FT_HANDLE ftHandle,
    USHORT FlowControl,
    UCHAR XonChar,
    UCHAR XoffChar
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_ResetDevice(
    FT_HANDLE ftHandle
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_SetDtr(
    FT_HANDLE ftHandle
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_ClrDtr(
    FT_HANDLE ftHandle
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_SetRts(
    FT_HANDLE ftHandle
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_ClrRts(
    FT_HANDLE ftHandle
    );

FTD2XX_API
    FT_STATUS WINAPI FT_SetTimeouts(
    FT_HANDLE ftHandle,
    ULONG ReadTimeout,
    ULONG WriteTimeout
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API FT_STATUS WINAPI FT_GetQueueStatus( FT_HANDLE ftHandle, DWORD *dwRxBytes ){
    // check if readable and set the dwRxBytes
    if (canRead((int)ftHandle)){
        *dwRxBytes = 1;
    }else{
        *dwRxBytes = 0;
    }
    //sleep_ms(2);
    return 0;
}

FTD2XX_API
    FT_STATUS WINAPI FT_SetBreakOn(
    FT_HANDLE ftHandle
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_SetBreakOff(
    FT_HANDLE ftHandle
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_GetEventStatus(
    FT_HANDLE ftHandle,
    DWORD *dwEventDWord
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_SetLatencyTimer(
    FT_HANDLE ftHandle,
    UCHAR ucLatency
    ){
        sleep_ms(2);
        return 0;
    }

FTD2XX_API
    FT_STATUS WINAPI FT_SetUSBParameters(
    FT_HANDLE ftHandle,
    ULONG ulInTransferSize,
    ULONG ulOutTransferSize
    ){
        sleep_ms(2);
        return 0;
    }
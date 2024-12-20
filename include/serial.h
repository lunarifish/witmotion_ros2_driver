
#ifndef SERIAL_H
#define SERIAL_H

#include <linux/types.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <linux/rtc.h>
#include <termios.h>

#ifdef __cplusplus
extern "C" {
#endif

void serial_close(int fd);

int serial_open(const char *dev, unsigned int baud);

int serial_read(int fd, unsigned char *val, int len);

int serial_write(int fd, unsigned char *val, int len);

#ifdef __cplusplus
}
#endif

#endif

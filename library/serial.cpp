#include "serial.h"
#include "readwrite.h"

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/ioctl.h>

static void setrts(int fd, int on)
{
	int controlbits;
	int ret;
	ret = ioctl(fd, TIOCMGET, &controlbits);
	printf("setrts: %d\n", ret);
	if (on) {
		controlbits |= TIOCM_RTS;
	} else {
		controlbits &= ~TIOCM_RTS;
	}
	ret = ioctl(fd, TIOCMSET, &controlbits);
	printf("setrts: %d\n", ret);
}

static void setdtr(int fd, int on)
{
	int controlbits = TIOCM_DTR;
	ioctl(fd, (on ? TIOCMBIS : TIOCMBIC), &controlbits);
}


/*
 * Opens non-blocking.
 * Returns whether successul.
 */
int serial_open(struct serial *ser, const char *port, int baud)
{
	int tcbaud;

	memset(ser, 0, sizeof(*ser));
	ser->fd = -1;

	switch (baud) {
	case 230400: tcbaud = B230400; break;
	case 115200: tcbaud = B115200; break;
	case  57600: tcbaud = B57600; break;
	case  38400: tcbaud = B38400; break;
	case  19200: tcbaud = B19200; break;
	case   9600: tcbaud = B9600; break;
	case   4800: tcbaud = B4800; break;
	case   2400: tcbaud = B2400; break;
	case   1200: tcbaud = B1200; break;
	case    300: tcbaud = B300; break;
	default:
		fprintf(stderr, "%s: baud not supported %d\n",
				__FUNCTION__, baud);
		return 0;
	}
	ser->baud = baud;
	ser->fd = open(port, O_RDWR | O_NOCTTY);
	if (ser->fd == -1) {
		perror(port);
		return 0;
	}
	tcgetattr(ser->fd, &ser->oldtio);
	ser->newtio.c_cflag = CS8 | CLOCAL | CREAD;
	ser->newtio.c_iflag = IGNPAR;
	ser->newtio.c_oflag = 0;
	ser->newtio.c_lflag = 0;
	ser->newtio.c_cc[VMIN]  = 0; /* allow read() to return zero */
	ser->newtio.c_cc[VTIME] = 1; /* don't wait for any char */
	cfsetospeed(&ser->newtio, tcbaud);
	cfsetispeed(&ser->newtio, tcbaud);
	tcflush(ser->fd, TCIOFLUSH);
	tcsetattr(ser->fd, TCSANOW, &ser->newtio);

	setdtr(ser->fd, 0);

	return 1;
}

void serial_close(struct serial *ser)
{
	if (ser == 0 || ser->fd == -1)
		return;
	tcsetattr(ser->fd, TCSANOW, &ser->oldtio); /* restore settings */
	close(ser->fd);
	ser->fd = -1;
}

int serial_read(struct serial *ser, void *buf, int len, int timeout_ms)
{
	return read_timed(ser->fd, buf, len, timeout_ms);
}

/*
 * Read bytes into buf until end char is found, len bytes are received,
 * or no bytes are received for timeout milliseconds (negative for infinite).
 * Returns number of bytes actually read.
 */
int serial_read_until_char(struct serial *ser, void *buf, int len,
		char end, int timeout)
{
	read_until_char_timed(ser->fd, buf, len, end, timeout);
	return 0;
}

#if 0
static void print_data(unsigned char *buf, int len)
{
	if (len) {
		fprintf(stderr, "wrote: ");
		while (len--) fprintf(stderr, "%02X ", *buf++);
		fflush(stderr);
		fprintf(stderr, "\n");
	}
}
#endif

int serial_write(struct serial *ser, void *buf, int len)
{
	int ret;
	ret = write(ser->fd, buf, len);
	if (ret == -1) {
		perror(__FUNCTION__);
		return -1;
	} else if (ret < len) {
		fprintf(stderr, "%s: short write\n", __FUNCTION__);
	}
//	print_data(buf, ret);
	return ret;
}

int serial_bytes_available(struct serial *ser)
{
	int bytes;
	ioctl(ser->fd, FIONREAD, &bytes);
	return bytes;
}


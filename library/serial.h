#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <termios.h>
#include <unistd.h>

struct serial {
	int fd;
	int baud;
	struct termios oldtio,newtio;
};

int serial_open(struct serial *ser, const char *port, int baud);
int serial_write(struct serial *ser, void *buf, int len);
int serial_bytes_available(struct serial *ser);
int serial_read(struct serial *ser, void *buf, int len, int timeout_ms);
int serial_read_until_char(struct serial *ser, void *buf, int len,
		char end, int timeout_ms);
void serial_close(struct serial *ser);


#endif /* ndef __SERIAL_H__ */


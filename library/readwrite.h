#ifndef __READWRITE_H__
#define __READWRITE_H__

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/poll.h>

int read_timed(int fd, void *buf, int len, int timeout_ms);
int read_until_char_timed(int fd, void *buf, int len, char end, int timeout_ms);


#endif /* ndef __READWRITE_H__ */


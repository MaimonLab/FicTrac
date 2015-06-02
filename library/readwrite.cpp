#include "readwrite.h"


/*
 * Negative timeout for infinite wait.
 * May take longer than timeout_ms since timeout is restarted after each
 * data chunk is received.
 * Returns number of bytes actually read, which is zero on timeout or error.
 */
int read_timed(int fd, void *buf, int len, int timeout_ms)
{
	struct pollfd pollfd;
	int remain = len;
	char *pbuf = (char*)buf;

	pollfd.fd = fd;
	pollfd.events = POLLIN;
	while (remain > 0) {
		int ret = poll(&pollfd, 1, timeout_ms);
		if (ret == 0) {
			/* timeout */
			break;
		} else if (ret == -1 && errno == EINTR) {
			fprintf(stderr, "%s: poll interrupted\n", __FUNCTION__);
			break;
		} else if (ret == -1) {
			perror(__FUNCTION__);
			break;
		}

		ret = read(fd, pbuf, remain);
		if (ret == 0) {
			/* should not happen */
			fprintf(stderr, "%s: zero read\n", __FUNCTION__);
			break;
		} else if (ret == -1) {
			perror(__FUNCTION__);
			break;
		}

		pbuf += ret;
		remain -= ret;
	}
	return len - remain;
}

/*
 * Due to its nature, this function must read a single byte at a time
 * which can be slow for large reads.
 *
 * When return is less than "len", "end" may have been found or there may
 * have been an error or timeout.
 */
int read_until_char_timed(int fd, void *buf, int len, char end, int timeout_ms)
{
	int remain = len;
	char *pbuf = (char*)buf;

	while (remain > 0) {
		int ret = read_timed(fd, pbuf, 1, timeout_ms);
		if (ret == 0) {
			/* timeout or error */
			break;
		}
		remain--;
		if (*pbuf == end)
			break;
		pbuf++;
	}
	return len - remain;
}


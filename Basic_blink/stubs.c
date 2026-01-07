#include <sys/stat.h>
#include <unistd.h>

int _close(int fd)
{
	(void)fd;
	return -1;
}

int _lseek(int fd, int offset, int whence)
{
	(void)fd;
	(void)offset;
	(void)whence;
	return 0;
}

int _read(int fd, char* buf, int count)
{
	(void)fd;
	(void)buf;
	(void)count;
	return 0;
}

int _write(int fd, const char* buf, int count)
{
	(void)fd;
	(void)buf;
	return count;
}

void _exit(int code)
{
	(void)code;
	while (1)
		;
}

void* _sbrk(int incr)
{
	(void)incr;
	return (void*)-1;
}

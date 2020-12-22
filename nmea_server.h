#ifndef _NMEA_SERVER_H
#define _NMEA_SERVER_H

#include <stdio.h>
#include <sys/select.h>
#include <sys/types.h>

int nmeasrv_init(fd_set *fds);
void nmeasrv_cleanup();

int nmeasrv_task(fd_set *fds, fd_set *select_fds);
int nmeasrv_broadcast(const char *fmt, ...);

#endif

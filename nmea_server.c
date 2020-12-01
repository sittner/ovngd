#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <syslog.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "nmea_server.h"

#define OVNG_PORT 4353

static int server_fd = -1;
static fd_set client_fds;

int nmeasrv_init(fd_set *fds) {
  server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    syslog(LOG_ERR, "Could not create socket");
    goto fail0;
  }

  struct sockaddr_in server;
  server.sin_family = AF_INET;
  server.sin_port = htons(OVNG_PORT);
  server.sin_addr.s_addr = htonl(INADDR_ANY);

  int opt_val = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt_val, sizeof opt_val);

  if (bind(server_fd, (struct sockaddr *) &server, sizeof(server)) < 0) {
    syslog(LOG_ERR, "Could not bind socket");
    goto fail1;
  }

  if (listen(server_fd, 4) < 0) {
    syslog(LOG_ERR, "Could not listen on socket");
    goto fail1;
  }

  FD_SET(server_fd, fds);
  FD_ZERO(&client_fds);

  return 0;

fail1:
  close(server_fd);
fail0:
  return -1;
}

void nmeasrv_cleanup() {
  int cfd;

  // close client connections
  for (cfd = 0; cfd < FD_SETSIZE; cfd++) {
    if (FD_ISSET(cfd, &client_fds)) {
      close(cfd);
    }
  }

  // close server connection
  close(server_fd);
}

int nmeasrv_task(fd_set *select_fds, fd_set *fds) {
  int cfd;
  struct sockaddr_in client_addr;
  socklen_t client_addr_size;
  char client_addr_str[INET_ADDRSTRLEN];
  int opt_val;
  char rx_buf[1024];
  ssize_t rx_len;

  // handle new connections
  if (FD_ISSET(server_fd, fds)) {
    client_addr_size = sizeof(client_addr);
    cfd = accept(server_fd, (struct sockaddr *) &client_addr, &client_addr_size);
    if (cfd < 0) {
      syslog(LOG_ERR, "Failed on client accept");
      return -1;
    }

    client_addr_str[0] = 0;
    inet_ntop(AF_INET, &client_addr.sin_addr, client_addr_str, sizeof(client_addr_str));
    syslog(LOG_INFO, "connect from host %s, port %u.\n", client_addr_str, ntohs(client_addr.sin_port));

    opt_val = 1;
    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &opt_val, sizeof opt_val);

    FD_SET(cfd, select_fds);
    FD_SET(cfd, &client_fds);
  }

  // process client fds
  for (cfd = 0; cfd < FD_SETSIZE; cfd++) {
    if (FD_ISSET(cfd, &client_fds) && FD_ISSET(cfd, fds)) {
      // read data from client
      rx_len = read(cfd, rx_buf, sizeof(rx_buf));
      if (rx_len > 0) {
        // ignore data
      } else {
        // close connection
        close(cfd);        
        FD_CLR(cfd, select_fds);
        FD_CLR(cfd, &client_fds);
      }
    }
  }

  return 0;
}

int nmeasrv_broadcast(const char *fmt, ...) {
  va_list ap;
  char msg[256];
  unsigned char cs;
  char *p;
  int cfd, len;

  // generate formatted message
  va_start(ap, fmt);
  len = vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);

  // check error and size
  if (len < 0) {
    return len;
  }
  if (len >= (sizeof(msg) - 4)) {
    errno = -EINVAL;
    return -1;
  }

  // first char must be a '$'
  if (msg[0] != '$') {
    errno = -EINVAL;
    return -1;
  }

  // calc checksum
  for (cs = 0, p = &msg[1], len = 1; *p != 0; p++, len++) {
    cs ^= *p;
  }

  // add ckecksum and linefeed to message
  len += sprintf(p, "*%02X\n", cs);

  // send to all clients
  for (cfd = 0; cfd < FD_SETSIZE; cfd++) {
    if (FD_ISSET(cfd, &client_fds)) {
      write(cfd, msg, len);
    }
  }

  return len;
}


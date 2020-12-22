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
#define MAX_CLIENTS 16
#define RX_BUF_SIZE 1024

typedef struct {
  int fd;
  struct sockaddr_in addr;
  char rx_buf[RX_BUF_SIZE];
  int rx_buf_pos;
} CLIENT_DATA_T;

static int server_fd = -1;
static CLIENT_DATA_T clients[MAX_CLIENTS];

static CLIENT_DATA_T *find_free_client();
static void handle_client_data(CLIENT_DATA_T *client);

static CLIENT_DATA_T *find_free_client() {
  int i;
  CLIENT_DATA_T *client;

  for (i=0, client = clients; i<MAX_CLIENTS; i++, client++) {
    if (client->fd < 0) {
      return client;
    }
  }

  return NULL;
}

static void handle_client_data(CLIENT_DATA_T *client) {
}

int nmeasrv_init(fd_set *fds) {
  int i;
  CLIENT_DATA_T *client;

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
  for (i=0, client = clients; i<MAX_CLIENTS; i++, client++) {
    memset(client, 0, sizeof(CLIENT_DATA_T));
    client->fd = -1;
  }

  return 0;

fail1:
  close(server_fd);
fail0:
  return -1;
}

void nmeasrv_cleanup() {
  int i;
  CLIENT_DATA_T *client;

  // close client connections
  for (i=0, client = clients; i<MAX_CLIENTS; i++, client++) {
    if (client->fd >= 0) {
      close(client->fd);
    }
  }

  // close server connection
  close(server_fd);
}

int nmeasrv_task(fd_set *fds, fd_set *select_fds) {
  int i, j;
  CLIENT_DATA_T *client;
  int cfd;
  struct sockaddr_in client_addr;
  socklen_t client_addr_size;
  char client_addr_str[INET_ADDRSTRLEN];
  int opt_val;
  char rx_buf[RX_BUF_SIZE];
  char *p;
  ssize_t rx_len;

  // handle new connections
  if (FD_ISSET(server_fd, select_fds)) {
    client_addr_size = sizeof(client_addr);
    cfd = accept(server_fd, (struct sockaddr *) &client_addr, &client_addr_size);
    if (cfd < 0) {
      syslog(LOG_ERR, "Failed on client accept");
      return -1;
    }

    client_addr_str[0] = 0;
    inet_ntop(AF_INET, &client_addr.sin_addr, client_addr_str, sizeof(client_addr_str));

    client = find_free_client();
    if (client == NULL) {
      syslog(LOG_INFO, "reject connect from host %s, port %u. Maximim client connections exceeded.", client_addr_str, ntohs(client_addr.sin_port));
      close(cfd);
    } else {
      syslog(LOG_INFO, "connect from host %s, port %u.", client_addr_str, ntohs(client_addr.sin_port));

      opt_val = 1;
      setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &opt_val, sizeof opt_val);

      memset(client, 0, sizeof(CLIENT_DATA_T));
      client->fd = cfd;
      client->addr = client_addr;
      FD_SET(cfd, fds);
    }
  }

  // process clients
  for (i=0, client = clients; i<MAX_CLIENTS; i++, client++) {
    if (client->fd >= 0 && FD_ISSET(client->fd, select_fds)) {
      // read data from client
      rx_len = read(client->fd, rx_buf, sizeof(rx_buf));
      if (rx_len > 0) {
        // receive data
        for (j = 0, p = rx_buf; j <rx_len; j++, p++) {
          if (*p == '\r') {
            continue;
          }
          if (*p == '\n') {
            client->rx_buf[client->rx_buf_pos] = 0;
            client->rx_buf_pos = 0;
            handle_client_data(client);
            continue;
          }
          if (client->rx_buf_pos < (RX_BUF_SIZE - 1)) {
            client->rx_buf[(client->rx_buf_pos)++] = *p;
          }
        }
      } else {
        // close connection
        close(client->fd);
        FD_CLR(client->fd, fds);
        client->fd = -1;
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
  int len;
  int i;
  CLIENT_DATA_T *client;

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
  for (i=0, client = clients; i<MAX_CLIENTS; i++, client++) {
    if (client->fd >= 0) {
      write(client->fd, msg, len);
    }
  }

  return len;
}


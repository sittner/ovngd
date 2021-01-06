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
#include "baro.h"

#define OVNG_PORT 4353
#define MAX_CLIENTS 16
#define RX_BUF_SIZE 1024

#define BARO_REF_MIN 500.0
#define BARO_REF_MAX 1500.0


typedef struct {
  int fd;
  struct sockaddr_in addr;
  char rx_buf[RX_BUF_SIZE];
  int rx_buf_pos;
} CLIENT_DATA_T;

static fd_set *main_fds;
static int server_fd = -1;
static CLIENT_DATA_T clients[MAX_CLIENTS];

static CLIENT_DATA_T *find_free_client();
static CLIENT_DATA_T *accept_client(struct sockaddr_in client_addr, int cfd);
static void disconnect_client(CLIENT_DATA_T * client);
static int read_client_data(CLIENT_DATA_T *client);
static void handle_client_data(CLIENT_DATA_T *client);
static void handle_client_data_calib(CLIENT_DATA_T *client, char *pos);
static void handle_client_data_calib_baro(CLIENT_DATA_T *client, char *pos);
static char *next_token(char **pos);

static CLIENT_DATA_T *find_free_client() {
  int i;
  CLIENT_DATA_T *client;

  for (i = 0, client = clients; i < MAX_CLIENTS; i++, client++) {
    if (client->fd < 0) {
      return client;
    }
  }

  return NULL;
}

static CLIENT_DATA_T *accept_client(struct sockaddr_in client_addr, int cfd) {
  CLIENT_DATA_T *client;
  char client_addr_str[INET_ADDRSTRLEN];
  int opt_val;

  client_addr_str[0] = 0;
  inet_ntop(AF_INET, &client_addr.sin_addr, client_addr_str, sizeof(client_addr_str));

  client = find_free_client();
  if (client == NULL) {
    syslog(LOG_INFO, "reject connect from host %s, port %u. Maximim client connections exceeded.", client_addr_str, ntohs(client_addr.sin_port));
    close(cfd);
    return NULL;
  }

  syslog(LOG_INFO, "connect from host %s, port %u.", client_addr_str, ntohs(client_addr.sin_port));

  opt_val = 1;
  setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &opt_val, sizeof opt_val);

  memset(client, 0, sizeof(CLIENT_DATA_T));
  client->fd = cfd;
  client->addr = client_addr;
  FD_SET(cfd, main_fds);

  return client;
}

static void disconnect_client(CLIENT_DATA_T * client) {
  if (client->fd < 0) {
    return;
  }

  close(client->fd);
  FD_CLR(client->fd, main_fds);
  client->fd = -1;
}

static int read_client_data(CLIENT_DATA_T *client) {
  int i;
  char rx_buf[RX_BUF_SIZE];
  char *p;
  ssize_t rx_len;

  // read data from client
  rx_len = recv(client->fd, rx_buf, sizeof(rx_buf), 0);
  if (rx_len < 0) {
    disconnect_client(client);
    return -1;
  }

  // receive data
  for (i = 0, p = rx_buf; i <rx_len; i++, p++) {
    if (*p == '\b') {
      if (client->rx_buf_pos > 0) {
        (client->rx_buf_pos)--;
      }
      continue;
    }

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

  return 0;
}

static void handle_client_data(CLIENT_DATA_T *client) {
  char *line = client->rx_buf;
  char *pos;
  unsigned char csum_rx, csum;
  char *type;

  // check for start delimiter
  if (line[0] != '$') {
    return;
  }
  line++;

  // search for csum delimiter
  pos = strchr(line, '*');
  if (pos == NULL) {
    return;
  }

  // parse received checksum
  *(pos++) = 0;
  csum_rx = (unsigned char) strtol(pos, NULL, 16);

  // calculate local checksum
  for (csum = 0, pos = line; *pos; pos++) {
    csum ^= *pos;
  }

  // verify checksum
  if (csum_rx != csum) {
    return;
  }

  // check type
  type = next_token(&line);
  if (type == NULL) {
    return;
  }

  // dispatch types
  if (strcmp(type, "POVCAL") == 0) {
    handle_client_data_calib(client, line);
    return;
  }
}

static void handle_client_data_calib(CLIENT_DATA_T *client, char *pos) {
  char *subtype;

  // get subtype
  subtype = next_token(&pos);
  if (subtype == NULL) {
    return;
  }

  // dispatch subtypes
  if (strcmp(subtype, "B") == 0) {
    handle_client_data_calib_baro(client, pos);
    return;
  }
}

static void handle_client_data_calib_baro(CLIENT_DATA_T *client, char *pos) {
  char *subtype;
  char *s;
  int baro_autoref;
  double baro_ref;

  // get command
  subtype = next_token(&pos);
  if (subtype == NULL) {
    return;
  }

  // dispatch commands
  if (strcmp(subtype, "S") == 0) {
    // get baro_ref
    s = next_token(&pos);
    if (s == NULL) {
      // use autoref
      baro_autoref = 1;
      baro_ref = 0.0;
    } else {
      // verify baro_ref
      baro_autoref = 0;
      baro_ref = atof(s);
      if (baro_ref < BARO_REF_MIN || baro_ref > BARO_REF_MAX) {
        return;
      }
    }

    // start calibration
    baro_start_calib(baro_autoref, baro_ref);
  }
}

static char *next_token(char **pos) {
  char *val, *sep;

  if (*pos == NULL) {
    return NULL;
  }

  val = *pos;
  sep = strchr(val, ',');
  if (sep == NULL) {
    *pos = NULL;
  } else {
    *sep = 0;
    *pos = sep + 1;
  }

  return val;
}

int nmeasrv_init(fd_set *fds) {
  int i;
  CLIENT_DATA_T *client;

  main_fds = fds;

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

  FD_SET(server_fd, main_fds);
  for (i = 0, client = clients; i < MAX_CLIENTS; i++, client++) {
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
  for (i=0, client = clients; i < MAX_CLIENTS; i++, client++) {
    if (client->fd >= 0) {
      close(client->fd);
    }
  }

  // close server connection
  close(server_fd);
}

int nmeasrv_task(fd_set *select_fds) {
  int i;
  CLIENT_DATA_T *client;
  int cfd;
  struct sockaddr_in client_addr;
  socklen_t client_addr_size;

  // handle new connections
  if (FD_ISSET(server_fd, select_fds)) {
    client_addr_size = sizeof(client_addr);
    cfd = accept(server_fd, (struct sockaddr *) &client_addr, &client_addr_size);
    if (cfd < 0) {
      syslog(LOG_ERR, "Failed on client accept");
      return -1;
    }

    accept_client(client_addr, cfd);
  }

  // process clients
  for (i = 0, client = clients; i < MAX_CLIENTS; i++, client++) {
    if (client->fd >= 0 && FD_ISSET(client->fd, select_fds)) {
      read_client_data(client);
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
  for (i = 0, client = clients; i < MAX_CLIENTS; i++, client++) {
    if (client->fd >= 0) {
      if (send(client->fd, msg, len, MSG_NOSIGNAL) < 0) {
        disconnect_client(client);
      }
    }
  }

  return len;
}


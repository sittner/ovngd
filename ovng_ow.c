#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <sys/eventfd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "ovng_ow.h"

#define W1_READ_SEP_TIME_US	2000000

typedef enum {
  STATE_STOP = 0,
  STATE_RUN,
  STATE_EXIT
} STATE_T;

static OVOW_TEMP_T *temp_list;
static STATE_T temp_thread_state;
static int temp_event;
static pthread_mutex_t temp_mutex;
static pthread_t temp_thread;

typedef struct {
  unsigned char rom[9];
  unsigned char crc;
  int verdict;
  unsigned char family_data[9];
  int temp;
} w1fs_temp_t;

static int read_w1fs_temp(const char *fn, w1fs_temp_t *t) {
  int fd;
  ssize_t len;
  char buf[4096];
  char vs[4];
  int cnt;
  int ret = -1;

  fd = open(fn, O_RDONLY);
  if (fd < 0) {
    goto fail0;
  }

  len = read(fd, &buf, sizeof(buf));
  if (len < 0 || len >= sizeof(buf)) {
    goto fail1;
  } 

  buf[len] = 0;

  cnt = sscanf(buf,
    "%02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx : crc=%02hhx %3s\n"
    "%02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx t=%d",
    &t->rom[0], &t->rom[1], &t->rom[2], &t->rom[3],
    &t->rom[4], &t->rom[5], &t->rom[6], &t->rom[7],
    &t->rom[8], &t->crc, &vs[0],
    &t->family_data[0], &t->family_data[1], &t->family_data[2], &t->family_data[3],
    &t->family_data[4], &t->family_data[5], &t->family_data[6], &t->family_data[7],
    &t->family_data[8], &t->temp);
  if (cnt != 21) {
    goto fail1;
  }

  t->verdict = (strcmp(vs, "YES") == 0);
  ret = 0;

fail1:
  close(fd);
fail0:
  return ret;
}

static void read_temp(OVOW_TEMP_T *node) {
  w1fs_temp_t t;
  uint64_t u = 1;

  if (read_w1fs_temp(node->fname, &t) < 0) {
    return;
  }

  if (t.verdict) {
    pthread_mutex_lock(&temp_mutex);
    node->temp = (double) t.temp * 0.001;
    node->has_data = 1;
    pthread_mutex_unlock(&temp_mutex);
    write(temp_event, &u, sizeof(u));
  }
}

static void *temp_thread_proc(void *p) {
  OVOW_TEMP_T *node;

  while (1) {
    for (node = temp_list; node != NULL; node = node->next) {
      if (temp_thread_state == STATE_EXIT) {
        return NULL;
      }
      read_temp(node);
      usleep(W1_READ_SEP_TIME_US);
    }
  }
}

void ovow_init(void) {
  temp_list = NULL;
  temp_thread_state = STATE_STOP;
  temp_event = -1;
  pthread_mutex_init(&temp_mutex, NULL);
}

int ovow_temp_add(const char *id, const char *name, ovow_data_cb_t callback) {
  OVOW_TEMP_T *node, *last;

  // create new node
  node = malloc(sizeof(OVOW_TEMP_T));
  if (node == NULL) {
    errno = -ENOMEM;
    goto fail0;
  }

  // build path
  if (asprintf(&node->fname, "/sys/bus/w1/devices/%s/w1_slave", id) < 0) {
    goto fail1;
  }

  // initialize fields
  node->callback = callback;
  node->temp = NAN;
  node->has_data = 0;
  node->next = NULL;

  node->id = strdup(id);
  if (node->id == NULL) {
    errno = -ENOMEM;
    goto fail2;
  }

  node->name = strdup(name);
  if (node->name == NULL) {
    errno = -ENOMEM;
    goto fail3;
  }

  // initialize list
  if (temp_list == NULL) {
    temp_list = node;
    return 0;
  }

  // find last node and add new node
  for (last = temp_list; last->next != NULL; last = last->next);
  last->next = node;

  return 0;

fail3:
  free(node->id);
fail2:
  free(node->fname);
fail1:
  free(node);
fail0:
  return -1;
}

int ovow_temp_start(fd_set *fds) {
  // do not start if list is empty
  if (temp_list == NULL) {
    return 0;
  }

  temp_event = eventfd(0, 0);
  if (temp_event < 0) {
    goto fail0;
  }

  if (pthread_create(&temp_thread, NULL, temp_thread_proc, NULL) < 0) {
    goto fail1;
  }

  temp_thread_state = STATE_RUN;
  FD_SET(temp_event, fds);
  return 0;

fail1:
  close(temp_event);
fail0:
  return -1;
}

void ovow_temp_cleanup(void) {
  OVOW_TEMP_T *node, *next;

  if (temp_thread_state == STATE_RUN) {
    temp_thread_state = STATE_EXIT;
    pthread_join(temp_thread, NULL);
  }

  node = temp_list;
  while (node != NULL) {
    next = node->next;
    free(node->name);
    free(node->id);
    free(node);
    node = next;
  }

  if (temp_event >= 0) {
    close(temp_event);
  }

  pthread_mutex_destroy(&temp_mutex);
}

void ovow_temp_task(fd_set *fds) {
  OVOW_TEMP_T *node;
  uint64_t u;

  if (FD_ISSET(temp_event, fds)) {
    read(temp_event, &u, sizeof(u));

    pthread_mutex_lock(&temp_mutex);
    for (node = temp_list; node != NULL; node = node->next) {
      if (node->has_data) {
        if (node->callback != NULL) {
          node->callback(node);
        }
        node->has_data = 0;
      }
    }
    pthread_mutex_unlock(&temp_mutex);
  }
}


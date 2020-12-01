#ifndef _OVNG_OW_H
#define _OVNG_OW_H

#include <pthread.h>

struct OVOW_TEMP;
typedef struct OVOW_TEMP OVOW_TEMP_T;

typedef void (*ovow_data_cb_t)(const OVOW_TEMP_T *data);

struct OVOW_TEMP {
  OVOW_TEMP_T *next;
  char *id;
  char *name;
  ovow_data_cb_t callback;
  char *fname;
  double temp;
  int has_data;
};

void ovow_init(void);
int ovow_temp_add(const char *id, const char *name, ovow_data_cb_t callback);
int ovow_temp_start(fd_set *fds);
void ovow_temp_cleanup(void);
void ovow_temp_task(fd_set *fds);

#endif

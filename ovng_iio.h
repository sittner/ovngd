#ifndef _OVNG_IIO_H
#define _OVNG_IIO_H

#include <sys/select.h>
#include <sys/types.h>

#include "iio_utils.h"
#include "vector.h"

#define IIO_SAMPLE_FREQ 20

typedef struct {
  const char *attr;
  const char *val;
} OVIIO_ATTR_INIT_T;

typedef void (*oviio_1d_data_cb_t)(double data);
typedef void (*oviio_3d_data_cb_t)(vector3d_t data);
typedef void (*oviio_done_cb_t)(void);

typedef struct {
  oviio_1d_data_cb_t vbat_cb;
  oviio_1d_data_cb_t press_stat_cb;
  oviio_1d_data_cb_t press_tek_cb;
  oviio_1d_data_cb_t press_dyn_cb;
  oviio_3d_data_cb_t accel_cb;
  oviio_3d_data_cb_t anglvel_cb;
  oviio_3d_data_cb_t magn_cb;
  oviio_done_cb_t imu_done_cb;
} OVIIO_DATA_CALLBACKS_T;

typedef struct {
  oviio_1d_data_cb_t callback;
} OVIIO_DIRECT_CHAN_T;

typedef struct {
  struct iioutils_chaninfo *ch;
  oviio_1d_data_cb_t callback;
} OVIIO_1D_CHAN_T;

typedef struct {
  struct iioutils_mount_matrix *mnt_mtx;
  struct iioutils_chaninfo *ch_x;
  struct iioutils_chaninfo *ch_y;
  struct iioutils_chaninfo *ch_z;
  oviio_3d_data_cb_t callback;
} OVIIO_3D_CHAN_T;

typedef struct {
  const char *name;
  OVIIO_DIRECT_CHAN_T *ch_direct;
  OVIIO_1D_CHAN_T *ch1d;
  OVIIO_3D_CHAN_T *ch3d;
  double scale;
} OVIIO_CHAN_DESC_T;

struct OVIIO_DEV_DESC;
typedef struct OVIIO_DEV_DESC OVIIO_DEV_DESC_T;

typedef struct {
  const OVIIO_DEV_DESC_T *desc;
  int index;
  struct iioutils_chaninfo *ci_array;
  int ci_count;
  int scan_size;
  char *buffer;
  int buffer_fd;
  oviio_done_cb_t scan_done;
} OVIIO_DEV_T;

struct OVIIO_DEV_DESC {
  enum IIOUTILS_TYPE type;
  const char *name;
  int match_of_name;
  int trigger_freq;
  const OVIIO_ATTR_INIT_T *init;
  const OVIIO_CHAN_DESC_T *cd;
  OVIIO_DEV_T *trigger;
  OVIIO_DEV_T *dev;
  struct iioutils_mount_matrix *mnt_mtx;
};

int oviio_init(fd_set *fds, const OVIIO_DATA_CALLBACKS_T *callbacks, int direct_period);
void oviio_cleanup();

int oviio_task(fd_set *fds);

#endif


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>

#include "ovng_iio.h"

#define VBAT_REF	1.8
#define VBAT_RES	4095.0
#define VBAT_R_VCC	120.0
#define VBAT_R_GND	10.0

#define VBAT_SCALE	((VBAT_REF / VBAT_RES) * ((VBAT_R_VCC + VBAT_R_GND) / VBAT_R_GND))

static OVIIO_DEV_T oviio_dev_trigger;
static OVIIO_DEV_T oviio_dev_adc;
static OVIIO_DEV_T oviio_dev_press_stat;
static OVIIO_DEV_T oviio_dev_press_tek;
static OVIIO_DEV_T oviio_dev_press_dyn;
static OVIIO_DEV_T oviio_dev_imu;

static OVIIO_DIRECT_CHAN_T oviio_ch_vbat;

static OVIIO_1D_CHAN_T oviio_ch_press_stat;
static OVIIO_1D_CHAN_T oviio_ch_press_tek;
static OVIIO_1D_CHAN_T oviio_ch_press_dyn;

static OVIIO_3D_CHAN_T oviio_ch_accel;
static OVIIO_3D_CHAN_T oviio_ch_anglvel;
static OVIIO_3D_CHAN_T oviio_ch_magn;

static struct iioutils_mount_matrix oviio_mnt_mtx_imu;

static const OVIIO_ATTR_INIT_T oviio_attr_init_ms5611[] = {
  { "in_pressure_oversampling_ratio", "4096" },
  { }
};

static const OVIIO_ATTR_INIT_T oviio_attr_init_imu[] = {
  { "in_accel_scale", "0.004788403" },
  { "in_accel_filter_low_pass_3db_frequency", "11.5" },
  { "in_anglvel_scale", "0.000266316" },
  { "in_anglvel_filter_low_pass_3db_frequency", "11.6" },
  { }
};

static const OVIIO_CHAN_DESC_T oviio_adc_chdecs[] = {
  { .name = "in_voltage5_raw", .ch_direct = &oviio_ch_vbat, .scale = VBAT_SCALE },
  {}
};

static const OVIIO_CHAN_DESC_T oviio_press_stat_chdecs[] = {
  { .name = "in_pressure", .ch1d = &oviio_ch_press_stat, .scale = 10.0 }, // static pressure in hPa
  {}
};

static const OVIIO_CHAN_DESC_T oviio_press_tek_chdecs[] = {
  { .name = "in_pressure", .ch1d = &oviio_ch_press_tek, .scale = 10.0 }, // TEK pressure in hPa
  {}
};

static const OVIIO_CHAN_DESC_T oviio_press_dyn_chdecs[] = {
  { .name = "in_pressure", .ch1d = &oviio_ch_press_dyn, .scale = 1000.0 }, // dynamic pressure in Pa
  {}
};

static const OVIIO_CHAN_DESC_T oviio_imu_chdecs[] = {
  { .name = "in_accel", .ch3d = &oviio_ch_accel, .scale = 1.0 },
  { .name = "in_anglvel", .ch3d = &oviio_ch_anglvel, .scale = 1.0 },
  { .name = "in_magn", .ch3d = &oviio_ch_magn, .scale = 1.0 },
  {}
};

static const OVIIO_DEV_DESC_T oviio_dev_descs[] = {
  {
    .type = IIO_TYPE_TRIGGER,
    .name = "ovng-trigger",
    .trigger_freq = IIO_SAMPLE_FREQ,
    .dev = &oviio_dev_trigger
  }, {
    .type = IIO_TYPE_DEVICE,
    .name = "adc",
    .match_of_name = 1,
    .cd = oviio_adc_chdecs,
    .dev = &oviio_dev_adc
  }, {
    .type = IIO_TYPE_DEVICE,
    .name = "press_stat",
    .match_of_name = 1,
    .cd = oviio_press_stat_chdecs,
    .init = oviio_attr_init_ms5611,
    .trigger = &oviio_dev_trigger,
    .dev = &oviio_dev_press_stat
  }, {
    .type = IIO_TYPE_DEVICE,
    .name = "press_tek",
    .match_of_name = 1,
    .cd = oviio_press_tek_chdecs,
    .init = oviio_attr_init_ms5611,
    .trigger = &oviio_dev_trigger,
    .dev = &oviio_dev_press_tek
  }, {
    .type = IIO_TYPE_DEVICE,
    .name = "press_dyn",
    .match_of_name = 1,
    .cd = oviio_press_dyn_chdecs,
    .trigger = &oviio_dev_trigger,
    .dev = &oviio_dev_press_dyn
  }, {
    .type = IIO_TYPE_DEVICE,
    .name = "imu",
    .match_of_name = 1,
    .cd = oviio_imu_chdecs,
    .init = oviio_attr_init_imu,
    .trigger = &oviio_dev_trigger,
    .dev = &oviio_dev_imu,
    .mnt_mtx = &oviio_mnt_mtx_imu
  },
  { }
};

static int create_timer(int period_ms) {
  int fd;
  struct itimerspec timspec;

  fd = timerfd_create(CLOCK_MONOTONIC, 0);
  if (fd < 0) {
    goto fail0;
  }

  memset(&timspec, 0, sizeof(timspec));
  timspec.it_interval.tv_sec = period_ms / 1000;
  timspec.it_interval.tv_nsec = (period_ms % 1000) * 1000000;
  timspec.it_value.tv_sec = 0;
  timspec.it_value.tv_nsec = 1;

  if (timerfd_settime(fd, 0, &timspec, 0) < 0) {
    goto fail1;
  }

  return fd;

fail1:
  close(fd);
fail0:
  return -1;
}

static int direct_timer;

static struct iioutils_chaninfo *oviio_find_channel(OVIIO_DEV_T *dev, const char *name) {
  struct iioutils_chaninfo *ci;
  int i;

  for (i = 0, ci = dev->ci_array; i < dev->ci_count; i++, ci++) {
    if (strcmp(name, ci->name) == 0) {
      return ci;
    }
  }

  return NULL;
}

static int oviio_enable_1d_channel(OVIIO_DEV_T *dev, const char *name) {
  char buf[IIOUTILS_MAX_NAME_LENGTH];

  snprintf(buf, IIOUTILS_MAX_NAME_LENGTH, "scan_elements/%s_en", name);
  if (iioutils_write_int(dev->desc->type, dev->index, buf, 1) < 0) {
    return -1;
  }

  return 0;
}

static int oviio_find_1d_channel(OVIIO_DEV_T *dev, OVIIO_1D_CHAN_T *chan, const char *name) {
  chan->ch = oviio_find_channel(dev, name);
  if (chan->ch == NULL) {
    return -1;
  }

  return 0;
}

static int oviio_enable_3d_channel(OVIIO_DEV_T *dev, const char *name) {
  char buf[IIOUTILS_MAX_NAME_LENGTH];

  snprintf(buf, IIOUTILS_MAX_NAME_LENGTH, "scan_elements/%s_x_en", name);
  if (iioutils_write_int(dev->desc->type, dev->index, buf, 1) < 0) {
    return -1;
  }

  snprintf(buf, IIOUTILS_MAX_NAME_LENGTH, "scan_elements/%s_y_en", name);
  if (iioutils_write_int(dev->desc->type, dev->index, buf, 1) < 0) {
    return -1;
  }

  snprintf(buf, IIOUTILS_MAX_NAME_LENGTH, "scan_elements/%s_z_en", name);
  if (iioutils_write_int(dev->desc->type, dev->index, buf, 1) < 0) {
    return -1;
  }

  return 0;
}

static int oviio_find_3d_channel(OVIIO_DEV_T *dev, OVIIO_3D_CHAN_T *chan, const char *name) {
  char buf[IIOUTILS_MAX_NAME_LENGTH];

  snprintf(buf, IIOUTILS_MAX_NAME_LENGTH, "%s_x", name);
  chan->ch_x = oviio_find_channel(dev, buf);
  if (chan->ch_x == NULL) {
    return -1;
  }

  snprintf(buf, IIOUTILS_MAX_NAME_LENGTH, "%s_y", name);
  chan->ch_y = oviio_find_channel(dev, buf);
  if (chan->ch_y == NULL) {
    return -1;
  }

  snprintf(buf, IIOUTILS_MAX_NAME_LENGTH, "%s_z", name);
  chan->ch_z = oviio_find_channel(dev, buf);
  if (chan->ch_z == NULL) {
    return -1;
  }

  return 0;
}


int oviio_init(fd_set *fds, const OVIIO_DATA_CALLBACKS_T *callbacks, int direct_period) {
  int idx;
  const OVIIO_DEV_DESC_T *desc;
  OVIIO_DEV_T *dev;
  const OVIIO_ATTR_INIT_T *init;
  const OVIIO_CHAN_DESC_T *cd;
  char buf[IIOUTILS_MAX_NAME_LENGTH];

  // try to find devices
  for (desc = oviio_dev_descs; desc->name != NULL; desc++) {
    // create hrt triggers as needed
    if (desc->trigger_freq > 0) {
      if (iioutils_create_hrt_trigger(desc->name)) {
        fprintf(stderr, "ERROR: unable to create trigger '%s'.\n", desc->name);
        goto fail0;
      }
    }

    // initialize device structure
    dev = desc->dev;
    memset(dev, 0, sizeof(OVIIO_DEV_T));
    dev->desc = desc;
    dev->index = -1;
    dev->buffer_fd = -1;

    // search device
    idx = iioutils_find_by_name(desc->name, desc->type, desc->match_of_name);
    if (idx < 0) {
      fprintf(stderr, "ERROR: IIO device '%s' not found.\n", desc->name);
      continue;
    }
    dev->index = idx;
  }

  // initialize device attributes
  for (desc = oviio_dev_descs; desc->name != NULL; desc++) {
    dev = desc->dev;

    if (desc->trigger_freq > 0) {
      if (iioutils_write_int(desc->type, dev->index, "sampling_frequency", desc->trigger_freq) < 0) {
        fprintf(stderr, "ERROR: Unable to set sampling_frequency for trigger '%s'.\n", desc->name);
        goto fail0;
      }
    }

    for (init = desc->init; init != NULL && init->attr != NULL; init++) {
      if (iioutils_write_string(desc->type, dev->index, init->attr, init->val) < 0) {
        fprintf(stderr, "ERROR: Unable to write init attribute '%s' for device '%s'.\n",
          init->attr, desc->name);
        goto fail0;
      }
    }

    if (desc->mnt_mtx != NULL) {
      if (iioutils_read_mount_matrix(desc->type, dev->index, desc->mnt_mtx) < 0) {
        fprintf(stderr, "WARNING: Unable to read mount_matrix for device '%s'. Using 1:1 mapping.\n",
          desc->name);
      }
    }
  }

  // map and enable channels
  for (desc = oviio_dev_descs; desc->name != NULL; desc++) {
    dev = desc->dev;

    // process only devices with channels
    if (desc->cd == NULL) {
      continue;
    }

    // enable required channels
    idx = 0;
    for (cd = desc->cd; cd->name != NULL; cd++) {
      if (cd->ch1d != NULL) {
        idx++;
        if (oviio_enable_1d_channel(dev, cd->name) < 0) {
          fprintf(stderr, "ERROR: Unable to enable 1d channel '%s' on device '%s'.\n",
            cd->name, desc->name);
          goto fail1;
        }
      }

      if (cd->ch3d != NULL) {
        idx++;
        if (oviio_enable_3d_channel(dev, cd->name) < 0) {
          fprintf(stderr, "ERROR: Unable to enable 3d channel '%s' on device '%s'.\n",
            cd->name, desc->name);
          goto fail1;
        }
      }
    }

    // skip device if only direct channels found
    if (idx == 0) {
      continue;
    }

    // build channel array for device
    if (iioutils_build_channel_array(desc->type, dev->index, &dev->ci_array, &dev->ci_count) < 0) {
      fprintf(stderr, "ERROR: Unable read channel info for device '%s'.\n", desc->name);
      goto fail1;
    }

    // map channels
    for (cd = desc->cd; cd->name != NULL; cd++) {
      if (cd->ch1d != NULL) {
        if (oviio_find_1d_channel(dev, cd->ch1d, cd->name) < 0) {
          fprintf(stderr, "ERROR: Unable to find 1d channel '%s' on device '%s'.\n",
            cd->name, desc->name);
          goto fail1;
        }
      }

      if (cd->ch3d != NULL) {
        if (oviio_find_3d_channel(dev, cd->ch3d, cd->name) < 0) {
          fprintf(stderr, "ERROR: Unable to find 3d channel '%s' on device '%s'.\n",
            cd->name, desc->name);
          goto fail1;
        }
        cd->ch3d->mnt_mtx = desc->mnt_mtx;
      }
    }
  }

  // create buffers and open devices
  for (desc = oviio_dev_descs; desc->name != NULL; desc++) {
    dev = desc->dev;
    if (dev->ci_array == NULL) {
      continue;
    }

    // set trigger
    if (desc->trigger != NULL) {
      if (iioutils_read_string(desc->trigger->desc->type, desc->trigger->index, "name", buf) < 0) {
        fprintf(stderr, "ERROR: Unable to get trigger name for device '%s'.\n", desc->name);
        goto fail1;
      }

      if (iioutils_write_string(desc->type, dev->index, "trigger/current_trigger", buf) < 0) {
        fprintf(stderr, "ERROR: Unable to set trigger for device '%s'.\n", desc->name);
        goto fail1;
      }
    }

    // enable buffer
    if (iioutils_write_int(desc->type, dev->index, "buffer/length", 2) < 0) {
      fprintf(stderr, "ERROR: Unable to enable buffer for device '%s'.\n", desc->name);
      goto fail1;
    }
    if (iioutils_write_int(desc->type, dev->index, "buffer/enable", 1) < 0) {
      fprintf(stderr, "ERROR: Unable to enable buffer for device '%s'.\n", desc->name);
      goto fail1;
    }

    // allocate buffer memory
    dev->scan_size = iioutils_get_scan_size(dev->ci_array, dev->ci_count);
    dev->buffer = malloc(dev->scan_size);
    if (dev->buffer == NULL) {
      fprintf(stderr, "ERROR: Unable to alloc buffer for device '%s'.\n", desc->name);
      goto fail1;
    }

    // open buffer device
    snprintf(buf, sizeof(buf), "/dev/iio:device%d", dev->index);
    dev->buffer_fd = open(buf, O_RDONLY | O_NONBLOCK);
    if (dev->buffer_fd < 0) {
      fprintf(stderr, "ERROR: Unable to open buffer access for device '%s'.\n", desc->name);
      goto fail1;
    }

    // add to global FD set
    FD_SET(dev->buffer_fd, fds);
  }

  // create timer for direct polling if needed
  if (direct_period > 0) {
    direct_timer = create_timer(direct_period);
    if (direct_timer < 0) {
      fprintf(stderr, "ERROR: Unable to create timer fd for direct iio polling.\n");
      goto fail1;
    }
    FD_SET(direct_timer, fds);
  } else {
    direct_timer = -1;
  }

  // setup callbacks
  oviio_ch_vbat.callback = callbacks->vbat_cb;
  oviio_ch_press_stat.callback = callbacks->press_stat_cb;
  oviio_ch_press_tek.callback = callbacks->press_tek_cb;
  oviio_ch_press_dyn.callback = callbacks->press_dyn_cb;
  oviio_ch_accel.callback = callbacks->accel_cb;
  oviio_ch_anglvel.callback = callbacks->anglvel_cb;
  oviio_ch_magn.callback = callbacks->magn_cb;
  oviio_dev_imu.scan_done = callbacks->imu_done_cb;

  return 0;

fail1:
  oviio_cleanup();

fail0:
  return -1;
}

void oviio_cleanup() {
  const OVIIO_DEV_DESC_T *desc;
  OVIIO_DEV_T *dev;

  if (direct_timer >= 0) {
    close(direct_timer);
  }

  // free channel arrays
  for (desc = oviio_dev_descs; desc->name != NULL; desc++) {
    dev = desc->dev;

    if (dev->buffer_fd >= 0) {
      close(dev->buffer_fd);
      iioutils_write_int(desc->type, dev->index, "buffer/enable", 0);
    }

    free(dev->buffer);

    iioutils_free_channel_array(dev->ci_array, dev->ci_count);
  }
}

int oviio_task(fd_set *fds) {
  const OVIIO_DEV_DESC_T *desc;
  OVIIO_DEV_T *dev;
  ssize_t read_size;
  const OVIIO_CHAN_DESC_T *cd;
  OVIIO_1D_CHAN_T *ch1d;
  OVIIO_3D_CHAN_T *ch3d;
  vector3d_t v3d;
  OVIIO_DIRECT_CHAN_T *ch_direct;
  double val;
  uint64_t u;

  for (desc = oviio_dev_descs; desc->name != NULL; desc++) {
    dev = desc->dev;

    // check for data on device
    if (dev->buffer_fd >= 0 && FD_ISSET(dev->buffer_fd, fds)) {
      // read device data
      read_size = read(dev->buffer_fd, dev->buffer, dev->scan_size);
      if (read_size != dev->scan_size) {
        if (read_size >= 0) {
          errno = -ENODATA;
        }
        return -1;
      }

      // map data to channels
      for (cd = desc->cd; cd->name != NULL; cd++) {
        // process 1d data
        ch1d = cd->ch1d;
        if (ch1d != NULL && ch1d->callback != NULL) {
          ch1d->callback(iioutils_read_scan_double(ch1d->ch, dev->buffer) * cd->scale);
        }

        // process 3d data
        ch3d = cd->ch3d;
        if (ch3d != NULL && ch3d->callback != NULL) {
          v3d.x = iioutils_read_scan_double(ch3d->ch_x, dev->buffer) * cd->scale;
          v3d.y = iioutils_read_scan_double(ch3d->ch_y, dev->buffer) * cd->scale;
          v3d.z = iioutils_read_scan_double(ch3d->ch_z, dev->buffer) * cd->scale;

          if (ch3d->mnt_mtx != NULL) {
            v3d = iioutils_transform_mount_matrix(ch3d->mnt_mtx, v3d);
          }

          ch3d->callback(v3d);
        }
      }

      // trigger scan_done
      if (dev->scan_done != NULL) {
        dev->scan_done();
      }
    }
  }

  // process direct
  if (direct_timer >= 0 && FD_ISSET(direct_timer, fds)) {
    read(direct_timer, &u, sizeof(u));

    for (desc = oviio_dev_descs; desc->name != NULL; desc++) {
      dev = desc->dev;
      for (cd = desc->cd; cd != NULL && cd->name != NULL; cd++) {
        ch_direct = cd->ch_direct;
        if (ch_direct != NULL) {
          if (iioutils_read_double(desc->type, dev->index, cd->name, &val) < 0) {
            return -1;
          }

          ch_direct->callback(val * cd->scale);
        }
      }
    }
  }

  return 0;
}


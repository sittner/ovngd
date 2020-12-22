#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <syslog.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/eventfd.h>
#include <sys/stat.h>

#include "cfgfile.h"
#include "eeprom.h"
#include "ovng_iio.h"
#include "ovng_ow.h"
#include "nmea_server.h"
#include "baro.h"
#include "ahrs.h"

static int exit_fd;

static void sighandler(int sig)
{
  uint64_t u = 1;

  switch (sig)
  {
    case SIGINT:
    case SIGTERM:
      write(exit_fd, &u, sizeof(u));
      break;
    case SIGHUP:
      break;
    default:
      syslog(LOG_DEBUG, "Unhandled signal %d", sig);
  }
}

static void proc_v_bat(double data) {
  nmeasrv_broadcast("$POV,V,%0.2f", data);
}

static void proc_ow_temp(const OVOW_TEMP_T *data) {
  nmeasrv_broadcast("$POV,%s,%0.2f", data->name, data->temp);
}

const OVIIO_DATA_CALLBACKS_T callbacks = {
  .vbat_cb = proc_v_bat,
  .press_stat_cb = baro_stat_data,
  .press_tek_cb = baro_tek_data,
  .press_dyn_cb = baro_dyn_data,
  .accel_cb = ahrs_accel_data,
  .anglvel_cb = ahrs_anglvel_data,
  .magn_cb = ahrs_magn_data,
  .imu_done_cb = ahrs_scan_done
};

int main(int argc, char* argv[]) {
  int ret = 1;
  int err;
  struct sigaction act;
  fd_set fds, select_fds;
  uint64_t u;
  OVNGD_CONF_T conf;

  // try to read config file
  cfgfile_read(&conf, (argc >= 2) ? argv[1] : NULL);

  // try to read eeprom data
  eeprom_init(conf.eeprom_path);

  FD_ZERO(&fds);

  exit_fd = eventfd(0, 0);
  if (exit_fd < 0) {
    fprintf(stderr, "ERROR: unable to create exit event fd.\n");
    goto fail;
  }
  FD_SET(exit_fd, &fds);

  memset(&act, 0, sizeof(act));
  act.sa_handler = &sighandler;
  sigaction(SIGINT, &act, NULL);
  sigaction(SIGTERM, &act, NULL);
  sigaction(SIGHUP, &act, NULL);

  if (oviio_init(&fds, &callbacks, 1000)) {
    goto fail_exit_fd;
  }

  ovow_init();
  if (conf.temp_id[0] != 0) {
    ovow_temp_add(conf.temp_id, "T", proc_ow_temp);
  }
  if (ovow_temp_start(&fds)) {
    goto fail_ovow_temp;
  }

  if (nmeasrv_init(&fds)) {
    goto fail_ovow_temp;
  }

  baro_init(&conf);
  ahrs_init(conf.ahrs_madg_beta);

  while (1) {
    select_fds = fds;

    err = select(FD_SETSIZE, &select_fds, NULL, NULL, NULL);
    if (err < 0) {
      if (errno == EINTR) {
        continue;
      }
      fprintf(stderr, "ERROR: Failed on select.\n");
      goto fail_nmea_server;
    }

    if (FD_ISSET(exit_fd, &select_fds)) {
      read(exit_fd, &u, sizeof(u));
      break;
    }

    err = oviio_task(&select_fds);
    if (err < 0) {
      fprintf(stderr, "ERROR: Failed to process IIO data.\n");
      goto fail_nmea_server;
    }

    ovow_temp_task(&select_fds);

    err = nmeasrv_task(&fds, &select_fds);
    if (err < 0) {
      goto fail_nmea_server;
    }
  }

fail_nmea_server:
  nmeasrv_cleanup();
fail_ovow_temp:
  ovow_temp_cleanup();
  oviio_cleanup();
fail_exit_fd:
  close(exit_fd);
fail:
  return ret;
}


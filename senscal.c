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
#include <math.h>

#include "ovng_iio.h"

// Sea level standard atmospheric pressure [hPa]
#define BARO_P0 (1013.25)

// Temperature lapse rate for dry air [K/m]
#define BARO_L (0.00976)

// Constant-pressure specific heat [J/(kg·K)]
#define BARO_CP (1004.68506)

// Sea level standard temperature [K]
#define BARO_T0 (288.16)

// Earth-surface gravitational acceleration [m/s2]
#define BARO_G (9.80665)

// Molar mass of dry air [kg/mol]
#define BARO_M (0.02896968)

// Universal gas constant [J/(mol·K)]
#define BARO_R0 (8.314462618)

// convert QNH to QFE
// see https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation
#define BARO_QNH_TO_QFE(p, h) ((p) * pow(1 - ((BARO_L * (h)) / BARO_T0), (BARO_G * BARO_M) / (BARO_R0 * BARO_L)))


#define P_SAMPLE_TIME	10
#define P_SAMPLE_COUNT	(P_SAMPLE_TIME * IIO_SAMPLE_FREQ)

static int exit_fd;

static int p_stat_cnt;
static double p_stat_sum;

static int p_tek_cnt;
static double p_tek_sum;

static int p_dyn_cnt;
static double p_dyn_sum;

static vector3d_t accel;
static vector3d_t anglvel;
static vector3d_t magn;

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

static void proc_p_stat(double data) {
  if (p_stat_cnt > 0) {
    p_stat_sum += data;
    p_stat_cnt--;
  }
}

static void proc_p_tek(double data) {
  if (p_tek_cnt > 0) {
    p_tek_sum += data;
    p_tek_cnt--;
  }
}

static void proc_p_dyn(double data) {
  if (p_dyn_cnt > 0) {
    p_dyn_sum += data;
    p_dyn_cnt--;
  }
}

static void proc_accel(vector3d_t data) {
  accel = data;
}

static void proc_anglvel(vector3d_t data) {
  anglvel = data;
}

static void proc_magn(vector3d_t data) {
  magn = data;
}

static void proc_imu(void) {
  printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
  (int) (accel.x / 0.004788403), (int) (accel.y / 0.004788403), (int) (accel.z / 0.004788403),
  (int) (anglvel.x / 0.000266316), (int) (anglvel.y / 0.000266316), (int) (anglvel.z / 0.000266316),
  (int) (magn.x / 0.001499755), (int) (magn.y / 0.001499755), (int) (magn.z / 0.001499755));
}

const OVIIO_DATA_CALLBACKS_T callbacks = {
  .vbat_cb = NULL,
  .press_stat_cb = proc_p_stat,
  .press_tek_cb = proc_p_tek,
  .press_dyn_cb = proc_p_dyn,
  .accel_cb = proc_accel,
  .anglvel_cb = proc_anglvel,
  .magn_cb = proc_magn,
  .imu_done_cb = proc_imu
};

int main(void) {
  int ret = 1;
  int err;
  struct sigaction act;
  fd_set fds, select_fds;
  uint64_t u;

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

  p_stat_cnt = 0;//P_SAMPLE_COUNT;
  p_stat_sum = 0.0;
  p_tek_cnt = 0;//P_SAMPLE_COUNT;
  p_tek_sum = 0.0;
  p_dyn_cnt = 0;//P_SAMPLE_COUNT;
  p_dyn_sum = 0.0;

  if (oviio_init(&fds, &callbacks, 0)) {
    goto fail_exit_fd;
  }

//  while (p_stat_cnt > 0 || p_tek_cnt > 0 || p_dyn_cnt > 0) {
  while (1) {
    select_fds = fds;

    err = select(FD_SETSIZE, &select_fds, NULL, NULL, NULL);
    if (err < 0) {
      if (errno == EINTR) {
        continue;
      }
      fprintf(stderr, "ERROR: Failed on select.\n");
      goto fail_oviio;
    }

    if (FD_ISSET(exit_fd, &select_fds)) {
      read(exit_fd, &u, sizeof(u));
      break;
    }

    err = oviio_task(&select_fds);
    if (err < 0) {
      fprintf(stderr, "ERROR: Failed to process IIO data.\n");
      goto fail_oviio;
    }
  }

  p_stat_sum /= (double) P_SAMPLE_COUNT;
  p_tek_sum /= (double) P_SAMPLE_COUNT;
  p_dyn_sum /= (double) P_SAMPLE_COUNT;

  printf("stat: %f, tek: %f, dyn: %f\n", p_stat_sum, p_tek_sum, p_dyn_sum);

fail_oviio:
  oviio_cleanup();
fail_exit_fd:
  close(exit_fd);
fail:
  return ret;
}


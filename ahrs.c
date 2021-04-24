#include "ahrs.h"
#include "vector.h"
#include "ovng_iio.h"
#include "nmea_server.h"
#include "eeprom.h"
#include "complementary_filter.h"

#include <math.h>
#include <syslog.h>

#define A_TO_G (1.0 / CF_GRAVITY)
#define SAMPLE_PERIOD (1.0 / (double) IIO_SAMPLE_FREQ)

static int send_raw;
static int use_mag;
static vector3d_t a, w, m;
static double gload;
static CF_DATA_T filter;

void ahrs_init(const AHRS_CONF_T *conf) {
  send_raw = conf->send_raw;
  use_mag = conf->use_mag;

  cfInit(&filter);
  cfSetGainAcc(&filter, conf->flt_gain_acc);
  cfSetGainMag(&filter, conf->flt_gain_mag);
  cfSetBiasAlpha(&filter, conf->flt_bias_alpha);
  filter.do_bias_estimation = conf->flt_do_bias_estim;
  filter.do_adaptive_gain = conf->flt_do_adaptive_gain;

  vector3d_init(&a);
  vector3d_init(&w);
  vector3d_init(&m);

  gload = 0.0;
}

void ahrs_eeprom_init(void) {
  vector3d_float_init(&eeprom_data.payload.ahrs.mag.offset);
  vector3d_float_set(&eeprom_data.payload.ahrs.mag.map[0], 1.0, 0.0, 0.0);
  vector3d_float_set(&eeprom_data.payload.ahrs.mag.map[1], 0.0, 1.0, 0.0);
  vector3d_float_set(&eeprom_data.payload.ahrs.mag.map[2], 0.0, 0.0, 1.0);
}

void ahrs_accel_data(vector3d_t data) {
  // gravity from sensor has wrong sign
  a = vector3d_scale(data, -1.0);

  // set gload
  double mag = vector3d_mag(data);
  gload = A_TO_G * ((data.z > 0.0) ? -mag : mag);
}

void ahrs_anglvel_data(vector3d_t data) {
  w = data;
}

void ahrs_magn_data(vector3d_t data) {
  m = data;
}

void ahrs_scan_done(void) {
  euler_t pos;
  int mag_yaw;

  if (send_raw) {
    nmeasrv_broadcast("$POVIMU,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f",
      a.x, a.y, a.z,
      w.x, w.y, w.z,
      m.x, m.y, m.z);
  }

  m = vector3d_rotate_by_matrix_float(vector3d_sub_float(m, eeprom_data.payload.ahrs.mag.offset), eeprom_data.payload.ahrs.mag.map);

  mag_yaw = use_mag && ((eeprom_data.payload.flags & EE_FLAG_AHRS_IS_CALIBRATED) != 0);
  if (mag_yaw) {
    cfUpdateMag(&filter, a, w, m, SAMPLE_PERIOD);
  } else {
    cfUpdate(&filter, a, w, SAMPLE_PERIOD);
  }

  pos = quaternion_to_euler(cfGetOrientation(&filter));
  nmeasrv_broadcast("$POVAHRS,%0.2f,%0.2f,%0.2f,%d,%0.2f",
    pos.roll * RAD_TO_DEGREE,
    pos.pitch * RAD_TO_DEGREE,
    pos.yaw * RAD_TO_DEGREE,
    mag_yaw, gload);
}

void ahrs_calib_fusion_reset(void) {
  cfReset(&filter);
}

void ahrs_calib_magn(const vector3d_float_t *os, const vector3d_float_t *map) {
  eeprom_data.payload.flags |= EE_FLAG_AHRS_IS_CALIBRATED;
  memcpy(&eeprom_data.payload.ahrs.mag.offset, os, sizeof(vector3d_float_t));
  memcpy(&eeprom_data.payload.ahrs.mag.map, map, sizeof(vector3d_float_t) * 3);

  eeprom_save();
  syslog(LOG_INFO, "AHRS calibration results saved to EEPROM.");
}


#include "ahrs.h"
#include "vector.h"
#include "ovng_iio.h"
#include "nmea_server.h"

#include <math.h>

#include "complementary_filter.h"

#define A_TO_G (1.0 / CF_GRAVITY)
#define SAMPLE_PERIOD (1.0 / (double) IIO_SAMPLE_FREQ)

// https://wiki.paparazziuav.org/wiki/ImuCalibration
// https://github.com/shannon112/imu_calibration
// https%3A%2F%2Fwww.evernote.com%2Fshard%2Fs315%2Fsh%2F988c52e5-8ad9-405e-a416-c2c17d7996ef%2F65b6d82d47a45cdd&title=%255BIMU%255D%2Bcalibration%2BRazor-AHRS%252Frazor-9dof-ahrs

static int send_raw; // send raw data
static vector3d_t a, w, m; // raw input values
static double gload;
static CF_DATA_T filter;

void ahrs_init(const AHRS_CONF_T *conf) {
  send_raw = conf->send_raw;

  vector3d_init(&a);
  vector3d_init(&w);
  vector3d_init(&m);

  cfInit(&filter);

  gload = 0.0;
}

void ahrs_accel_data(vector3d_t data) {
  a = data;

  // TODO: offset/scale calib

  // set gload
  double mag = vector3d_mag(data);
  gload = A_TO_G * ((data.z < 0.0) ? -mag : mag);
}

void ahrs_anglvel_data(vector3d_t data) {
  // TODO: offset calib
  // TODO: why negative??
  w = vector3d_scale(data, -1.0);
}

void ahrs_magn_data(vector3d_t data) {
  m = data;
}

void ahrs_scan_done(void) {
  if (send_raw) {
    nmeasrv_broadcast("$POV,A,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f",
      a.x, a.y, a.z,
      w.x, w.y, w.z,
      m.x, m.y, m.z);
  }

  //cfUpdate(&filter, a, w, SAMPLE_PERIOD);
  cfUpdateMag(&filter, a, w, m, SAMPLE_PERIOD);

  quaternion_t pos = cfGetOrientation(&filter);

  // TODO
  double roll = atan2(pos.q0*pos.q1 + pos.q2*pos.q3, 0.5 - pos.q1*pos.q1 - pos.q2*pos.q2);
  double pitch = asin(-2.0 * (pos.q1*pos.q3 - pos.q0*pos.q2));
  double yaw = atan2(pos.q1*pos.q2 + pos.q0*pos.q3, 0.5 - pos.q2*pos.q2 - pos.q3*pos.q3);

  nmeasrv_broadcast("$POV,b,%0.2f,p,%0.2f,h,%0.2f,g,%0.2f",
    roll * RAD_TO_DEGREE,
    pitch * RAD_TO_DEGREE,
    yaw * RAD_TO_DEGREE,
    gload);
}


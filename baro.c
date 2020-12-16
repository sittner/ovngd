#include "baro.h"
#include "filter.h"
#include "ovng_iio.h"
#include "nmea_server.h"

#include <math.h>

#define VARIO_FACTOR	-2260.389548275485
#define VARIO_EXPONENT	-0.8097374740609689

#define FLT_PERIOD (1.0 / (double) IIO_SAMPLE_FREQ)

static pt1_flt_t p_stat;
static pt1_flt_t p_dyn;

static kalman_flt_t vkf;

static int use_tek;
static double kalman_z_abs;

static inline double computeNoncompVario(double pressure, double d_pressure) {
  return VARIO_FACTOR * pow(pressure, VARIO_EXPONENT) * d_pressure;
}

void baro_init(const OVNGD_CONF_T *conf) {
  pt1_init(&p_stat, conf->baro_stat_filter_tau, FLT_PERIOD);
  pt1_init(&p_dyn, conf->baro_dyn_filter_tau, FLT_PERIOD);
  kalman_init(&vkf, FLT_PERIOD, conf->baro_tek_kalman_x_accel);

  use_tek = conf->baro_use_tek;
  kalman_z_abs = conf->baro_tek_kalman_z_abs;
}

void baro_stat_data(double data) {
  data += BARO_STAT_OFFSET_HPA;
  pt1_update(&p_stat, data);
  nmeasrv_broadcast("$POV,P,%0.2f", p_stat.x);
}

void baro_tek_data(double data) {
  data += BARO_TEK_OFFSET_HPA;
  kalman_update(&vkf, data, kalman_z_abs);
  if (use_tek) {
    nmeasrv_broadcast("$POV,E,%0.2f", computeNoncompVario(vkf.x_abs, vkf.x_vel));
  }
}

void baro_dyn_data(double data) {
  data += BARO_DYN_OFFSET_PA;
  pt1_update(&p_dyn, data);
  // mask speeds < ~10km/h
  nmeasrv_broadcast("$POV,Q,%0.2f", (p_dyn.x < 4.0) ? 0.0 : p_dyn.x);
}


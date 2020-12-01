#include "baro.h"
#include "filter.h"
#include "ovng_iio.h"
#include "nmea_server.h"

#include <math.h>

#define VARIO_FACTOR	-2260.389548275485
#define VARIO_EXPONENT	-0.8097374740609689

#define FLT_PERIOD (1.0 / (double) IIO_SAMPLE_FREQ)

#define FLT_STAT_DT 0.5
#define FLT_DYN_DT 0.5

static pt1_flt_t p_stat;
static pt1_flt_t p_dyn;

static kalman_flt_t vkf;

// TODO: config
static int use_tek = 1;

static inline double computeNoncompVario(double pressure, double d_pressure) {
  return VARIO_FACTOR * pow(pressure, VARIO_EXPONENT) * d_pressure;
}

void baro_init(void) {
  // TODO: config
  pt1_init(&p_stat, 0.5, FLT_PERIOD);
  pt1_init(&p_dyn, 0.5, FLT_PERIOD);
  kalman_init(&vkf, FLT_PERIOD, 0.3);
}

void baro_stat_data(double data) {
  data += BARO_STAT_OFFSET_HPA;
  pt1_update(&p_stat, data);
  nmeasrv_broadcast("$POV,P,%0.2f", p_stat.x);
}

void baro_tek_data(double data) {
  // TODO: config
  data += BARO_TEK_OFFSET_HPA;
  kalman_update(&vkf, data, 0.25);
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


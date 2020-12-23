#include "baro.h"
#include "filter.h"
#include "ovng_iio.h"
#include "nmea_server.h"
#include "eeprom.h"

#include <math.h>
#include <syslog.h>

#define VARIO_FACTOR	-2260.389548275485
#define VARIO_EXPONENT	-0.8097374740609689

#define FLT_PERIOD (1.0 / (double) IIO_SAMPLE_FREQ)

#define CALIB_PERIODS (5 * IIO_SAMPLE_FREQ)

typedef struct {
  int count;
  double accu;
} CALIB_DATA_T;

static pt1_flt_t p_stat;
static pt1_flt_t p_dyn;

static kalman_flt_t vkf;

static int use_tek;
static double kalman_z_abs;

#define CALIB_STAT 0
#define CALIB_TEK 1
#define CALIB_DYN 2
#define CALIB_COUNT 3

#define MAX_OFFSET_BARO 100.0
#define MAX_OFFSET_DIFF 100.0

static int calib_baro_autoref;
static double calib_baro_ref;
static CALIB_DATA_T calib_data[CALIB_COUNT];

static void calib_init(int start, int baro_autoref, double baro_ref);
static void calib_update(CALIB_DATA_T *calib, double data);
static void calib_finish(void);

static double computeNoncompVario(double pressure, double d_pressure);


static void calib_init(int start, int baro_autoref, double baro_ref) {
  int count, i;
  CALIB_DATA_T *calib;

  calib_baro_autoref = baro_autoref;
  calib_baro_ref = baro_ref;
  count = start ? CALIB_PERIODS : 0;
  for (i = 0, calib = calib_data; i < CALIB_COUNT; i++, calib++) {
    calib->count = count;
    calib->accu = 0.0;
  }
}

static void calib_update(CALIB_DATA_T *calib, double data) {
  if (calib->count > 0) {
    (calib->count)--;
    calib->accu += data;
    if (calib->count == 0) {
      calib->accu /= (double) CALIB_PERIODS;
      calib_finish();
    }
  }
}

static void calib_finish(void) {
  int i;
  CALIB_DATA_T *calib;
  double stat_offset, tek_offset, dyn_offset;

  // check if all calibs are finnished
  for (i = 0, calib = calib_data; i < CALIB_COUNT; i++, calib++) {
    if (calib->count > 0) {
      return;
    }
  }

  // calculate autoref reference
  if (calib_baro_autoref) {
    calib_baro_ref = (calib_data[CALIB_STAT].accu + calib_data[CALIB_TEK].accu) / 2.0;
  }

  // calculate offsets
  stat_offset = calib_data[CALIB_STAT].accu - calib_baro_ref;
  tek_offset = calib_data[CALIB_TEK].accu - calib_baro_ref;
  dyn_offset = calib_data[CALIB_DYN].accu;

  syslog(LOG_INFO, "Baro calibration done (calib_baro_ref = %f, stat_offset = %f, tek_offset = %f, dyn_offset = %f).",
    calib_baro_ref, stat_offset, tek_offset, dyn_offset);

  // check result
  if (fabs(stat_offset) > MAX_OFFSET_BARO || fabs(tek_offset) > MAX_OFFSET_BARO || fabs(dyn_offset) > MAX_OFFSET_DIFF) {
    nmeasrv_broadcast("$POV,M,C,B,F,%f,%f,%f,%f", calib_baro_ref, stat_offset, tek_offset, dyn_offset);
    syslog(LOG_ERR, "Baro calibration offset verification failed.");
    return;
  }

  eeprom_data.payload.baro.is_calibrated = 1;
  eeprom_data.payload.baro.stat_offset = stat_offset;
  eeprom_data.payload.baro.tek_offset = tek_offset;
  eeprom_data.payload.baro.dyn_offset = dyn_offset;

  eeprom_save();
  syslog(LOG_INFO, "Baro calibration results saved to EEPROM.");
  nmeasrv_broadcast("$POV,M,C,B,A,%f,%f,%f,%f", calib_baro_ref, stat_offset, tek_offset, dyn_offset);
}

static double computeNoncompVario(double pressure, double d_pressure) {
  return VARIO_FACTOR * pow(pressure, VARIO_EXPONENT) * d_pressure;
}

void baro_init(const BARO_CONF_T *conf) {
  pt1_init(&p_stat, conf->stat_filter_tau, FLT_PERIOD);
  pt1_init(&p_dyn, conf->dyn_filter_tau, FLT_PERIOD);
  kalman_init(&vkf, FLT_PERIOD, conf->tek_kalman_x_accel);

  use_tek = conf->use_tek;
  kalman_z_abs = conf->tek_kalman_z_abs;

  calib_init(0, 0, 0.0);
}

void baro_start_calib(int baro_autoref, double baro_ref) {
  syslog(LOG_INFO, "Starting baro calibration (baro_autoref = %d, baro_ref = %f).",
    baro_autoref, baro_ref);
  nmeasrv_broadcast("$POV,M,C,B,S,%d,%f", baro_autoref, baro_ref);
  calib_init(1, baro_autoref, baro_ref);
}

void baro_eeprom_init(void) {
  eeprom_data.payload.baro.is_calibrated = 0;
  eeprom_data.payload.baro.stat_offset = 0.0;
  eeprom_data.payload.baro.tek_offset = 0.0;
  eeprom_data.payload.baro.dyn_offset = 0.0;
}

void baro_stat_data(double data) {
  calib_update(&calib_data[CALIB_STAT], data);

  // process only if calibrated
  if (!eeprom_data.payload.baro.is_calibrated) {
    return;
  }

  // apply offset
  data -= eeprom_data.payload.baro.stat_offset;

  // apply filter
  pt1_update(&p_stat, data);

  // send NMEA message
  nmeasrv_broadcast("$POV,P,%0.2f", p_stat.x);
}

void baro_tek_data(double data) {
  calib_update(&calib_data[CALIB_TEK], data);

  // process only if calibrated
  if (!eeprom_data.payload.baro.is_calibrated) {
    return;
  }

  // process only if TEK is used
  if (!use_tek) {
    return;
  }

  // apply offset
  data -= eeprom_data.payload.baro.tek_offset;

  // apply filter
  kalman_update(&vkf, data, kalman_z_abs);

  // send NMEA message
  nmeasrv_broadcast("$POV,E,%0.2f", computeNoncompVario(vkf.x_abs, vkf.x_vel));
}

void baro_dyn_data(double data) {
  calib_update(&calib_data[CALIB_DYN], data);

  // process only if calibrated
  if (!eeprom_data.payload.baro.is_calibrated) {
    return;
  }

  // apply offset
  data -= eeprom_data.payload.baro.dyn_offset;

  // apply filter
  pt1_update(&p_dyn, data);
  data = p_dyn.x;

  // mask speeds < ~10km/h
  if (data < 4.0) {
    data = 0.0;
  }

  // send NMEA message
  nmeasrv_broadcast("$POV,Q,%0.2f", data);
}


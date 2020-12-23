#ifndef _BARO_H
#define _BARO_H

#include "cfgfile.h"

typedef struct {
  int is_calibrated;
  double stat_offset;
  double tek_offset;
  double dyn_offset;
} BARO_EEPROM_T;

// config defaults
#define BARO_USE_TEK		1
#define BARO_STAT_FILTER_TAU	0.5
#define BARO_DYN_FILTER_TAU	0.5
#define BARO_TEK_KALMAN_X_ACCEL	0.3
#define BARO_TEK_KALMAN_Z_ABS	0.25

void baro_init(const BARO_CONF_T *conf);
void baro_start_calib(int baro_autoref, double baro_ref);

void baro_eeprom_init(void);

void baro_stat_data(double data);
void baro_tek_data(double data);
void baro_dyn_data(double data);

#endif

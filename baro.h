#ifndef _BARO_H
#define _BARO_H

#include "cfgfile.h"

// TODO: move to EEPROM
#define BARO_STAT_OFFSET_HPA	28.26
#define BARO_TEK_OFFSET_HPA	10.91
#define BARO_DYN_OFFSET_PA	0.0

// config defaults
#define BARO_USE_TEK		1
#define BARO_STAT_FILTER_TAU	0.5
#define BARO_DYN_FILTER_TAU	0.5
#define BARO_TEK_KALMAN_X_ACCEL	0.3
#define BARO_TEK_KALMAN_Z_ABS	0.25

void baro_init(const OVNGD_CONF_T *conf);

void baro_stat_data(double data);
void baro_tek_data(double data);
void baro_dyn_data(double data);

#endif

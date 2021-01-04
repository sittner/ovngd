#ifndef _AHRS_H_
#define _AHRS_H_

#include "cfgfile.h"
#include "vector.h"

typedef struct {
  int is_calibrated;
  vector3d_t offset;
  vector3d_t scale;
} AHRS_SCALE_EEPROM_T;

typedef struct {
  AHRS_SCALE_EEPROM_T mag;
} AHRS_EEPROM_T;

void ahrs_init(const AHRS_CONF_T *conf);

void ahrs_eeprom_init(void);

void ahrs_accel_data(vector3d_t data);
void ahrs_anglvel_data(vector3d_t data);
void ahrs_magn_data(vector3d_t data);

void ahrs_scan_done(void);

#endif

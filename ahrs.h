#ifndef _AHRS_H_
#define _AHRS_H_

#include "cfgfile.h"
#include "vector.h"

#define MADG_BETA_DEFLT 0.1

void ahrs_init(const AHRS_CONF_T *conf);

void ahrs_accel_data(vector3d_t data);
void ahrs_anglvel_data(vector3d_t data);
void ahrs_magn_data(vector3d_t data);

void ahrs_scan_done(void);

#endif

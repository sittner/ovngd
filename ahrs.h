#ifndef _AHRS_H_
#define _AHRS_H_

#include "vector.h"

#define MADG_BETA_DEFLT 0.1

void ahrs_init(double _beta, int _send_raw);

void ahrs_accel_data(vector3d_t data);
void ahrs_anglvel_data(vector3d_t data);
void ahrs_magn_data(vector3d_t data);

void ahrs_scan_done(void);

#endif

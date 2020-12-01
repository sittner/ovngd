#ifndef _AHRS_H_
#define _AHRS_H_

#include "vector.h"

void ahrs_init(void);

void ahrs_accel_data(vector3d_t data);
void ahrs_anglvel_data(vector3d_t data);
void ahrs_magn_data(vector3d_t data);

void ahrs_scan_done(void);

#endif

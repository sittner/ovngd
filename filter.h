#ifndef _FILTER_H
#define _FILTER_H

typedef struct {
  double k;
  double x;
} pt1_flt_t;

void pt1_init(pt1_flt_t *flt, double tau, double dt);
void pt1_update(pt1_flt_t *flt, double z);

typedef struct {
  double dt, dt2, dt3, dt4;

  double x_abs;		// the absolute quantity x
  double x_vel;		// the rate of change of x, in x units per second squared.

  // Covariance matrix for the state
  double p_abs_abs;
  double p_abs_vel;
  double p_vel_vel;

  // The variance of the acceleration noise input to the system model
  double var_x_accel;
} kalman_flt_t;

void kalman_init(kalman_flt_t *flt, double dt, double var_x_accel);
void kalman_reset(kalman_flt_t *flt);
void kalman_update(kalman_flt_t *flt, double z_abs, double var_z_abs);

#endif

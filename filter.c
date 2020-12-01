#include "filter.h"

#include <math.h>

void pt1_init(pt1_flt_t *flt, double tau, double dt) {
  flt->k = dt / tau;
  flt->x = 0.0;
}

void pt1_update(pt1_flt_t *flt, double z) {
  flt->x += (z - flt->x) * flt->k;
}

void kalman_init(kalman_flt_t *flt, double dt, double var_x_accel) {
  // precalc dt constants
  flt->dt = dt;
  flt->dt2 = dt * flt->dt;
  flt->dt3 = dt * flt->dt2;
  flt->dt4 = dt * flt->dt3;

  flt->var_x_accel = var_x_accel;

  // reset filter
  kalman_reset(flt);
}

void kalman_reset(kalman_flt_t *flt) {
  flt->x_abs = 0.0;
  flt->x_vel = 0.0;

  flt->p_abs_abs = 0.0;
  flt->p_abs_vel = 0.0;
  flt->p_vel_vel = 0.0;
}

void kalman_update(kalman_flt_t *flt, double z_abs, double var_z_abs) {
  double y;
  double s_inv;
  double k_abs;
  double k_vel;
  
  // check if dt is positive
  if (flt->dt <= 0.0) {
    return;
  }
  
  // Predict step
  // update state estimate
  flt->x_abs += flt->x_vel * flt->dt;
  
  flt->p_abs_abs += 2.0 * (flt->dt * flt->p_abs_vel) + flt->dt2 * flt->p_vel_vel + 0.25 * (flt->var_x_accel * flt->dt4);
  flt->p_abs_vel += flt->dt * flt->p_vel_vel + (flt->var_x_accel * flt->dt3) / 2.0;
  flt->p_vel_vel += flt->var_x_accel * flt->dt2;
  
  // Update step
  y = z_abs - flt->x_abs; // Innovation
  s_inv = 1.0 / (flt->p_abs_abs + var_z_abs); // Innovation precision 
  k_abs = flt->p_abs_abs * s_inv; // Kalman gain
  k_vel = flt->p_abs_vel * s_inv;
  
  // Update state estimate
  flt->x_abs += k_abs * y;
  flt->x_vel += k_vel * y;
  
  // Update state covariance.
  flt->p_vel_vel -= flt->p_abs_vel * k_vel;
  flt->p_abs_vel -= flt->p_abs_vel * k_abs;
  flt->p_abs_abs -= flt->p_abs_abs * k_abs;
}


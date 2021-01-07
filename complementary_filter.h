#ifndef _COMPLEMENTARY_FILTER_H_
#define _COMPLEMENTARY_FILTER_H_

#include <stdbool.h>
#include "vector.h"

#define CF_GRAVITY (9.81)


typedef struct {
  // Gain parameter for the complementary filter, belongs in [0, 1].
  double gain_acc;
  double gain_mag;

  // Bias estimation gain parameter, belongs in [0, 1].
  double bias_alpha;

  // Parameter whether to do bias estimation or not.
  bool do_bias_estimation;

  // Parameter whether to do adaptive gain or not.
  bool do_adaptive_gain;

  bool initialized;

  // When the filter is in the steady state, bias estimation will occur (if the
  // parameter is enabled).
  bool steady_state;

  // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
  // the orientation of the fixed frame wrt the body frame.
  quaternion_t pos;

  // Bias in angular velocities
  vector3d_t w_prev;

  // Bias in angular velocities;
  vector3d_t w_bias;
} CF_DATA_T;

void cfInit(CF_DATA_T *cf);
void cfReset(CF_DATA_T *cf);

bool cfSetGainAcc(CF_DATA_T *cf, double gain);
bool cfSetGainMag(CF_DATA_T *cf, double gain);
bool cfSetBiasAlpha(CF_DATA_T *cf, double bias_alpha);

// Set the orientation, as a Hamilton Quaternion, of the body frame wrt the
// fixed frame.
void cfSetOrientation(CF_DATA_T *cf, quaternion_t q);

// Get the orientation, as a Hamilton Quaternion, of the body frame wrt the
// fixed frame.
quaternion_t cfGetOrientation(CF_DATA_T *cf);

// Update from accelerometer and gyroscope data.
// [ax, ay, az]: Normalized gravity vector.
// [wx, wy, wz]: Angular veloctiy, in rad / s.
// dt: time delta, in seconds.
void cfUpdate(CF_DATA_T *cf, vector3d_t a, vector3d_t w, double dt);

// Update from accelerometer, gyroscope, and magnetometer data.
// [ax, ay, az]: Normalized gravity vector.
// [wx, wy, wz]: Angular veloctiy, in rad / s.
// [mx, my, mz]: Magnetic field, units irrelevant.
// dt: time delta, in seconds.
void cfUpdateMag(CF_DATA_T *cf, vector3d_t a, vector3d_t w, vector3d_t m, double dt);

#endif

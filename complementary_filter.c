/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.q2om>

  @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.q2cny.q2uny.edu>
        All rights reserved.

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
        ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
        WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
        DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
        DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
        (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
        LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
        ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
        SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "complementary_filter.h"

#include <math.h>

// Bias estimation steady state thresholds
static const double kAngularVelocityThreshold = 0.2;
static const double kAccelerationThreshold = 0.1;
static const double kDeltaAngularVelocityThreshold = 0.01;

static quaternion_t scaleQuaternion(double gain, quaternion_t q)
{
  if (q.q0 < 0.9) {
    // Slerp (Spherical linear interpolation):
    double angle = acos(q.q0);
    double A = sin(angle * (1.0 - gain)) / sin(angle);
    double B = sin(angle * gain) / sin(angle);
    q.q0 = A + B * q.q0;
    q.q1 = B * q.q1;
    q.q2 = B * q.q2;
    q.q3 = B * q.q3;
  } else {
    // Lerp (Linear interpolation):
    q.q0 = (1.0 - gain) + gain * q.q0;
    q.q1 = gain * q.q1;
    q.q2 = gain * q.q2;
    q.q3 = gain * q.q3;
  }

  return quaternion_normalize(q);
}

static bool checkState(CF_DATA_T *cf, vector3d_t a, vector3d_t w)
{
  double acc_magnitude = vector3d_mag(a);
  if (fabs(acc_magnitude - CF_GRAVITY) > kAccelerationThreshold) {
    return false;
  }

  acc_magnitude = vector3d_mag(vector3d_sub(w, cf->w_prev));
  if (acc_magnitude > kDeltaAngularVelocityThreshold) {
    return false;
  }

  acc_magnitude = vector3d_mag(vector3d_sub(w, cf->w_bias));
  if (acc_magnitude > kAngularVelocityThreshold) {
    return false;
  }

  return true;
}

static void updateBiases(CF_DATA_T *cf, vector3d_t a, vector3d_t w)
{
  cf->steady_state = checkState(cf, a, w);

  if (cf->steady_state) {
    cf->w_bias = vector3d_add(cf->w_bias, vector3d_scale(vector3d_sub(w, cf->w_bias), cf->bias_alpha));
  }

  cf->w_prev = w;
}

quaternion_t getPrediction(CF_DATA_T *cf, vector3d_t w, double dt)
{
  vector3d_t w_unb;
  w_unb = vector3d_sub(w, cf->w_bias);

  quaternion_t pred;
  pred.q0 = cf->pos.q0 + 0.5 * dt * ( w_unb.x * cf->pos.q1 + w_unb.y * cf->pos.q2 + w_unb.z * cf->pos.q3);
  pred.q1 = cf->pos.q1 + 0.5 * dt * (-w_unb.x * cf->pos.q0 - w_unb.y * cf->pos.q3 + w_unb.z * cf->pos.q2);
  pred.q2 = cf->pos.q2 + 0.5 * dt * ( w_unb.x * cf->pos.q3 - w_unb.y * cf->pos.q0 - w_unb.z * cf->pos.q1);
  pred.q3 = cf->pos.q3 + 0.5 * dt * (-w_unb.x * cf->pos.q2 + w_unb.y * cf->pos.q1 - w_unb.z * cf->pos.q0);

  return quaternion_normalize(pred);
}

static quaternion_t getMeasurement(vector3d_t a)
{
  // q_acc is the quaternion obtained from the acceleration vector representing
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
  // this is basically an optimized version of quaternion_from_vectors
  // for the special case of u=q and v={0.0,0.0,1.0}
  quaternion_t meas;
  meas.q0 = vector3d_mag(a) + a.z;
  meas.q1 = -a.y;
  meas.q2 = a.x;
  meas.q3 = 0.0;
  return meas;
}

static quaternion_t getAccCorrection(vector3d_t a, quaternion_t p)
{
  // Normalize acceleration vector.
  a = vector3d_normalize(a);

  // Acceleration reading rotated into the world frame by the inverse predicted
  // quaternion (predicted gravity):
  vector3d_t g;
  g = vector3d_rotate_by_quaternion(a, quaternion_invert(p));

  // Delta quaternion that rotates the predicted gravity into the real gravity:
  return getMeasurement(g);
}

static quaternion_t getMagCorrection(vector3d_t m, quaternion_t p)
{
  // Magnetic reading rotated into the world frame by the inverse predicted
  // quaternion:
  vector3d_t l;
  l = vector3d_rotate_by_quaternion(m, quaternion_invert(p));

  // Delta quaternion that rotates the l so that it lies in the xz-plane
  // (points north):
  double gamma = l.x*l.x + l.y*l.y;
  double beta = sqrt(gamma + l.x*sqrt(gamma));
  quaternion_t dq;
  dq.q0 = beta / (sqrt(2.0 * gamma));
  dq.q1 = 0.0;
  dq.q2 = 0.0;
  dq.q3 = l.y / (SQRT_2 * beta);
  return dq;
}

static double getAdaptiveGain(double alpha, vector3d_t a)
{
  double a_mag = vector3d_mag(a);
  double error = fabs(a_mag - CF_GRAVITY) / CF_GRAVITY;
  double factor;
  double error1 = 0.1;
  double error2 = 0.2;
  double m = 1.0 / (error1 - error2);
  double b = 1.0 - m * error1;

  if (error < error1) {
    factor = 1.0;
  } else if (error < error2) {
    factor = m * error + b;
  } else {
    factor = 0.0;
  }

  return factor * alpha;
}

static quaternion_t getMeasurementMag(vector3d_t a, vector3d_t m)
{
  // q_acc is the quaternion obtained from the acceleration vector representing
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
  quaternion_t acc = getMeasurement(a);

  // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
  // frame by the inverse of q_acc.
  // l = R(q_acc)^-1 m
  double lx = (acc.q0 * acc.q0 + acc.q1 * acc.q1 - acc.q2 * acc.q2) * m.x
      + 2.0 * (acc.q1 * acc.q2) * m.y
      - 2.0 * (acc.q0 * acc.q2) * m.z;
  double ly = 2.0 * (acc.q1 * acc.q2) * m.x
      + (acc.q0 * acc.q0 - acc.q1 * acc.q1 + acc.q2*acc.q2) * m.y
      + 2.0 * (acc.q0*acc.q1) * m.z;

  // q_mag is the quaternion that rotates the Global frame (North West Up) into
  // the intermediary frame. q1_mag and q2_mag are defined as 0.
  double gamma = lx * lx + ly * ly;
  double beta = sqrt(gamma + lx * sqrt(gamma));
  quaternion_t mag;
  mag.q0 = beta / (sqrt(2.0 * gamma));
  mag.q1 = 0.0;
  mag.q2 = 0.0;
  mag.q3 = ly / (SQRT_2 * beta);

  // The quaternion multiplication between q_acc and q_mag represents the
  // quaternion, orientation of the Global frame wrt the local frame.
  // q = q_acc times q_mag
  return quaternion_multiply(acc, mag);
  //q0_meas = q0_acc*q0_mag;
  //q1_meas = q1_acc*q0_mag + q2_acc*q3_mag;
  //q2_meas = q2_acc*q0_mag - q1_acc*q3_mag;
  //q3_meas = q0_acc*q3_mag;
}

void cfInit(CF_DATA_T *cf) {
  cf->gain_acc = 0.01;
  cf->gain_mag = 0.01;
  cf->bias_alpha = 0.01;
  cf->do_bias_estimation = true;
  cf->do_adaptive_gain = true;
  cf->initialized = false;
  cf->steady_state = false;
  quaternion_init(&cf->pos);
  vector3d_init(&cf->w_prev);
  vector3d_init(&cf->w_bias);
}

bool cfSetGainAcc(CF_DATA_T *cf, double gain)
{
  if (gain >= 0.0 && gain <= 1.0) {
    cf->gain_acc = gain;
    return true;
  }

  return false;
}
bool cfSetGainMag(CF_DATA_T *cf, double gain)
{
  if (gain >= 0.0 && gain <= 1.0) {
    cf->gain_mag = gain;
    return true;
  }

  return false;
}

bool cfSetBiasAlpha(CF_DATA_T *cf, double bias_alpha)
{
  if (bias_alpha >= 0.0 && bias_alpha <= 1.0) {
    cf->bias_alpha = bias_alpha;
    return true;
  }

  return false;
}

void cfSetOrientation(CF_DATA_T *cf, quaternion_t q)
{
  // Set the state to inverse (state is fixed wrt body).
  cf->pos = quaternion_invert(q);
}

quaternion_t cfGetOrientation(CF_DATA_T *cf)
{
  // Return the inverse of the state (state is fixed wrt body).
  return quaternion_invert(cf->pos);
}

void cfUpdate(CF_DATA_T *cf, vector3d_t a, vector3d_t w, double dt)
{
  if (cf->initialized) {
    // First time - ignore prediction:
    cf->pos = getMeasurement(a);
    cf->initialized = true;
    return;
  }

  // Bias estimation.
  if (cf->do_bias_estimation) {
    updateBiases(cf, a, w);
  }

  // Prediction.
  quaternion_t pred;
  pred = getPrediction(cf, w, dt);

  // Correction (from acc):
  // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  quaternion_t acc;
  acc = getAccCorrection(a, pred);

  double gain = cf->gain_acc;
  if (cf->do_adaptive_gain) {
    gain = getAdaptiveGain(cf->gain_acc, a);
  }

  acc = scaleQuaternion(gain, acc);

  cf->pos = quaternion_normalize(quaternion_multiply(pred, acc));
}

void cfUpdateMag(CF_DATA_T *cf, vector3d_t a, vector3d_t w, vector3d_t m, double dt)
{
  if (cf->initialized) {
    // First time - ignore prediction:
    cf->pos = getMeasurementMag(a, m);
    cf->initialized = true;
    return;
  }

  // Bias estimation.
  if (cf->do_bias_estimation) {
    updateBiases(cf, a, w);
  }

  // Prediction.
  quaternion_t pred;
  pred = getPrediction(cf, w, dt);

  // Correction (from acc):
  // q_temp = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  quaternion_t acc;
  acc = getAccCorrection(a, pred);
  double alpha = cf->gain_acc;
  if (cf->do_adaptive_gain) {
     alpha = getAdaptiveGain(cf->gain_acc, a);
  }
  acc = scaleQuaternion(alpha, acc);

  quaternion_t temp;
  temp = quaternion_multiply(pred, acc);

  // Correction (from mag):
  // q_ = q_temp * [(1-gain) * qI + gain * dq_mag]
  // where qI = identity quaternion
  quaternion_t mag;
  mag = scaleQuaternion(cf->gain_mag, getMagCorrection(m, temp));

  cf->pos = quaternion_normalize(quaternion_multiply(temp, mag));
}


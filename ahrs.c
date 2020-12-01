//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

#include "ahrs.h"
#include "vector.h"
#include "ovng_iio.h"
#include "nmea_server.h"

#include <math.h>

#define A_TO_G (1.0 / 9.80665)
#define SAMPLE_PERIOD (1.0 / (double) IIO_SAMPLE_FREQ)

#define MADG_BETA_DEFLT 0.1

static double beta; // algorithm gain

static vector3d_t g, a, m; // input values

static double q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

static double gload;

static void madgwick_update_imu(void) {
  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-q1 * g.x - q2 * g.y - q3 * g.z);
  qDot2 = 0.5 * (q0 * g.x + q2 * g.z - q3 * g.y);
  qDot3 = 0.5 * (q0 * g.y - q1 * g.z + q3 * g.x);
  qDot4 = 0.5 * (q0 * g.z + q1 * g.y - q2 * g.x);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((a.x == 0.0) && (a.y == 0.0) && (a.z == 0.0))) {
    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0 * q0;
    _2q1 = 2.0 * q1;
    _2q2 = 2.0 * q2;
    _2q3 = 2.0 * q3;
    _4q0 = 4.0 * q0;
    _4q1 = 4.0 * q1;
    _4q2 = 4.0 * q2;
    _8q1 = 8.0 * q1;
    _8q2 = 8.0 * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
    s1 = _4q1 * q3q3 - _2q3 * a.x + 4.0 * q0q0 * q1 - _2q0 * a.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z;
    s2 = 4.0 * q0q0 * q2 + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z;
    s3 = 4.0 * q1q1 * q3 - _2q1 * a.x + 4.0 * q2q2 * q3 - _2q2 * a.y;
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * SAMPLE_PERIOD;
  q1 += qDot2 * SAMPLE_PERIOD;
  q2 += qDot3 * SAMPLE_PERIOD;
  q3 += qDot4 * SAMPLE_PERIOD;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 * recipNorm;
  q1 = q1 * recipNorm;
  q2 = q2 * recipNorm;
  q3 = q3 * recipNorm;
}

static void madgwick_update(void) {
  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double hx, hy;
  double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((m.x == 0.0) && (m.y == 0.0) && (m.z == 0.0)) {
    madgwick_update_imu();
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-q1 * g.x - q2 * g.y - q3 * g.z);
  qDot2 = 0.5 * (q0 * g.x + q2 * g.z - q3 * g.y);
  qDot3 = 0.5 * (q0 * g.y - q1 * g.z + q3 * g.x);
  qDot4 = 0.5 * (q0 * g.z + q1 * g.y - q2 * g.x);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((a.x == 0.0) && (a.y == 0.0) && (a.z == 0.0))) {
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0 * q0 * m.x;
    _2q0my = 2.0 * q0 * m.y;
    _2q0mz = 2.0 * q0 * m.z;
    _2q1mx = 2.0 * q1 * m.x;
    _2q0 = 2.0 * q0;
    _2q1 = 2.0 * q1;
    _2q2 = 2.0 * q2;
    _2q3 = 2.0 * q3;
    _2q0q2 = 2.0 * q0 * q2;
    _2q2q3 = 2.0 * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = m.x * q0q0 - _2q0my * q3 + _2q0mz * q2 + m.x * q1q1 + _2q1 * m.y * q2 + _2q1 * m.z * q3 - m.x * q2q2 - m.x * q3q3;
    hy = _2q0mx * q3 + m.y * q0q0 - _2q0mz * q1 + _2q1mx * q2 - m.y * q1q1 + m.y * q2q2 + _2q2 * m.z * q3 - m.y * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + m.z * q0q0 + _2q1mx * q3 - m.z * q1q1 + _2q2 * m.y * q3 - m.z * q2q2 + m.z * q3q3;
    _4bx = 2.0 * _2bx;
    _4bz = 2.0 * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q1 * (2.0 * q0q1 + _2q2q3 - a.y) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
    s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q0 * (2.0 * q0q1 + _2q2q3 - a.y) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - a.z) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
    s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q3 * (2.0 * q0q1 + _2q2q3 - a.y) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - a.z) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
    s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - a.x) + _2q2 * (2.0 * q0q1 + _2q2q3 - a.y) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m.z);
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * SAMPLE_PERIOD;
  q1 += qDot2 * SAMPLE_PERIOD;
  q2 += qDot3 * SAMPLE_PERIOD;
  q3 += qDot4 * SAMPLE_PERIOD;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 * recipNorm;
  q1 = q1 * recipNorm;
  q2 = q2 * recipNorm;
  q3 = q3 * recipNorm;
}

static inline double get_roll(void) {
  return atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
}

static inline double get_pitch(void) {
  return asin(-2.0 * (q1*q3 - q0*q2));
}

static inline double get_yaw(void) {
  return atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
}

void ahrs_init(void) {
  beta = MADG_BETA_DEFLT; // TODO: config

  vector3d_init(&g);
  vector3d_init(&a);
  vector3d_init(&m);

  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;

  gload = 0.0;
}

void ahrs_accel_data(vector3d_t data) {
  // noarmalize vector
  double mag = vector3d_mag(data);
  if (mag > 0.0) {
    a = vector3d_scale(data, 1.0 / mag);
  } else {
    vector3d_init(&a);
  }

  // set gload
  gload = A_TO_G * ((data.z < 0.0) ? -mag : mag);
}

void ahrs_anglvel_data(vector3d_t data) {
  // TODO: why negative??
  g = vector3d_scale(data, -1.0);
}

void ahrs_magn_data(vector3d_t data) {
  data.x -= 0.853;  // 0 deg
  data.y -= -1.223; // -90 deg
  data.z -= -0.121; // ???

  // noarmalize vector
  double mag = vector3d_mag(data);
  if (mag > 0.0) {
    m = vector3d_scale(data, 1.0 / mag);
  } else {
    vector3d_init(&m);
  }

/*
printf("(%+06.3f %+06.3f %+06.3f) -> %.3f\n",
  m.x, m.y, m.z,
atan2(tmp.x, tmp.y) * RAD_TO_DEGREE);
*/

}

void ahrs_scan_done(void) {
  madgwick_update();
/*
printf("(%+06.3f %+06.3f %+06.3f) (%+06.1f %+06.1f %+06.1f) (%+06.3f %+06.3f %+06.3f) -> %+06.1f %+06.1f %+06.1f %+06.1f\n",
  a.x, a.y, a.z,
  g.x * RAD_TO_DEGREE, g.y * RAD_TO_DEGREE, g.z * RAD_TO_DEGREE,
  m.x, m.y, m.z,
atan2(m.y, m.x) * RAD_TO_DEGREE,
    get_roll() * RAD_TO_DEGREE,
    get_pitch() * RAD_TO_DEGREE,
    get_yaw() * RAD_TO_DEGREE
);
*/
  nmeasrv_broadcast("$POV,b,%0.2f,p,%0.2f,h,%0.2f,g,%0.2f",
    get_roll() * RAD_TO_DEGREE,
    get_pitch() * RAD_TO_DEGREE,
    get_yaw() * RAD_TO_DEGREE,
    gload);
}


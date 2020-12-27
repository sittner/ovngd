#ifndef _VECTOR_3D_H_
#define _VECTOR_3D_H_

#include <math.h>

#define DEGREE_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEGREE (180.0 / M_PI)

#define SQRT_2 (1.4142135623730950488)

typedef struct {
  double x;
  double y;
  double z;
} vector3d_t;

typedef struct {
  double q0;
  double q1;
  double q2;
  double q3;
} quaternion_t;

static inline void vector3d_init(vector3d_t *d)
{
  d->x = 0.0;
  d->y = 0.0;
  d->z = 0.0;
}

static inline double vector3d_dot_product(vector3d_t a, vector3d_t b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline double vector3d_mag(vector3d_t v)
{
  return sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

static inline vector3d_t vector3d_scale(vector3d_t v, double s)
{
  v.x *= s;
  v.y *= s;
  v.z *= s;
  return v;
}

static inline vector3d_t vector3d_normalize(vector3d_t v)
{
  return vector3d_scale(v, 1.0 / vector3d_mag(v));
}

static inline vector3d_t vector3d_add(vector3d_t a, vector3d_t b)
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  return a;
}

static inline vector3d_t vector3d_sub(vector3d_t a, vector3d_t b)
{
  a.x -= b.x;
  a.y -= b.y;
  a.z -= b.z;
  return a;
}

static inline void quaternion_init(quaternion_t *q)
{
  q->q0 = 1.0;
  q->q1 = 0.0;
  q->q2 = 0.0;
  q->q3 = 0.0;
}

static inline quaternion_t quaternion_normalize(quaternion_t q)
{
  double norm = 1.0 / sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
  q.q0 *= norm;
  q.q1 *= norm;
  q.q2 *= norm;
  q.q3 *= norm;
  return q;
}

static inline quaternion_t quaternion_invert(quaternion_t q)
{
  // Assumes quaternion is normalized.
  q.q1 = -q.q1;
  q.q2 = -q.q2;
  q.q3 = -q.q3;
  return q;
}

static inline quaternion_t quaternion_multiply(quaternion_t p, quaternion_t q)
{
  // r = p q
  quaternion_t r;
  r.q0 = p.q0 * q.q0 - p.q1 * q.q1 - p.q2 * q.q2 - p.q3 * q.q3;
  r.q1 = p.q0 * q.q1 + p.q1 * q.q0 + p.q2 * q.q3 - p.q3 * q.q2;
  r.q2 = p.q0 * q.q2 - p.q1 * q.q3 + p.q2 * q.q0 + p.q3 * q.q1;
  r.q3 = p.q0 * q.q3 + p.q1 * q.q2 - p.q2 * q.q1 + p.q3 * q.q0;
  return r;
}

static inline vector3d_t vector3d_rotate_by_quaternion(vector3d_t v, quaternion_t q)
{
  vector3d_t r;
  r.x = (q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3) * v.x + 2.0 * (q.q1 * q.q2 - q.q0 * q.q3) * v.y + 2.0 * (q.q1 * q.q3 + q.q0 * q.q2) * v.z;
  r.y = 2.0 * (q.q1 * q.q2 + q.q0 * q.q3) * v.x + (q.q0 * q.q0 - q.q1 * q.q1 + q.q2 * q.q2 - q.q3 * q.q3) * v.y + 2.0 * (q.q2 * q.q3 - q.q0 * q.q1) * v.z;
  r.z = 2.0 * (q.q1 * q.q3 - q.q0 * q.q2) * v.x + 2.0 * (q.q2 * q.q3 + q.q0 * q.q1) * v.y + (q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3) * v.z;
  return r;
}

#endif


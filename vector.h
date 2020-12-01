////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _VECTOR_3D_H_
#define _VECTOR_3D_H_

#include <math.h>

#define	DEGREE_TO_RAD		(M_PI / 180.0)
#define	RAD_TO_DEGREE		(180.0 / M_PI)

#define TWO_PI			(2.0 * M_PI)

typedef struct {
  double x;
  double y;
  double z;
} vector3d_t;

typedef struct {
  double w;
  double x;
  double y;
  double z;
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

#endif


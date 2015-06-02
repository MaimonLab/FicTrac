#ifndef _CAMERA_MODEL_PRIVATE_H
#define _CAMERA_MODEL_PRIVATE_H 1


#include "CameraModel.h"
#include "CmPoint.h"
#include <opencv2/opencv.hpp>

///-----------------------------------------------------------------------------
/// Some math...
///-----------------------------------------------------------------------------

#include <math.h>

static inline CmReal vec3dot(const CmReal a[3], const CmReal b[3])
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline void vec3cross(const CmReal a[3], const CmReal b[3], CmReal v[3])
{
	v[0] = a[1]*b[2] - a[2]*b[1];
	v[1] = a[2]*b[0] - a[0]*b[2];
	v[2] = a[0]*b[1] - a[1]*b[0];
}

// returns magnitude of resulting vector
static CmReal vec3crossMag(const CmReal a[3], const CmReal b[3])
{
	CmReal v[3];
	vec3cross(a, b, v);
	return sqrt(vec3dot(v,v));
}

static inline void vec3scale(CmReal a[3], CmReal s)
{
	a[0] *= s;
	a[1] *= s;
	a[2] *= s;
}

static inline void vec3scale(const CmReal a[3], CmReal b[3], CmReal s)
{
	b[0] = a[0] * s;
	b[1] = a[1] * s;
	b[2] = a[2] * s;
}

static CmReal vec3normalise(CmReal a[3])
{
	CmReal mag = sqrt(vec3dot(a,a));
	if (mag != 0)
		vec3scale(a, 1.0/mag);
	return mag;
}

static inline CmReal clamp(CmReal x, CmReal min, CmReal max)
{
	if (x <= min)
		return min;
	if (x >= max)
		return max;
	return x;
}


#endif // _CAMERA_MODEL_PRIVATE_H


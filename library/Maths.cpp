#include "Maths.h"

#include <cstring>
#include <cmath>

using namespace rjdm;

///
/// Constants.
///

const double Maths::PI		= 3.1415926535898;
const double Maths::R2D	= 180.0/PI;
const double Maths::D2R	= PI/180.0;
const double Maths::G		= 9.80665;	// m/s^2

///
/// Utility functions.
///

double Maths::RND(long double num, int dig)
{
	double tmp = pow(10.0,dig);
	return double(round(num*tmp)/tmp);
}

///
/// Vector functions.
///

void Maths::MUL_VEC(double a[3], const double b[3])
{
	a[0] *= b[0];
	a[1] *= b[1];
	a[2] *= b[2];
}

float Maths::NORMALISE_VEC(float a[3])
{
	float mag = sqrt(DOT_VEC(a,a));
	if (mag != 0)
		SCALE_VEC(a, 1.0/mag);
	return mag;
}

double Maths::NORMALISE_VEC(double a[3])
{
	double mag = sqrt(DOT_VEC(a,a));
	if (mag != 0)
		SCALE_VEC(a, 1.0/mag);
	return mag;
}

void Maths::MAT_T(double mat[9])
{
	double tmp3 = 0, tmp6 = 0, tmp7 = 0;
	tmp3 = mat[3]; tmp6 = mat[6]; tmp7 = mat[7];
	mat[3] = mat[1]; mat[6] = mat[2]; mat[7] = mat[5];
	mat[1] = tmp3; mat[2] = tmp6; mat[5] = tmp7;
}

void Maths::MUL_MAT(const double lmat[9], const double rmat[9], double res[9])
{
#if 0
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			// element i==r,j==c in result matrix
			double sum = 0;
			for (int k = 0; k < 3; ++k) {
				sum += lmat[i*3+k]*rmat[k*3+j];
			}
			res[i*3+j] = sum;
		}
	}
#else
	res[0] = lmat[0]*rmat[0] + lmat[1]*rmat[3] + lmat[2]*rmat[6];
	res[1] = lmat[0]*rmat[1] + lmat[1]*rmat[4] + lmat[2]*rmat[7];
	res[2] = lmat[0]*rmat[2] + lmat[1]*rmat[5] + lmat[2]*rmat[8];

	res[3] = lmat[3]*rmat[0] + lmat[4]*rmat[3] + lmat[5]*rmat[6];
	res[4] = lmat[3]*rmat[1] + lmat[4]*rmat[4] + lmat[5]*rmat[7];
	res[5] = lmat[3]*rmat[2] + lmat[4]*rmat[5] + lmat[5]*rmat[8];

	res[6] = lmat[6]*rmat[0] + lmat[7]*rmat[3] + lmat[8]*rmat[6];
	res[7] = lmat[6]*rmat[1] + lmat[7]*rmat[4] + lmat[8]*rmat[7];
	res[8] = lmat[6]*rmat[2] + lmat[7]*rmat[5] + lmat[8]*rmat[8];
#endif
}

void Maths::ANGLE_UNIT_AXIS_TO_MAT(const double angle, const double axis[3],
		double m[9])
{
#if 0
	double x=axis[0], y=axis[1], z=axis[2];
	double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;
    m[0] = c + x*x*t;
    m[4] = c + y*y*t;
    m[8] = c + z*z*t;
    double tmp1 = x*y*t;
    double tmp2 = z*s;
    m[3] = tmp1 + tmp2;
    m[1] = tmp1 - tmp2;
    tmp1 = x*z*t;
    tmp2 = y*s;
    m[6] = tmp1 - tmp2;
    m[2] = tmp1 + tmp2;
    tmp1 = y*z*t;
    tmp2 = x*s;
    m[7] = tmp1 + tmp2;
    m[5] = tmp1 - tmp2;
#else
	double x=axis[0], y=axis[1], z=axis[2];
	double c = cos(angle);
	double d = 1.0 - c;
	double dx = d*x;
	double dy = d*y;
	double dz = d*z;
	double dxy = dx*y;
	double dxz = dx*z;
	double dyz = dy*z;
	double s = sin(angle);
	double sx = s*x;
	double sy = s*y;
	double sz = s*z;
	m[0] = c + dx*x;
	m[1] = dxy - sz;
	m[2] = dxz + sy;
	m[3] = dxy + sz;
	m[4] = c + dy*y;
	m[5] = dyz - sx;
	m[6] = dxz - sy;
	m[7] = dyz + sx;
	m[8] = c + dz*z;
#endif
}

/// aka rodrigues()
void Maths::ANGLE_AXIS_TO_MAT(const double angleAxis[3], double m[9])
{
	double angle = sqrt(DOT_VEC(angleAxis, angleAxis));
	if (angle <= 1e-7)
		return MAKE_IDENTITY_MAT(m);
	double s = 1.0 / angle;
	double v[3] = {s*angleAxis[0], s*angleAxis[1], s*angleAxis[2]};
	ANGLE_UNIT_AXIS_TO_MAT(angle, v, m);
}

void Maths::ANGLE_AXIS_FROM_MAT(const double m[9], double angleAxis[3])
{
	// make sure m is not ill conditioned
	double angle = acos(Maths::CLAMP((m[0]+m[4]+m[8]-1)/2.0, -1.0, 1.0));
	double sin_angle = sin(angle);
	if( sin_angle != 0 ) { angle /= 2.0*sin_angle; }
	angleAxis[0] = angle*(m[7]-m[5]);
	angleAxis[1] = angle*(m[2]-m[6]);
	angleAxis[2] = angle*(m[3]-m[1]);
}

/// vecs must be normalised!!
void Maths::ROT_MAT_FROM_VECS(const double vec1[3], const double vec2[3],
		double m[9])
{
//	NORMALISE_VEC(vec1);
//	NORMALISE_VEC(vec2);
	double axis[3] = {0};
	CROSS_VEC(vec1, vec2, axis);
	double angle = asin(NORMALISE_VEC(axis));
	ANGLE_UNIT_AXIS_TO_MAT(angle, axis, m);
}

/// vecs must be normalised!!
void Maths::ANGLE_AXIS_FROM_VECS(const double vec1[3], const double vec2[3],
		double angleAxis[3])
{
//	NORMALISE_VEC(vec1);
//	NORMALISE_VEC(vec2);
	CROSS_VEC(vec1, vec2, angleAxis);
	double angle = asin(NORMALISE_VEC(angleAxis));
	SCALE_VEC(angleAxis, angle);
}

void Maths::ANGLE_AXIS_FROM_VEC(const double vec2[3], double angleAxis[3])
{
	const double vec1[3] = {0,0,1};		// forward in camera coordinates
	ANGLE_AXIS_FROM_VECS(vec1, vec2, angleAxis);
}

void Maths::ROT_VEC_X_AXIS(double vec[3], const double phi)
{
	double m[9] = {0};
	m[0] = 1;			m[1] = 0;			m[2] = 0;
	m[3] = 0;			m[4] = cos(phi);	m[5] = -sin(phi);
	m[6] = 0;			m[7] = sin(phi);	m[8] = cos(phi);
	double init[3] = {0};
	memcpy(init, vec, 3*sizeof(double));
	MAT_MUL_VEC(m, init, vec);
}

void Maths::ROT_VEC_Y_AXIS(double vec[3], const double phi)
{
	double m[9] = {0};
	m[0] = cos(phi);	m[1] = 0;			m[2] = sin(phi);
	m[3] = 0;			m[4] = 1;			m[5] = 0;
	m[6] = -sin(phi);	m[7] = 0;			m[8] = cos(phi);
	double init[3] = {0};
	memcpy(init, vec, 3*sizeof(double));
	MAT_MUL_VEC(m, init, vec);
}

void Maths::ROT_VEC_Z_AXIS(double vec[3], const double phi)
{
	double m[9] = {0};
	m[0] = cos(phi);	m[1] = -sin(phi);	m[2] = 0;
	m[3] = sin(phi);	m[4] = cos(phi);	m[5] = 0;
	m[6] = 0;			m[7] = 0;			m[8] = 1;
	double init[3] = {0};
	memcpy(init, vec, 3*sizeof(double));
	MAT_MUL_VEC(m, init, vec);
}

void Maths::ROT_MAT_X_AXIS(double mat[9], const double phi)
{
	double m[9] = {0};
	m[0] = 1;			m[1] = 0;			m[2] = 0;
	m[3] = 0;			m[4] = cos(phi);	m[5] = -sin(phi);
	m[6] = 0;			m[7] = sin(phi);	m[8] = cos(phi);
	double init[9] = {0};
	memcpy(init, mat, 9*sizeof(double));
	MUL_MAT(m, init, mat);
}

void Maths::ROT_MAT_Y_AXIS(double mat[9], const double phi)
{
	double m[9] = {0};
	m[0] = cos(phi);	m[1] = 0;			m[2] = sin(phi);
	m[3] = 0;			m[4] = 1;			m[5] = 0;
	m[6] = -sin(phi);	m[7] = 0;			m[8] = cos(phi);
	double init[9] = {0};
	memcpy(init, mat, 9*sizeof(double));
	MUL_MAT(m, init, mat);
}

void Maths::ROT_MAT_Z_AXIS(double mat[9], const double phi)
{
	double m[9] = {0};
	m[0] = cos(phi);	m[1] = -sin(phi);	m[2] = 0;
	m[3] = sin(phi);	m[4] = cos(phi);	m[5] = 0;
	m[6] = 0;			m[7] = 0;			m[8] = 1;
	double init[9] = {0};
	memcpy(init, mat, 9*sizeof(double));
	MUL_MAT(m, init, mat);
}

void Maths::MAKE_EULER_MAT(const double roll, const double pitch,
		const double heading, double mat[9])
{
	// Vl = M*Ve	Where Vl and Ve are vectors expressed in the body and
	//				earth frames respectively.
	mat[0] = cos(heading)*cos(pitch);
	mat[1] = sin(heading)*cos(pitch);
	mat[2] = -sin(pitch);
	mat[3] = cos(heading)*sin(pitch)*sin(roll)-sin(heading)*cos(roll);
	mat[4] = sin(heading)*sin(pitch)*sin(roll)+cos(heading)*cos(roll);
	mat[5] = cos(pitch)*sin(roll);
	mat[6] = cos(heading)*sin(pitch)*cos(roll)+sin(heading)*sin(roll);
	mat[7] = sin(heading)*sin(pitch)*cos(roll)-cos(heading)*sin(roll);
	mat[8] = cos(pitch)*cos(roll);
}

void Maths::EULER_ANGLES_FROM_MAT(const double mat[9],
		double& roll, double& pitch, double& heading)
{
	roll = atan2(mat[5], mat[8]);
	pitch = asin(-mat[2]);
	heading = atan2(mat[1], mat[0]);
}

void Maths::ROT_VEC_WORLD_TO_BODY(const double world[3],
		const double roll, const double pitch, const double heading,
		double body[3])
{
	double m[9];
	MAKE_EULER_MAT(roll, pitch, heading, m);
	MAT_MUL_VEC(m, world, body);
}

void Maths::ROT_VEC_WORLD_TO_BODY(double vec[3],
		const double roll, const double pitch, const double heading)
{
	double m[9] = {0};
	MAKE_EULER_MAT(roll, pitch, heading, m);
	double body[3] = {0};
	MAT_MUL_VEC(m, vec, body);
	vec[0] = body[0];
	vec[1] = body[1];
	vec[2] = body[2];
}

void Maths::ROT_VEC_WORLD_TO_BODY(const double world[3],
		const double m[9], double body[3])
{
	MAT_MUL_VEC(m, world, body);
}

void Maths::ROT_VEC_WORLD_TO_BODY(double vec[3],
		const double m[9])
{
	double body[3] = {0};
	MAT_MUL_VEC(m, vec, body);
	vec[0] = body[0];
	vec[1] = body[1];
	vec[2] = body[2];
}

void Maths::ROT_VEC_BODY_TO_WORLD(const double body[3],
		const double roll, const double pitch, const double heading,
		double world[3])
{
	double m[9];
	MAKE_EULER_MAT(roll, pitch, heading, m);
	MAT_T_MUL_VEC(m, body, world);
}

void Maths::ROT_VEC_BODY_TO_WORLD(double vec[3],
		const double roll, const double pitch, const double heading)
{
	double m[9] = {0};
	MAKE_EULER_MAT(roll, pitch, heading, m);
	double world[3] = {0};
	MAT_T_MUL_VEC(m, vec, world);
	vec[0] = world[0];
	vec[1] = world[1];
	vec[2] = world[2];
}

void Maths::ROT_VEC_BODY_TO_WORLD(const double body[3],
		const double m[9], double world[3])
{
	MAT_T_MUL_VEC(m, body, world);
}

void Maths::ROT_VEC_BODY_TO_WORLD(double vec[3],
		const double m[9])
{
	double world[3] = {0};
	MAT_T_MUL_VEC(m, vec, world);
	vec[0] = world[0];
	vec[1] = world[1];
	vec[2] = world[2];
}

#include "CmPoint.h"
#include "CameraModel.h"
#include <cmath>


///-----------------------------------------------------------------------------
/// Non-inline implementation - all <cmath> dependencies are in here
///-----------------------------------------------------------------------------


template <typename T>
static inline void angleUnitAxisToMatrix(
	T cosAngle, T sinAngle, const CmPointT<T>& axis, T m[9])
{
	T x=axis.x, y=axis.y, z=axis.z;
	T c = cosAngle;
	T d = (T)1.0 - c;
	T dx = d*x;
	T dy = d*y;
	T dz = d*z;
	T dxy = dx*y;
	T dxz = dx*z;
	T dyz = dy*z;
	T s = sinAngle;
	T sx = s*x;
	T sy = s*y;
	T sz = s*z;
	m[0] = c + dx*x;
	m[1] = dxy - sz;
	m[2] = dxz + sy;
	m[3] = dxy + sz;
	m[4] = c + dy*y;
	m[5] = dyz - sx;
	m[6] = dxz - sy;
	m[7] = dyz + sx;
	m[8] = c + dz*z;
}

template <typename T>
static inline void matMul(const T a[9], const T b[9], T m[9])
{
	m[0] = a[0]*b[0] + a[1]*b[3] + a[2]*b[6];
	m[1] = a[0]*b[1] + a[1]*b[4] + a[2]*b[7];
	m[2] = a[0]*b[2] + a[1]*b[5] + a[2]*b[8];

	m[3] = a[3]*b[0] + a[4]*b[3] + a[5]*b[6];
	m[4] = a[3]*b[1] + a[4]*b[4] + a[5]*b[7];
	m[5] = a[3]*b[2] + a[4]*b[5] + a[5]*b[8];

	m[6] = a[6]*b[0] + a[7]*b[3] + a[8]*b[6];
	m[7] = a[6]*b[1] + a[7]*b[4] + a[8]*b[7];
	m[8] = a[6]*b[2] + a[7]*b[5] + a[8]*b[8];
}

template <typename T>
static inline T cmQuatNormalise(T q[4])
{
	T x=q[0], y=q[1], z=q[2], w=q[3];
	T mag = std::sqrt(x*x + y*y + z*z + w*w);
	if (mag != 0) {
		T s = (T)1.0 / mag;
		q[0] *= s;
		q[1] *= s;
		q[2] *= s;
		q[3] *= s;
	}
	return mag;
}

template <typename T>
static inline void cmRotateMatrixToQuat(const T m[9], T q[4])
{
	T t = 1 + m[0] + m[4] + m[8];
	if (t > (T)1e-7) {
		T S = std::sqrt(t) * (T)2.0;
		q[0] = (m[7] - m[5]) / S;
		q[1] = (m[2] - m[6]) / S;
		q[2] = (m[3] - m[1]) / S;
		q[3] = (T)0.25 * S;
	} else {
		if (m[0] > m[4] && m[0] > m[8])  {
			T S  = std::sqrt((T)1.0 + m[0] - m[4] - m[8]) * (T)2.0;
			q[0] = (T)0.25 * S;
			q[1] = (m[3] + m[1] ) / S;
			q[2] = (m[2] + m[6] ) / S;
			q[3] = (m[7] - m[5] ) / S;
		} else if (m[4] > m[8]) {
			T S  = std::sqrt((T)1.0 + m[4] - m[0] - m[8]) * (T)2.0;
			q[0] = (m[3] + m[1] ) / S;
			q[1] = (T)0.25 * S;
			q[2] = (m[7] + m[5] ) / S;
			q[3] = (m[2] - m[6] ) / S;
		} else {
			T S  = std::sqrt((T)1.0 + m[8] - m[0] - m[4]) * (T)2.0;
			q[0] = (m[2] + m[6] ) / S;
			q[1] = (m[7] + m[5] ) / S;
			q[2] = (T)0.25 * S;
			q[3] = (m[3] - m[1] ) / S;
		}
	}
	cmQuatNormalise(q);
}

/// assumes a normalised quaternion is given
template <typename T>
static inline void cmQuatToAngleAxis(const T q[4], T& angle, CmPointT<T>& axis)
{
	T x=q[0], y=q[1], z=q[2], w=q[3];
	T cos_a = w;
	T sin_a = std::sqrt((T)1.0 - cos_a * cos_a);
	if (std::fabs(sin_a) < (T)0.0005)
		sin_a = (T)1.0;
	angle = std::acos(cos_a) * (T)2.0;
	axis.x = x / sin_a;
	axis.y = y / sin_a;
	axis.z = z / sin_a;
}

#if 0
/// assumes a normalised quaternion is given
template <typename T>
static inline void cmQuatToRotateMatrix(const T q[4], T m[9])
{
	T wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;
	T x=q[0], y=q[1], z=q[2], w=q[3];
	x2 = x + x;
	y2 = y + y;
	z2 = z + z;
	xx = x * x2;
	xy = x * y2;
	xz = x * z2;
	yy = y * y2;
	yz = y * z2;
	zz = z * z2;
	wx = w * x2;
	wy = w * y2;
	wz = w * z2;
	m[0] = (T)1.0 - (yy + zz);
	m[1] = xy - wz;
	m[2] = xz + wy;
	m[3] = xy + wz;
	m[4] = (T)1.0 - (xx + zz);
	m[5] = yz - wx;
	m[6] = xz - wy;
	m[7] = yz + wx;
	m[8] = (T)1.0 - (xx + yy);
}

template <typename T>
static inline void cmAngleAxisToQuat(T angle, CmPointT<T> axis, T q[4])
{
	axis.normalise();
	T cos_a = std::cos(angle * (T)0.5);
	T sin_a = std::sin(angle * (T)0.5);
	q[0] = axis.x * sin_a;
	q[1] = axis.y * sin_a;
	q[2] = axis.z * sin_a;
	q[3] = cos_a;
}
#endif

template <typename T>
T CmPointT<T>::normalise()
{
	T mag = len();
	if (mag != 0)
		(*this) *= (T)1.0 / mag;
	return mag;
}

template <typename T>
CmPointT<T> CmPointT<T>::getNormalised() const
{
	CmPointT<T> ret = *this;
	ret.normalise();
	return ret;
}

template <typename T>
T CmPointT<T>::len() const
{
	return std::sqrt(len2());
}

template <typename T>
T CmPointT<T>::getAngleToNorm(const CmPointT<T>& b) const
{
	return std::acos(*this % b);
}

///
/// Be careful with precision of acos() when using single-precision float.
///
template <>
float CmPointT<float>::getAngleToNorm(const CmPointT<float>& b) const
{
	const CmPointT<float> &a = *this;
	float cosAngle = a % b;
	const float thresh = 0.95; /// prefer acos since it's cheaper
	if (std::fabs(cosAngle) < thresh) {
		return std::acos(cosAngle);
	} else {
		/// asin more precise around 0,180 degrees
		float angle = std::asin((a ^ b).len());
		if (cosAngle < 0)
			angle = (float)CM_PI - angle;
		return angle;
	}
}

template <typename T>
void CmPointT<T>::omegaToMatrix(float R[9]) const
{
	CmPointT<T> v = *this;
	T angle = v.normalise();
	angleUnitAxisToMatrix<float>(std::cos(angle), std::sin(angle), v, R);
}

template <typename T>
void CmPointT<T>::omegaToMatrix(double R[9]) const
{
	CmPointT<T> v = *this;
	T angle = v.normalise();
	angleUnitAxisToMatrix<double>(std::cos(angle), std::sin(angle), v, R);
}

template <typename T>
void CmPointT<T>::getRotationAbout(T angle, T R[9]) const
{
	CmPointT<T> a = *this;
	a.normalise();
	a.getRotationAboutNorm(angle, R);
}

template <typename T>
void CmPointT<T>::getRotationAboutNorm(T angle, T R[9]) const
{
	angleUnitAxisToMatrix(std::cos(angle), std::sin(angle), *this, R);
}

///-----------------------------------------------------------------------------
/// Explicitly instantiate supported types.
///-----------------------------------------------------------------------------

template class CmPointT<float>;
template class CmPointT<double>;


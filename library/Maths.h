/**
 * Maths
 *
 * Class provides some basic maths-related functions.
 * 
 * Richard Moore, 2009.
 * rjdmoore@gmail.com
 */

#ifndef MATHS_H_
#define MATHS_H_

#include <boost/shared_array.hpp>

#include <stdint.h>

namespace rjdm
{
	class Maths
	{
		public:
			/// Constants.
			static const double	PI;
			static const double	R2D;
			static const double	D2R;
			static const double	G;
			
			/// Utility functions.
			static double RND(long double num, int dig);
			static double CLAMP(double x, double min, double max)
					{ return (x<=min) ? min : (x>=max) ? max : x; }
			static int CLAMP(int x, int min, int max)
					{ return (x<=min) ? min : (x>=max) ? max : x; }
			static double DET(const double m[9])
				{ return m[0]*m[4]*m[8] + m[1]*m[5]*m[6] + m[2]*m[3]*m[7] - m[2]*m[4]*m[6] - m[1]*m[3]*m[8] - m[0]*m[5]*m[7]; }
			
			/// Vector functions.
			static void MUL_VEC(double a[3], const double b[3]);
			static void CROSS_VEC(const double a[3], const double b[3], double v[3])
					{	v[0] = a[1]*b[2] - a[2]*b[1];
						v[1] = a[2]*b[0] - a[0]*b[2];
						v[2] = a[0]*b[1] - a[1]*b[0]; }
			static float DOT_VEC(const float a[3], const float b[3])
					{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
			static double DOT_VEC(const double a[3], const double b[3])
					{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
			static void SCALE_VEC(float a[3], float s)
					{ a[0] *= s; a[1] *= s; a[2] *= s; }
			static void SCALE_VEC(double a[3], double s)
					{ a[0] *= s; a[1] *= s; a[2] *= s; }
			static void MAT_MUL_VEC(const double m[9], const double v[3], double r[3])
					{	r[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
						r[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
						r[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2]; }
			static void MAT_T_MUL_VEC(const double m[9], const double v[3], double r[3])
					{	r[2] = m[2]*v[0] + m[5]*v[1] + m[8]*v[2];
						r[1] = m[1]*v[0] + m[4]*v[1] + m[7]*v[2];
						r[0] = m[0]*v[0] + m[3]*v[1] + m[6]*v[2]; }
			static float NORMALISE_VEC(float a[3]);
			static double NORMALISE_VEC(double a[3]);
			static void MAKE_IDENTITY_MAT(double m[9]) {
				m[0] = m[4] = m[8] = 1.0; m[1]=m[2]=m[3]=m[5]=m[6]=m[7] = 0.0;
			}
			static void MAT_T(double mat[9]);
			static void MUL_MAT(const double lmat[9], const double rmat[9],
					double res[9]);
			static void ANGLE_UNIT_AXIS_TO_MAT(const double angle,
					const double axis[3], double m[9]);
			static void ANGLE_AXIS_TO_MAT(const double angleAxis[3], double m[9]);	// aka rodrigues
			static void ANGLE_AXIS_FROM_MAT(const double m[9], double angleAxis[3]);	// aka rodrigues
			static void ANGLE_AXIS_FROM_VECS(const double vec1[3],
					const double vec2[3], double angleAxis[3]);
			static void ANGLE_AXIS_FROM_VEC(const double vec2[3],
					double angleAxis[3]);
			static void ROT_MAT_FROM_VECS(const double vec1[3],
					const double vec2[3], double m[9]);

			/// Rotation functions.
			static void ROT_VEC_X_AXIS(double vec[3], const double phi);
			static void ROT_VEC_Y_AXIS(double vec[3], const double phi);
			static void ROT_VEC_Z_AXIS(double vec[3], const double phi);

			static void ROT_MAT_X_AXIS(double mat[9], const double phi);
			static void ROT_MAT_Y_AXIS(double mat[9], const double phi);
			static void ROT_MAT_Z_AXIS(double mat[9], const double phi);

			static void MAKE_EULER_MAT(const double roll, const double pitch,
					const double heading, double mat[9]);
			static void EULER_ANGLES_FROM_MAT(const double mat[9],
					double& roll, double& pitch, double& heading);

			/// In all the following, BODY is given in inertial coords - i.e. NED
			static inline void ROT_VEC_CAM_TO_BODY(double vec[3])
			{
				double t = vec[0]; vec[0] = vec[2]; vec[2] = vec[1]; vec[1] = t;
			}
			static inline void ROT_VEC_BODY_TO_CAM(double vec[3])
			{
				double t = vec[0]; vec[0] = vec[1]; vec[1] = vec[2]; vec[2] = t;
			}

			static void ROT_VEC_WORLD_TO_BODY(const double world[3],
					const double roll, const double pitch, const double heading,
					double body[3]);
			static void ROT_VEC_WORLD_TO_BODY(double vec[3],
					const double roll, const double pitch, const double heading);

			static void ROT_VEC_WORLD_TO_CAM(const double world[3],
					const double roll, const double pitch, const double heading,
					double cam[3])
			{
				ROT_VEC_WORLD_TO_BODY(world, roll, pitch, heading, cam);
				ROT_VEC_BODY_TO_CAM(cam);
			}
			static void ROT_VEC_WORLD_TO_CAM(double vec[3],
					const double roll, const double pitch, const double heading)
			{
				ROT_VEC_WORLD_TO_BODY(vec, roll, pitch, heading);
				ROT_VEC_BODY_TO_CAM(vec);
			}

			static void ROT_VEC_WORLD_TO_BODY(const double world[3],
					const double m[9], double body[3]);
			static void ROT_VEC_WORLD_TO_BODY(double vec[3],
					const double m[9]);

			static void ROT_VEC_WORLD_TO_CAM(const double world[3],
					const double m[9], double cam[3])
			{
				ROT_VEC_WORLD_TO_BODY(world, m, cam);
				ROT_VEC_BODY_TO_CAM(cam);
			}
			static void ROT_VEC_WORLD_TO_CAM(double vec[3],
					const double m[9])
			{
				ROT_VEC_WORLD_TO_BODY(vec, m);
				ROT_VEC_BODY_TO_CAM(vec);
			}

			static void ROT_VEC_BODY_TO_WORLD(const double body[3],
					const double roll, const double pitch, const double heading,
					double world[3]);
			static void ROT_VEC_BODY_TO_WORLD(double vec[3],
					const double roll, const double pitch, const double heading);

			static void ROT_VEC_CAM_TO_WORLD(const double cam[3],
					const double roll, const double pitch, const double heading,
					double world[3])
			{
				double vec[3] = {cam[0], cam[1], cam[2]};
				ROT_VEC_CAM_TO_BODY(vec);
				ROT_VEC_BODY_TO_WORLD(vec, roll, pitch, heading, world);
			}
			static void ROT_VEC_CAM_TO_WORLD(double vec[3],
					const double roll, const double pitch, const double heading)
			{
				ROT_VEC_CAM_TO_BODY(vec);
				ROT_VEC_BODY_TO_WORLD(vec, roll, pitch, heading);
			}

			static void ROT_VEC_BODY_TO_WORLD(const double body[3],
					const double m[9], double world[3]);
			static void ROT_VEC_BODY_TO_WORLD(double vec[3],
					const double m[9]);

			static void ROT_VEC_CAM_TO_WORLD(const double cam[3],
					const double m[9], double world[3])
			{
				double vec[3] = {cam[0], cam[1], cam[2]};
				ROT_VEC_CAM_TO_BODY(vec);
				ROT_VEC_BODY_TO_WORLD(vec, m, world);
			}
			static void ROT_VEC_CAM_TO_WORLD(double vec[3],
					const double m[9])
			{
				ROT_VEC_CAM_TO_BODY(vec);
				ROT_VEC_BODY_TO_WORLD(vec, m);
			}
	};
}

#endif /*MATHS_H_*/


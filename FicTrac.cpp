/*
 * FicTrac.cpp
 *
 *  Created on: 26/05/2011
 *      Author: Richard Moore (rjdmoore@uqconnect.edu.au)
 */

#define BUILD_STRING "2015.03.26"

#include "CameraModel.h"
#include "CameraRemap.h"
SHARED_PTR(CameraRemap);
#include "BasicRemapper.h"
#include "AVWriter.h"
#include "NLoptFunc.h"
#include "Maths.h"
#include "Utils.h"
#include "VsDraw.h"
#include "serial.h"
#include "CVSource.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <deque>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <iosfwd>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <netinet/in.h>
#include <signal.h>

#include "fmfwrapper.h"

//added by Pablo for MCC USB 3101 6/30/14
#include <fcntl.h>
#include <ctype.h>
#include <asm/types.h>

#include "pmd.h"
#include "usb-3100.h"
//these two files and its headers/pointers were added to Fictrac's folder
//---------------------------------

#define ENABLE_VOLTAGE_OUT 1  //set to 0 if running without HID devices (ie MCC 3101)
#define EXTRA_DEBUG_WINDOWS 0
#define LOG_TIMING 1

#ifdef PGR_CAMERA
	#include "PGRSource.h"
#endif

using rjdm::Maths;
using std::deque;
using std::vector;
using std::string;
using std::max;
using std::ofstream;
using std::ifstream;
using std::stringstream;
using std::endl;
using cv::Mat;
using cv::Scalar;
using cv::Size;
using cv::Rect;
using cv::Point;
using cv::Point3f;
using cv::waitKey;
using cv::namedWindow;
using cv::destroyWindow;
using cv::setWindowProperty;
using cv::startWindowThread;
using cv::WINDOW_NORMAL;
using cv::BORDER_CONSTANT;
using cv::BORDER_REPLICATE;
using cv::imread;

bool SPHERE_INIT = false;

bool ACTIVE = true;

enum CAM_MODEL_TYPE { RECTILINEAR, FISHEYE };

//FIXME: multi-thread printf calls

//static void equalPointsOnSphere(int n)
//{
////	dlong := pi*(3-sqrt(5))  /* ~2.39996323 */
////	dz    := 2.0/N
////	long := 0
////	z    := 1 - dz/2
////	for k := 0 .. N-1
////		r    := sqrt(1-z*z)
////		node[k] := (cos(long)*r, sin(long)*r, z)
////		z    := z - dz
////		long := long + dlong
////	end
//
//	if( n < 2 ) { return; }
//
//	double dlong = PI*(3-sqrt(5.0));
//	double dz = 2.0/n;
//	double long = 0;
//	double z = 1.0-dz/2.0;
//}

static void TERMINATE(int) { ACTIVE = false; }

void drawLine(uint8_t* img, int w, int h, int c, int step,
		double x1, double y1, double x2, double y2,
		uint8_t r=255, uint8_t g=255, uint8_t b=255, int thickness=1)
{
	IplImage* ipl = cvCreateImageHeader(cvSize(w,h), IPL_DEPTH_8U, c);
	ipl->imageData = (char*)img;
	ipl->widthStep = step;

	int x1i = round(x1*16);
	int y1i = round(y1*16);
	int x2i = round(x2*16);
	int y2i = round(y2*16);

	cvLine(ipl, cvPoint(x1i,y1i), cvPoint(x2i,y2i), CV_RGB(b,g,r), thickness, CV_AA, 4);
	cvReleaseImageHeader(&ipl);
}

void drawLine(Mat& img,
		double x1, double y1, double x2, double y2,
		uint8_t r=255, uint8_t g=255, uint8_t b=255, int thickness=1)
{
	return drawLine(img.data, img.cols, img.rows, img.channels(), img.step,
			x1, y1, x2, y2,
			r, g, b, thickness);
}

void drawText(uint8_t* img, int w, int h, int c, int step,
		std::string text, int x, int y,
		double scale, int r=-1, int g=-1, int b=-1,
		bool shadow=false, int shadow_val=-1)
{
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5*scale, 0.5*scale, 0, 1*scale, CV_AA);
	IplImage* ipl = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, c);
	ipl->imageData = (char*)img;
	ipl->widthStep = step;

	if( r < 0 || g < 0 || b < 0 ) { r = g = b = 255; }

	if( shadow ) {
		if( shadow_val < 0 ) { shadow_val = 0; }
		cvPutText(ipl,text.c_str(),cvPoint(x+1,y+1),&font,CV_RGB(shadow_val,shadow_val,shadow_val));
	}

	cvPutText(ipl,text.c_str(),cvPoint(x,y),&font,CV_RGB(b,g,r));
	cvReleaseImageHeader(&ipl);
}

void drawText(Mat& img,
		std::string text, int x, int y, double scale,
		int r=-1, int g=-1, int b=-1,
		bool shadow=false, int shadow_val=-1)
{
	return drawText(img.data, img.cols, img.rows, img.channels(), img.step,
			text, x, y, scale, r, g, b, shadow, shadow_val);
}

void makeSphereRotMaps(CameraModelPtr cam_model,
		Mat mapX, Mat mapY, Mat mask,
		double sphere_r_d_ratio, double rot_angle_axis[3])
{
	double sc[3] = {0, 0, 1};
	double r = sphere_r_d_ratio;
	double* rot = rot_angle_axis;
	int w = mapX.cols, h = mapX.rows;
	float *mapx = (float*)mapX.data, *mapy = (float*)mapY.data;
	for( int i = 0, it = 0; i < h; ++i ) {
		for( int j = 0; j < w; ++j, ++it ) {
			if( mask.data[i*mask.step+j] < 255 ) {
				mapx[it] = -1;
				mapy[it] = -1;
				continue;
			}

			double l[3] = {0};
			cam_model->pixelIndexToVector(j, i, l);
			Maths::NORMALISE_VEC(l);

			double a = 1.0;
			double b = 2.0*(l[0]*-sc[0]+l[1]*-sc[1]+l[2]*-sc[2]);
			double c = sc[0]*sc[0]+sc[1]*sc[1]+sc[2]*sc[2]-r*r;

			double quad = b*b-4*a*c;
			if( quad < 0 ) {
				mapx[it] = -1;
				mapy[it] = -1;
//				mask.data[it] = 0;
				continue;
			}

			// get point on surface of sphere (camera coords)
			//FIXME: always choses point on front surface of sphere!!
//			double u1 = (-b+sqrt(quad))/(2.0*a);
			double u2 = (-b-sqrt(quad))/(2.0*a);
//			double u = std::min(u1,u2);
			double p1[3] = {l[0]*u2, l[1]*u2, l[2]*u2};

			// switch to sphere coords
			double p1s[3] = {p1[0]-sc[0], p1[1]-sc[1], p1[2]-sc[2]};

			// rotate point about rotation axis (sphere coords)
			double rmat[9] = {0};
			Maths::ANGLE_AXIS_TO_MAT(rot, rmat);
			double p2s[3] = {0};
			Maths::MAT_MUL_VEC(rmat, p1s, p2s);

			// check whether point has disappeared
			if( Maths::DOT_VEC(p2s, sc) > 0 ) {
				mapx[it] = -1;
				mapy[it] = -1;
//				mask.data[it] = 128;
				continue;
			}

			// switch back to camera coords
			double p2[3] = {p2s[0]+sc[0], p2s[1]+sc[1], p2s[2]+sc[2]};

			// re-intersect with camera model
			double x2 = 0, y2 = 0;
			cam_model->vectorToPixelIndex(p2, x2, y2);

			mapx[it] = float(x2);
			mapy[it] = float(y2);
//			mask.data[it] = 255;
		}
	}
}

bool intersectSphere(double camVec[3], double sphereVec[3], double r)
{
	double q = camVec[2]*camVec[2]+r*r;

	if( q < 1 ) { return false; }

	// get point on front surface of sphere (camera coords)
	double u = camVec[2]-sqrt(q-1);

	// switch to sphere coords
	sphereVec[0] = camVec[0]*u;
	sphereVec[1] = camVec[1]*u;
	sphereVec[2] = camVec[2]*u-1;

	return true;
}

class MatchSphereAbs : public NLoptFunc
{
public:
	MatchSphereAbs(CameraModelPtr cam_model,
			double roi_rot_mat[9],
			int sphere_w, int sphere_h,
			double sphere_r_d_ratio,
			Mat mask=Mat(), Mat sphere_template=Mat(),
			bool do_display=false, bool do_update=true,
			double lower_bound=-0.5, double upper_bound=0.5,
			double tol=1e-4, int max_eval=100)
	: _cam_model(cam_model), _w(sphere_w), _h(sphere_h),
	  _input_w(cam_model->width()), _input_h(cam_model->height()),
	  _r(sphere_r_d_ratio),  _do_display(do_display), _do_update(do_update)
	{
		/*
		 * The model sphere has it's own coordinate system, because we arbitrarily set
		 * the view vector corresponding to the centre of the projection of the tracking
		 * ball to be the principal axis of the virtual camera (cam_model).
		 *
		 * All incoming and outgoing vectors and matrices must thus be transposed to/from
		 * this coordinate system: world <= (Rw) => cam <= (roi_rot_mat) => ROI.
		 */

		init(NLOPT_LN_BOBYQA, 3);
		setLowerBounds(lower_bound);
		setUpperBounds(upper_bound);
		setFtol(tol);
		setXtol(tol);
		setMaxEval(max_eval);

		_sphere_model = CameraModel::createEquiArea(_w, _h);

		_R[0] = _testR[0] = _tmpR1[0] = _tmpR2[0] = 1;
		_R[1] = _testR[1] = _tmpR1[1] = _tmpR2[1] = 0;
		_R[2] = _testR[2] = _tmpR1[2] = _tmpR2[2] = 0;
		_R[3] = _testR[3] = _tmpR1[3] = _tmpR2[3] = 0;
		_R[4] = _testR[4] = _tmpR1[4] = _tmpR2[4] = 1;
		_R[5] = _testR[5] = _tmpR1[5] = _tmpR2[5] = 0;
		_R[6] = _testR[6] = _tmpR1[6] = _tmpR2[6] = 0;
		_R[7] = _testR[7] = _tmpR1[7] = _tmpR2[7] = 0;
		_R[8] = _testR[8] = _tmpR1[8] = _tmpR2[8] = 1;

		if( !mask.empty() ) {
			_mask = mask.clone();
		} else {
			_mask.create(_input_h, _input_w, CV_8UC1);
			_mask.setTo(Scalar::all(255));
		}

		_sphere_hist = boost::shared_array<int>(new int[_w*_h*256]);
		memset(_sphere_hist.get(), 0, _w*_h*256*sizeof(int));

		_sphere_max.create(_h, _w, CV_32SC1);
		_sphere_max.setTo(Scalar::all(0));

		if( !sphere_template.empty() ) {
			_sphere = sphere_template.clone();
			for( int i = 0; i < _h; i++ ) {
				int32_t* pmax = (int32_t*)_sphere_max.ptr(i);
				uint8_t* psphere = _sphere.ptr(i);
				for( int j = 0; j < _w; j++ ) {
					uint8_t d = abs(psphere[j]-128);
					pmax[j] = d;
					_sphere_hist[(i*_w+j)*256+psphere[j]] = d;
				}
			}
		} else {
			_sphere.create(_h, _w, CV_8UC1);
			_sphere.setTo(Scalar::all(128));
		}

		_orient.create(_h, _w, CV_8UC1);
		_orient.setTo(Scalar::all(255));

		_p1s_lut = boost::shared_array<double>(new double[_input_w*_input_h*3]);
		memset(_p1s_lut.get(), 0, _input_w*_input_h*3*sizeof(double));
		for( int i = 0; i < _input_h; i++ ) {
			uint8_t* pmask = _mask.ptr(i);
			for( int j = 0; j < _input_w; j++ ) {
				if( pmask[j] < 255 ) { continue; }

				double l[3] = {0};
				_cam_model->pixelIndexToVector(j, i, l);
				Maths::NORMALISE_VEC(l);

				double* s = &_p1s_lut[(i*_input_w+j)*3];
				if( !intersectSphere(l, s, _r) ) { pmask[j] = 128; }
			}
		}

//		memcpy(_roi_rot_mat, roi_rot_mat, 9*sizeof(double));
//		Maths::MAKE_IDENTITY_MAT(_roi_rot_mat);
	}

	void setImage(Mat input)
	{
		_input = input;
	}

	void clearSphere()
	{
		_sphere.setTo(Scalar::all(128));
		_sphere_max.setTo(Scalar::all(0));
		memset(_sphere_hist.get(), 0, _w*_h*256*sizeof(int));
	}

	double* updateSphere(double angleAxis[3])
	{
		Maths::ANGLE_AXIS_TO_MAT(angleAxis, _tmpR1);	// relative rotation in camera frame
		Maths::MUL_MAT(_R, _tmpR1, _tmpR2);				// absolute orientation in camera frame
		memcpy(_R, _tmpR2, 9*sizeof(double));			// save absolute orientation
//		Maths::MUL_MAT(_roi_rot_mat, _R, _tmpR1);		// transpose to ROI coordinate frame

		if( _do_display ) {
			_orient.setTo(Scalar::all(128));
		}

		static Mat diff(_h, _w, CV_8UC1);
		#if EXTRA_DEBUG_WINDOWS
			diff.setTo(Scalar::all(128));
		#endif

		int cnt = 0, good = 0;
		for( int i = 0; i < _input_h; i++ ) {
			for( int j = 0; j < _input_w; j++ ) {
				if( _mask.data[i*_mask.step+j] < 255 ) { continue; }
				cnt++;

				double* p1s = &_p1s_lut[(i*_input_w+j)*3];

				// rotate point about rotation axis (sphere coords)
				double p2s[3] = {0};
				Maths::MAT_MUL_VEC(_tmpR2, p1s, p2s);

				// map vector in sphere coords to pixel
				int x = 0, y = 0;
				_sphere_model->vectorToPixelIndex(p2s, x, y);

				#if EXTRA_DEBUG_WINDOWS
					diff.data[y*diff.step+x] = abs(_input.data[i*_input.step+j]-_sphere.data[y*_sphere.step+x]);
				#endif

				if( _do_update ) {
					int32_t* smax = &_sphere_max.at<int32_t>(y,x);
					if( *smax > 0 ) { good++; }

					uint8_t d = _input.data[i*_input.step+j];
					int h = ++(_sphere_hist[(y*_w+x)*256+d]);
					if( h > *smax ) {
						*smax = h;
						_sphere.data[y*_sphere.step+x] = d;
					}
				}

				// display
				if( _do_display ) {
					_orient.data[y*_orient.step+x] = _input.data[i*_input.step+j];
				}
			}
		}

		double r = good/double(cnt);
		printf("match overlap: %.1f%%\n", 100*r);

		#if EXTRA_DEBUG_WINDOWS
			namedWindow("FicTrac-diff", 0);
			imshow("FicTrac-diff", diff);
			cvResizeWindow("FicTrac-diff", 600, 300);
		#endif

		return _R;
	}

	double testRotation(const double x[3])
	{
		Maths::ANGLE_AXIS_TO_MAT(x, _tmpR1);			// relative rotation in camera frame
		Maths::MUL_MAT(_R, _tmpR1, _testR);				// absolute orientation in camera frame
//		Maths::MUL_MAT(_roi_rot_mat, _tmpR2, _testR);	// transpose to ROI coordinate frame

//		printf("%d: testing %6.3f %6.3f %6.3f...",
//				_nEval, x[0], x[1], x[2]);

//		_orient.setTo(Scalar::all(128));

		double err = 0;
		int cnt = 0, good = 0;
		for( int i = 0; i < _input_h; i++ ) {
			for( int j = 0; j < _input_w; j++ ) {
				if( _mask.data[i*_mask.step+j] < 255 ) { continue; }
				cnt++;

				double* p1s = &_p1s_lut[(i*_input_w+j)*3];

				// rotate point about rotation axis (sphere coords)
				double p2s[3] = {0};
				Maths::MAT_MUL_VEC(_testR, p1s, p2s);

				// map vector in sphere coords to pixel
				int x = 0, y = 0;
				_sphere_model->vectorToPixelIndex(p2s, x, y);

				if( _sphere_max.at<int32_t>(y,x) == 0 ) { continue; }
				int px = _input.data[i*_input.step+j];
				if( abs(px-128) < 50 ) { continue; }
				double diff = (px-_sphere.data[y*_sphere.step+x]);
				err += diff*diff;
				good++;

//				_orient.data[y*_orient.step+x] = _input.data[i*_input.step+j];
			}
		}

		if( cnt > 0 ) {
			double r = good/double(cnt);
			if( r > 0.25 || !SPHERE_INIT ) {
				err /= good;
			} else {
				err = DBL_MAX;
			}
		} else {
			err = DBL_MAX;
		}

//		printf(" %6.3f\n", diff);

//		namedWindow("debug-test1");
//		imshow("debug-test1", _orient);

		return err;
	}

	virtual double objective(unsigned n, const double* x, double* grad)
	{
		return testRotation(x);
	}

	void drawDebug(Mat& sphere, Mat& orient)
	{
		if( _do_display ) {
			resize(_sphere, sphere, sphere.size());
			resize(_orient, orient, orient.size());
		}
	}

	double* getR() { return _R; }
	double* getTestR() { return _testR; }
	Mat& getTemplate() { return _sphere; }
	double* getLUT() { return _p1s_lut.get(); }
	CameraModelPtr getSphereModel() { return _sphere_model; }

private:
	CameraModelPtr _cam_model, _sphere_model;
	int _w, _h;
	int _input_w, _input_h;
	double _r;

	double _R[9], _testR[9], _tmpR1[9], _tmpR2[9];
//	double _roi_rot_mat[9];

	Mat _sphere, _weights, _input, _mask, _orient;

	Mat _sphere_max;

	boost::shared_array<double> _p1s_lut;

	boost::shared_array<int> _sphere_hist;

	bool _do_display, _do_update;
};

void getCornerVecs(
		const double x[6],
		double tl2[3], double tr2[3],
		double br2[3], double bl2[3])
{
	double T[3] = {x[0], x[1], x[2]};
	double R[3] = {x[3], x[4], x[5]};
	double M[9] = {0};
	Maths::ANGLE_AXIS_TO_MAT(R, M);

	// get initial corner positions
	double tl0[3] = {0.5, -0.5, 0};
	double tr0[3] = {0.5, 0.5, 0};
	double br0[3] = {-0.5, 0.5, 0};
	double bl0[3] = {-0.5, -0.5, 0};

	// rotate corners
	double tl1[3] = {0};
	Maths::MAT_MUL_VEC(M, tl0, tl1);
	double tr1[3] = {0};
	Maths::MAT_MUL_VEC(M, tr0, tr1);
	double br1[3] = {0};
	Maths::MAT_MUL_VEC(M, br0, br1);
	double bl1[3] = {0};
	Maths::MAT_MUL_VEC(M, bl0, bl1);

	// plus translation
	tl2[0] = tl1[0]+T[0]; tl2[1] = tl1[1]+T[1]; tl2[2] = tl1[2]+T[2];
	tr2[0] = tr1[0]+T[0]; tr2[1] = tr1[1]+T[1]; tr2[2] = tr1[2]+T[2];
	br2[0] = br1[0]+T[0]; br2[1] = br1[1]+T[1]; br2[2] = br1[2]+T[2];
	bl2[0] = bl1[0]+T[0]; bl2[1] = bl1[1]+T[1]; bl2[2] = bl1[2]+T[2];

	Maths::NORMALISE_VEC(tl2);
	Maths::NORMALISE_VEC(tr2);
	Maths::NORMALISE_VEC(br2);
	Maths::NORMALISE_VEC(bl2);
}

void getAxesVecs(
		const double nlopt_x[6],
		double origin[3], double xaxis[3],
		double yaxis[3], double zaxis[3])
{
	double T[3] = {nlopt_x[0], nlopt_x[1], nlopt_x[2]};
	double R[3] = {nlopt_x[3], nlopt_x[4], nlopt_x[5]};
	double M[9] = {0,0,0,0,0,0,0,0,0};
	Maths::ANGLE_AXIS_TO_MAT(R, M);

	// get initial axes
	double o0[3] = {0.0, 0.0, 0.0};
	double x0[3] = {0.25, 0.0, 0.0};
	double y0[3] = {0.0, 0.25, 0.0};
	double z0[3] = {0.0, 0.0, 0.25};

	// rotate axes
	double o1[3] = {0};
	Maths::MAT_MUL_VEC(M, o0, o1);
	double x1[3] = {0};
	Maths::MAT_MUL_VEC(M, x0, x1);
	double y1[3] = {0};
	Maths::MAT_MUL_VEC(M, y0, y1);
	double z1[3] = {0};
	Maths::MAT_MUL_VEC(M, z0, z1);

	// plus translation
	origin[0] = o1[0]+T[0]; origin[1] = o1[1]+T[1]; origin[2] = o1[2]+T[2];
	xaxis[0] = x1[0]+T[0]; xaxis[1] = x1[1]+T[1]; xaxis[2] = x1[2]+T[2];
	yaxis[0] = y1[0]+T[0]; yaxis[1] = y1[1]+T[1]; yaxis[2] = y1[2]+T[2];
	zaxis[0] = z1[0]+T[0]; zaxis[1] = z1[1]+T[1]; zaxis[2] = z1[2]+T[2];

	Maths::NORMALISE_VEC(origin);
	Maths::NORMALISE_VEC(xaxis);
	Maths::NORMALISE_VEC(yaxis);
	Maths::NORMALISE_VEC(zaxis);
}

class PlaneHomography : public NLoptFunc
{
public:
	PlaneHomography(vector<Point3f>& corners)
	:	_corners(corners)
	{
		// tx ty tz rx ry rz
		double lb[6] = {-1e3, -1e3, 0, -CM_PI, -CM_PI, -CM_PI};
		double ub[6] = {1e3, 1e3, 1e3, CM_PI, CM_PI, CM_PI};
		init(NLOPT_LN_BOBYQA, 6);
		setLowerBounds(lb);
		setUpperBounds(ub);
		setXtol(1e-8);
		setMaxEval(1e4);

		double init_step[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
		setInitialStep(init_step);
	}

	virtual double objective(unsigned n, const double* x, double* grad)
	{
//		printf("%d:\tT: %.2f %.2f %.2f  R: %.2f %.2f %.2f  ",
//				_nEval, x[0], x[1], x[2], x[3], x[4], x[5]);

		double tl[3] = {_corners[0].x, _corners[0].y, _corners[0].z};
		double tr[3] = {_corners[1].x, _corners[1].y, _corners[1].z};
		double br[3] = {_corners[2].x, _corners[2].y, _corners[2].z};
		double bl[3] = {_corners[3].x, _corners[3].y, _corners[3].z};

		double tl2[3], tr2[3], br2[3], bl2[3];
		getCornerVecs(x, tl2, tr2, br2, bl2);

		double err = 0;
//		err += acos(CLAMP(Maths::DOT_VEC(tl, tl2), -1.0, 1.0));
//		err += acos(CLAMP(Maths::DOT_VEC(tr, tr2), -1.0, 1.0));
//		err += acos(CLAMP(Maths::DOT_VEC(br, br2), -1.0, 1.0));
//		err += acos(CLAMP(Maths::DOT_VEC(bl, bl2), -1.0, 1.0));

		err += ((tl[0]-tl2[0])*(tl[0]-tl2[0])+(tl[1]-tl2[1])*(tl[1]-tl2[1])+(tl[2]-tl2[2])*(tl[2]-tl2[2]));
		err += ((tr[0]-tr2[0])*(tr[0]-tr2[0])+(tr[1]-tr2[1])*(tr[1]-tr2[1])+(tr[2]-tr2[2])*(tr[2]-tr2[2]));
		err += ((br[0]-br2[0])*(br[0]-br2[0])+(br[1]-br2[1])*(br[1]-br2[1])+(br[2]-br2[2])*(br[2]-br2[2]));
		err += ((bl[0]-bl2[0])*(bl[0]-bl2[0])+(bl[1]-bl2[1])*(bl[1]-bl2[1])+(bl[2]-bl2[2])*(bl[2]-bl2[2]));

//		printf("(%.2f)\n", err);

		return err;
	}

private:
	vector<Point3f>& _corners;
};

void mouseCallback(int event, int x, int y, int flags, void* param)
{
	vector<Point>* click = (vector<Point>*)param;

	switch (event) {
//	case CV_EVENT_MOUSEMOVE:
//		break;
	case CV_EVENT_LBUTTONDOWN:
//		printf("mouse: %d %d\n", x, y);
		click->push_back(Point(x,y));
		break;
	case CV_EVENT_RBUTTONDOWN:
//		printf("right mouse: %d %d\n", x, y);
		click->push_back(Point(-1,-1));
//		pthread_mutex_lock(&_GPSPATH_MUTX);
//		printf("terminating\n");
//		_GPSPATH_ACTIVE = false;
//		pthread_cond_broadcast(&_GPSPATH_COND);
//		pthread_mutex_unlock(&_GPSPATH_MUTX);
		break;
	}
}

struct s_input {
	bool cam_input;
	boost::shared_ptr<ImgSource> cap;
	deque<Mat> frame_q, remap_q;
	deque<double> timestamp_q;
	pthread_t thread;
	pthread_mutex_t	mutex;
	pthread_cond_t cond;
	unsigned int buflen;
	int width, height;
	int remap_width, remap_height;
	CameraModelPtr cam_model, remap_model;
	CameraRemapPtr remapper;
	RemapTransformPtr roi_transform;
	Mat remap_mask;
	bool use_ball_colour;
	CmPoint32f fg_yuv, bg_yuv;
	double thresh_win;
	double thresh_ratio;
	int frame_skip;
};

double SEGMENT_TIME = 0;

void* grabInputFrames(void* obj)
{
	Utils::SET_PROCESS_PRIORITY(-5);

	s_input* input = (s_input*)obj;
	int rheight = input->remap_height, rwidth = input->remap_width;
	Mat frame_grey(input->height, input->width, CV_8UC1);
	Mat remap_bgr(rheight, rwidth, CV_8UC3);
	remap_bgr.setTo(Scalar::all(0));
	Mat remap_yuv(rheight, rwidth, CV_8UC3);
	remap_yuv.setTo(Scalar::all(0));
	Mat remap_blur(rheight, rwidth, CV_8UC1);
	remap_blur.setTo(Scalar::all(0));

	double thresh_ratio = input->thresh_ratio;
	int thresh_win = (int)round(input->thresh_win*rwidth) | 0x01;
	int thresh_rad = (thresh_win-1)/2;
//	int thresh_max_x = rwidth-thresh_win-1;
//	int thresh_max_y = rheight-thresh_win-1;
	Mat thresh_min(rheight, rwidth, CV_8UC1);
	Mat thresh_max(rheight, rwidth, CV_8UC1);

	Mat remap_mask = input->remap_mask.clone();

	bool use_ball_colour = input->use_ball_colour;
	double fgu = input->fg_yuv.y, fgv = input->fg_yuv.z;
	double bgu = input->bg_yuv.y, bgv = input->bg_yuv.z;

#if EXTRA_DEBUG_WINDOWS
	namedWindow("roi", WINDOW_NORMAL);
	namedWindow("thresh", WINDOW_NORMAL);
#endif // EXTRA_DEBUG_WINDOWS

	// rewind to video start
	if( !input->cam_input ) { input->cap->rewind(); }

	// run while active
	int grab_errs = 0;
	int frame_cnt = 0;
	while( ACTIVE ) {
		pthread_mutex_lock(&input->mutex);
		while( ACTIVE && input->frame_q.size() >= input->buflen ) {
			pthread_cond_wait(&input->cond, &input->mutex);
		}
		pthread_mutex_unlock(&input->mutex);
		if( !ACTIVE ) { break; }

		#if LOG_TIMING
			double t1 = Utils::GET_CLOCK();
		#endif

		Mat frame_bgr(input->height, input->width, CV_8UC3);
		if( !input->cap->grab(frame_bgr) ) {
			fprintf(stderr, "ERROR: Can't get new frame!\n");
			fflush(stderr);
			if( !(input->cam_input) && (++grab_errs > 0) ) {
				pthread_mutex_lock(&input->mutex);
				ACTIVE = false;
				pthread_cond_broadcast(&(input->cond));
				pthread_mutex_unlock(&input->mutex);
				break;
			} else { continue; }
		} else { grab_errs = 0; frame_cnt++; }
		if( frame_cnt <= input->frame_skip ) { continue; }
		double timestamp = input->cap->getTimestamp();

		// output thresholded image
		Mat remap_grey(rheight, rwidth, CV_8UC1);
		remap_grey.setTo(Scalar::all(128));

		// vars for cached min/max
		int win_it = 0;
		uint8_t win_max_hist[thresh_win];
		uint8_t win_min_hist[thresh_win];

//		double t1 = Utils::GET_CLOCK();

		if( use_ball_colour ) {
			/// Threshold ROI using UV colour space.
			input->remapper->apply(frame_bgr, remap_bgr);

#if EXTRA_DEBUG_WINDOWS
			imshow("roi", remap_bgr);
#endif // EXTRA_DEBUG_WINDOWS

			cvtColor(remap_bgr, remap_yuv, CV_BGR2YUV);
			for( int i = 0; i < rheight; i++ ) {
				uint8_t* pyuv = remap_yuv.ptr(i);
				uint8_t* pgrey = remap_grey.ptr(i);
				uint8_t* pmask = remap_mask.ptr(i);
				for( int j = 0; j < rwidth; j++ ) {
					if( pmask[j] < 250 ) { continue; }
					uint8_t u = pyuv[3*j+1];
					uint8_t v = pyuv[3*j+2];
					double fgd = sqrt((u-fgu)*(u-fgu)+(v-fgv)*(v-fgv));
					double bgd = sqrt((u-bgu)*(u-bgu)+(v-bgv)*(v-bgv));
					if( (thresh_ratio*bgd) <= fgd ) {
						pgrey[j] = 0;
					} else {
						pgrey[j] = 255;
					}
				}
			}
		} else {
			/// Threshold ROI using adaptive algorithm.
			cvtColor(frame_bgr, frame_grey, CV_BGR2GRAY);
			input->remapper->apply(frame_grey, remap_grey);

#if EXTRA_DEBUG_WINDOWS
			imshow("roi", remap_grey);
#endif // EXTRA_DEBUG_WINDOWS

			// pre-processing
			//blur(remap_grey, remap_blur, Size(3,3), Point(-1,-1), BORDER_REPLICATE);
			medianBlur(remap_grey, remap_blur, 3);

			thresh_min.setTo(Scalar::all(255));
			thresh_max.setTo(Scalar::all(0));

#if 1
			/** cached min/max **/
			// pre-fill first col
			uint8_t max = 0, min = 255;
			for( int i = 0; i < thresh_rad; i++ ) {		// last row is computed in next block (before testing)
				max = 0; min = 255;
				uint8_t* pmask = remap_mask.ptr(i);
				uint8_t* pgrey = remap_blur.ptr(i);
				for( int j = 0; j <= thresh_rad; j++ ) {
					if( pmask[j] < 250 ) { continue; }
					uint8_t g = pgrey[j];
					if( (g > max) && (g < 255) ) { max = g; }	// ignore overexposed regions
					if( g < min ) { min = g; }
				}
				win_max_hist[win_it++] = max;
				win_min_hist[win_it++] = min;
			}

			// compute window min/max
			uint8_t* pthrmax = thresh_max.data;
			uint8_t* pthrmin = thresh_min.data;
			for( int j = 0; j < rwidth; j++ ) {
				for( int i = 0; i < rheight; i++ ) {
					// add row
					max = 0; min = 255;
					if( (i+thresh_rad) < rheight ) {
						uint8_t* pmask = remap_mask.ptr(i+thresh_rad);
						uint8_t* pgrey = remap_blur.ptr(i+thresh_rad);
						for( int s = -thresh_rad; s <= thresh_rad; s++ ) {
							int js = j+s;
							if( (js < 0) || (js >= rwidth) ) { continue; }
							if( pmask[js] < 250 ) { continue; }
							uint8_t g = pgrey[js];
							if( (g > max) && (g < 255) ) { max = g; }	// ignore overexposed regions
							if( g < min ) { min = g; }
						}
					} else {
						// pre-fill next cols
						uint8_t* pmask = remap_mask.ptr(i+thresh_rad-rheight);
						uint8_t* pgrey = remap_blur.ptr(i+thresh_rad-rheight);
						for( int s = -thresh_rad; s <= thresh_rad; s++ ) {
							int js = j+s+1;
							if( (js < 0) || (js >= rwidth) ) { continue; }
							if( pmask[js] < 250 ) { continue; }
							uint8_t g = pgrey[js];
							if( (g > max) && (g < 255) ) { max = g; }	// ignore overexposed regions
							if( g < min ) { min = g; }
						}
					}
					win_max_hist[win_it] = max;
					win_min_hist[win_it] = min;

					// find window max/min
					max = 0; min =255;
					for( int k = 0; k < thresh_win; k++ ) {
						int ik = i+thresh_rad-k;
						if( (ik >= rheight) || (ik < 0) ) { continue; }
						int wk = win_it-k;
						if( wk < 0 ) { wk += thresh_win; }
						uint8_t mx = win_max_hist[wk];
						if( mx > max ) { max = mx; }
						uint8_t mn = win_min_hist[wk];
						if( mn < min ) { min = mn; }
					}
					pthrmax[i*thresh_max.step+j] = max;
					pthrmin[i*thresh_min.step+j] = min;
					if( ++win_it >= thresh_win ) { win_it -= thresh_win; }
				}
			}
#else
			/** naive min/max **/
			for( int i = 0; i < rheight; i++ ) {
				uint8_t* pmask = remap_mask.ptr(i);
				uint8_t* pthrmin = thresh_min.ptr(i);
				uint8_t* pthrmax = thresh_max.ptr(i);

				// handle borders
				int y = i-thresh_rad;
				int thresh_win_y = thresh_win;
				if( y < 0 ) {
					thresh_win_y += y;
					y = 0;
				} else if( y > thresh_max_y ) {
					thresh_win_y += thresh_max_y-y;
					y = thresh_max_y;
				}
				for( int j = 0; j < rwidth; j++ ) {
					if( pmask[j] < 250 ) { continue; }

					// handle borders
					int x = j-thresh_rad;
					int thresh_win_x = thresh_win;
					if( x < 0 ) {
						thresh_win_x += x;
						x = 0;
					} else if( x > thresh_max_x ) {
						thresh_win_x += thresh_max_x-x;
						x = thresh_max_x;
					}

					Mat thresh_roi = remap_blur(Rect(x,y,thresh_win_x,thresh_win_y));
					Mat mask_roi = remap_mask(Rect(x,y,thresh_win_x,thresh_win_y));
					for( int s = 0; s < thresh_win_y; s++ ) {
						uint8_t* ppmask = mask_roi.ptr(s);
						uint8_t* ppthresh = thresh_roi.ptr(s);
						for( int t = 0; t < thresh_win_x; t++ ) {
							if( ppmask[t] < 250 ) { continue; }
							uint8_t v = ppthresh[t];
							if( v < pthrmin[j] ) { pthrmin[j] = v; }
							if( v > pthrmax[j] ) { pthrmax[j] = v; }
						}
					}
				}
			}
#endif
			for( int i = 0; i < rheight; i++ ) {
				uint8_t* pmask = remap_mask.ptr(i);
				uint8_t* premap = remap_grey.ptr(i);
				uint8_t* pthrmin = thresh_min.ptr(i);
				uint8_t* pthrmax = thresh_max.ptr(i);
				for( int j = 0; j < rwidth; j++ ) {
					if( pmask[j] < 250 ) {
						premap[j] = 128;
						continue;
					}
					if( (thresh_ratio*(premap[j]-pthrmin[j])) <= (pthrmax[j]-premap[j]) ) {
						premap[j] = 0;
					} else {
						premap[j] = 255;
					}
				}
			}
		}

//		double t2 = Utils::GET_CLOCK();
//		printf("  %s loop time: %.1fms\n", __func__, (t2-t1)*1e3);

		/// Segmentation.
//		{
//			static int hist[256];
//			memset(hist, 0, 256*sizeof(int));
//			int cnt = 0;
//			for( int i = 0; i < rheight; i+=2 ) {
//				uint8_t* pmask = thresh_mask.ptr(i);
//				uint8_t* premap = thresh_remap.ptr(i);
//				for( int j = 0; j < rwidth; j+=2 ) {
//					if( pmask[j] < 255 ) { continue; }
//					hist[premap[j]]++;
//					cnt++;
//				}
//			}
//			int lo_pc = round(0.25*cnt), hi_pc = round(0.75*cnt);
//			uint8_t lo = 0, hi = 255;
//			int sum = 0;
//			for( int i = 0; i < 256; i++ ) {
//				sum += hist[i];
//				if( (lo == 0) && (sum >= lo_pc) ) { lo = i; }
//				if( (hi == 255) && (sum >= hi_pc) ) { hi = i; }
//			}
//			for( int i = 0; i < rheight; i++ ) {
//				uint8_t* pmask = thresh_mask.ptr(i);
//				uint8_t* premap = thresh_remap.ptr(i);
//				for( int j = 0; j < rwidth; j++ ) {
//					if( pmask[j] < 255 ) {
//						premap[j] = 128;
//					} else {
//						premap[j] = Maths::CLAMP((premap[j]+64-lo)*192.0/(hi+64-lo)+0.5, 0.0, 255.0);
//					}
//				}
//			}
//
//			int adapt_win = 65;
//			adapt_win |= 1;
//			adaptiveThreshold(thresh_remap, thresh_remap, 255,
//					ADAPTIVE_THRESH_GAUSSIAN_C,
//					THRESH_BINARY,
//					adapt_win, -10);
//
//			namedWindow("thresh_debug");
//			imshow("thresh_debug", thresh_remap);
//			waitKey(0);
//
//			resize(thresh_remap, remap, remap.size());
//			GaussianBlur(remap, remap, cvSize(3, 3), 0);
//		}

		#if EXTRA_DEBUG_WINDOWS
			imshow("thresh", remap_grey);
		#endif // EXTRA_DEBUG_WINDOWS

		pthread_mutex_lock(&(input->mutex));
		input->frame_q.push_back(frame_bgr);
		input->remap_q.push_back(remap_grey);
		input->timestamp_q.push_back(timestamp);
		pthread_cond_broadcast(&(input->cond));
		pthread_mutex_unlock(&(input->mutex));

		#if LOG_TIMING
			double t2 = Utils::GET_CLOCK();
			SEGMENT_TIME = (t2-t1);
		#endif
	}
	printf("%s terminated\n", __func__);
	return NULL;
}

bool getInputFrames(boost::shared_ptr<s_input> input, Mat& frame, Mat& remap, double& timestamp)
{
	pthread_mutex_lock(&input->mutex);
	while( ACTIVE && input->frame_q.empty() ) {
		pthread_cond_wait(&input->cond, &input->mutex);
	}
	if( !ACTIVE ) {
		pthread_mutex_unlock(&input->mutex);
		return false;
	}
	frame = input->frame_q.front();
	remap = input->remap_q.front();
	timestamp = input->timestamp_q.front();
	input->frame_q.pop_front();
	input->remap_q.pop_front();
	input->timestamp_q.pop_front();
	pthread_cond_broadcast(&(input->cond));
	pthread_mutex_unlock(&input->mutex);
	return true;
}

struct s_socket {
	pthread_t thread;
	pthread_mutex_t	mutex;
	int portno;
	int frameno;
	double px, py, vx, vy, heading;
	int state;
	int sockfd;
};

//FIXME: make continuous stream to avoid open/close overhead each frame?
void* socketListener(void* params)
{
	Utils::SET_PROCESS_PRIORITY(-10);

	s_socket* _socket = (s_socket*)params;
	const int buflen = 128;
	char buffer[128];
	struct sockaddr_in serv_addr, cli_addr;
	socklen_t len = sizeof(sockaddr_in);
	_socket->sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if( _socket->sockfd < 0 ) {
		fprintf(stderr, "ERROR: opening socket!\n");
		fflush(stderr);
		_socket->state = -1;
		return NULL;
	}
	fcntl(_socket->sockfd, F_SETFL, O_NONBLOCK);
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(0);
	if( bind(_socket->sockfd, (struct sockaddr *)&serv_addr, len) < 0 ) {
		fprintf(stderr, "ERROR: binding socket!\n");
		fflush(stderr);
		_socket->state = -1;
		return NULL;
	}

	if( listen(_socket->sockfd, 1) < 0 ) {
		fprintf(stderr, "ERROR: listening on socket!\n");
		fflush(stderr);
		_socket->state = -1;
		return NULL;
	}
	getsockname(_socket->sockfd, (struct sockaddr *)&serv_addr, &len);
	_socket->portno = ntohs(serv_addr.sin_port);
	printf("** SOCKET BOUND ON PORT %d **\n", _socket->portno);
	fflush(stdout);
	_socket->state = 1;

	while( ACTIVE ) {
		int newsockfd = -1;
		while( ACTIVE && newsockfd < 0 ) {
			fd_set rfds;
			FD_ZERO(&rfds);
			FD_SET(_socket->sockfd, &rfds);
			timeval tv;
			tv.tv_sec = 0;
			tv.tv_usec = 100000;
			int retval = select(_socket->sockfd+1, &rfds, NULL, NULL, &tv);
			if( retval > 0 ) {
				newsockfd = accept(_socket->sockfd, (struct sockaddr *) &cli_addr, &len);
			}
		}
		if( !ACTIVE ) { close(newsockfd); break; }

		// do something
		pthread_mutex_lock(&_socket->mutex);
		int frameno = _socket->frameno;
		double px = _socket->px;
		double py = _socket->py;
		double vx = _socket->vx;
		double vy = _socket->vy;
		double heading = _socket->heading;
		pthread_mutex_unlock(&_socket->mutex);
		bzero(buffer,buflen);
		int bytes = sprintf(buffer, "%d %.4f %.4f %.4f %.4f %.4f", frameno, px, py, vx, vy, heading);
		int n = write(newsockfd, buffer, bytes);	// FIXME: write only max buflen bytes
		if( n < bytes ) {
			fprintf(stderr, "ERROR: writing to socket!\n");
			fflush(stderr);
		}
		close(newsockfd);
		newsockfd = -1;
	}
	close(_socket->sockfd);
	printf("%s terminated\n", __func__);
	fflush(stdout);
	return NULL;
}

struct s_logger {
	string fn;
	deque<string> msgq;
	pthread_t thread;
	pthread_mutex_t	mutex;
	pthread_cond_t cond;
	int state;
};

void* logger(void* params)
{
	Utils::SET_PROCESS_PRIORITY(-2);

	s_logger* _log = (s_logger*)params;
	ofstream ofs(_log->fn.c_str(), ofstream::out);
	if( !ofs.is_open() ) {
		fprintf(stderr, "ERROR: opening output data file (%s)!\n", _log->fn.c_str());
		fflush(stderr);
		_log->state = -1;
		return NULL;
	}
	ofs.precision(14);
	printf("Output data file: %s\n", _log->fn.c_str());
	_log->state = 1;

	while( ACTIVE ) {
		pthread_mutex_lock(&_log->mutex);
		while( ACTIVE && _log->msgq.empty() ) {
			pthread_cond_wait(&_log->cond, &_log->mutex);
		}
		if( !ACTIVE ) {
			pthread_mutex_unlock(&_log->mutex);
			break;
		}
		string msg = _log->msgq.front();
		_log->msgq.pop_front();
		pthread_mutex_unlock(&_log->mutex);

		ofs << msg << endl;
	}
	ofs.close();
	printf("%s terminated\n", __func__);
	fflush(stdout);
	return NULL;
}

int main(int argc, char *argv[])
{
	///
	/// LICENSE
	///
	{
		printf("\n\n\n#######################################################################\n");
		printf("#                                                                     #\n");
		printf("# FicTrac: A webcam-based method for generating fictive animal paths. #\n");
		printf("# Version: public release (build %s)                          #\n", BUILD_STRING);
		printf("# Copyright (C) 2011-2015 Richard Moore (rjdmoore@uqconnect.edu)      #\n");
		printf("#                                                                     #\n");
		printf("#######################################################################\n");
		printf("#                                                                     #\n");
		printf("# This software uses libraries from the following projects:           #\n");
		printf("#     - OpenCV (http://opencv.org)                                    #\n");
		printf("#     - Cairo (http://www.cairographics.org)                          #\n");
		printf("#     - NLopt (http://ab-initio.mit.edu/nlopt/)                       #\n");
		printf("# These libraries are not distributed with this release but are       #\n");
		printf("# freely available as open-source projects.                           #\n");
		printf("#                                                                     #\n");
		printf("# This work is licensed under the Creative Commons                    #\n");
		printf("# Attribution-NonCommercial-ShareAlike 3.0 Unported License.          #\n");
		printf("# To view a copy of this license, visit                               #\n");
		printf("# http://creativecommons.org/licenses/by-nc-sa/3.0/                   #\n");
		printf("#                                                                     #\n");
		printf("# THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY           #\n");
		printf("# KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          #\n");
		printf("# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR             #\n");
		printf("# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR       #\n");
		printf("# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         #\n");
		printf("# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,     #\n");
		printf("# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE      #\n");
		printf("# USE OR OTHER DEALINGS IN THE SOFTWARE.                              #\n");
		printf("#                                                                     #\n");
		printf("#######################################################################\n\n\n");
		fflush(stdout);
		sleep(1);
	}

	// Setup OpenCV drawing thread.
	startWindowThread();

	// Catch cntl-c
	signal(SIGINT, TERMINATE);

	// Set high priority (when run as SU)
	Utils::SET_PROCESS_PRIORITY(-15);

	///
	/// LOAD PROGRAM VARIABLES
	///

	printf("Reading configuration file...\n");
	fflush(stdout);

	string input_vid_fn = "", mask_fn = "", transform_fn = "", template_fn = "";
	string output_fn = "", config_img_fn = "";
	string closed_loop_fn = "";
	string debug_video_fn = "", frames_video_fn = "";
	int fps = -1, cam_index = 0;
	unsigned int frame_skip = 0, frame_step = 1;
	bool do_display = true, do_config = false, do_search = true;
	bool save_video = false, save_input_video = false, load_template = true, no_prompts = false;
	bool do_update = true;
	bool fisheye = false, cam_input = false, use_ball_colour = false;
	double vfov = 45*Maths::D2R;
	int quality_factor = 6;
	int remap_width = 60, remap_height = 60;
	int sphere_width = 180, sphere_height = 90;
	double nlopt_ftol = 1e-3, error_thresh = 6000;
	int nlopt_max_eval = 100, max_bad_frames = 0;
	double nlopt_res = 0.5;
	double thresh_win = 0.2;
	double thresh_ratio = 1.25;
	bool do_led_display = false, do_socket_out = false;
	BAYER_TYPE bayer_type = BAYER_NONE;
	double sphere_orient[3] = {0,0,0};
	bool force_draw_config = false;
    	string fmf_save = "";

	enum CAM_MODEL_TYPE m_cam_model = RECTILINEAR;

	// serial port
	serial _serial;
	bool do_serial_out = false;
	bool output_position= false;  //added by Pablo 07/2014
	int serial_baud = 115200;
	string serial_port = "/dev/ttyS0";

	// read in config file
	if( argc <= 1 ) {
		printf("Input config file required (invocation: ./fictrac /path/to/config.txt)!\n");
		fflush(stdout);
		exit(-1);
	}
	std::ifstream file(argv[1],std::ifstream::in);
	if( !file.is_open() ) {
		fprintf(stderr, "%s: Error, unable to open file (%s)!\n", __func__, argv[1]);
		exit(-1);
	}

	string line;
	deque<string> tokens;
	getline(file, line);
	while(!file.eof()) {
		Utils::TOKENISE(line, tokens);
		if( tokens.size() > 1 ) {
			if( tokens.front().compare("input_vid_fn") == 0 ) {
				tokens.pop_front();
				input_vid_fn = tokens.front();
			} else if( tokens.front().compare("output_fn") == 0 ) {
				tokens.pop_front();
				output_fn = tokens.front();
			} else if( tokens.front().compare("mask_fn") == 0 ) {
				tokens.pop_front();
				mask_fn = tokens.front();
			} else if( tokens.front().compare("transform_fn") == 0 ) {
				tokens.pop_front();
				transform_fn = tokens.front();
			} else if( tokens.front().compare("template_fn") == 0 ) {
				tokens.pop_front();
				template_fn = tokens.front();
			} else if( tokens.front().compare("closed_loop_fn") == 0 ) {
				tokens.pop_front();
				closed_loop_fn = tokens.front();
			} else if( tokens.front().compare("frame_skip") == 0 ) {
				tokens.pop_front();
				frame_skip = atoi(tokens.front().c_str());
			} else if( tokens.front().compare("frame_step") == 0 ) {
				tokens.pop_front();
				frame_step = max(atoi(tokens.front().c_str()), 1);
			} else if( tokens.front().compare("do_display") == 0 ) {
				tokens.pop_front();
				do_display = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("no_prompts") == 0 ) {
				tokens.pop_front();
				no_prompts = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("do_config") == 0 ) {
				tokens.pop_front();
				do_config = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("save_video") == 0 ) {
				tokens.pop_front();
				save_video = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("save_input_video") == 0 ) {
				tokens.pop_front();
				save_input_video = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("do_search") == 0 ) {
				tokens.pop_front();
				do_search = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("do_update") == 0 ) {
				tokens.pop_front();
				do_update = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("load_template") == 0 ) {
				tokens.pop_front();
				load_template = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("fisheye") == 0 ) {
				tokens.pop_front();
				fisheye = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("cam_input") == 0 ) {
				tokens.pop_front();
				cam_input = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("fps") == 0 ) {
				tokens.pop_front();
				fps = atoi(tokens.front().c_str());
			} else if( tokens.front().compare("cam_index") == 0 ) {
				tokens.pop_front();
				cam_index = atoi(tokens.front().c_str());
			} else if( tokens.front().compare("use_ball_colour") == 0 ) {
				tokens.pop_front();
				use_ball_colour = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("vfov") == 0 ) {
				tokens.pop_front();
				vfov = Utils::STR2NUM(tokens.front())*Maths::D2R;
			} else if( tokens.front().compare("quality_factor") == 0 ) {
				tokens.pop_front();
				quality_factor = atoi(tokens.front().c_str());
			} else if( tokens.front().compare("nlopt_ftol") == 0 ) {
				tokens.pop_front();
				nlopt_ftol = Utils::STR2NUM(tokens.front());
			} else if( tokens.front().compare("nlopt_max_eval") == 0 ) {
				tokens.pop_front();
				nlopt_max_eval = atoi(tokens.front().c_str());
			} else if( tokens.front().compare("error_thresh") == 0 ) {
				tokens.pop_front();
				error_thresh = Utils::STR2NUM(tokens.front());
			} else if( tokens.front().compare("thresh_win") == 0 ) {
				tokens.pop_front();
				thresh_win = Utils::STR2NUM(tokens.front());
			} else if( tokens.front().compare("thresh_ratio") == 0 ) {
				tokens.pop_front();
				thresh_ratio = Utils::STR2NUM(tokens.front());
			} else if( tokens.front().compare("max_bad_frames") == 0 ) {
				tokens.pop_front();
				max_bad_frames = atoi(tokens.front().c_str());
			} else if( tokens.front().compare("do_serial_out") == 0 ) {
				tokens.pop_front();
				do_serial_out = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("serial_baud") == 0 ) {
				tokens.pop_front();
				serial_baud = atoi(tokens.front().c_str());
			} else if( tokens.front().compare("serial_port") == 0 ) {
				tokens.pop_front();
				serial_port = tokens.front();
			} else if( tokens.front().compare("do_led_display") == 0 ) {
				tokens.pop_front();
				do_led_display = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("do_socket_out") == 0 ) {
				tokens.pop_front();
				do_socket_out = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("sphere_orient") == 0 ) {
						tokens.pop_front();
						sphere_orient[0] = atof(tokens.front().c_str());
						tokens.pop_front();
						sphere_orient[1] = atof(tokens.front().c_str());
						tokens.pop_front();
						sphere_orient[2] = atof(tokens.front().c_str());
			} else if( tokens.front().compare("bayer_type") == 0 ) {
				tokens.pop_front();
				int tmp = atoi(tokens.front().c_str());
				switch( tmp ) {
					case BAYER_BGGR:
						bayer_type = BAYER_BGGR;
						break;
					case BAYER_GBRG:
						bayer_type = BAYER_GBRG;
						break;
					case BAYER_GRBG:
						bayer_type = BAYER_GRBG;
						break;
					case BAYER_RGGB:
						bayer_type = BAYER_RGGB;
						break;
					default:
						printf("Unknown BAYER_TYPE (%d)! Defaulting to BAYER_NONE.\n", tmp);
						bayer_type = BAYER_NONE;
						break;
				}
			} else if( tokens.front().compare("force_draw_config") == 0 ) {
				tokens.pop_front();
				force_draw_config = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("output_position") == 0 ) {  //else if clause added by Pablo on 07/2014
				tokens.pop_front();
				output_position = bool(atoi(tokens.front().c_str()));
			} else if( tokens.front().compare("fmf_save") == 0) {
                tokens.pop_front();
                fmf_save = tokens.front();
            }
		}
		// ignore the remainder of the line
		getline(file, line);
	}

	if( !cam_input && input_vid_fn.empty() ) {
		fprintf(stderr, "ERROR: input_vid_fn must be specified in config file!\n");
		fflush(stderr);
		exit(-1);
	}

	if( cam_input && output_fn.empty() ) {
		fprintf(stderr, "ERROR: output_fn must be specified in config file!\n");
		fflush(stderr);
		exit(-1);
	}

	if( !cam_input && output_fn.empty() && !input_vid_fn.empty() ) {
		output_fn = input_vid_fn.substr(0, input_vid_fn.length()-4)+".dat";
	}

	if( output_fn.empty() ) {
		fprintf(stderr, "ERROR: no output_fn specified!\n");
		fflush(stderr);
		exit(-1);
	}

	if( (thresh_win <= 0) || (thresh_win >= 1) ) {
		fprintf(stderr, "ERROR: thesh_win is now defined as a %% of the ROI window and must be within the range (0,1)!\n");
		fflush(stderr);
		exit(-1);
	}

	if( mask_fn.empty() ) {
		mask_fn = output_fn.substr(0, output_fn.length()-4)+"-mask.jpg";
	}
	if( transform_fn.empty() ) {
		transform_fn = output_fn.substr(0, output_fn.length()-4)+"-transform.dat";
	}
	if( template_fn.empty() ) {
		template_fn = output_fn.substr(0, output_fn.length()-4)+"-template.jpg";
	}
	if( debug_video_fn.empty() ) {
		debug_video_fn = output_fn.substr(0, output_fn.length()-4)+"-debug";
	}
	if( frames_video_fn.empty() ) {
		frames_video_fn = output_fn.substr(0, output_fn.length()-4)+"-raw_frames";
	}

	config_img_fn = output_fn.substr(0, output_fn.length()-4)+"-configImg.jpg";

	if( quality_factor > 0 ) {
		remap_width = remap_height = 10*quality_factor;
		sphere_height = 1.5*remap_width;
		sphere_width = 2*sphere_height;
	}

	if( fisheye ) {
		m_cam_model = FISHEYE;
	}

	printf("\nInitialising program variables:\n");
	printf("input_vid_fn: .  .  '%s'\n", input_vid_fn.c_str());
	printf("output_fn: .  .  .  '%s'\n", output_fn.c_str());
	printf("mask_fn:.  .  .  .  '%s'\n", mask_fn.c_str());
	printf("transform_fn: .  .  '%s'\n", transform_fn.c_str());
	printf("template_fn:  .  .  '%s'\n", template_fn.c_str());
	printf("closed_loop_fn:  .  '%s'\n", closed_loop_fn.c_str());
	printf("debug_video_fn:  .  '%s'\n", debug_video_fn.c_str());
	printf("frames_video_fn: .  '%s'\n", frames_video_fn.c_str());
   	printf("fmf_save: .   .  .  '%s'\n", fmf_save.c_str());
	printf("frame_skip:.  .  .  %d\n", frame_skip);
	printf("frame_step:.  .  .  %d\n", frame_step);
	printf("do_display:.  .  .  %d\n", do_display);
	printf("no_prompts:.  .  .  %d\n", no_prompts);
	printf("do_config: .  .  .  %d\n", do_config);
	printf("save_video:.  .  .  %d\n", save_video);
	printf("save_input_video:.  %d\n", save_input_video);
	printf("do_search: .  .  .  %d\n", do_search);
	printf("do_update: .  .  .  %d\n", do_update);
	printf("load_template:.  .  %d\n", load_template);
	printf("fisheye:.  .  .  .  %d\n", fisheye);
	printf("cam_input: .  .  .  %d\n", cam_input);
	printf("fps: .  .  .  .  .  %d\n", fps);
	printf("cam_index: .  .  .  %d\n", cam_index);
	printf("use_ball_colour: .  %d\n", use_ball_colour);
	printf("vfov:.  .  .  .  .  %f\n", vfov*Maths::R2D);
	printf("quality_factor:  .  %d\n", quality_factor);
	printf("remap_width:  .  .  %d\n", remap_width);
	printf("remap_height: .  .  %d\n", remap_height);
	printf("sphere_width: .  .  %d\n", sphere_width);
	printf("sphere_height:.  .  %d\n", sphere_height);
	printf("nlopt_ftol:.  .  .  %f\n", nlopt_ftol);
	printf("nlopt_max_eval:  .  %d\n", nlopt_max_eval);
	printf("error_thresh: .  .  %f\n", error_thresh);
	printf("thresh_win:.  .  .  %f (%d px)\n", thresh_win, (int)round(thresh_win*remap_width) | 0x01);
	printf("thresh_ratio: .  .  %f\n", thresh_ratio);
	printf("do_serial_out:.  .  %d\n", do_serial_out);
	printf("serial_baud:  .  .  %d\n", serial_baud);
	printf("serial_port:  .  .  '%s'\n", serial_port.c_str());
	printf("do_led_display:  .  %d\n", do_led_display);
	printf("do_socket_out:.  .  %d\n", do_socket_out);
	printf("sphere_orient:.  .  %f %f %f\n",
			sphere_orient[0], sphere_orient[1], sphere_orient[2]);
	printf("bayer_type:.  .  .  %d\n", bayer_type);
	printf("force_draw_config:  %d\n", force_draw_config);
	printf("\n");

	fflush(stdout);
	if( !no_prompts ) {
		printf("\n  Press ENTER to continue...\n");
		fflush(stdout);
		getchar();
	}

	///
	/// INPUT
	///

	boost::shared_ptr<ImgSource> cap;
	if( cam_input ) {
		// input camera
		#ifdef PGR_CAMERA
			cap = boost::shared_ptr<PGRSource>(new PGRSource(cam_index));
		#else
			cap = boost::shared_ptr<CVSource>(new CVSource(cam_index));
		#endif

		// try to set fps
		cap->setFPS(fps);

		// set bayer mode
		cap->setBayerType(bayer_type);
	} else {
		// input video
		cap = boost::shared_ptr<CVSource>(new CVSource(input_vid_fn));
	}
	if( !cap->isOpen() ) {
		fprintf(stderr, "ERROR: opening capture device!\n");
		fflush(stderr);
		exit(-1);
	}
	int width = cap->getWidth();
	int height = cap->getHeight();

	printf("input image size: %d x %d\n", width, height);

	//////////////////////////////////////////////////////////////////////////////////////
	///                                                                                ///
	/// ESTIMATE CAMERA HOMOGRAPHY & SPHERE MODEL                                      ///
	///                                                                                ///
	//////////////////////////////////////////////////////////////////////////////////////

	double sphere_centre[3] = {0};
	double sphere_fov = 0;
	double Rw[9] = {0};
	double Tw[3] = {0};
	CmPoint32f fg_yuv(0,0,0);
	CmPoint32f bg_yuv(0,0,0);

	/// Attempt to load config params from file.
	ifstream ifs(transform_fn, ifstream::in);
	if( !do_config && !ifs.good() ) {
		fprintf(stderr, "ERROR: opening transform file (%s)!\n",
				transform_fn.c_str());
		fflush(stderr);
		do_config = true;
	}

	if( do_config ) {

		Mat frame_bgr;
		for( unsigned int i = 0; i <= frame_skip; i++ ) {
			if( !cap->grab(frame_bgr) ) {
				fprintf(stderr, "ERROR: grabbing camera frame!\n");
				fflush(stderr);
				exit(-1);
			}
		}

		printf("\nInitiating configuration procedure..\n");

		///
		/// Initiate configuration procedure.
		///

		CameraModelPtr cam_model;
		if( fisheye ) {
			cam_model = CameraModel::createFisheye(
					width, height, vfov/height, 360*Maths::D2R);
		} else {
			cam_model = CameraModel::createRectilinear(width, height, vfov);
		}

		// display
		namedWindow("FicTrac-config");
		VsDraw::Ptr vsdraw = VsDraw::Ptr(new VsDraw);
		vsdraw->openImage(cam_model, frame_bgr);
		vsdraw->grey(255);
		vsdraw->thickness(1.5);
		vsdraw->display("FicTrac-config");
		vector<Point> click;
		cvSetMouseCallback("FicTrac-config", mouseCallback, &click);

		// zoomed sphere roi for precision
		VsDraw::Ptr vsdraw_zoom = VsDraw::Ptr(new VsDraw);
		CameraModelPtr zoom_model = CameraModel::createFisheye(
				400, 400, vfov/400.0, 360*Maths::D2R);

		// rotation from forward
		double omega[3] = {0};
		Maths::ANGLE_AXIS_FROM_VEC(sphere_centre, omega);
		CmPoint32f omegav(omega[0], omega[1], omega[2]);
		RemapTransformPtr roi_transform = MatrixRemapTransform::createFromOmega(omegav);
		CameraRemapPtr zoom_remapper = CameraRemapPtr(
				new CameraRemap(cam_model, zoom_model, roi_transform));
		Mat zoom_bgr(400,400,CV_8UC3);

		Mat save_img(height, width, CV_8UC3);

		// select sphere roi
		printf("\n  Select the centre of the sphere's ROI.\n");
		fflush(stdout);
		click.clear();
		int roi_state = 0, roi_state_cnt = 0;
		bool roi_done = false;
		double sphere_cx = 0, sphere_cy = 0;
		while( !roi_done ) {
			switch( roi_state ) {
				// select sphere centre
				case 0:
				{
					waitKey(10);
					if( click.size() > 0 ) {
						if( click[0].x < 0 || click[0].y < 0 ) { click.clear(); continue; }
						sphere_cx = click[0].x;
						sphere_cy = click[0].y;
						click.clear();
						cam_model->pixelIndexToVector(sphere_cx, sphere_cy, sphere_centre);
						Maths::NORMALISE_VEC(sphere_centre);
						CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
						vsdraw->cross(axis, -4);
						vsdraw->display("FicTrac-config");
						roi_state++;

						printf("\n  Select the radius of the sphere's ROI.\n");
						fflush(stdout);
					}
					break;
				}

				// select sphere radius
				case 1:
				{
					waitKey(10);
					if( click.size() > 0 ) {
						if( click[0].x < 0 || click[0].y < 0 ) { click.clear(); continue; }
						Point sc = click.front();
						click.clear();
						double sphere_circum[3] = {0};
						cam_model->pixelIndexToVector(sc.x, sc.y, sphere_circum);
						Maths::NORMALISE_VEC(sphere_circum);
						sphere_fov = acos(Maths::DOT_VEC(sphere_centre, sphere_circum))*2.0;
						printf("sphere FoV: %.2f degrees\n", sphere_fov*Maths::R2D);
						CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
						vsdraw->alpha(0.4);
						vsdraw->circle(axis, sphere_fov/2.0);
						vsdraw->display("FicTrac-config");
						roi_state++;

						printf("\n  Adjust the centre and radius using the arrow keys and +/- respectively.\n  Press ENTER when done.\n\n");
						fflush(stdout);
					}
					break;
				}

				// adjust centre/radius
				case 2:
				{
					uint16_t key = waitKey(50);
					switch( key ) {
						case 0xFF51:
							sphere_cx -= 0.1;
							cam_model->pixelIndexToVector(sphere_cx, sphere_cy, sphere_centre);
							Maths::NORMALISE_VEC(sphere_centre);
							{
								CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
								vsdraw->openImage(cam_model, frame_bgr);
								vsdraw->grey(255);
								vsdraw->thickness(1.5);
								vsdraw->cross(axis, -8);
								vsdraw->alpha(0.4);
								vsdraw->circle(axis, sphere_fov/2.0);
								vsdraw->display("FicTrac-config");
							}
							break;
						case 0xFF52:
							sphere_cy -= 0.1;
							cam_model->pixelIndexToVector(sphere_cx, sphere_cy, sphere_centre);
							Maths::NORMALISE_VEC(sphere_centre);
							{
								CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
								vsdraw->openImage(cam_model, frame_bgr);
								vsdraw->grey(255);
								vsdraw->thickness(1.5);
								vsdraw->cross(axis, -8);
								vsdraw->alpha(0.4);
								vsdraw->circle(axis, sphere_fov/2.0);
								vsdraw->display("FicTrac-config");
							}
							break;
						case 0xFF53:
							sphere_cx += 0.1;
							cam_model->pixelIndexToVector(sphere_cx, sphere_cy, sphere_centre);
							Maths::NORMALISE_VEC(sphere_centre);
							{
								CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
								vsdraw->openImage(cam_model, frame_bgr);
								vsdraw->grey(255);
								vsdraw->thickness(1.5);
								vsdraw->cross(axis, -8);
								vsdraw->alpha(0.4);
								vsdraw->circle(axis, sphere_fov/2.0);
								vsdraw->display("FicTrac-config");
							}
							break;
						case 0xFF54:
							sphere_cy += 0.1;
							cam_model->pixelIndexToVector(sphere_cx, sphere_cy, sphere_centre);
							Maths::NORMALISE_VEC(sphere_centre);
							{
								CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
								vsdraw->openImage(cam_model, frame_bgr);
								vsdraw->grey(255);
								vsdraw->thickness(1.5);
								vsdraw->cross(axis, -8);
								vsdraw->alpha(0.4);
								vsdraw->circle(axis, sphere_fov/2.0);
								vsdraw->display("FicTrac-config");
							}
							break;
						case 0x2D:
							sphere_fov -= 0.1*(vfov*Maths::R2D/45.0)*Maths::D2R;
							{
								CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
								vsdraw->openImage(cam_model, frame_bgr);
								vsdraw->grey(255);
								vsdraw->thickness(1.5);
								vsdraw->cross(axis, -8);
								vsdraw->alpha(0.4);
								vsdraw->circle(axis, sphere_fov/2.0);
								vsdraw->display("FicTrac-config");
								printf("sphere FoV: %.1f degrees\n", sphere_fov*Maths::R2D);
							}
							break;
						case 0x3D:
							sphere_fov += 0.1*(vfov*Maths::R2D/45.0)*Maths::D2R;
							{
								CmPoint32f axis(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
								vsdraw->openImage(cam_model, frame_bgr);
								vsdraw->grey(255);
								vsdraw->thickness(1.5);
								vsdraw->cross(axis, -8);
								vsdraw->alpha(0.4);
								vsdraw->circle(axis, sphere_fov/2.0);
								vsdraw->display("FicTrac-config");
								printf("sphere FoV: %.1f degrees\n", sphere_fov*Maths::R2D);
							}
							break;
						case 0x0A:
						case 0x0D:
						case 0xFF8D:
							roi_done = true;
							break;
						default:
//							if( key != 0xFFFF ) { printf("key: %x\n", key); }	// TODO: remove
							break;
					}
					// update zoom image if key pressed
					if( key != 0xFFFF ) {
						// update zoom camera model
						zoom_model = CameraModel::createFisheye(
								400, 400, sphere_fov*1.25/400.0, 360*Maths::D2R);
						Maths::ANGLE_AXIS_FROM_VEC(sphere_centre, omega);
						omegav = CmPoint32f(omega[0], omega[1], omega[2]);
						roi_transform = MatrixRemapTransform::createFromOmega(omegav);
						zoom_remapper = CameraRemapPtr(
								new CameraRemap(cam_model, zoom_model, roi_transform));
						zoom_remapper->apply(frame_bgr, zoom_bgr);

						// display zoom image
						namedWindow("Zoom ROI");
						CmPoint32f axis(0,0,1);
						vsdraw_zoom->openImage(zoom_model, zoom_bgr);
						vsdraw_zoom->grey(255);
						vsdraw_zoom->thickness(1.5);
						vsdraw_zoom->cross(axis, -8);
						vsdraw_zoom->alpha(0.4);
						vsdraw_zoom->circle(axis, sphere_fov/2.0);
						vsdraw_zoom->display("Zoom ROI");
					}
					break;
				}

				default:
				{
					waitKey(10);
					break;
				}
			}
			roi_state_cnt++;
		}

		// destroy zoom window
		destroyWindow("Zoom ROI");

		printf("sphere ROI centre: (%.2f, %.2f, %.2f) radius: %.1f degrees\n",
				sphere_centre[0], sphere_centre[1], sphere_centre[2],
				sphere_fov*Maths::R2D/2.0);
		fflush(stdout);

		vector<Point3f> corners;	// fl fr br bl
		if( fisheye ) {
			// select fl, fr, br, bl corners
			printf("\n  Select each corner of the plate.\n  Begin with the FL, then FR, BR, and BL.\n\n");
			fflush(stdout);
			click.clear();
			unsigned int clicks = 0;
			while( 1 ) {
				if( click.size() > clicks ) {
					if( click.back().x < 0 || click.back().y < 0 ) { click.clear(); continue; }
					double vec[3] = {0};
					cam_model->pixelIndexToVector(click.back().x, click.back().y, vec);
					Maths::NORMALISE_VEC(vec);
					Point3f cnr(vec[0], vec[1], vec[2]);
					corners.push_back(cnr);
					CmPoint32f axis(vec[0], vec[1], vec[2]);
					vsdraw->point(axis, -4, false, true);
					vsdraw->display("FicTrac-config");
					clicks++;
				}
				if( clicks >= 4 ) { break; }
				waitKey(10);
			}
		} else {
			// select left, top, right, bottom sides
			printf("\n  Select 2 points on each side of the plate.\n  Begin with the left side, then front, right, and rear.\n\n");
			fflush(stdout);
			click.clear();
			unsigned int clicks = 0;
			while( 1 ) {
				if( click.size() > clicks ) {
					{
						double vec[3] = {0};
						cam_model->pixelIndexToVector(click.back().x, click.back().y, vec);
						CmPoint32f axis(vec[0], vec[1], vec[2]);
						vsdraw->point(axis, -4, false, false);
						vsdraw->display("FicTrac-config");
					}
					clicks++;
				}
				if( clicks >= 8 ) { break; }
				waitKey(10);
			}

			// draw edges
			for( int i = 0; i < 4; i++ ) {
				double vec1[3] = {0};
				cam_model->pixelIndexToVector(click[2*i+0].x, click[2*i+0].y, vec1);
				CmPoint32f axis1(vec1[0], vec1[1], vec1[2]);
				double vec2[3] = {0};
				cam_model->pixelIndexToVector(click[2*i+1].x, click[2*i+1].y, vec2);
				CmPoint32f axis2(vec2[0], vec2[1], vec2[2]);
				vsdraw->line(axis1, axis2);
			}
			vsdraw->display("FicTrac-config");

			// extract image corners
			click.push_back(click[0]);
			click.push_back(click[1]);
			printf("corners:\n");
			for( int i = 0; i < 4; i++ ) {
				double x1 = click[2*i+0].x;
				double y1 = click[2*i+0].y;
				double x2 = click[2*i+1].x;
				double y2 = click[2*i+1].y;
				double x3 = click[2*i+2].x;
				double y3 = click[2*i+2].y;
				double x4 = click[2*i+3].x;
				double y4 = click[2*i+3].y;
				double px = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
				double py = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));

				double vec[3] = {0};
				cam_model->pixelIndexToVector(px, py, vec);
				Maths::NORMALISE_VEC(vec);
				corners.push_back(Point3f(vec[0], vec[1], vec[2]));

//				printf("(%6.1f, %6.1f): %.1f %.1f %.1f\n",
//						px, py, vec[0], vec[1], vec[2]);
				printf("(%6.1f, %6.1f)\n", px, py);

				CmPoint32f axis(vec[0], vec[1], vec[2]);
				vsdraw->point(axis, -6, false, true);
			}
			vsdraw->display("FicTrac-config");
		}
		fflush(stdout);

		// select colours
		if( use_ball_colour ) {
			vsdraw->copyImage(save_img);
			printf("\n  Select foreground region colour.\n  Right mouse click to close polygon.\n\n");
			fflush(stdout);
			click.clear();
			unsigned int clicks = 0;
			while( 1 ) {
				if( click.size() > clicks ) {
					bool ending = false;
					if( click.back().x < 0 || click.back().y < 0 ) {
						click.pop_back();
						if( clicks < 3 ) {
							printf("\n Please select at least 3 points!\n");
							fflush(stdout);
							continue;
						}
						click.push_back(click.front());
						ending = true;
					}

					double vec[3] = {0};
					cam_model->pixelIndexToVector(click.back().x, click.back().y, vec);
					CmPoint32f axis(vec[0], vec[1], vec[2]);
					static CmPoint32f ppt = axis;
					vsdraw->point(axis, -1, false);
					if( clicks > 0 ) {
						vsdraw->line(ppt, axis, 1);
						ppt = axis;
					}
					vsdraw->display("FicTrac-config");

					if( ending ) { break; }
					clicks++;
				}
				waitKey(10);
			}
			Mat fg_mask(height, width, CV_8UC1);
			fg_mask.setTo(Scalar::all((0)));
			fillConvexPoly(fg_mask, &click[0], click.size(), CV_RGB(255,255,255), 8, 0);

			printf("\n  Select background region colour.\n  Right mouse click to close polygon.\n\n");
			fflush(stdout);
			click.clear();
			clicks = 0;
			while( 1 ) {
				if( click.size() > clicks ) {
					bool ending = false;
					if( click.back().x < 0 || click.back().y < 0 ) {
						click.pop_back();
						if( clicks < 3 ) {
							printf("\n Please select at least 3 points!\n");
							fflush(stdout);
							continue;
						}
						click.push_back(click.front());
						ending = true;
					}

					double vec[3] = {0};
					cam_model->pixelIndexToVector(click.back().x, click.back().y, vec);
					CmPoint32f axis(vec[0], vec[1], vec[2]);
					static CmPoint32f ppt = axis;
					vsdraw->point(axis, -1, false);
					if( clicks > 0 ) {
						vsdraw->line(ppt, axis, 1);
						ppt = axis;
					}
					vsdraw->display("FicTrac-config");

					if( ending ) { break; }
					clicks++;
				}
				waitKey(10);
			}
			Mat bg_mask(height, width, CV_8UC1);
			bg_mask.setTo(Scalar::all((0)));
			fillConvexPoly(bg_mask, &click[0], click.size(), CV_RGB(255,255,255), 8, 0);

			Mat frame_yuv;
			cvtColor(frame_bgr, frame_yuv, CV_BGR2YUV);

			vector<uint8_t> fgu, fgv, bgu, bgv;
			for( int i = 0; i < height; i++ ) {
				uint8_t* pfgmask = fg_mask.ptr(i);
				uint8_t* pbgmask = bg_mask.ptr(i);
				uint8_t* pframe = frame_yuv.ptr(i);
				for( int j = 0; j < width; j++ ) {
					if( pfgmask[j] == 255 ) {
						fgu.push_back(pframe[3*j+1]);
						fgv.push_back(pframe[3*j+2]);
					}
					if( pbgmask[j] == 255 ) {
						bgu.push_back(pframe[3*j+1]);
						bgv.push_back(pframe[3*j+2]);
					}
				}
			}

			std::sort(fgu.begin(), fgu.end());
			std::sort(fgv.begin(), fgv.end());
			std::sort(bgu.begin(), bgu.end());
			std::sort(bgv.begin(), bgv.end());

			fg_yuv.y = fgu[fgu.size()/2];
			fg_yuv.z = fgv[fgv.size()/2];
			bg_yuv.y = bgu[bgu.size()/2];
			bg_yuv.z = bgv[bgv.size()/2];

			printf("foreground colour: %.1f %.1f\nbackground colour: %.1f %.1f\n",
					fg_yuv.y, fg_yuv.z, bg_yuv.y, bg_yuv.z);

			vsdraw->openImage(cam_model, save_img);
			vsdraw->grey(255);
			vsdraw->thickness(1.5);
		}

		// Estimate transform from square bracket to camera
		// R is square to camera rotation matrix
		// T is translation to square centre in camera frame coordinates
		PlaneHomography plhomo(corners);

		double err = 0;
		double guess[6] = {0, 0, 1,
				Utils::GEN_RAND_DBL(-0.05, 0.05),
				Utils::GEN_RAND_DBL(-0.05, 0.05),
				Utils::GEN_RAND_DBL(-0.05, 0.05)};

		// save config image state
		vsdraw->copyImage(save_img);

		Mat axes_img;
		bool not_done = true;
		while( not_done ) {
			plhomo.optimize(guess);
			plhomo.getOptX(guess);
			err = plhomo.getOptF();

			printf("Minimised T: %.3f %.3f %.3f   R: %.3f %.3f %.3f   (%.4f)\n",
					guess[0], guess[1], guess[2], guess[3], guess[4], guess[5], err);

			// reload config image state
			vsdraw->openImage(cam_model, save_img);
			vsdraw->grey(255);
			vsdraw->thickness(1.5);

			// draw corners
			{
				double tl2[3], tr2[3], br2[3], bl2[3];
				getCornerVecs(guess, tl2, tr2, br2, bl2);

				printf("re-projected corners:\n");
				double px = 0, py = 0;
				cam_model->vectorToPixelIndex(tl2, px, py);
				printf("(%6.1f, %6.1f)\n", px, py);
				cam_model->vectorToPixelIndex(tr2, px, py);
				printf("(%6.1f, %6.1f)\n", px, py);
				cam_model->vectorToPixelIndex(br2, px, py);
				printf("(%6.1f, %6.1f)\n", px, py);
				cam_model->vectorToPixelIndex(bl2, px, py);
				printf("(%6.1f, %6.1f)\n", px, py);

				CmPoint32f axis(tl2[0], tl2[1], tl2[2]);
				vsdraw->cross(axis, -6, true);
				axis[0] = tr2[0]; axis[1] = tr2[1]; axis[2] = tr2[2];
				vsdraw->cross(axis, -6, true);
				axis[0] = br2[0]; axis[1] = br2[1]; axis[2] = br2[2];
				vsdraw->cross(axis, -6, true);
				axis[0] = bl2[0]; axis[1] = bl2[1]; axis[2] = bl2[2];
				vsdraw->cross(axis, -6, true);
			}

			// draw quad axes
			{
				double origin[3], xaxis[3], yaxis[3], zaxis[3];
				getAxesVecs(guess, origin, xaxis, yaxis, zaxis);
				CmPoint32f vo(origin[0], origin[1], origin[2]);
				CmPoint32f vx(xaxis[0], xaxis[1], xaxis[2]);
				CmPoint32f vy(yaxis[0], yaxis[1], yaxis[2]);
				CmPoint32f vz(zaxis[0], zaxis[1], zaxis[2]);

				vsdraw->line(vo, vz, 32, true);
				vsdraw->line(vo, vy, 32, true);
				vsdraw->line(vo, vx, 32, true);
				vsdraw->point(vo, -1, true);

				// return rgb image
				vsdraw->copyImage(axes_img);
				cvtColor(axes_img, axes_img, CV_BGR2RGB);

				double px = 0, py = 0;
				cam_model->vectorToPixelIndex(xaxis, px, py);
				drawText(axes_img, "x",
						px, py, 1, 255, 255, 255, true, 0);
				cam_model->vectorToPixelIndex(yaxis, px, py);
				drawText(axes_img, "y",
						px, py, 1, 255, 255, 255, true, 0);
				cam_model->vectorToPixelIndex(zaxis, px, py);
				drawText(axes_img, "z",
						px, py, 1, 255, 255, 255, true, 0);

				cvtColor(axes_img, axes_img, CV_RGB2BGR);
				imshow("FicTrac-config", axes_img);
			}

			printf("\n  Inspect the configuration image.\n\n  Check that the re-projected corners (crosses) match the projected corners (large circles),\n  and that the projected laboratory coordinate frame is correct.\n  The laboratory axes should form a proper right-handed coordinate frame.\n");
			printf("\n  To accept the current configuration, press ENTER,\n  otherwise use keys X Y Z to adjust the estimated transformation,\n  then press ENTER to re-run the minimisation.\n\n");

			fflush(stdout);

			double O[9];
			Maths::ANGLE_AXIS_TO_MAT(&(guess[3]), O);
			bool adjusting = true, adjusted = false, redraw = false;
			while( adjusting ) {
				if( redraw ) {
					// reload config image state
					vsdraw->openImage(cam_model, save_img);
					vsdraw->grey(255);
					vsdraw->thickness(1.5);

					// get current rotation guess
					double r[3];
					Maths::ANGLE_AXIS_FROM_MAT(O,r);
					double new_guess[6] = {guess[0], guess[1], guess[2], r[0], r[1], r[2]};

					// redraw quad axes
					{
						double origin[3], xaxis[3], yaxis[3], zaxis[3];
						getAxesVecs(new_guess, origin, xaxis, yaxis, zaxis);
						CmPoint32f vo(origin[0], origin[1], origin[2]);
						CmPoint32f vx(xaxis[0], xaxis[1], xaxis[2]);
						CmPoint32f vy(yaxis[0], yaxis[1], yaxis[2]);
						CmPoint32f vz(zaxis[0], zaxis[1], zaxis[2]);

						vsdraw->line(vo, vz, 32, true);
						vsdraw->line(vo, vy, 32, true);
						vsdraw->line(vo, vx, 32, true);
						vsdraw->point(vo, -1, true);

						// return rgb image
						vsdraw->copyImage(axes_img);
						cvtColor(axes_img, axes_img, CV_BGR2RGB);

						double px = 0, py = 0;
						cam_model->vectorToPixelIndex(xaxis, px, py);
						drawText(axes_img, "x",
								px, py, 1, 255, 255, 255, true, 0);
						cam_model->vectorToPixelIndex(yaxis, px, py);
						drawText(axes_img, "y",
								px, py, 1, 255, 255, 255, true, 0);
						cam_model->vectorToPixelIndex(zaxis, px, py);
						drawText(axes_img, "z",
								px, py, 1, 255, 255, 255, true, 0);

						cvtColor(axes_img, axes_img, CV_RGB2BGR);
						imshow("FicTrac-config", axes_img);
					}
				}

				// do manual adjustment
				redraw = false;
				uint16_t key = waitKey(10);
				switch(key) {
					case 0x78:
						Maths::ROT_MAT_X_AXIS(O, 2.5*Maths::D2R);
						adjusted = redraw = true;
						break;
					case 0x79:
						Maths::ROT_MAT_Y_AXIS(O, 2.5*Maths::D2R);
						adjusted = redraw = true;
						break;
					case 0x7A:
						Maths::ROT_MAT_Z_AXIS(O, 2.5*Maths::D2R);
						adjusted = redraw = true;
						break;
					case 0x0A:
					case 0x0D:
					case 0xFF8D:
						adjusting = false;
						break;
					case 0xFFFF:
						break;
					default:
						printf("\n  Press ENTER to accept the current transformation!\n");
						break;
				}
			}
			if( adjusted ) {
				// update guess
				double r[3];
				Maths::ANGLE_AXIS_FROM_MAT(O,r);
				guess[3] = r[0]; guess[4] = r[1]; guess[5] = r[2];
			} else {
				// accept current minimisation
				not_done = false;
			}
		}
		// save minimised configuration
		axes_img.copyTo(save_img);

		// Output config params to file
		printf("Transform data file: %s\n", transform_fn.c_str());
		ofstream ofs(transform_fn.c_str(), ofstream::out);

		Maths::ANGLE_AXIS_TO_MAT(&guess[3], Rw);
		Maths::MAT_T(Rw);
		for( int i = 0, it = 0; i < 3; i++ ) {
			for( int j = 0; j < 3; j++, it++ ) {
				ofs << Rw[it] << " ";
			}
			ofs << endl;
		}
		ofs << guess[0] << " " << guess[1] << " " << guess[2] << endl;
		ofs << sphere_centre[0] << " " << sphere_centre[1] << " " << sphere_centre[2] << endl;
		ofs << sphere_fov << endl;
		ofs << fg_yuv.x << " " << fg_yuv.y << " " << fg_yuv.z << endl;
		ofs << bg_yuv.x << " " << bg_yuv.y << " " << bg_yuv.z << endl;
		ofs << vfov << endl;
		ofs << m_cam_model << endl;
		ofs.close();

		printf("Output image file: %s\n", config_img_fn.c_str());
		imwrite(config_img_fn, save_img);

		fflush(stdout);
		if( !no_prompts ) {
			printf("\n  Press any key to continue...\n\n");
			fflush(stdout);
			waitKey(0);
		}
		destroyWindow("FicTrac-config");
	} else {

		/// Load config params from file.
		printf("cam-world rotation matrix:\n");
		for( int i = 0, it = 0; i < 3; i++ ) {
			for( int j = 0; j < 3; j++, it++ ) {
				ifs >> Rw[it];
				printf("\t%6.3f ", Rw[it]);
			}
			printf("\n");
		}

		// load translation vector
		ifs >> Tw[0];
		ifs >> Tw[1];
		ifs >> Tw[2];

		printf("cam-world translation vector:\n%6.3f %6.3f %6.3f\n",
				Tw[0], Tw[1], Tw[2]);

		// load sphere roi params
		ifs >> sphere_centre[0];
		ifs >> sphere_centre[1];
		ifs >> sphere_centre[2];
		Maths::NORMALISE_VEC(sphere_centre);
		ifs >> sphere_fov;
		ifs >> fg_yuv.x;
		ifs >> fg_yuv.y;
		ifs >> fg_yuv.z;
		ifs >> bg_yuv.x;
		ifs >> bg_yuv.y;
		ifs >> bg_yuv.z;
		ifs >> vfov;		// overwrites variable if present in transform file
		int tmp = -1;
		ifs >> tmp;
		if( tmp > 0 ) {
			m_cam_model = (CAM_MODEL_TYPE)tmp;	// overwrites variable if present in transform file
		}

		printf("sphere ROI centre: (%.2f, %.2f, %.2f) radius: %.1f degrees\n",
				sphere_centre[0], sphere_centre[1], sphere_centre[2],
				sphere_fov*Maths::R2D/2.0);
		printf("using a ");
		switch( m_cam_model ) {
			case RECTILINEAR:
				printf("rectilinear camera model\n");
				break;
			case FISHEYE:
				fisheye = true;
				printf("fisheye camera model\n");
				break;
		}

		if( force_draw_config ) {
			// copy T|R from transform file
			double tmp_mat[9];
			memcpy(tmp_mat, Rw, 9*sizeof(double));
			Maths::MAT_T(tmp_mat);
			double tmp_vec[6];
			Maths::ANGLE_AXIS_FROM_MAT(tmp_mat, &(tmp_vec[3]));
			memcpy(tmp_vec, Tw, 3*sizeof(double));

			CameraModelPtr cam_model;
			if( fisheye ) {
				cam_model = CameraModel::createFisheye(
						width, height, vfov/height, 360*Maths::D2R);
			} else {
				cam_model = CameraModel::createRectilinear(width, height, vfov);
			}

			Mat frame_bgr, save_img;
			for( unsigned int i = 0; i <= frame_skip; i++ ) {
				if( !cap->grab(frame_bgr) ) {
					fprintf(stderr, "ERROR: grabbing camera frame!\n");
					fflush(stderr);
					exit(-1);
				}
			}

			VsDraw::Ptr vsdraw = VsDraw::Ptr(new VsDraw);
			vsdraw->openImage(cam_model, frame_bgr);
			vsdraw->grey(255);
			vsdraw->thickness(1.5);

			// draw ROI
			CmPoint32f sc(sphere_centre[0], sphere_centre[1], sphere_centre[2]);
			vsdraw->cross(sc, -4);
			vsdraw->circle(sc, sphere_fov/2.0);

			// draw corners
			double tl2[3], tr2[3], br2[3], bl2[3];
			getCornerVecs(tmp_vec, tl2, tr2, br2, bl2);

			printf("re-projected corners:\n");
			double px = 0, py = 0;
			cam_model->vectorToPixelIndex(tl2, px, py);
			printf("(%6.1f, %6.1f)\n", px, py);
			cam_model->vectorToPixelIndex(tr2, px, py);
			printf("(%6.1f, %6.1f)\n", px, py);
			cam_model->vectorToPixelIndex(br2, px, py);
			printf("(%6.1f, %6.1f)\n", px, py);
			cam_model->vectorToPixelIndex(bl2, px, py);
			printf("(%6.1f, %6.1f)\n", px, py);

			CmPoint32f axis(tl2[0], tl2[1], tl2[2]);
			vsdraw->cross(axis, -6, true);
			axis[0] = tr2[0]; axis[1] = tr2[1]; axis[2] = tr2[2];
			vsdraw->cross(axis, -6, true);
			axis[0] = br2[0]; axis[1] = br2[1]; axis[2] = br2[2];
			vsdraw->cross(axis, -6, true);
			axis[0] = bl2[0]; axis[1] = bl2[1]; axis[2] = bl2[2];
			vsdraw->cross(axis, -6, true);
//			vsdraw->display("FicTrac-config");

			// draw quad axes
			double origin[3], xaxis[3], yaxis[3], zaxis[3];
			getAxesVecs(tmp_vec, origin, xaxis, yaxis, zaxis);
			CmPoint32f vo(origin[0], origin[1], origin[2]);
			CmPoint32f vx(xaxis[0], xaxis[1], xaxis[2]);
			CmPoint32f vy(yaxis[0], yaxis[1], yaxis[2]);
			CmPoint32f vz(zaxis[0], zaxis[1], zaxis[2]);

			vsdraw->line(vo, vz, 32, true);
			vsdraw->line(vo, vy, 32, true);
			vsdraw->line(vo, vx, 32, true);
			vsdraw->point(vo, -1, true);

			// return rgb image
			vsdraw->copyImage(save_img);
			cvtColor(save_img, save_img, CV_BGR2RGB);

			px = 0; py = 0;
			cam_model->vectorToPixelIndex(xaxis, px, py);
			drawText(save_img, "x",
					px, py, 1, 255, 255, 255, true, 0);
			cam_model->vectorToPixelIndex(yaxis, px, py);
			drawText(save_img, "y",
					px, py, 1, 255, 255, 255, true, 0);
			cam_model->vectorToPixelIndex(zaxis, px, py);
			drawText(save_img, "z",
					px, py, 1, 255, 255, 255, true, 0);

			cvtColor(save_img, save_img, CV_RGB2BGR);

			printf("Output image file: %s\n", config_img_fn.c_str());
			imwrite(config_img_fn, save_img);
		}

		fflush(stdout);
	}

	//////////////////////////////////////////////////////////////////////////////////////
	///                                                                                ///
	/// START SPHERE ORIENTATION TRACKER                                               ///
	///                                                                                ///
	//////////////////////////////////////////////////////////////////////////////////////

	int draw_size = 160;
	boost::shared_ptr<AVWriter> writer, input_writer;
	if( save_video ) {
		writer = boost::shared_ptr<AVWriter>(new AVWriter(
				debug_video_fn,
				4*draw_size, 3*draw_size,
				512, 384,
				AV_PIX_FMT_RGB24, 25, -1, 4));
	}
	if( save_input_video ) {
		input_writer = boost::shared_ptr<AVWriter>(new AVWriter(
				frames_video_fn,
				width, height,
				width, height,
				AV_PIX_FMT_BGR24, 25, -1, 4));
	}

	///
	/// CAMERA_MODELS
	///

	// sphere roi camera model
	CameraRemapPtr remapper;
	CameraModelPtr cam_model, dest_model;
	RemapTransformPtr roi_transform;
	double roi_rot_mat[9];
	Maths::MAKE_IDENTITY_MAT(roi_rot_mat);
	{
		if( fisheye ) {
			cam_model = CameraModel::createFisheye(
					width, height, vfov/height, 360*Maths::D2R);
		} else {
			cam_model = CameraModel::createRectilinear(width, height, vfov);
		}

		double sphere_radPerPix = sphere_fov/remap_width;
		dest_model = CameraModel::createFisheye(
				remap_width, remap_height, sphere_radPerPix, sphere_fov);

		// rotation from forward
		double omega[3] = {0};
		Maths::ANGLE_AXIS_FROM_VEC(sphere_centre, omega);
		CmPoint32f omegav(omega[0], omega[1], omega[2]);
		roi_transform = MatrixRemapTransform::createFromOmega(omegav);
		remapper = CameraRemapPtr(new CameraRemap(cam_model, dest_model, roi_transform));

		printf("sphere rotation from forward: %6.3f %6.3f %6.3f\n",
				omegav.x, omegav.y, omegav.z);

		double fwd[3] = {0,0,1};
		Maths::ROT_MAT_FROM_VECS(fwd, sphere_centre, roi_rot_mat);
//		Maths::MAT_T(roi_rot_mat);
	}

	///
	/// LOAD MASK AND TEMPLATE
	///

	/// Load mask.
	Mat mask = imread(mask_fn.c_str(), 0);
	if( mask.empty() ) {
		fprintf(stderr, "ERROR: loading mask (%s)!\n", mask_fn.c_str());
		fflush(stderr);
		exit(-1);
	}
	printf("input mask size: %d x %d\n", mask.cols, mask.rows);

	// sphere roi mask
	Mat mask_remap(remap_height, remap_width, CV_8UC1);
	mask_remap.setTo(Scalar::all(0));
	remapper->apply(mask, mask_remap);
	int erode_its = 1;//max(int(3+round(log(remap_width/60.0)/log(2))),1);	// magic!
	erode(mask_remap, mask_remap, Mat(), Point(-1,-1), erode_its, BORDER_CONSTANT, 0);

	// erode can fail near border, so erase.
//	for( int i = 0; i <= erode_its; i++ ) {
//		uint8_t* pmask_top = mask_remap.ptr(i);
//		uint8_t* pmask_bot = mask_remap.ptr(remap_height-i-1);
//		for( int j = 0; j < remap_width; j++ ) {
//			pmask_top[j] = 0;
//			pmask_bot[j] = 0;
//		}
//	}
//	for( int i = 0; i < remap_height; i++ ) {
//		uint8_t* pmask = mask_remap.ptr(i);
//		for( int j = 0; j <= erode_its; j++ ) {
//			pmask[j] = 0;
//			pmask[remap_width-j-1] = 0;
//		}
//	}

#if EXTRA_DEBUG_WINDOWS
	namedWindow("mask", CV_NORMAL);
	imshow("mask", mask_remap);
#endif // EXTRA_DEBUG_WINDOWS

	/// Load sphere template.
	Mat sphere_template = Mat();
	if( load_template ) {
		sphere_template = imread(template_fn.c_str(), 0);
		if( sphere_template.empty() || sphere_template.cols != sphere_width || sphere_template.rows != sphere_height || sphere_template.channels() > 1 ) {
			fprintf(stderr, "ERROR: loading template (%s)!\n", template_fn.c_str());
			fflush(stderr);
			sphere_template = Mat();
			load_template = false;
		} else {
			SPHERE_INIT = true;
		}
	}

	if( !do_update && !load_template ) {
		fprintf(stderr, "ERROR: must do either do_update or load_template!\n");
		fflush(stderr);
		exit(-1);
	}

	///
	/// INPUT
	///

	boost::shared_ptr<s_input> input = boost::shared_ptr<s_input>(new s_input);
	input->cap = cap;
	input->cam_input = cam_input;
	pthread_cond_init(&input->cond, NULL);
	pthread_mutex_init(&input->mutex, NULL);
	input->buflen = 10*frame_step;
	input->width = width;
	input->height = height;
	input->remap_width = remap_width;
	input->remap_height = remap_height;
	input->cam_model = cam_model;
	input->remap_model = dest_model;
	input->remapper = remapper;
	input->roi_transform = roi_transform;
	input->remap_mask = mask_remap;
	input->use_ball_colour = use_ball_colour;
	input->fg_yuv = fg_yuv;
	input->bg_yuv = bg_yuv;
	input->thresh_win = thresh_win;
	input->thresh_ratio = thresh_ratio;
	input->frame_skip = frame_skip;
	pthread_create(&input->thread, NULL, &grabInputFrames, input.get());
	printf("input thread initialised\n");

	///
	/// SPHERE
	///

	// NOTE: sin rather than tan to correct for apparent size of sphere
	double sphere_radius_distance_ratio = sin(sphere_fov/2.0);
	printf("sphere radius/distance ratio: %.3f\n", sphere_radius_distance_ratio);

	// sphere roi model
	MatchSphereAbs sphere(
			dest_model, roi_rot_mat,
			sphere_width, sphere_height,
			sphere_radius_distance_ratio,
			mask_remap, sphere_template,
			do_display, do_update,
			-nlopt_res, nlopt_res,
			nlopt_ftol, nlopt_max_eval);

	// initialise sphere orientation
	if( sqrt(Maths::DOT_VEC(sphere_orient, sphere_orient)) > 1e-3 ) {
		Maths::ANGLE_AXIS_TO_MAT(sphere_orient, sphere.getR());
	}

	/// Finished init.
	if( !no_prompts ) {
		printf("\n  Initialisation complete.\n\n  During execution, if the display is active,\n  press 's' at any time to save the sphere template to disk,\n  'p' to pause the program, and 'ESC' to quit.\n\n  Press ENTER to continue...\n");
		fflush(stdout);
		getchar();
	}

	///
	/// LOGGING AND OUTPUT
	///

	boost::shared_ptr<s_logger> log = boost::shared_ptr<s_logger>(new s_logger);
	log->fn = output_fn;
	log->state = 0;
	pthread_cond_init(&log->cond, NULL);
	pthread_mutex_init(&log->mutex, NULL);
	pthread_create(&log->thread, NULL, &logger, log.get());
	while( !log->state ) { usleep(10000); }
	if( log->state < 0 ) {
		fprintf(stderr, "ERROR: initialising logger!\n");
		fflush(stderr);
		exit(-1);
	}

	#if LOG_TIMING
		ofstream timing("timing.dat", ofstream::out);
		timing.precision(8);
	#endif

	ofstream clfs;
	bool do_closed_loop = false;
	if( !closed_loop_fn.empty() ) {
		clfs.open(closed_loop_fn.c_str(), ofstream::out);
		do_closed_loop = true;
	}

	if( do_serial_out ) {
		if( !serial_open(&_serial, serial_port.c_str(), serial_baud) ) {
			fprintf(stderr, "ERROR: opening serial port (%s)!\n", serial_port.c_str());
			fflush(stderr);
			do_serial_out = false;
		}
	}

	boost::shared_ptr<s_socket> _socket;
	if( do_socket_out ) {
		_socket = boost::shared_ptr<s_socket>(new s_socket);
		pthread_mutex_init(&_socket->mutex, NULL);
		_socket->portno = -1;
		_socket->state = 0;
		_socket->sockfd = -1;
		_socket->frameno = -1;
		_socket->px = 0;
		_socket->py = 0;
		_socket->vx = 0;
		_socket->vy = 0;
		_socket->heading = 0;

		printf("Waiting for socket to initialise...\n");
		pthread_create(&_socket->thread, NULL, &socketListener, _socket.get());
		while( !_socket->state ) { usleep(10000); }
		if( _socket->state < 0 ) {
			exit(-1);
		} else {
			ofstream tmpfs("socket.port", ofstream::out);
			tmpfs << _socket->portno << endl;
			tmpfs.close();
			if( !no_prompts ) {
				printf("\n  Press any key to continue...\n\n");
				fflush(stdout);
				getchar();
			}
		}
	}

    // set up the fmf saving functionality
    //create an empty fmf wrapper object (placed here to avoid scoping issues)
    boost::shared_ptr<fmfwrapper> fmf = boost::shared_ptr<fmfwrapper>(new fmfwrapper());  
    if( fmf_save.compare("") != 0 ) {
        try{
            fmf->fmfopen(fmf_save);  //create the actual fmf saver python object
        }catch(const std::exception &e){
            std::cout << "C++ exception in fmfwrapper:  " << e.what() << std::endl;
        }
    }

	fflush(stdout);

#if LOG_TIMING
	double t0 = Utils::GET_CLOCK();
	
#if ENABLE_VOLTAGE_OUT
	//added for MCC USB 3101 by Pablo 7/1/14
	HIDInterface*  hid = 0x0;
	__u8 channel;
	__u16 value;
	hid_return ret;
	int idx;
	int nInterfaces = 0;

	ret=hid_init();
	if (ret!=HID_RET_SUCCESS) {
	fprintf(stderr, "hid_init failed with return code %d\n", ret);
	return -1;
	}
	if ((nInterfaces = PMD_Find_Interface(&hid, 0, USB3101_PID)) >= 0) {
    fprintf(stderr, "USB 3101 Device is found! Number of Interfaces = %d\n", nInterfaces);
	}
	
	/* config mask DIO_DIR_OUT (0x00) means all outputs */
	usbDConfigPort_USB31XX(hid, DIO_DIR_OUT);
	usbDOut_USB31XX(hid, 0);
	
	// Configure all analog channels for 0-10V output
	for (idx = 0; idx < 8; idx++) {
	usbAOutConfig_USB31XX(hid, idx, UP_10_00V);
	}
	
	//^^^Pablo-------------------------------------------
#endif /* ENABLE_VOLTAGE_OUT */

#endif // LOG_TIMING

	unsigned int nframes = 0;
	double av_err = 0, av_exec_time = 0, av_loop_time = 0, total_dist = 0;

	///
	/// PROGRAM LOOP
	///

	for( unsigned int cnt = 1, seq_n = 1; ; cnt++, seq_n++ ) {

		nframes++;

		printf("\ndoing frame %d\n", cnt);

		double t1 = Utils::GET_CLOCK();

		/// Grab frame and remap.

		double timestamp = 0;
		Mat frame_bgr, remap;
		bool grab_error = false;
		for( unsigned int i = 0; i < frame_step; i++ ) {
			if( !getInputFrames(input, frame_bgr, remap, timestamp) ) {
				fprintf(stderr, "ERROR: No more frames!\n");
				fflush(stdout);
				grab_error = true;
				break;
			}
		}
		if( grab_error ) { break; }

		double t2 = Utils::GET_CLOCK();

		/// Minimise rotation.

		double err = 0;
		bool bad_frame = false;
		static bool first_good_frame = true;
		static int nbad_frames = 0;

		static double av_guess[3] = {0,0,0};

		double guess[3] = {av_guess[0], av_guess[1], av_guess[2]};

		double max_err = DBL_MAX;
		if( error_thresh > 0 ) {
			max_err = error_thresh;
		}

		// init search
		{
			double lb[3] = {guess[0]-nlopt_res, guess[1]-nlopt_res, guess[2]-nlopt_res};
			double ub[3] = {guess[0]+nlopt_res, guess[1]+nlopt_res, guess[2]+nlopt_res};
			sphere.setLowerBounds(lb);
			sphere.setUpperBounds(ub);
			sphere.setImage(remap);
		}

		if( (cnt > 1) || SPHERE_INIT ) {
			// guess small rotation
//			guess[0] = Utils::GEN_RAND_DBL(-0.01, 0.01);
//			guess[1] = Utils::GEN_RAND_DBL(-0.01, 0.01);
//			guess[2] = Utils::GEN_RAND_DBL(-0.01, 0.01);

			printf("initial guess:\t%.2f %.2f %.2f\n",
					guess[0], guess[1], guess[2]);

			sphere.optimize(guess);
			sphere.getOptX(guess);
			err = sphere.getOptF();

			bad_frame = err > max_err;

			printf("initial minimisation:\t%.2f %.2f %.2f  (%.3f)\n",
					guess[0], guess[1], guess[2], err);

//			printf("recursive mean/stdv: %.1f / %.1f\n",
//					recur_mean_err, recur_stdv_err);

			// if bad, do global search to re-localise
			if( do_search && bad_frame && SPHERE_INIT ) {
				double best_score = err;
				double best_guess[3] = {guess[0], guess[1], guess[2]};
				double test[3] = {0};
				int nsearch_pts = 1000;
				int search_state = 0;
				printf("performing global search... ");
				fflush(stdout);
				for( int i = 1; i <= nsearch_pts; i++ ) {
					float pc = float(i)/nsearch_pts;
					switch( search_state ) {
						case 0:
							if( pc >= 0.1 ) {
								printf("10%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 1:
							if( pc >= 0.2 ) {
								printf("20%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 2:
							if( pc >= 0.3 ) {
								printf("30%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 3:
							if( pc >= 0.4 ) {
								printf("40%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 4:
							if( pc >= 0.5 ) {
								printf("50%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 5:
							if( pc >= 0.6 ) {
								printf("60%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 6:
							if( pc >= 0.7 ) {
								printf("70%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 7:
							if( pc >= 0.8 ) {
								printf("80%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 8:
							if( pc >= 0.9 ) {
								printf("90%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						case 9:
							if( pc >= 1 ) {
								printf("100%% ");
								fflush(stdout);
								search_state++;
							}
							break;
						default:
							break;
					}
					fflush(stdout);

					guess[0] = Utils::GEN_RAND_GSN(1,0);
					guess[1] = Utils::GEN_RAND_GSN(1,0);
					guess[2] = Utils::GEN_RAND_GSN(1,0);
					Maths::NORMALISE_VEC(guess);

					for( int mag = -180; mag < 180; mag+=10 ) {
						double rmag = mag*Maths::D2R;
						test[0] = rmag*guess[0];
						test[1] = rmag*guess[1];
						test[2] = rmag*guess[2];

						err = sphere.testRotation(test);

						if( err < best_score ) {
							best_score = err;
							best_guess[0] = test[0];
							best_guess[1] = test[1];
							best_guess[2] = test[2];
						}

//						if( !(bad_frame = err > max_err) ) { break; }
					}
//					if( !bad_frame ) { break; }
				}
				printf("\nsearch:\t\t%.2f %.2f %.2f  (%.3f)\n",
						best_guess[0], best_guess[1], best_guess[2], best_score);

				double lb[3] = {best_guess[0]-nlopt_res, best_guess[1]-nlopt_res, best_guess[2]-nlopt_res};
				double ub[3] = {best_guess[0]+nlopt_res, best_guess[1]+nlopt_res, best_guess[2]+nlopt_res};
				sphere.setLowerBounds(lb);
				sphere.setUpperBounds(ub);
				sphere.optimize(best_guess);
				sphere.getOptX(guess);
				err = sphere.getOptF();

				bad_frame = err > max_err;

				printf("minimised:\t%.2f %.2f %.2f  (%.3f)\n",
						guess[0], guess[1], guess[2], err);
			}
		}
		if( bad_frame ) {
			nbad_frames++;

			if( (max_bad_frames > 0) && (nbad_frames > max_bad_frames) ) {
				// reset output data
				seq_n = 1;
				nbad_frames = 0;
				SPHERE_INIT = false;
				sphere.clearSphere();
				first_good_frame = true;
			}
		} else {
			nbad_frames = 0;
			av_guess[0] = 0.90*guess[0]+0.10*av_guess[0];
			av_guess[1] = 0.90*guess[1]+0.10*av_guess[1];
			av_guess[2] = 0.90*guess[2]+0.10*av_guess[2];
		}

		double t3 = Utils::GET_CLOCK();

		///
		/// SAVING
		///

		// draw pos hist stuff
		const int hist_length = 250;
		static double pos_hist[hist_length*3];
		static int hist_it = -1;
		if( hist_it < 0 ) { memset(pos_hist, 0, hist_length*3*sizeof(double)); }
		static std::deque<CmPoint> map_hist;

		static double posx = 0;
		static double posy = 0;
		static double velx = 0;
		static double vely = 0;
		static double heading = 0;
		static double intx = 0;
		static double inty = 0;
		
		//w - rel vec world
		//moved and made "static" by Pablo 07/2014
		static double w[3] = {0,0,0};

		// save if good
		if( !bad_frame || !SPHERE_INIT ) {

			sphere.updateSphere(guess);

			// r - rel vec cam (-ve of guess???)
//			double r[3] = {-guess[0], -guess[1], -guess[2]};
			double r[3] = {0,0,0};
			Maths::MAT_MUL_VEC(roi_rot_mat, guess, r);
			Maths::SCALE_VEC(r, -1);

			// R - abs mat cam
//			double* R = sphere.getR();
			double Rcam[9] = {1,0,0,0,1,0,0,0,1};
			Maths::MUL_MAT(roi_rot_mat, sphere.getR(), Rcam);
			Maths::MAT_T(Rcam);

			// store initial rotation from template (if any)
			static double Rf[9] = {1,0,0,0,1,0,0,0,1};
			if( first_good_frame ) {
				memcpy(Rf, Rcam, 9*sizeof(double));
				Maths::MAT_T(Rf);

				// clear rel vec cam
				r[0] = r[1] = r[2] = 0;

				first_good_frame = false;
			}

			Maths::MAT_MUL_VEC(Rw, r, w);

			// Rc - abs mat cam (corrected for initial orientation)
			double Rc[9] = {1,0,0,0,1,0,0,0,1};
			Maths::MUL_MAT(Rf, Rcam, Rc);

			// Rv - abs vec cam
			double Rcv[3] = {0,0,0};
			Maths::ANGLE_AXIS_FROM_MAT(Rc, Rcv);

			// FIXME: hack to avoid bee pos history drawing failing
			// (path hist on ball was failing when Rf != 1)
			double Rv[3] = {0,0,0};
			Maths::ANGLE_AXIS_FROM_MAT(Rcam, Rv);

			// Wv - abs vec world
			double Wv[3] = {0,0,0};
			Maths::MAT_MUL_VEC(Rw, Rcv, Wv);

			// running speed, radians/frame (-ve rotation around x-axis causes y-axis translation, vice-versa!!)
			velx = w[1];
			vely = -w[0];
			double speed = sqrt(velx*velx+vely*vely); //FIXME: this is probably an approximation, affects integrated x/y?

			// running direction
			double direction = atan2(vely, velx);
			if( direction < 0*Maths::D2R ) { direction += 360*Maths::D2R; }

			// integrated x/y pos (optical mouse style)
			intx += velx;
			inty += vely;

			// integrate 2d position
			{
				int steps = 4;	// increasing this doesn't help much??
				double step = speed/steps;
				static double prev_heading = heading;

				// integrate bee heading
				heading -= w[2];
				while( heading < 0*Maths::D2R ) { heading += 360*Maths::D2R; }
				while( heading >= 360*Maths::D2R ) { heading -= 360*Maths::D2R; }

				double heading_step = heading-prev_heading;
				if( heading_step >= 180*Maths::D2R ) { heading_step -= 360*Maths::D2R; }
				if( heading_step < -180*Maths::D2R ) { heading_step += 360*Maths::D2R; }
				heading_step /= steps;

				double dir[3] = {velx, vely, 0};
				Maths::NORMALISE_VEC(dir);
				Maths::ROT_VEC_Z_AXIS(dir, prev_heading+heading_step/2.0);
				for( int i = 0; i < steps; i++ ) {
					posx += step*dir[0];
					posy += step*dir[1];
					Maths::ROT_VEC_Z_AXIS(dir, heading_step);
				}
				prev_heading = heading;
			}

			printf("optimum rotation:\t%.3f %.3f %.3f  (%.3f/%d)\n",
					guess[0], guess[1], guess[2], err, sphere.getNumEval());

			// frame_count
			stringstream ofs;
			ofs.precision(14);
			ofs << cnt << ", ";
			// rel_vec_cam[3] | error
			ofs << r[0] << ", " << r[1] << ", " << r[2] << ", " << err << ", ";
			// rel_vec_world[3]
			ofs << w[0] << ", " << w[1] << ", " << w[2] << ", ";
			// abs_vec_cam[3]
			ofs << Rcv[0] << ", " << Rcv[1] << ", " << Rcv[2] << ", ";
			// abs_vec_world[3]
			ofs << Wv[0] << ", " << Wv[1] << ", " << Wv[2] << ", ";
			// integrated xpos | integrated ypos | integrated heading
			ofs << posx << ", " << posy << ", " << heading << ", ";
			// direction (radians) | speed (radians/frame)
			ofs << direction << ", " << speed << ", ";
			// integrated x movement | integrated y movement (mouse output equivalent)
			ofs << intx << ", " << inty << ", ";
			// timestamp
			ofs << timestamp << ", " << seq_n << ", ";

			pthread_mutex_lock(&(log->mutex));
			log->msgq.push_back(ofs.str());
			pthread_cond_broadcast(&(log->cond));
			pthread_mutex_unlock(&(log->mutex));

			// update pos hist (in ROI-space!)
			hist_it++;
			pos_hist[3*(hist_it%hist_length)+0] = -Rv[0];
			pos_hist[3*(hist_it%hist_length)+1] = -Rv[1];
			pos_hist[3*(hist_it%hist_length)+2] = -Rv[2];

			map_hist.push_back(CmPoint(posx, posy, heading));

			// draw heading history
			#if EXTRA_DEBUG_WINDOWS
				// drawable bounds [2-397], [2-197]
				static Mat heading_plot(200, 400, CV_8UC1);
				static deque<float> heading_hist;
				heading_hist.push_front((2*Maths::PI-heading)*195.0/(2*Maths::PI)+2);
				if( heading_hist.size() > 395 ) { heading_hist.pop_back(); }

				heading_plot.setTo(Scalar::all(0));
				cv::line(heading_plot,
						cvPoint2D32f(round(2*16), round(99.5*16)),
						cvPoint2D32f(round(397*16), round(99.5*16)),
						CV_RGB(127,127,127), 1, CV_AA, 4);
				cv::line(heading_plot,
						cvPoint2D32f(round(2*16), round(2*16)),
						cvPoint2D32f(round(2*16), round(197*16)),
						CV_RGB(127,127,127), 1, CV_AA, 4);
				cv::line(heading_plot,
						cvPoint2D32f(round(397*16), round(2*16)),
						cvPoint2D32f(round(397*16), round(197*16)),
						CV_RGB(127,127,127), 1, CV_AA, 4);
				cv::line(heading_plot,
						cvPoint2D32f(round(2*16), round(2*16)),
						cvPoint2D32f(round(397*16), round(2*16)),
						CV_RGB(127,127,127), 1, CV_AA, 4);
				cv::line(heading_plot,
						cvPoint2D32f(round(2*16), round(50.75*16)),
						cvPoint2D32f(round(397*16), round(50.75*16)),
						CV_RGB(127,127,127), 1, CV_AA, 4);
				cv::line(heading_plot,
						cvPoint2D32f(round(2*16), round(148.25*16)),
						cvPoint2D32f(round(397*16), round(148.25*16)),
						CV_RGB(127,127,127), 1, CV_AA, 4);
				cv::line(heading_plot,
						cvPoint2D32f(round(2*16), round(197*16)),
						cvPoint2D32f(round(397*16), round(197*16)),
						CV_RGB(127,127,127), 1, CV_AA, 4);

				for( unsigned int i = 1; i < heading_hist.size(); i++ ) {
					cv::line(heading_plot,
							cvPoint2D32f(round((397-(i-1))*16), round(heading_hist[i-1]*16)),
							cvPoint2D32f(round((397-i)*16), round(heading_hist[i]*16)),
							CV_RGB(255,255,255), 1, CV_AA, 4);
				}

				namedWindow("heading_hist");
				imshow("heading_hist", heading_plot);
			#endif

			// draw movement history
			#if EXTRA_DEBUG_WINDOWS
				// drawable bounds [2-397], [2-197]
				static Mat move_plot(200, 400, CV_8UC1);
				static deque<float> move_hist_x, move_hist_y;
				move_hist_x.push_front(-w[1]);
				if( move_hist_x.size() > 100 ) { move_hist_x.pop_back(); }
				move_hist_y.push_front(w[0]);
				if( move_hist_y.size() > 100 ) { move_hist_y.pop_back(); }

				float cum_hist_x[101];
				float cum_hist_y[101];
				memset(cum_hist_x, 0, 101*sizeof(float));
				memset(cum_hist_y, 0, 101*sizeof(float));

				float min_x = FLT_MAX, max_x = -FLT_MAX;
				float min_y = FLT_MAX, max_y = -FLT_MAX;
				for( unsigned int i = 1; i <= move_hist_x.size(); i++ ) {
					cum_hist_x[i] = cum_hist_x[i-1]+move_hist_x[i-1];
					cum_hist_y[i] = cum_hist_y[i-1]+move_hist_y[i-1];
					if( cum_hist_x[i] < min_x ) {
						min_x = cum_hist_x[i];
					}
					if( cum_hist_x[i] > max_x ) {
						max_x = cum_hist_x[i];
					}
					if( cum_hist_y[i] < min_y ) {
						min_y = cum_hist_y[i];
					}
					if( cum_hist_y[i] > max_y ) {
						max_y = cum_hist_y[i];
					}
				}

				float x_scl = std::min(200.0/fabs(max_x), 200.0/fabs(min_x));
				float y_scl = std::min(100.0/fabs(max_y), 100.0/fabs(min_y));
				float scl = std::min(x_scl,y_scl);

				float ppx = 200, ppy = 100;
				move_plot.setTo(Scalar::all(0));
				for( unsigned int i = 1; i <= move_hist_x.size(); i++ ) {
					float px = 200+scl*cum_hist_x[i], py = 100+scl*cum_hist_y[i];
					cv::line(move_plot,
							cvPoint2D32f(round(ppx*16), round(ppy*16)),
							cvPoint2D32f(round(px*16), round(py*16)),
							CV_RGB(255,255,255), 1, CV_AA, 4);
					ppx = px;
					ppy = py;
				}

				namedWindow("move_hist");
				imshow("move_hist", move_plot);
			#endif

			// stats
			total_dist += sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]);

			SPHERE_INIT = true;
		} else {
			printf("FRAME DROPPED!!\n");
		}

		// stats
		av_err += err;
		av_exec_time += (t3-t2)*1e3;

		///
		/// CLOSED LOOP
		///

		if( do_closed_loop ) {
			clfs.seekp(0, std::ios::beg);
			clfs << cnt << ", " << intx << ", " << inty << ", " << heading << ", " << cnt << endl;
		}

		// Serial Out modified by Pablo for MCC USB 3101  7/1/14 
		if( do_serial_out ) {
//			static char out[8] = {0,0,0,0,0,0,0,0};
//			int velx_int = Maths::CLAMP((int)round(65535.0*(velx/nlopt_res+1)/2.0), 0, 65535);		// [-0.5,0.5] -> [0,65535]
//			int vely_int = Maths::CLAMP((int)round(65535.0*(vely/nlopt_res+1)/2.0), 0, 65535);		// [-0.5,0.5] -> [0,65535]
//			int heading_int = round(65536*heading/(2*Maths::PI));						// [0,2pi) -> [0,65535]
//			if( heading_int >= 65536 ) { heading_int -= 65536; }
//			out[0] = 0x52;
//			out[1] = (velx_int >> 8) & 0xFF;
//			out[2] = (velx_int) & 0xFF;
//			out[3] = (vely_int >> 8) & 0xFF;
//			out[4] = (vely_int) & 0xFF;
//			out[5] = (heading_int >> 8) & 0xFF;
//			out[6] = (heading_int) & 0xFF;
//			out[7] = 0x4D;

			int heading_int = round(256*heading/(2*Maths::PI));
			if( heading_int >= 256 ) { heading_int -= 256; }
			uint8_t heading_8bit = heading_int;

			if( serial_write(&_serial, &heading_8bit, 1) != 1 ) {
				fprintf(stderr, "ERROR: Short write to serial (%s)!\n", serial_port.c_str());
			}
			
			if (output_position) {
				int comp0 = round(65536.0*intx/(2*Maths::PI));
				int comp1 = round(65536.0*heading/(2*Maths::PI));
				int comp2 = round(65536.0*inty/(2*Maths::PI));		
					
                #if ENABLE_VOLTAGE_OUT
				usbAOut_USB31XX(hid, 0, (__u16) comp0, 0);
				usbAOut_USB31XX(hid, 2, (__u16) comp2, 0);
				usbAOut_USB31XX(hid, 1, (__u16) comp1, 0);
                #endif /* ENABLE_VOLTAGE_OUT */
			}
			else {
				int comp0 = Maths::CLAMP((int)round(65535.0*(vely/nlopt_res+1)/2.0), 0, 65535);
				int comp1 = Maths::CLAMP((int)round(65535.0*(w[2]/nlopt_res+1)/2.0), 0, 65535);
				int comp2 = Maths::CLAMP((int)round(65535.0*(vely/nlopt_res+1)/2.0), 0, 65535);

                #if ENABLE_VOLTAGE_OUT
				usbAOut_USB31XX(hid, 0, (__u16) comp0, 0);
				usbAOut_USB31XX(hid, 1, (__u16) comp1, 0);
				usbAOut_USB31XX(hid, 2, (__u16) comp2, 0);
                #endif /* ENABLE_VOLTAGE_OUT */
			}
		}

		if( do_socket_out ) {
			pthread_mutex_lock(&(_socket->mutex));
			_socket->frameno = cnt;
			_socket->px = posx;
			_socket->py = posy;
			_socket->vx = velx;
			_socket->vy = vely;
			_socket->heading = heading*Maths::R2D;
			pthread_mutex_unlock(&(_socket->mutex));
		}

		double t4 = Utils::GET_CLOCK();

		///
		/// DRAWING
		///

		if( do_led_display ) {
			static Mat led_display(32, 128, CV_8UC3);
			rectangle(led_display, cvPoint(0,0), cvPoint(127,31), CV_RGB(0,255,255), CV_FILLED, 8, 0);

			// draw heading bar
			int hx = 64+round(128*heading/(2*Maths::PI));
			if( hx >= 128 ) { hx -= 128; }
			int lhx = hx-4, rhx = hx;
			if( lhx < 0 ) {
				int rem = -lhx;
				// draw lhx to right border
				rectangle(led_display, cvPoint(128-rem,0), cvPoint(127,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
				// draw remainder of lhx from 0
				rectangle(led_display, cvPoint(0,0), cvPoint(4-rem,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
				// draw rhx
				rectangle(led_display, cvPoint(rhx,0), cvPoint(rhx+3,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
			} else if( (rhx+3) >= 128 ) {
				int rem = rhx+3-127;
				// draw rhx to right border
				rectangle(led_display, cvPoint(128-4+rem,0), cvPoint(127,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
				// draw remainder of rhx from 0
				rectangle(led_display, cvPoint(0,0), cvPoint(rem-1,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
				// draw lhx
				rectangle(led_display, cvPoint(lhx,0), cvPoint(lhx+3,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
			} else {
				// draw as normal
				rectangle(led_display, cvPoint(lhx,0), cvPoint(lhx+3,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
				rectangle(led_display, cvPoint(rhx,0), cvPoint(rhx+3,31), CV_RGB(0,0,0), CV_FILLED, 8, 0);
			}

			namedWindow("FicTrac-LED_panel", 0);
			imshow("FicTrac-LED_panel", led_display);

			{
				uint16_t key = waitKey(1);
				if( key == 0x6D ) {
					setWindowProperty("FicTrac-LED_panel", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
				}
			}
		}

		if( do_display || save_video ) {
			static Mat draw(3*draw_size, 4*draw_size, CV_8UC3);
			draw.setTo(Scalar::all(0));

			// input image
			double radPerPix = sphere_fov*1.5/(2.0*draw_size);
			static CameraModelPtr draw_camera = CameraModel::createFisheye(
					2*draw_size, 2*draw_size, radPerPix, 360*Maths::D2R);
			static CameraRemapPtr draw_remapper = CameraRemapPtr(new CameraRemap(
					cam_model, draw_camera, roi_transform));
			Mat draw_input = draw(Rect(0, 0, 2*draw_size, 2*draw_size));
//			draw_input.setTo(Scalar::all(128));
			draw_remapper->apply(frame_bgr, draw_input);

			// draw rotation axis
//			{
//				double cx = draw_input.cols/2.0, cy = draw_input.rows/2.0;
//
//				circle(draw_input,
//						Point(round(cx*16), round(16*cy)),
//						8, CV_RGB(255,255,255), 1, CV_AA, 4);
//
//				double c[3];
//				draw_camera->pixelIndexToVector(cx, cy, c);
//				Maths::NORMALISE_VEC(c);
//				double r[3] = {-guess[0], -guess[1], -guess[2]};
//				double w[3] = {0};
//				Maths::MAT_MUL_VEC(Rw, r, w);
//				Maths::NORMALISE_VEC(r);
//				r[0] = c[0]+0.1*r[0];
//				r[1] = c[1]+0.1*r[1];
//				r[2] = c[2]+0.1*r[2];
//
//				double rx, ry;
//				draw_camera->vectorToPixelIndex(r, rx, ry);
//
//				line(draw_input,
//						Point(round(cx*16), round(cy*16)),
//						Point(round(rx*16), round(ry*16)),
//						CV_RGB(255, 255, 255), 1, CV_AA, 4);
//
//				double d[3] = {w[1], -w[0], 0};
//				Maths::NORMALISE_VEC(d);
//				Maths::MAT_T_MUL_VEC(Rw, d, w);
//				d[0] = c[0]+0.1*w[0];
//				d[1] = c[1]+0.1*w[1];
//				d[2] = c[2]+0.1*w[2];
//
//				double dx, dy;
//				draw_camera->vectorToPixelIndex(d, dx, dy);
//
//				line(draw_input,
//						Point(round(cx*16), round(cy*16)),
//						Point(round(dx*16), round(dy*16)),
//						CV_RGB(127, 127, 127), 1, CV_AA, 4);
//			}


			// comparison diff image
			static Mat prev_remap = remap;
			static Mat comp(remap_height, remap_width, CV_8UC1);
			comp.setTo(Scalar::all(0));
			for( int i = 0; i < remap_height; ++i ) {
				for( int j = 0; j < remap_width; ++j ) {
					if( mask_remap.data[i*mask_remap.step+j] < 255 ) { continue; }
					comp.data[i*comp.step[0]+j] = abs(prev_remap.data[i*prev_remap.step[0]+j]-remap.data[i*remap.step[0]+j]);
				}
			}

			// sphere warping
			static Mat mapX(remap_height, remap_width, CV_32FC1);
			mapX.setTo(Scalar::all(-1));
			static Mat mapY(remap_height, remap_width, CV_32FC1);
			mapY.setTo(Scalar::all(-1));
			makeSphereRotMaps(dest_model, mapX, mapY, mask_remap,
					sphere_radius_distance_ratio, guess);
			boost::shared_ptr<BasicRemapper> warper = boost::shared_ptr<BasicRemapper>(
					new BasicRemapper(remap_width, remap_height, mapX, mapY));
			static Mat warp(remap_height, remap_width, CV_8UC1);
			warp.setTo(Scalar::all(0));
			warper->apply(prev_remap, warp);

			// diff image
			static Mat diff(remap_height, remap_width, CV_8UC1);
			diff.setTo(Scalar::all(0));
			for( int i = 0; i < remap_height; ++i ) {
				for( int j = 0; j < remap_width; ++j ) {
					if( mask_remap.data[i*mask_remap.step+j] < 255 ) { continue; }
					diff.data[i*diff.step[0]+j] = abs(warp.data[i*warp.step[0]+j]-remap.data[i*remap.step[0]+j]);
				}
			}

			// draw roi/comp/warp/diff
			static Mat draw_roi(draw_size, draw_size, CV_8UC1);
			static Mat draw_comp(draw_size, draw_size, CV_8UC1);
			resize(remap, draw_roi, draw_roi.size());
			resize(comp, draw_comp, draw_comp.size());
			static Mat draw_warp(draw_size, draw_size, CV_8UC1);
			static Mat draw_diff(draw_size, draw_size, CV_8UC1);
			resize(warp, draw_warp, draw_warp.size());
			resize(diff, draw_diff, draw_diff.size());
			for( int i = 0; i < draw_size; i++ ) {
				uint8_t* pdraw = &draw.data[i*draw.step];
				uint8_t* proi = &draw_roi.data[i*draw_roi.step];
//				uint8_t* pcomp = &draw_comp.data[i*draw_comp.step];
//				uint8_t* pwarp = &draw_warp.data[i*draw_warp.step];
				uint8_t* pdiff = &draw_diff.data[i*draw_diff.step];
				for( int j = 0; j < draw_size; j++ ) {
					pdraw[3*(j+2*draw_size)+0] = proi[j];
					pdraw[3*(j+2*draw_size)+1] = proi[j];
					pdraw[3*(j+2*draw_size)+2] = proi[j];

					pdraw[3*(j+3*draw_size)+0] = pdiff[j];
					pdraw[3*(j+3*draw_size)+1] = pdiff[j];
					pdraw[3*(j+3*draw_size)+2] = pdiff[j];
				}
			}

			// sphere/orient
			static Mat draw_sphere(draw_size, 2*draw_size, CV_8UC1);
			static Mat draw_orient(draw_size, 2*draw_size, CV_8UC1);
			sphere.drawDebug(draw_sphere, draw_orient);
			for( int i = 0; i < draw_sphere.rows; i++ ) {
				uint8_t* psphere = &draw_sphere.data[i*draw_sphere.step];
				uint8_t* porient = &draw_orient.data[i*draw_orient.step];
				for( int j = 0; j < draw_sphere.cols; j++ ) {
					draw.data[(i+draw_size)*draw.step+3*(j+2*draw_size)+0] = porient[j];
					draw.data[(i+draw_size)*draw.step+3*(j+2*draw_size)+1] = porient[j];
					draw.data[(i+draw_size)*draw.step+3*(j+2*draw_size)+2] = porient[j];

					draw.data[(i+2*draw_size)*draw.step+3*(j+2*draw_size)+0] = psphere[j];
					draw.data[(i+2*draw_size)*draw.step+3*(j+2*draw_size)+1] = psphere[j];
					draw.data[(i+2*draw_size)*draw.step+3*(j+2*draw_size)+2] = psphere[j];
				}
			}

			// paper map
			{
				int npts = std::min(int(map_hist.size()), 1000);
				double res = map_hist.size()/double(npts);
				double minx = 0, maxx = 0, miny = 0, maxy = 0;
				for( int i = 0; i < npts; i++ ) {
					int it = i*res+0.5;
					double x = map_hist[it].x, y = map_hist[it].y;
					if( x < minx )
						minx = x;
					if( x > maxx )
						maxx = x;
					if( y < miny )
						miny = y;
					if( y > maxy )
						maxy = y;
				}
				double scl = 1;
				if( npts > 1 ) {
					double sclx = double(draw_size-8)/(2.0*std::max(fabs(minx), fabs(maxx)));
					double scly = double(draw_size-4)/std::max(fabs(miny), fabs(maxy));
					scl = std::min(sclx, scly);
				}
				double cx = draw_size, cy = 2.5*draw_size;
				double ppx = cx, ppy = cy;
				for( int i = 0; i < npts; i++ ) {
					int it = i*res+0.5;
					double px = cx+scl*map_hist[it].y, py = cy-scl*map_hist[it].x;
					cv::line(draw,
							Point(round(ppx*16), round(ppy*16)),
							Point(round(px*16), round(py*16)),
							CV_RGB(255, 255, 255), 1, CV_AA, 4);
					ppx = px;
					ppy = py;
				}
			}

			// draw quad axes
//			{
//				// get initial corner positions
//				double o0[3] = {0.0, 0.0, 0.0};
//				double x0[3] = {50.0, 0.0, 0.0};
//				double y0[3] = {0.0, 50.0, 0.0};
//				double z0[3] = {0.0, 0.0, 50.0};
//
//				// rotate corners
//				double o1[3] = {0};
//				Maths::MAT_T_MUL_VEC(Rw, o0, o1);
//				double x1[3] = {0};
//				Maths::MAT_T_MUL_VEC(Rw, x0, x1);
//				double y1[3] = {0};
//				Maths::MAT_T_MUL_VEC(Rw, y0, y1);
//				double z1[3] = {0};
//				Maths::MAT_T_MUL_VEC(Rw, z0, z1);
//
//				double cx = draw_input.cols/2.0, cy = draw_input.rows/2.0;
//
//				circle(draw_input,
//						Point(round(cx*16), round(16*cy)),
//						8, CV_RGB(255,255,255), 1, CV_AA, 4);
//
//				line(draw_input,
//						Point(round(cx*16), round(cy*16)),
//						Point(round((cx+x1[0])*16), round((cy+x1[1])*16)),
//						CV_RGB(255, 255, 255), 1, CV_AA, 4);
//				line(draw_input,
//						Point(round(cx*16), round(cy*16)),
//						Point(round((cx+y1[0])*16), round((cy+y1[1])*16)),
//						CV_RGB(255, 255, 255), 1, CV_AA, 4);
//				line(draw_input,
//						Point(round(cx*16), round(cy*16)),
//						Point(round((cx+z1[0])*16), round((cy+z1[1])*16)),
//						CV_RGB(255, 255, 255), 1, CV_AA, 4);
//
//				drawText(draw_input, "x",
//						cx+x1[0], cy+x1[1], 1, 255, 255, 255, true, 0);
//				drawText(draw_input, "y",
//						cx+y1[0], cy+y1[1], 1, 255, 255, 255, true, 0);
//				drawText(draw_input, "z",
//						cx+z1[0], cy+z1[1], 1, 255, 255, 255, true, 0);
//			}

			// draw bee position history
			{
				double up[3] = {0,0,-1};
				double up_cam[3];
				Maths::MAT_T_MUL_VEC(Rw, up, up_cam);
				// store current pos
				double curr_pos[3] = {0,0,0}, curr_pos_mat[9];
				Maths::MAT_T_MUL_VEC(roi_rot_mat, &pos_hist[3*(hist_it%hist_length)], curr_pos);
				Maths::ANGLE_AXIS_TO_MAT(curr_pos, curr_pos_mat);
				double vec[3] = {up_cam[0], up_cam[1], up_cam[2]};
				Maths::SCALE_VEC(vec, sphere_radius_distance_ratio);
				double ppx = draw_input.cols/2.0, ppy = draw_input.rows/2.0;
				if( vec[2] < 0 ) {
					vec[2] += 1;
					draw_camera->vectorToPixelIndex(vec, ppx, ppy);
				}
				for( int i = 1; i < hist_length; i++ ) {
					int j = hist_it-i;
					while( j < 0 ) { j += hist_length; }
					double pos[3] = {0,0,0};
					Maths::MAT_T_MUL_VEC(roi_rot_mat, &pos_hist[3*(j%hist_length)], pos);

					double pos_mat[9];
					Maths::ANGLE_AXIS_TO_MAT(pos, pos_mat);

					// sphere-relative pos in cam coords
					double vec_tmp[3];
					Maths::MAT_MUL_VEC(pos_mat, up_cam, vec_tmp);
					double vec[3];
					Maths::MAT_T_MUL_VEC(curr_pos_mat, vec_tmp, vec);
					Maths::NORMALISE_VEC(vec);
					Maths::SCALE_VEC(vec, sphere_radius_distance_ratio);

					// sphere is centred at (0,0,1) cam coords, with r
					double px = draw_input.cols/2.0, py = draw_input.rows/2.0;

					// cam-relative pos in cam coords
					vec[2] += 1;

					draw_camera->vectorToPixelIndex(vec, px, py);
					float d = sqrt((px-ppx)*(px-ppx)+(py-ppy)*(py-ppy));
					if( (vec[2] < 1) && (d < draw_size/4.0) ) {
//						int r = 0;
//						int g = int(255.0*(1.0-i/float(hist_length))+0.5);
//						int b = int(255.0*(1.0-i/float(hist_length))+0.5);

						float mix = (i+0.5)/float(hist_length);

						int r = mix*draw_input.data[int(py+0.5)*draw_input.step+3*int(px+0.5)+2]+(1-mix)*0;
						int g = mix*draw_input.data[int(py+0.5)*draw_input.step+3*int(px+0.5)+1]+(1-mix)*255;
						int b = mix*draw_input.data[int(py+0.5)*draw_input.step+3*int(px+0.5)+0]+(1-mix)*255;

						cv::line(draw_input,
								Point(round(px*16), round(py*16)),
								Point(round(ppx*16), round(ppy*16)),
								CV_RGB(r,g,b), 1, CV_AA, 4);
					}
					ppx = px;
					ppy = py;
				}
			}

			// draw text
			char times[128];
			time_t t = time(NULL);
			strftime(times, 20, "FicTrac %d/%m/%Y", localtime(&t));
			drawText(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					string(times),
					2, 15, 1.0, 0, 255, 255, true, -1);
			drawText(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					string("input image"),
					2, 2*draw_size-8, 1.0, 0, 255, 255, true, -1);
			drawText(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					string("flat path"),
					2, 3*draw_size-8, 1.0, 0, 255, 255, true, -1);
			drawText(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					string("accumulated map"),
					2*draw_size+3, 3*draw_size-8, 1.0, 0, 255, 255, true, -1);
			drawText(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					string("sphere ROI"),
					2*draw_size+3, 1*draw_size-8, 1.0, 0, 255, 255, true, -1);
			drawText(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					string("instant map"),
					2*draw_size+3, 2*draw_size-8, 1.0, 0, 255, 255, true, -1);
			drawText(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					string("warped diff"),
					3*draw_size+3, 1*draw_size-8, 1.0, 0, 255, 255, true, -1);

			// draw lines
			drawLine(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					2*draw_size, 0, 2*draw_size, 3*draw_size,
					255, 255, 255, 2);
			drawLine(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					0, 2*draw_size, 4*draw_size, 2*draw_size,
					255, 255, 255, 2);
			drawLine(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					3*draw_size, 0, 3*draw_size, 1*draw_size,
					255, 255, 255, 2);
			drawLine(draw.data, draw.cols, draw.rows,
					draw.channels(), draw.step,
					2*draw_size, 1*draw_size, 4*draw_size, 1*draw_size,
					255, 255, 255, 2);

			if( do_display ) {
				namedWindow("FicTrac-debug");
				imshow("FicTrac-debug", draw);
	//			cvMoveWindow("FicTrac-debug", 10, 10);
			}

			if( save_video ) {
				cvtColor(draw, draw, CV_BGR2RGB);
				writer->enqueue_frame(draw);
			}

            // Add a frame to the fmf movie
            if( fmf_save.compare("") != 0 ) {
                try{
                    //To save the full black and white image
                    //Mat channel;
                    //cvtColor(draw, channel, CV_BGR2GRAY);
                    //fmf->enqueue_frame(channel, Utils::GET_CLOCK());
                    
                    //To save the full color image
                    cvtColor(draw, draw, CV_BGR2RGB);
                    fmf->enqueue_frame(draw, Utils::GET_CLOCK());
                    
                    //To save subregions follow a similar pattern
                    //For black and white:
                    //
                    //Mat channel;
                    //cvtColor(draw, channel, CV_BGR2GRAY);
                    //Mat submat = channel(Rect(36, 10, 252, 200));
                    //fmf->enqueue_frame(submat, Utils::GET_CLOCK());
                    //
                    //For color:
                    //cvtColor(draw, draw, CV_BGR2RGB);
                    //Mat submat = draw(Rect(36, 10, 252, 200));
                    //fmf->enqueue_frame(submat, Utils::GET_CLOCK());

                }catch(const std::exception &e){
                    std::cout << "C++ exception in fmfwrapper:  " << e.what() << std::endl;
                }
            }

			prev_remap = remap;
		}

		if( save_input_video ) { input_writer->enqueue_frame(frame_bgr); }

		double t5 = Utils::GET_CLOCK();

		double curr_time = Utils::GET_CLOCK();
		static double prev_time = curr_time;
		double time_waited = 1000*(curr_time-prev_time);
		prev_time = curr_time;
		double fps_wait = 1;
		if( !cam_input && fps > 0 && fps < 1000 ) {
			fps_wait = 1000.0/fps;
		}
		static double wait_time = fps_wait;
		wait_time += 0.5*(fps_wait-time_waited);
		wait_time = Maths::CLAMP(wait_time, 1.0, 10000.0);

		uint16_t key = 0;
		if( fps == 0 ) {
			// wait until user presses a key
			if( do_display ) {
				key = waitKey(0);
			} else {
				getchar();
			}
		} else if( fps > 0) {
			// if cam_input, wait time required for fps, else wait 1ms
			key = waitKey(int(wait_time+0.5));
		} else if( do_display ) {
			key = waitKey(1);
		}
		if( key == 0x73 ) {
			printf("Saving sphere template to disk (%s)...\n", template_fn.c_str());
			fflush(stdout);
			imwrite(template_fn, sphere.getTemplate());
		} else if( key == 0x70 ) {
			printf("\n  Press any key to continue...\n");
			fflush(stdout);
			if( do_display) { waitKey(0); }
		} else if( key == 0x1B ) {
			printf("Exiting program...\n");
			fflush(stdout);
			break;
//			} else {
//				printf("key pressed: %X\n", key);
		}

		double t6 = Utils::GET_CLOCK();

		printf("grab/test/save/draw/wait/total time: %.1fms/%.1fms/%.1fms/%.1fms/%.1fms/%.1fms\n",
				(t2-t1)*1e3, (t3-t2)*1e3, (t4-t3)*1e3, (t5-t4)*1e3, (t6-t5)*1e3, (t6-t1)*1e3);

		#if LOG_TIMING
			timing << (t6-t0) << " " << SEGMENT_TIME << " " << (t2-t1) << " " << (t3-t2) << " " << (t4-t3) << " " << (t5-t4) << " " << (t6-t5) << " " << (t6-t1) << endl;
		#endif

		double curr_loop = Utils::GET_CLOCK();
		static double prev_loop = curr_loop;
		static double av_loop = max(curr_loop-prev_loop, 0.001);
		av_loop += 0.05*(max(curr_loop-prev_loop, 0.001)-av_loop);
		av_loop_time += (curr_loop-prev_loop);
		printf("Loop FPS: %.1fHz\n", 1.0/av_loop);
		prev_loop = curr_loop;
	}

	/// Statistics.
	{
		if( nframes > 0 ) {
			av_err /= nframes;
			av_exec_time /= nframes;
			av_loop_time /= (nframes/1000.0);
		}
		printf("\nTotal rotation: %.1f degrees\nAverage matching error: %.1f\nAverage loop (minimisation) time: %.1fms (%.1fms)\n",
				total_dist*Maths::R2D, av_err, av_loop_time, av_exec_time);
	}

	/// Save template state.
	{
		printf("\nSaving sphere template to disk (%s)...\n", template_fn.c_str());
		fflush(stdout);
		imwrite(template_fn, sphere.getTemplate());
	}

	ACTIVE = false;

	printf("Waiting for threads to die...\n");
	fflush(stdout);

	pthread_mutex_lock(&(input->mutex));
	pthread_cond_broadcast(&(input->cond));
	pthread_mutex_unlock(&(input->mutex));
	pthread_join(input->thread, 0);
	pthread_cond_destroy(&input->cond);
	pthread_mutex_destroy(&input->mutex);

	pthread_mutex_lock(&(log->mutex));
	pthread_cond_broadcast(&(log->cond));
	pthread_mutex_unlock(&(log->mutex));
	pthread_join(log->thread, 0);
	pthread_cond_destroy(&log->cond);
	pthread_mutex_destroy(&log->mutex);

	if( do_socket_out ) {
		pthread_join(_socket->thread, 0);
		pthread_mutex_destroy(&_socket->mutex);
	}

	#if LOG_TIMING
		timing.close();
	#endif

	if( do_serial_out ) { serial_close(&_serial); }
}


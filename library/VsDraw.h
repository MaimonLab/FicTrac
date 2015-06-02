/*
 * VsDraw.h
 *
 *  Created on: 14/09/2011
 *      Author: Saul Thurrowgood (saul@fastmail.com.au)
 */

#ifndef VSDRAW_H_
#define VSDRAW_H_


#include "CameraModel.h"
#include <opencv2/opencv.hpp>
#include <string>


///
/// Drawing on the 3D view sphere. The most basic operation is the drawing
/// of an arc on the view sphere.
///
class VsDraw
{
public:
	typedef boost::shared_ptr<VsDraw> Ptr;

	VsDraw();
	VsDraw(const VsDraw& copy);
	~VsDraw();

	void openImage(const CameraModelPtr& cam, cv::Mat& bgr);

	///
	/// Display is only valid for image targets and prior to close().
	///
	void display(const std::string& name);
	void close();

	///
	/// Drawing onto the view sphere by segmenting into shorter lines
	/// so that mapping onto arbitrary image projections gives the
	/// appearance of correct 3D view sphere geometry.
	///
	void arc(CmPoint axis, double radius, CmPoint start, double extent, int nSeg=32, bool filled=false);
	void line(CmPoint p0, CmPoint p1, int nSeg=32);
	void circle(CmPoint axis, double radius, int nSeg=64, bool filled=false);

	///
	/// Non-segmented primitives directly projected onto view sphere.
	/// The difference is that these do not perform a piecewise approximation
	/// of a curve on the view sphere, so when projected to arbitrary
	/// image mappings the path is not related to the 3D path, only the
	/// end points are correct.
	///
	/// Point radius is given in radians and the actual size of the circle
	/// drawn in image space will be determined by the local derivative of
	/// pixels per radian.  A negative radius forces the point to be drawn
	/// in units of pixels radius rather than angle.
	///
	void point(CmPoint p, double radius, bool filled=true);
	void cross(CmPoint centre, double radius);

	///
	/// State that affects drawing.
	///
	void reset();
	void alpha(double a);
	void color(const cv::Scalar& c) { _color=c; _color[3]=255; }
	void color(int r, int g, int b, int a=255) { _color = cv::Scalar(b,g,r,a); }
	void grey(int g, int a=255) { _color = cv::Scalar(g,g,g,a); }
	void thickness(double i) { _thickness = i; }
	void lineType(double i) { _lineType = i; }
	void shift(int i) { _shift = i; _scale = 1<<i; }

	// rjdm hack
	void copyImage(cv::Mat& rgb) { cv::cvtColor(_img, rgb, CV_BGRA2BGR); }

private:
	void _start();
	bool _linePoint(CmPoint& prev, CmPoint& prevPix, const CmPoint& p, bool force=false);
	bool _testLinePoint(CmPoint& prev, CmPoint& prevPix, const CmPoint& p);
	void _end();

	/// avoid infecting world with GTK-style include file nightmare!
	SHARED_PTR(Internal);
	InternalPtr _internal;

	CameraModelPtr _cam;
	cv::Mat _img, _srcImg;
	int _w, _h;
	double _alpha, _thickness;
	int _lineType, _shift, _scale;
	cv::Scalar _color;
	double _offX, _offY;
	CmPoint _polyP0, _polyPrevP, _polyPrevPix;
};


#endif /* VSDRAW_H_ */

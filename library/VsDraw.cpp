/*
 * VsDraw.cpp
 *
 *  Created on: 14/09/2011
 *      Author: Saul Thurrowgood (saul@fastmail.com.au)
 */

#include "VsDraw.h"

#include <opencv2/highgui/highgui.hpp>

#define CAIRO_HAS_PNG_FUNCTIONS 1
#define CAIRO_HAS_PDF_SURFACE 1
#define CAIRO_HAS_PS_SURFACE 1
#define CAIRO_HAS_SVG_SURFACE 1
#include <cairo/cairo.h>
#include <cairomm/cairomm.h>
#include <sigc++/sigc++.h>


class VsDraw::Internal
{
public:
	static InternalPtr create(Cairo::RefPtr<Cairo::Surface> surface)
	{
		InternalPtr ret = InternalPtr(new Internal);
		ret->_surface = surface;
		ret->_cr = Cairo::Context::create(surface);
		return ret;
	}

	bool isImageSurface()
	{
		return dynamic_cast<Cairo::ImageSurface*>(&surfaceRef());
	}

	Cairo::Context& contextRef()
	{
		///
		/// Dodgy use of dereference operator to get at internal pointer :)
		///
		return *_cr.operator->();
	}

	Cairo::Surface& surfaceRef()
	{
		///
		/// Dodgy use of dereference operator to get at internal pointer :)
		///
		return *_surface.operator->();
	}

	Cairo::RefPtr<Cairo::Surface>& surface()
	{
		return _surface;
	}

	InternalPtr clone()
	{
		InternalPtr ret = InternalPtr(new Internal);
		ret->_surface = _surface;
		ret->_cr = _cr;
		return ret;
	}

private:
	Internal() {}
	Cairo::RefPtr<Cairo::Surface> _surface;
	Cairo::RefPtr<Cairo::Context> _cr;
};


VsDraw::VsDraw()
{
	reset();
}

VsDraw::VsDraw(const VsDraw& copy) :
		_internal(copy._internal->clone()),
		_cam(copy._cam),
		_img(copy._img),
		_srcImg(copy._srcImg),
		_w(copy._w),
		_h(copy._h),
		_alpha(copy._alpha),
		_thickness(copy._thickness),
		_lineType(copy._lineType),
		_shift(copy._shift),
		_scale(copy._scale),
		_color(copy._color),
		_offX(copy._offX),
		_offY(copy._offY)
{
}

VsDraw::~VsDraw()
{
	close();
}

void VsDraw::reset()
{
	if (_cam) {
		_w = _cam->width();
		_h = _cam->height();
	}
	alpha(1);
	color(0,0,0);
	thickness(1);
	lineType(CV_AA);
	shift(8);
}

void VsDraw::alpha(double a)
{
	_alpha = std::max(0.0, std::min(1.0, a));
}

void VsDraw::openImage(const CameraModelPtr& cam, cv::Mat& bgr)
{
	_cam = cam;
	reset();
	if (bgr.cols != _w || bgr.rows != _h || bgr.type() != CV_8UC3) {
		bgr.create(_h, _w, CV_8UC3);
		bgr.setTo(cv::Scalar::all(255));
	}
	_srcImg = bgr;
	cv::cvtColor(_srcImg, _img, CV_BGR2BGRA);
	Cairo::RefPtr<Cairo::Surface> surface = Cairo::ImageSurface::create(
			_img.data, Cairo::FORMAT_ARGB32,
			_img.cols, _img.rows, _img.step);
	_internal = Internal::create(surface);
}

void VsDraw::display(const std::string& name)
{
	if (_internal && _internal->isImageSurface()) {
		cv::namedWindow(name);
		cv::imshow(name, _img);
	}
}

void VsDraw::close()
{
	if (_internal) {
		if (_internal->isImageSurface() && !_srcImg.empty())
			cv::cvtColor(_img, _srcImg, CV_BGRA2BGR);
		_internal = InternalPtr();
	}
}

void VsDraw::_start()
{
	Cairo::Context& cr = _internal->contextRef();
	cr.set_source_rgba(
			_color[2]*(1.0/255),
			_color[1]*(1.0/255),
			_color[0]*(1.0/255),
			_alpha * _color[3]*(1.0/255)); /// multiply global alpha value
	cr.set_line_width(_thickness);
	cr.set_line_cap(Cairo::LINE_CAP_ROUND);
	cr.set_line_join(Cairo::LINE_JOIN_ROUND);
}

void VsDraw::_end()
{
	Cairo::Context& cr = _internal->contextRef();
	cr.stroke();
}

void VsDraw::point(CmPoint p, double radius, bool filled, bool shadow)
{
	CmPoint pix;
	if (_cam->vectorToPixel(p, pix.x, pix.y)) {
		if (radius < 0) {
			radius = -radius;
		} else {
			CmPoint p1;
			if (!_cam->pixelToVector(pix.x+1e-3, pix.y+1e-3, p1))
				return;
			double dpixLen = 0.0014142136; /// 1e-3 * sqrt(2)
			double pixelPerRadian = dpixLen / p.getAngleTo(p1);
			radius *= pixelPerRadian;
		}
		if (radius > 1e6) {
			///
			/// Avoid inf radii which can crash the program.
			///
			radius = 1e6;
		}

		// shadow
		if( shadow ) {
			cv::Scalar c = _color;
			_color = cv::Scalar(0,0,0,255);
			_start();
			Cairo::Context& cr = _internal->contextRef();
			cr.arc(pix.x+_offX+1, pix.y+_offY+1, radius, 0, 2*CM_PI);
			_end();
			_color = c;
		}

		Cairo::Context& cr = _internal->contextRef();
		_start();
		cr.arc(pix.x+_offX, pix.y+_offY, radius, 0, 2*CM_PI);
		if (filled)
			cr.fill_preserve();
		_end();
	}
}

void VsDraw::cross(CmPoint centre, double radius, bool shadow)
{
	CmPoint pix;
	if (_cam->vectorToPixel(centre, pix.x, pix.y)) {
		if (radius < 0) {
			radius = -radius;
		} else {
			CmPoint p1;
			if (!_cam->pixelToVector(pix.x+1e-3, pix.y+1e-3, p1))
				return;
			double dpixLen = 0.0014142136; /// 1e-3 * sqrt(2)
			double pixelPerRadian = dpixLen / centre.getAngleTo(p1);
			radius *= pixelPerRadian;
		}
		if (radius > 1e6) {
			///
			/// Avoid inf radii which can crash the program.
			///
			radius = 1e6;
		}
		pix.x += _offX;
		pix.y += _offY;

		// shadow
		if( shadow ) {
			cv::Scalar c = _color;
			_color = cv::Scalar(0,0,0,255);
			_start();
			Cairo::Context& cr = _internal->contextRef();
			cr.move_to(pix.x-radius+1, pix.y-radius+1);
			cr.line_to(pix.x+radius+1, pix.y+radius+1);
			cr.move_to(pix.x+radius+1, pix.y-radius+1);
			cr.line_to(pix.x-radius+1, pix.y+radius+1);
			_end();
			_color = c;
		}

		_start();
		Cairo::Context& cr = _internal->contextRef();
		cr.move_to(pix.x-radius, pix.y-radius);
		cr.line_to(pix.x+radius, pix.y+radius);
		cr.move_to(pix.x+radius, pix.y-radius);
		cr.line_to(pix.x-radius, pix.y+radius);
		_end();
	}
}

///
/// Returns whether the path is unbroken i.e. whether it would be safe
/// to create a closed path from final path point.
///
bool VsDraw::_linePoint(CmPoint& prev, CmPoint& prevPix, const CmPoint& p, bool force, bool shadow)
{
	///
	/// NOTE: Cairo uses continuous pixel coordinates, the same as the
	///       CameraModel class, so we do *not* convert to pixel index.
	///

	Cairo::Context& cr = _internal->contextRef();
	if (prev.len2() < 1e-9) {
		///
		/// Begin a new line.
		///
		CmPoint pix;
		if (_cam->vectorToPixel(p, pix.x, pix.y)) {
			if( shadow ) { pix.x += 1; pix.y += 1; }
			cr.move_to(pix.x+_offX, pix.y+_offY);
			prev = p;
			prevPix = pix;
		} else {
			/// Indicate next point is a new line
			prev = 0;
			prevPix = -1e30;
			return false;
		}
	} else {
		CmPoint pix;
		if (_cam->vectorToPixel(p, pix.x, pix.y)) {
			if( shadow ) { pix.x += 1; pix.y += 1; }
			if (force || (
					fabs(prevPix.x-pix.x) < (_w/3)
					&& fabs(prevPix.y-pix.y) < (_h/3)))
			{
				cr.line_to(pix.x+_offX, pix.y+_offY);
				prev = p;
				prevPix = pix;
			} else {
				/// Indicate next point is a new line
				prev = 0;
				prevPix = -1e30;
				return false;
			}
		} else {
			/// Indicate next point is a new line
			prev = 0;
			prevPix = -1e30;
			return false;
		}
	}
	return true;
}

///
/// Copy of _linePoint() but without modifying anything.
///
bool VsDraw::_testLinePoint(CmPoint& prev, CmPoint& prevPix, const CmPoint& p, bool shadow)
{
	if (prev.len2() < 1e-9) {
		CmPoint pix;
		if (_cam->vectorToPixel(p, pix.x, pix.y)) {
			if( shadow ) { pix.x += 1; pix.y += 1; }
		} else {
			/// Indicate next point is a new line
			return false;
		}
	} else {
		CmPoint pix;
		if (_cam->vectorToPixel(p, pix.x, pix.y)) {
			if( shadow ) { pix.x += 1; pix.y += 1; }
			if (fabs(prevPix.x-pix.x) < (_w/3) && fabs(prevPix.y-pix.y) < (_h/3)) {
			} else {
				/// Indicate next point is a new line
				return false;
			}
		} else {
			/// Indicate next point is a new line
			return false;
		}
	}
	return true;
}

void VsDraw::arc(
		CmPoint axis, double radius, CmPoint start, double extent,
		int nSeg, bool filled, bool shadow)
{
	if (nSeg < 1)
		nSeg = 1;
	CmPoint first = axis.getRotatedAbout(axis^start, radius);
	CmPoint omega = axis.getNormalised() * (extent / nSeg);

	// shadow
	if( shadow ) {
		cv::Scalar c = _color;
		_color = cv::Scalar(0,0,0,255);
		_start();
		CmPoint prev, prevPix;
		_linePoint(prev, prevPix, first, false, true);
		Cairo::Context& cr = _internal->contextRef();
		bool unbroken = true;
		for (int i=1; i<nSeg; ++i) {
			CmPoint p = first.getRotatedAbout(i * omega);
			if (!_linePoint(prev, prevPix, p, false, true))
				unbroken = false;
		}
		CmPoint p = first.getRotatedAbout(nSeg * omega);
		if (fabs(extent) >= (2*CM_PI - 1e-9)
				&& unbroken && _testLinePoint(prev, prevPix, p, true))
		{
			cr.close_path();
		} else {
			_linePoint(prev, prevPix, p, true);
		}
		_end();
		_color = c;
	}

	_start();
	CmPoint prev, prevPix;
	_linePoint(prev, prevPix, first);
#if 0
	for (int i=1; i<=nSeg; ++i) {
		CmPoint p = first.getRotatedAbout(i * omega);
		_linePoint(prev, prevPix, p);
	}
#else
	Cairo::Context& cr = _internal->contextRef();
	bool unbroken = true;
	for (int i=1; i<nSeg; ++i) {
		CmPoint p = first.getRotatedAbout(i * omega);
		if (!_linePoint(prev, prevPix, p))
			unbroken = false;
	}
	CmPoint p = first.getRotatedAbout(nSeg * omega);
	if (fabs(extent) >= (2*CM_PI - 1e-9)
			&& unbroken && _testLinePoint(prev, prevPix, p))
	{
		cr.close_path();
	} else {
		_linePoint(prev, prevPix, p);
	}
	if (filled)
		cr.fill_preserve();
#endif
	_end();
}

void VsDraw::circle(CmPoint axis, double radius, int nSeg, bool filled)
{
	CmPoint perp;
	if (fabs(axis.x) < fabs(axis.z))
		perp = CmPoint(1,0,0) ^ axis;
	else
		perp = CmPoint(0,0,1) ^ axis;
	arc(axis, radius, perp, 360*CM_D2R, std::max(3,nSeg), filled);
}

void VsDraw::line(CmPoint p0, CmPoint p1, int nSeg, bool shadow)
{
	///
	/// Draw in angular pieces for reasonable use of resolution.
	/// Note that a straight line always passes along a great circle
	/// when projected onto the view sphere, so use arc().
	///
	/// This is undefined when p0 = -p1, but then the line truly
	/// maps to the entire view sphere since the observer is *inside*
	/// the line.
	///
	CmPoint axis = p0 ^ p1;
	double angle = p0.getAngleTo(p1);
	arc(axis, 90*CM_D2R, p0, angle, nSeg, false, shadow);
}

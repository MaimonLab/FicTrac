/*
 * PGRSource.h
 *
 *  Created on: May 25, 2013
 *      Author: rjdmoore@uqconnect.edu.au
 */
#ifdef PGR_CAMERA

#ifndef PGRSOURCE_H_
#define PGRSOURCE_H_

#include "ImgSource.h"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <flycapture/FlyCapture2.h>

class PGRSource : public ImgSource {
public:
	PGRSource(int index=0);
	virtual ~PGRSource();

	virtual void setFPS(int fps);
	virtual void rewind();
//	virtual void skip(unsigned int frames);
	virtual bool grab(cv::Mat& frame);

private:
	boost::shared_ptr<FlyCapture2::Camera> _cap;
	FlyCapture2::Image _frame_cap;
};

#endif /* PGRSOURCE_H_ */

#endif

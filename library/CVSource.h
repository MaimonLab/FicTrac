/*
 * CVSource.h
 *
 *  Created on: May 25, 2013
 *      Author: rjdmoore
 */

#ifndef CVSOURCE_H_
#define CVSOURCE_H_

#include "ImgSource.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/shared_ptr.hpp>

class CVSource : public ImgSource {
public:
	CVSource(int index=0);
	CVSource(std::string filename);
	virtual ~CVSource();

	virtual void setFPS(int fps);
	virtual void rewind();
//	virtual void skip(unsigned int frames);
	virtual bool grab(cv::Mat& frame);

private:
	boost::shared_ptr<cv::VideoCapture> _cap;
	cv::Mat _frame_cap;
};

#endif /* CVSOURCE_H_ */

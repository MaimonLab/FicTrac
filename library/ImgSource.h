/*
 * ImgSource.h
 *
 *  Created on: May 25, 2013
 *      Author: rjdmoore@uqconnect.edu.au
 */

#ifndef IMGSOURCE_H_
#define IMGSOURCE_H_

#include <opencv2/opencv.hpp>

enum BAYER_TYPE { BAYER_NONE, BAYER_RGGB, BAYER_GRBG, BAYER_GBRG, BAYER_BGGR };

class ImgSource {
public:
	ImgSource();
	virtual ~ImgSource();

	virtual void setFPS(int fps)=0;
	virtual void rewind()=0;
//	virtual void skip(unsigned int)=0;
	virtual bool grab(cv::Mat& frame)=0;

	bool isOpen() { return _open; }
	int getWidth() { return _width; }
	int getHeight() { return _height; }
	double getTimestamp() { return _timestamp; }
	void setBayerType(BAYER_TYPE bayer_type) { _bayerType = bayer_type; }

protected:
	bool _open;
	BAYER_TYPE _bayerType;
	int _width, _height;
	double _timestamp;
};

#endif /* IMGSOURCE_H_ */

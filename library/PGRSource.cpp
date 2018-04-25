/*
 * PGRSource.cpp
 *
 *  Created on: May 25, 2013
 *      Author: rjdmoore@uqconnect.edu.au
 */
  
#ifdef PGR_CAMERA
 
#include "PGRSource.h"
#include <flycapture/FlyCapture2.h>
#include <flycapture/Error.h>
#include <flycapture/Image.h>

using cv::Mat;
using namespace FlyCapture2;

PGRSource::PGRSource(int index)
{
	printf("%s: looking for camera at index %d...\n", __func__, index);

	BusManager busMgr;
	PGRGuid guid;
	Error error = busMgr.GetCameraFromIndex(index, &guid);
	if( error != PGRERROR_OK ) {
		fprintf(stderr, "%s: Error reading camera GUID!\n", __func__);
		return;
	}
	FlyCapture2::Camera *cam = new FlyCapture2::Camera();
	_cap = boost::shared_ptr<Camera>(new Camera());
	error = cam->Connect(&guid);
	if( error != PGRERROR_OK ) {
		fprintf(stderr, "%s: Error connecting to camera!\n", __func__);
		return;
	}

	CameraInfo camInfo;
	error = cam->GetCameraInfo(&camInfo);
	if( error != PGRERROR_OK ) {
		fprintf(stderr, "%s: Error retrieving camera information!\n", __func__);
		return;
	} else {
		printf("Connected to PGR camera (%s/%s max res: %s)\n", camInfo.modelName, camInfo.sensorInfo, camInfo.sensorResolution);
	}

	FlyCapture2::FC2Config cfg;
	cam->GetConfiguration(&cfg);
	cfg.numBuffers = 6;
	cam->SetConfiguration(&cfg);


	printf("Got Here 1\n");
	error = cam->StartCapture();
	if( error != PGRERROR_OK ) {
		fprintf(stderr, "%s: Error starting video capture!\n", __func__);
		return;
	}
	printf("Got Here 2\n");
	Image::SetDefaultColorProcessing(ColorProcessingAlgorithm::NEAREST_NEIGHBOR);
	printf("Got Here 3\n");
	// capture test image
	Image testImg;
	error = cam->RetrieveBuffer(&testImg);
	printf("Got Here 4\n");
	if( error != PGRERROR_OK ) {
		fprintf(stderr, "%s: Error capturing image!\n", __func__);
		return; 
	}
	_width = testImg.GetCols();
	_height = testImg.GetRows();
	_open = true;
	_cap = boost::shared_ptr<Camera>(cam);
}

void PGRSource::setFPS(int fps)
{
	// do nothing
}

void PGRSource::rewind()
{
	// do nothing
}

//void PGRSource::skip(unsigned int frames)
//{
//	// do nothing
//}

bool PGRSource::grab(cv::Mat& frame)
{
	if( !_open ) { return NULL; }

	Error error = _cap->RetrieveBuffer(&_frame_cap);
	if( error != PGRERROR_OK ) {
		fprintf(stderr, "%s: Error grabbing image frame!\n", __func__);
		//getchar();
		return false;
	}
	TimeStamp ts = _frame_cap.GetTimeStamp();
	_timestamp = ts.seconds+ts.microSeconds/(double)1e6;

	Image frame_bgr;
	error = _frame_cap.Convert(PIXEL_FORMAT_BGR, &frame_bgr);
	if( error != PGRERROR_OK ) {
		fprintf(stderr, "%s: Error converting image format!\n", __func__);
		return false;
	}
	Mat frame_cv(frame_bgr.GetRows(), frame_bgr.GetCols(), CV_8UC3, frame_bgr.GetData(), frame_bgr.GetStride());
	frame_cv.copyTo(frame);
	return true;
}

PGRSource::~PGRSource()
{
	if( _open ) {
		_cap->StopCapture();
	}
	_cap->Disconnect();
}

#endif

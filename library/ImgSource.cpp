/*
 * ImgSource.cpp
 *
 *  Created on: May 25, 2013
 *      Author: rjdmoore
 */

#include "ImgSource.h"

ImgSource::ImgSource()
	: _open(false), _bayerType(BAYER_NONE), _width(-1), _height(-1), _timestamp(-1)
{}

ImgSource::~ImgSource() {}


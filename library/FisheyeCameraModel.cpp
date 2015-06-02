///-----------------------------------------------------------------------------
/// FisheyeCameraModel implementation (Saul Thurrowgood, 2008)
///-----------------------------------------------------------------------------

#include "CameraModelPrivate.h"


class FisheyeCameraModel : public CameraModel
{
public:
	FisheyeCameraModel(
		int width, int height,
		CmReal radPerPixel, CmReal imageCircleFOV,
		CmReal centreX=-1, CmReal centreY=-1);
	virtual bool pixelToVector(CmReal x, CmReal y, CmReal direction[3]) const;
	virtual bool vectorToPixel(const CmReal point[3], CmReal& x, CmReal& y) const;
	virtual bool validPixel(CmReal x, CmReal y) const;
	virtual CmReal getFOV() const {
		return _imageCircleFOV;
	}

private:
	CmReal _radPerPixel;
	CmReal _imageCircleFOV, _imageCircleR2;
	CmReal _xc, _yc;

	bool _validPixel(CmReal x, CmReal y, CmReal R2) const {
		return (R2 <= _imageCircleR2) && _validXY(x,y);
	}
};

CameraModelPtr CameraModel::createFisheye(
	int width, int height, CmReal radPerPixel, CmReal imageCircleFOV,
	CmReal centreX, CmReal centreY)
{
	return CameraModelPtr(
		new FisheyeCameraModel(
			width, height,
			radPerPixel, imageCircleFOV,
			centreX, centreY));
}

FisheyeCameraModel::FisheyeCameraModel(
	int width, int height,
	CmReal radPerPixel, CmReal imageCircleFOV,
	CmReal centreX, CmReal centreY)
	: CameraModel(width, height)
{
	_radPerPixel = radPerPixel;
	_imageCircleFOV = imageCircleFOV;
	_xc = (centreX==-1) ? width*0.5 : centreX;
	_yc = (centreY==-1) ? height*0.5 : centreY;

	CmReal R = (imageCircleFOV * 0.5) / radPerPixel;
	_imageCircleR2 = R * R;
}

bool FisheyeCameraModel::pixelToVector(
	CmReal x, CmReal y, CmReal direction[3]) const
{
	CmReal dx = x - _xc;
	CmReal dy = y - _yc;
	CmReal R2 = dx*dx + dy*dy;
	CmReal R = sqrt(R2);
	CmReal alpha = R * _radPerPixel; // Angle from forward.
	CmReal sinAlpha = sin(alpha);
	CmReal xyScale = (R > 1e-7) ? (sinAlpha / R) : sinAlpha;
	direction[0] = dx * xyScale;
	direction[1] = dy * xyScale;
	direction[2] = cos(alpha);
	return _validPixel(x, y, R2);
}

bool FisheyeCameraModel::vectorToPixel(
	const CmReal point[3], CmReal& x, CmReal& y) const
{
	CmReal r[3] = {point[0], point[1], point[2]};
	vec3normalise(r);
	CmReal alpha = acos(r[2]); // Angle from forward.
	CmReal R = alpha / _radPerPixel;
	CmReal sinAlpha2 = r[0]*r[0] + r[1]*r[1];
	CmReal xyScale = (sinAlpha2 > 1e-14) ? R / sqrt(sinAlpha2) : 0.0;
	x = r[0]*xyScale + _xc;
	y = r[1]*xyScale + _yc;
	return _validPixel(x, y, R*R);
}

bool FisheyeCameraModel::validPixel(CmReal x, CmReal y) const
{
	CmReal dx = x - _xc;
	CmReal dy = y - _yc;
	return _validPixel(x, y, dx*dx + dy*dy);
}

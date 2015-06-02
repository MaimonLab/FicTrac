///
/// Saul Thurrowgood, 2008.
///

#include "CameraRemap.h"
#include <opencv2/highgui/highgui.hpp>

using namespace cv;


template<typename T>
static inline T clamp(T x, T min, T max)
{
	return (x <= min) ? min : (x >= max) ? max : x;
}


CameraRemap::CameraRemap(
	const CameraModelPtr& src,
	const CameraModelPtr& dst,
	const RemapTransformPtr& trans)
	: Remapper(src->width(), src->height(), dst->width(), dst->height()),
	_src(src), _dst(dst), _trans(trans)
{
	_mapX.resize(_dstW * _dstH);
	_mapY.resize(_dstW * _dstH);
	setTransform(_trans);
}

void CameraRemap::setTransform(RemapTransformPtr trans)
{
	for (int y = 0; y < _dstH; y++) {
		for (int x = 0; x < _dstW; x++) {
			CmPoint v;
			CmReal sx, sy;
			bool valid = false;
			do {
				if (!_dst->pixelIndexToVector(x, y, v))
					break;
				if (trans)
					v = trans->inverseTransform(v);
				if (!_src->vectorToPixelIndex(v, sx, sy))
					break;
				valid = true;
			} while (false);

			int i = y*_dstW + x;
			if (valid) {
				_mapX[i] = clamp<CmReal>(sx, 0.0, _srcW-1);
				_mapY[i] = clamp<CmReal>(sy, 0.0, _srcH-1);
			} else {
				/// Set a value outside of dst image ROI.
				_mapX[i] = INVALID_MAP_VAL;
				_mapY[i] = INVALID_MAP_VAL;
			}
		}
	}
}

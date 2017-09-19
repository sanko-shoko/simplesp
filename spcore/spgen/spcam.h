//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CAM_H__
#define __SP_CAM_H__

#include "spcore/spcom.h"

namespace sp{


	//--------------------------------------------------------------------------------
	// camera parameter
	//--------------------------------------------------------------------------------

	SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1, const double fx, const double fy, const double cx, const double cy){
		CamParam dst;
		dst.dsize[0] = dsize0;
		dst.dsize[1] = dsize1;

		dst.fx = fx;
		dst.fy = fy;
		dst.cx = cx;
		dst.cy = cy;

		dst.k1 = 0.0;
		dst.k2 = 0.0;
		dst.k3 = 0.0;
		dst.p1 = 0.0;
		dst.p2 = 0.0;
		return dst;
	}

	SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1) {
		// groundless camera parameter, but in many cases empirically, no big difference
		const double f = 0.8 * (dsize0 + dsize1);
		return getCamParam(dsize0, dsize1, f, f, (dsize0 - 1) * 0.5, (dsize1 - 1) * 0.5);
	}

	SP_GENFUNC CamParam getCamParam(const int *dsize) {
		return getCamParam(dsize[0], dsize[1]);
	}

	SP_GENFUNC void getMat(double *dst, const int rows, const int cols, const CamParam &cam) {
		dst[0 * cols + 0] = cam.fx;
		dst[0 * cols + 1] = 0.0;
		dst[0 * cols + 2] = cam.cx;

		dst[1 * cols + 0] = 0.0;
		dst[1 * cols + 1] = cam.fy;
		dst[1 * cols + 2] = cam.cy;

		dst[2 * cols + 0] = 0.0;
		dst[2 * cols + 1] = 0.0;
		dst[2 * cols + 2] = 1.0;
	}


	//--------------------------------------------------------------------------------
	// camera util
	//--------------------------------------------------------------------------------

	SP_GENFUNC Vec2 mulCam(const CamParam &cam, const Vec2 &npx) {
		Vec2 pix;
		pix.x = npx.x * cam.fx + cam.cx;
		pix.y = npx.y * cam.fy + cam.cy;
		return pix;
	}

	SP_GENFUNC Vec2 invCam(const CamParam &cam, const Vec2 &pix) {
		Vec2 npx;
		npx.x = (pix.x - cam.cx) / cam.fx;
		npx.y = (pix.y - cam.cy) / cam.fy;
		return npx;
	}


}


#endif
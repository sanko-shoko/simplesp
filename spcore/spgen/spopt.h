//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_OPT_H__
#define __SP_OPT_H__

#include "spcore/spcom.h"
#include "spcore/spwrap.h"
#include "spcore/spgen/spbase.h"
#include "spcore/spgen/spvec.h"
#include "spcore/spgen/spmath.h"

#include "spcore/spgen/spcam.h"
#include "spcore/spgen/sppose.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// jacob
	//--------------------------------------------------------------------------------

	SP_GENFUNC void jacobCamToPix(double *jacob, const CamParam &cam, const Vec2 &npx) {
		const double x2 = npx.x * npx.x;
		const double y2 = npx.y * npx.y;
		const double xy = npx.x * npx.y;

		const double r2 = x2 + y2;
		const double r4 = r2 * r2;
		const double r6 = r4 * r2;

		const double k = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;

		Vec2 dist;
		dist.x = npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2);
		dist.y = npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy);

		jacob[0 * 9 + 0] = dist.x;
		jacob[0 * 9 + 1] = 0.0;
		jacob[0 * 9 + 2] = 1.0;
		jacob[0 * 9 + 3] = 0.0;
		jacob[0 * 9 + 4] = cam.fx * (npx.x * r2);
		jacob[0 * 9 + 5] = cam.fx * (npx.x * r4);
		jacob[0 * 9 + 6] = cam.fx * (npx.x * r6);
		jacob[0 * 9 + 7] = cam.fx * (2.0 * xy);
		jacob[0 * 9 + 8] = cam.fx * (2.0 * x2 + r2);

		jacob[1 * 9 + 0] = 0.0;
		jacob[1 * 9 + 1] = dist.y;
		jacob[1 * 9 + 2] = 0.0;
		jacob[1 * 9 + 3] = 1.0;
		jacob[1 * 9 + 4] = cam.fy * (npx.y * r2);
		jacob[1 * 9 + 5] = cam.fy * (npx.y * r4);
		jacob[1 * 9 + 6] = cam.fy * (npx.y * r6);
		jacob[1 * 9 + 7] = cam.fy * (2.0 * y2 + r2);
		jacob[1 * 9 + 8] = cam.fy * (2.0 * xy);
	}

	SP_GENFUNC void jacobNpxToDist(double *jacob, const CamParam &cam, const Vec2 &npx) {
		const double x2 = npx.x * npx.x;
		const double y2 = npx.y * npx.y;
		const double xy = npx.x * npx.y;

		const double r2 = x2 + y2;
		const double r4 = r2 * r2;
		const double r6 = r4 * r2;

		const double k1 = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;
		const double k2 = 2 * cam.k1 + 4 * cam.k2 * r2 + 6 * cam.k3 * r4;

		jacob[0 * 2 + 0] = x2 * k2 + k1 + 6 * cam.p2 * npx.x + 2 * cam.p1 * npx.y;
		jacob[1 * 2 + 1] = y2 * k2 + k1 + 2 * cam.p2 * npx.x + 6 * cam.p1 * npx.y;

		jacob[0 * 2 + 1] = xy * k2 + 2 * cam.p1 * npx.x + 2 * cam.p2 * npx.y;
		jacob[1 * 2 + 0] = xy * k2 + 2 * cam.p1 * npx.x + 2 * cam.p2 * npx.y;
	}

	SP_GENFUNC void jacobNpxToPix(double *jacob, const CamParam &cam, const Vec2 &npx) {

		double jNpxToDist[2 * 2] = { 0 };
		jacobNpxToDist(jNpxToDist, cam, npx);

		double jDistToPix[2 * 2] = { 0 };
		jDistToPix[0 * 2 + 0] = cam.fx;
		jDistToPix[1 * 2 + 1] = cam.fy;

		mulMat(jacob, 2, 2, jDistToPix, 2, 2, jNpxToDist, 2, 2);
	}

	SP_GENFUNC void jacobPosToNpx(double *jacob, const Vec3 &pos) {
		double divz = (pos.z != 0) ? 1.0 / pos.z : 0.0;

		jacob[0 * 3 + 0] = divz; jacob[0 * 3 + 1] = 0.0; jacob[0 * 3 + 2] = -pos.x * divz * divz;
		jacob[1 * 3 + 0] = 0.0; jacob[1 * 3 + 1] = divz; jacob[1 * 3 + 2] = -pos.y * divz * divz;
	}

	SP_GENFUNC void jacobPosToPix(double *jacob, const CamParam &cam, const Vec3 &pos) {

		double jPosToNpz[2 * 3] = { 0 };
		jacobPosToNpx(jPosToNpz, pos);

		double jNpxToPix[2 * 2];
		jacobNpxToPix(jNpxToPix, cam, prjVec(pos));

		mulMat(jacob, 2, 3, jNpxToPix, 2, 2, jPosToNpz, 2, 3);
	}

	SP_GENFUNC void jacobPoseToPos(double *jacob, const Pose &pose, const Vec3 &pos) {
		double rmat[3 * 3];
		getMat(rmat, 3, 3, pose.rot);
		const Vec3 v = mulMat(rmat, 3, 3, pos);
		jacob[0 * 6 + 0] = +0.0; jacob[0 * 6 + 1] = +v.z; jacob[0 * 6 + 2] = -v.y;
		jacob[1 * 6 + 0] = -v.z; jacob[1 * 6 + 1] = +0.0; jacob[1 * 6 + 2] = +v.x;
		jacob[2 * 6 + 0] = +v.y; jacob[2 * 6 + 1] = -v.x; jacob[2 * 6 + 2] = +0.0;

		jacob[0 * 6 + 3] = 1.0; jacob[0 * 6 + 4] = 0.0; jacob[0 * 6 + 5] = 0.0;
		jacob[1 * 6 + 3] = 0.0; jacob[1 * 6 + 4] = 1.0; jacob[1 * 6 + 5] = 0.0;
		jacob[2 * 6 + 3] = 0.0; jacob[2 * 6 + 4] = 0.0; jacob[2 * 6 + 5] = 1.0;

	}

	SP_GENFUNC void jacobPoseToNpx(double *jacob, const Pose &pose, const Vec3 &pos) {
		double pmat[3 * 4];
		getMat(pmat, 3, 4, pose);

		double jPoseToPos[3 * 6] = { 0 };
		jacobPoseToPos(jPoseToPos, pose, pos);

		double jPosToNpx[2 * 3] = { 0 };
		jacobPosToNpx(jPosToNpx, mulMat(pmat, 3, 4, pos));

		mulMat(jacob, 2, 6, jPosToNpx, 2, 3, jPoseToPos, 3, 6);
	}

	SP_GENFUNC void jacobPoseToPix(double *jacob, const Pose &pose, const CamParam &cam, const Vec3 &pos) {
		double pmat[3 * 4];
		getMat(pmat, 3, 4, pose);

		double jPoseToPos[3 * 6] = { 0 };
		jacobPoseToPos(jPoseToPos, pose, pos);

		double jPosToPix[2 * 3];
		jacobPosToPix(jPosToPix, cam, mulMat(pmat, 3, 4, pos));

		mulMat(jacob, 2, 6, jPosToPix, 2, 3, jPoseToPos, 3, 6);
	}


	//--------------------------------------------------------------------------------
	// camera distortion
	//--------------------------------------------------------------------------------

	// distiortion
	SP_GENFUNC Vec2 npxDist(const CamParam &cam, const Vec2 &npx) {
		const double x2 = npx.x * npx.x;
		const double y2 = npx.y * npx.y;
		const double xy = npx.x * npx.y;

		const double r2 = x2 + y2;
		const double r4 = r2 * r2;
		const double r6 = r4 * r2;

		const double k = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;

		Vec2 dist;
		dist.x = npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2);
		dist.y = npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy);

		return dist;
	}

	// distiortion
	SP_GENFUNC Vec2 pixDist(const CamParam &cam, const Vec2 &pix) {
		return mulCam(cam, npxDist(cam, invCam(cam, pix)));
	}

	// undistortion
	SP_GENFUNC Vec2 npxUndist(const CamParam &cam, const Vec2 &npx) {
		const int maxit = 10;

		Vec2 undist = npx;
		for (int it = 0; it < maxit; it++) {
			const Vec2 err = npx - npxDist(cam, undist);
			if (normVec(err) < SP_SMALL) break;

			double J[2 * 2], inv[2 * 2];
			jacobNpxToDist(J, cam, undist);

			if (invMat22(inv, J) == false) break;

			undist += mulMat(inv, 2, 2, err);
		}

		return undist;
	}

	// undistortion
	SP_GENFUNC Vec2 pixUndist(const CamParam &cam, const Vec2 &pix) {
		return mulCam(cam, npxUndist(cam, invCam(cam, pix)));
	}


}


#endif
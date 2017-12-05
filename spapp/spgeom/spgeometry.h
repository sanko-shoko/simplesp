//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GEOMETRY_H__
#define __SP_GEOMETRY_H__

#include "spcore/spcore.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// jacob
	//--------------------------------------------------------------------------------

	SP_GENFUNC void jacobHMat(double *jacob, const double *img, const double *obj){
		jacob[0 * 9 + 0] = obj[0];
		jacob[0 * 9 + 1] = obj[1];
		jacob[0 * 9 + 2] = 1.0;
		jacob[0 * 9 + 3] = 0.0;
		jacob[0 * 9 + 4] = 0.0;
		jacob[0 * 9 + 5] = 0.0;
		jacob[0 * 9 + 6] = -img[0] * obj[0];
		jacob[0 * 9 + 7] = -img[0] * obj[1];
		jacob[0 * 9 + 8] = -img[0];

		jacob[1 * 9 + 0] = 0.0;
		jacob[1 * 9 + 1] = 0.0;
		jacob[1 * 9 + 2] = 0.0;
		jacob[1 * 9 + 3] = obj[0];
		jacob[1 * 9 + 4] = obj[1];
		jacob[1 * 9 + 5] = 1.0;
		jacob[1 * 9 + 6] = -img[1] * obj[0];
		jacob[1 * 9 + 7] = -img[1] * obj[1];
		jacob[1 * 9 + 8] = -img[1];
	}

	SP_GENFUNC void jacobHMat(double *jacob, const Vec2 &img, const Vec2 &obj){
		double pimg[2] = { img.x, img.y };
		double pobj[2] = { obj.x, obj.y };
		jacobHMat(jacob, pimg, pobj);
	}

	SP_GENFUNC void jacobFMat(double *jacob, const double *pix0, const double *pix1){
		jacob[0] = pix1[0] * pix0[0];
		jacob[1] = pix1[0] * pix0[1];
		jacob[2] = pix1[0];
		jacob[3] = pix1[1] * pix0[0];
		jacob[4] = pix1[1] * pix0[1];
		jacob[5] = pix1[1];
		jacob[6] = pix0[0];
		jacob[7] = pix0[1];
		jacob[8] = 1.0;
	}

	SP_GENFUNC void jacobFMat(double *jacob, const Vec2 &pix0, const Vec2 &pix1){
		double ppix0[2] = { pix0.x, pix0.y };
		double ppix1[2] = { pix1.x, pix1.y };
		jacobFMat(jacob, ppix0, ppix1);
	}

	SP_GENFUNC void jacobPMat(double *jacob, const double *img, const double *obj){
		jacob[0 * 12 + 0] = obj[0];
		jacob[0 * 12 + 1] = obj[1];
		jacob[0 * 12 + 2] = obj[2];
		jacob[0 * 12 + 3] = 1.0;
		jacob[0 * 12 + 4] = 0.0;
		jacob[0 * 12 + 5] = 0.0;
		jacob[0 * 12 + 6] = 0.0;
		jacob[0 * 12 + 7] = 0.0;
		jacob[0 * 12 + 8] = -img[0] * obj[0];
		jacob[0 * 12 + 9] = -img[0] * obj[1];
		jacob[0 * 12 + 10] = -img[0] * obj[2];
		jacob[0 * 12 + 11] = -img[0];

		jacob[1 * 12 + 0] = 0.0;
		jacob[1 * 12 + 1] = 0.0;
		jacob[1 * 12 + 2] = 0.0;
		jacob[1 * 12 + 3] = 0.0;
		jacob[1 * 12 + 4] = obj[0];
		jacob[1 * 12 + 5] = obj[1];
		jacob[1 * 12 + 6] = obj[2];
		jacob[1 * 12 + 7] = 1.0;
		jacob[1 * 12 + 8] = -img[1] * obj[0];
		jacob[1 * 12 + 9] = -img[1] * obj[1];
		jacob[1 * 12 + 10] = -img[1] * obj[2];
		jacob[1 * 12 + 11] = -img[1];
	}

	SP_GENFUNC void jacobPMat(double *jacob, const Vec2 &img, const Vec3 &obj){
		double pimg[2] = { img.x, img.y };
		double pobj[3] = { obj.x, obj.y, obj.z };
		jacobPMat(jacob, pimg, pobj);
	}


	//--------------------------------------------------------------------------------
	// triangulation
	//--------------------------------------------------------------------------------

	SP_CPUFUNC bool refinePnt3d(Vec3 &pnt, const Mem1<Pose> &poses, const Mem1<CamParam> &cams, const Mem1<Vec2> &pixs, const int maxit = 1) {
		SP_ASSERT(poses.size() == cams.size() && poses.size() == pixs.size());

		const int unit = 2;
		if (pixs.size() < unit) return false;

		Mat J(poses.size() * 2, 3);
		Mat E(poses.size() * 2, 1);
		Mem1<double> errs(poses.size());

		Mat jacob(2, 3);
		for (int it = 0; it < maxit; it++) {
			for (int i = 0; i < poses.size(); i++) {
				const Vec3 pos = poses[i] * pnt;

				jacobPosToPix(jacob.ptr, cams[i], pos);
				const Mat R = getMat(poses[i].rot);
				jacob = jacob * R;
				memcpy(&J(i * 2, 0), jacob.ptr, jacob.size() * sizeof(double));

				const Vec2 err = pixs[i] - mulCamD(cams[i], prjVec(pos));
				E(i * 2 + 0, 0) = err.x;
				E(i * 2 + 1, 0) = err.y;
				errs[i] = normVec(err);
			}

			Mat result;
			if (solveEq(result, J, E, errs) == false) return false;

			pnt += getVec(result[0], result[1], result[2]);
		}

		for (int i = 0; i < poses.size(); i++) {
			if ((poses[i] * pnt).z <= 0.0) return false;
		}

		return true;
	}

	SP_CPUFUNC bool calcPnt3d(Vec3 &pnt, const Mem1<Pose> &poses, const Mem1<CamParam> &cams, const Mem1<Vec2> &pixs) {
		SP_ASSERT(poses.size() == cams.size() && poses.size() == pixs.size());

		Mat M(poses.size() * 2, 3);
		Mat V(poses.size() * 2, 1);

		for (int i = 0; i < poses.size(); i++) {
			const Vec2 &npx = npxUndist(cams[i], invCam(cams[i], pixs[i]));

			const Mat R = getMat(poses[i].rot);
			const Vec3 &trn = poses[i].trn;

			M(i * 2 + 0, 0) = R(0, 0) - npx.x * R(2, 0);
			M(i * 2 + 0, 1) = R(0, 1) - npx.x * R(2, 1);
			M(i * 2 + 0, 2) = R(0, 2) - npx.x * R(2, 2);

			M(i * 2 + 1, 0) = R(1, 0) - npx.y * R(2, 0);
			M(i * 2 + 1, 1) = R(1, 1) - npx.y * R(2, 1);
			M(i * 2 + 1, 2) = R(1, 2) - npx.y * R(2, 2);

			V(i * 2 + 0, 0) = npx.x * trn.z - trn.x;
			V(i * 2 + 1, 0) = npx.y * trn.z - trn.y;
		}

		Mat result;
		if (solveEq(result, M, V) == false) return false;

		pnt = getVec(result[0], result[1], result[2]);
		
		for (int i = 0; i < poses.size(); i++) {
			if ((poses[i] * pnt).z <= 0.0) return false;
		}

		return true;
	}

	SP_CPUFUNC bool calcPnt3d(Vec3 &pnt, const Pose &pose0, const CamParam &cam0, const Vec2 &pix0, const Pose &pose1, const CamParam &cam1, const Vec2 &pix1) {
		const Pose _poses[2] = { pose0, pose1 };
		const CamParam _cams[2] = { cam0, cam1 };
		const Vec2 _pixs[2] = { pix0, pix1 };

		const Mem1<Pose> poses(2, _poses);
		const Mem1<CamParam> cams(2, _cams);
		const Mem1<Vec2> pixs(2, _pixs);

		return calcPnt3d(pnt, poses, cams, pixs);
	}


	//--------------------------------------------------------------------------------
	// homography
	//--------------------------------------------------------------------------------

	SP_CPUFUNC double errHMat(const Mat &H, const Vec2 &pix, const Vec2 &obj) {
		return normVec(pix - H * obj);
	}

	SP_CPUFUNC Mem1<double> errHMat(const Mat &H, const Mem<Vec2> &pixs, const Mem<Vec2> &objs) {
		SP_ASSERT(pixs.size() == objs.size());

		Mem1<double> errs(pixs.size());
		for (int i = 0; i < errs.size(); i++) {
			errs[i] = errHMat(H, pixs[i], objs[i]);
		}
		return errs;
	}

	// calc homography
	SP_CPUFUNC bool calcHMat(Mat &H, const Mem<Vec2> &pixs, const Mem<Vec2> &objs, const int maxit = 1){
		SP_ASSERT(pixs.size() == objs.size());

		const int unit = 4;
		if (pixs.size() < unit) return false;

		NrmData npix(2), nobj(2);
		if (npix.cnvData(pixs) == false) return false;
		if (nobj.cnvData(objs) == false) return false;

		Mat A(pixs.size() * 2, 9);
		for (int i = 0; i < pixs.size(); i++){
			jacobHMat(&A(i * 2, 0), &npix.V(i, 0), &nobj.V(i, 0));
		}

		Mem1<double> errs;
		for (int it = 0; it < maxit; it++){
			if (H.rows() == 3 && H.cols() == 3) {
				errs = errHMat(H, pixs, objs);
			}
		
			Mat result;
			if (solveEqZero(result, A, errs) == false) return false;

			H = invMat(npix.T) * Mat(3, 3, result.ptr) * nobj.T;

			const double norm = normMat(H);
			if (norm < SP_SMALL) return false;
			H /= norm;

			const Vec2 rx = getVec(H(0, 0), H(1, 0));
			const Vec2 ry = getVec(H(0, 1), H(1, 1));
			if (normVec(rx) < SP_SMALL || normVec(ry) < SP_SMALL) return false;
		}

		return true;
	}

	SP_CPUFUNC bool calcHMatRANSAC(Mat &H, const Mem<Vec2> &pixs, const Mem<Vec2> &objs, const double thresh = 5.0){
		SP_ASSERT(pixs.size() == objs.size());
		
		const int unit = 4;
		if (pixs.size() < unit * SP_RANSAC_NUM) return false;

		srand(0);
		int maxit = SP_RANSAC_ITMAX;

		Mem1<Vec2> spixs, rpixs;
		Mem1<Vec2> sobjs, robjs;

		double maxv = 0.0;
		for (int it = 0; it < maxit; it++) {
			const int p = it % (pixs.size() - unit);
			if (p == 0) {
				spixs = shuffle(pixs, it);
				sobjs = shuffle(objs, it);
			}
			rpixs.resize(unit, &spixs[p]);
			robjs.resize(unit, &sobjs[p]);

			Mat test;
			if (calcHMat(test, rpixs, robjs, 1) == false) continue;

			const Mem1<double> errs = errHMat(test, pixs, objs);
			const double eval = evalErr(errs, thresh);

			if (eval > maxv){
				//SP_PRINTD("eval %lf\n", eval);
				maxv = eval;
				maxit = adaptiveStop(eval, unit);

				H = test;
			}
		}
		if (maxv < SP_RANSAC_RATE) return false;

		// refine
		const Mem1<double> errs = errHMat(H, pixs, objs);
		const Mem1<Vec2> dpixs = denoise(pixs, errs, thresh);
		const Mem1<Vec2> dobjs = denoise(objs, errs, thresh);

		return calcHMat(H, dpixs, dobjs, 10);
	}


	//--------------------------------------------------------------------------------
	// fundamental matrix
	//--------------------------------------------------------------------------------

	SP_CPUFUNC double errFMat(const Mat &F, const Vec2 &upix0, const Vec2 &upix1) {
		const Vec3 line = F * getVec(upix0, 1.0);

		const double div = pythag(line.x, line.y);
		if (div < SP_SMALL) return SP_INFINITY;

		const double err = fabs(dotVec(getVec(upix1, 1.0), line)) / div;
		return err;
	}

	SP_CPUFUNC Mem1<double> errFMat(const Mat &F, const Mem1<Vec2> &upixs0, const Mem1<Vec2> &upixs1) {
		SP_ASSERT(upixs0.size() == upixs1.size());

		Mem1<double> errs(upixs0.size());
		for (int i = 0; i < errs.size(); i++) {
			errs[i] = errFMat(F, upixs0[i], upixs1[i]);
		}
		return errs;
	}

	// calc fundamental matrix using 8 points algorithm
	SP_CPUFUNC bool calcFMat(Mat &F, const Mem1<Vec2> &upixs0, const Mem1<Vec2> &upixs1, const int maxit = 1){
		SP_ASSERT(upixs0.size() == upixs1.size());

		const int unit = 8;
		if (upixs0.size() < unit) return false;

		NrmData npix0(2), npix1(2);
		npix0.cnvData(upixs0);
		npix1.cnvData(upixs1);

		Mat A(upixs0.size(), 9);
		for (int i = 0; i < upixs0.size(); i++){
			jacobFMat(&A(i, 0), &npix0.V(i, 0), &npix1.V(i, 0));
		}

		Mem1<double> errs;
		for (int it = 0; it < maxit; it++) {
			if (F.rows() == 3 && F.cols() == 3) {
				errs = errFMat(F, upixs0, upixs1);
			}

			Mat result;
			if (solveEqZero(result, A, errs) == false) return false;

			const Mat M = Mat(3, 3, result.ptr);

			Mat U, S, V;
			if (svdMat(U, S, V, M, false) == false) return false;

			const double div = pythag(S(0, 0), S(1, 1));
			if (div < SP_SMALL) return false;

			S(0, 0) /= div;
			S(1, 1) /= div;
			S(2, 2) = 0.0;

			F = trnMat(npix1.T) * U * S * trnMat(V) * npix0.T;
		}
		return true;
	}

	// 8 points algorithm
	SP_CPUFUNC bool calcFMatRANSAC(Mat &F, const Mem1<Vec2> &upixs0, const Mem1<Vec2> &upixs1, const double thresh = 5.0){
		SP_ASSERT(upixs0.size() == upixs1.size());

		const int unit = 8;
		if (upixs0.size() < unit * SP_RANSAC_NUM) return false;

		int maxit = SP_RANSAC_ITMAX;

		Mem1<Vec2> spixs0, rpixs0;
		Mem1<Vec2> spixs1, rpixs1;

		double maxv = 0.0;
		for (int it = 0; it < maxit; it++){
			const int p = it % (upixs0.size() - unit);
			if (p == 0) {
				spixs0 = shuffle(upixs0, it);
				spixs1 = shuffle(upixs1, it);
			}
			rpixs0.resize(unit, &spixs0[p]);
			rpixs1.resize(unit, &spixs1[p]);

			Mat test;
			if (calcFMat(test, rpixs0, rpixs1) == false) continue;

			const Mem1<double> errs = errFMat(test, upixs0, upixs1);
			const double eval = evalErr(errs, thresh);

			if (eval > maxv){
				//SP_PRINTD("eval %lf\n", eval);
				maxv = eval;
				maxit = minVal(maxit, adaptiveStop(eval, unit));

				F = test;
			}
		}
		if (maxv < SP_RANSAC_RATE) return false;

		// refine
		const Mem1<double> errs = errFMat(F, upixs0, upixs1);
		const Mem1<Vec2> dpixs0 = denoise(upixs0, errs, thresh);
		const Mem1<Vec2> dpixs1 = denoise(upixs1, errs, thresh);

		return calcFMat(F, dpixs0, dpixs1, 10);
	}

	SP_CPUFUNC bool calcFMat(Mat &F, const Pose &pose, const CamParam &cam0, const CamParam &cam1) {
		const Mat E = skewMat(pose.trn) * getMat(pose.rot);
		F = invMat(trnMat(getMat(cam1))) * E * invMat(getMat(cam0));
		return true;
	}

	SP_CPUFUNC bool dcmpFMat(Pose &pose, const Mat &F, const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const Mem1<Vec2> &pixs1) {
		SP_ASSERT(pixs0.size() == pixs1.size());

		const int unit = 8;
		if (pixs0.size() < unit) return false;

		const Mat E = trnMat(getMat(cam1)) * F * getMat(cam0);

		Mat U, S, V;
		if (svdMat(U, S, V, E, false) == false) return false;
		if (detMat(U * trnMat(V)) < 0) {
			V *= -1;
		}

		Rot R[2];
		Vec3 T[2];
		{
			Mat W = zeroMat(3, 3);
			W(0, 1) = -1.0;
			W(1, 0) = +1.0;
			W(2, 2) = +1.0;

			R[0] = getRot((U * W * trnMat(V)).ptr, 3, 3);
			R[1] = getRot((U * trnMat(W) * trnMat(V)).ptr, 3, 3);

			T[0] = getVec(U(0, 2), U(1, 2), U(2, 2));
			T[1] = getVec(U(0, 2), U(1, 2), U(2, 2)) * (-1);
		}

		const Mem1<double> errs = errFMat(F, pixs0, pixs1);
		const Mem1<Vec2> dpixs0 = denoise(pixs0, errs);
		const Mem1<Vec2> dpixs1 = denoise(pixs1, errs);

		int maxv = 0;
		for (int i = 0; i < 4; i++) {
			const Pose test = getPose(R[i % 2], T[i / 2]);

			Mem1<Vec3> pnts(dpixs0.size());

			int cnt = 0;
			for (int i = 0; i < dpixs0.size(); i++) {
				if (calcPnt3d(pnts[i], zeroPose(), cam0, dpixs0[i], test, cam1, dpixs1[i]) == false) continue;
				cnt++;
			}
			
			if (cnt > maxv) {
				maxv = cnt;
				pose = test;
			}
		}

		return true;
	}


	//--------------------------------------------------------------------------------
	// pose
	//--------------------------------------------------------------------------------

	SP_CPUFUNC double errPose(const Pose &pose, const CamParam &cam, const Vec2 &pix, const Vec3 &obj) {
		return normVec(pix - mulCamD(cam, prjVec(pose * obj)));
	}

	SP_CPUFUNC double errPose(const Pose &pose, const CamParam &cam, const Vec2 &pix, const Vec2 &obj) {
		return errPose(pose, cam, pix, getVec(obj, 0.0));
	}

	SP_CPUFUNC Mem1<double> errPose(const Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs) {
		SP_ASSERT(pixs.size() == objs.size());

		Mem1<double> errs(pixs.size());
		for (int i = 0; i < pixs.size(); i++) {
			errs[i] = errPose(pose, cam, pixs[i], objs[i]);
		}
		return errs;
	}

	SP_CPUFUNC Mem1<double> errPose(const Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec2> &objs) {
		return errPose(pose, cam, pixs, getVec(objs, 0.0));
	}

	SP_CPUFUNC bool refinePose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs, const int maxit = 10) {
		SP_ASSERT(pixs.size() == objs.size());

		const int unit = 3;
		if (pixs.size() < unit) return false;

		Mat J(2 * pixs.size(), 6);
		Mat E(2 * pixs.size(), 1);
		Mem1<double> errs(pixs.size());

		for (int it = 0; it < maxit; it++) {
			for (int i = 0; i < pixs.size(); i++) {
				jacobPoseToPix(&J(i * 2, 0), pose, cam, objs[i]);

				const Vec2 err = pixs[i] - mulCamD(cam, prjVec(pose * objs[i]));
				E(i * 2 + 0, 0) = err.x;
				E(i * 2 + 1, 0) = err.y;
				errs[i] = normVec(err);
			}
			Mat delta;
			if (solveEq(delta, J, E, errs) == false) return false;

			pose = updatePose(pose, delta.ptr);
		}

		return true;
	}

	SP_CPUFUNC bool refinePose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec2> &objs, const int maxit = 10) {
		return refinePose(pose, cam, pixs, getVec(objs, 0.0), maxit);
	}

	// 
	SP_CPUFUNC bool calcPose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs) {
		SP_ASSERT(pixs.size() == objs.size());

		const int unit = 6;
		if (pixs.size() < unit) return false;

		Mat A(pixs.size() * 2, 12);
		for (int i = 0; i < pixs.size(); i++) {
			jacobPMat(&A(i * 2, 0), npxUndist(cam, invCam(cam, pixs[i])), objs[i]);
		}

		// calc pose 
		{
			Mat U, S, V;
			if (svdMat(U, S, V, covMat(A), true) == false) return false;

			Mat R(3, 3);
			for (int r = 0; r < 3; r++) {
				for (int c = 0; c < 3; c++) {
					R(r, c) = U(r * 4 + c, 0);
				}
			}
			Vec3 T = getVec(U(3, 0), U(7, 0), U(11, 0));

			const Vec3 pos = R * meanVec(objs) + T;
			if (pos.z < 0.0) {
				R *= -1.0;
				T *= -1.0;
			}

			double scale = 1.0;
			for (int i = 0; i < 3; i++) {
				scale *= sqrt(square(R(0, i)) + square(R(1, i)) + square(R(2, i)));
			}
			scale = pow(scale, 1.0 / 3.0);

			if (scale < SP_SMALL) return false;

			if (svdMat(U, S, V, R) == false) return false;
			R = U * trnMat(V);
			T /= scale;

			pose = getPose(getRot(R.ptr, 3, 3), T);
		}

		return true;
	}

	SP_CPUFUNC bool calcPoseRANSAC(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs, const double thresh = 5.0) {
		SP_ASSERT(pixs.size() == objs.size());

		const int unit = 6;
		if (pixs.size() < unit * SP_RANSAC_NUM) return false;

		int maxit = SP_RANSAC_ITMAX;

		Mem1<Vec2> spixs, rpixs;
		Mem1<Vec3> sobjs, robjs;

		double maxv = 0.0;
		for (int it = 0; it < maxit; it++) {
			const int p = it % (pixs.size() - unit);
			if (p == 0) {
				spixs = shuffle(pixs, it);
				sobjs = shuffle(objs, it);
			}
			rpixs.resize(unit, &spixs[p]);
			robjs.resize(unit, &sobjs[p]);

			Pose test;
			if (calcPose(test, cam, rpixs, robjs) == false) continue;

			const Mem1<double> errs = errPose(test, cam, pixs, objs);
			const double eval = evalErr(errs, thresh);

			if (eval > maxv) {
				//SP_PRINTD("eval %lf\n", eval);
				maxv = eval;
				maxit = adaptiveStop(eval, unit);

				pose = test;
			}
		}
		if (maxv < SP_RANSAC_RATE) return false;

		// refine
		const Mem1<double> errs = errPose(pose, cam, pixs, objs);
		const Mem1<Vec2> dpixs = denoise(pixs, errs, thresh);
		const Mem1<Vec3> dobjs = denoise(objs, errs, thresh);

		return refinePose(pose, cam, dpixs, dobjs);
	}

	//
	SP_CPUFUNC bool calcPose(Pose &pose, const CamParam &cam, const Mat &hom) {
		SP_ASSERT(hom.rows() == 3 && hom.cols() == 3);

		const Mat h = hom / hom(2, 2);

		const Mat mat = invMat(getMat(cam)) * h;

		const Vec3 m0 = getVec(mat(0, 0), mat(1, 0), mat(2, 0));
		const Vec3 m1 = getVec(mat(0, 1), mat(1, 1), mat(2, 1));
		const Vec3 m2 = getVec(mat(0, 2), mat(1, 2), mat(2, 2));

		const Vec3 n0 = unitVec(m0);
		const Vec3 n1 = unitVec(m1);

		const Vec3 Z = unitVec(crsVec(n0, n1));
		const Vec3 A = unitVec(addVec(n0, n1));
		const Vec3 B = unitVec(crsVec(A, Z));

		const Vec3 X = unitVec(addVec(A, B));
		const Vec3 Y = unitVec(subVec(A, B));

		pose = getPose(getRotAxis(X, Y, Z), m2 / sqrt(normVec(m0) * normVec(m1)));
		return true;
	}

	SP_CPUFUNC bool calcPose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec2> &objs, const int maxit = 10) {
		SP_ASSERT(pixs.size() == objs.size());

		const int unit = 4;
		if (pixs.size() < unit) return false;

		Mat hom;
		if (calcHMat(hom, pixs, objs) == false) return false;

		if (calcPose(pose, cam, hom) == false) return false;
		return refinePose(pose, cam, pixs, getVec(objs, 0.0), maxit);
	}

	SP_CPUFUNC bool calcPose(Pose &pose, const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const Mem1<Vec2> &pixs1) {
		SP_ASSERT(pixs0.size() == pixs1.size());

		Mem1<Vec2> upixs0(pixs0.size());
		Mem1<Vec2> upixs1(pixs1.size());

		for (int i = 0; i < pixs0.size(); i++) {
			upixs0[i] = pixUndist(cam0, pixs0[i]);
			upixs1[i] = pixUndist(cam1, pixs1[i]);
		}

		Mat F;
		if (calcFMatRANSAC(F, upixs0, upixs1) == false) return false;

		if (dcmpFMat(pose, F, cam0, pixs0, cam1, pixs1) == false) return false;
		return true;
	}


}
#endif
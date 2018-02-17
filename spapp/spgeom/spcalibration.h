//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CALIBRATION_H__
#define __SP_CALIBRATION_H__

#include "spcore/spcore.h"


namespace sp{

	//--------------------------------------------------------------------------------
	// local method
	//--------------------------------------------------------------------------------
	
	namespace _calibration{

		SP_CPUFUNC bool initCam(CamParam &cam, const int *dsize, const Mem1<Mem1<Vec2> > &pixsList, const Mem1<Mem1<Vec2> > &objsList){

			Mem1<Mat> homs;

			// set valid data
			for (int i = 0; i < pixsList.size(); i++){
				const Mem1<Vec2> &pixs = pixsList[i] -(getVec(dsize[0] - 1, dsize[1] - 1) * 0.5);
				const Mem1<Vec2> &objs = objsList[i];

				Mat hom;
				if (calcHMat(hom, pixs, objs) == false) continue;

				homs.push(hom);
			}


			Mat M(homs.size() * 2, 3);

			// solve closed form
			for (int i = 0; i < homs.size(); i++){
				const Mat &hom = homs[i];
				Vec3 v[4];
				v[0] = getVec(hom(0, 0), hom(1, 0), hom(2, 0));
				v[1] = getVec(hom(0, 1), hom(1, 1), hom(2, 1));
				v[2] = v[0] + v[1];
				v[3] = v[0] - v[1];

				M(i * 2 + 0, 0) = v[0].x * v[1].x;
				M(i * 2 + 0, 1) = v[0].y * v[1].y;
				M(i * 2 + 0, 2) = v[0].z * v[1].z;

				M(i * 2 + 1, 0) = v[2].x * v[3].x;
				M(i * 2 + 1, 1) = v[2].y * v[3].y;
				M(i * 2 + 1, 2) = v[2].z * v[3].z;
			}

			Mat result;
			if (solveEqZero(result, M) == false) return false;
			
			if (fabs(result[2]) < SP_SMALL) return false;
			result /= result[2];

			if (result[0] < SP_SMALL || result[1] < SP_SMALL) return false;

			// set param
			cam = getCamParam(dsize);

			cam.fx = sqrt(1.0 / result[0]);
			cam.fy = sqrt(1.0 / result[1]);

			return true;
		}

		SP_CPUFUNC double optCam(CamParam &cam, const int *dsize, const Mem1<Mem1<Vec2> > &pixsList, const Mem1<Mem1<Vec2> > &objsList, const int maxit = 20){

			Mem1<Pose> vposes;
			Mem1<Mem1<Vec2> > vpixsList;
			Mem1<Mem1<Vec2> > vobjsList;

			// set valid data
			for (int i = 0; i < objsList.size(); i++){
				Pose pose;
				if (calcPose(pose, cam, pixsList[i], objsList[i]) == false) continue;

				vposes.push(pose);
				vpixsList.push(pixsList[i]);
				vobjsList.push(objsList[i]);
			}


			int pmax = 0;
			for (int i = 0; i < vposes.size(); i++){
				pmax += vpixsList[i].size();
			}

			double ret = -1.0;

			// gauss-newton
			for (int it = 0; it < maxit; it++){
				Mat J(pmax * 2, 9 + 6 * vposes.size());
				J.zero();

				Mat E(pmax * 2, 1);
				Mem1<double> errs(pmax);

				int cnt = 0;
				for (int i = 0; i < vposes.size(); i++){

					const Pose &pose = vposes[i];

					Mat jCamToPix(2, 9);
					Mat jPoseToPix(2, 6);

					const Mem1<Vec2> &vpixs = vpixsList[i];
					const Mem1<Vec2> &vobjs = vobjsList[i];
					for (int p = 0; p < vpixs.size(); p++){
						const Vec2 pix = vpixs[p];
						const Vec3 obj = getVec(vobjs[p].x, vobjs[p].y, 0.0);
						const Vec2 vec = mulCamD(cam, prjVec(pose * obj));

						const Vec2 err = pix - vec;
						E(cnt * 2 + 0, 0) = err.x;
						E(cnt * 2 + 1, 0) = err.y;

						jacobCamToPix(jCamToPix.ptr, cam, prjVec(pose * obj));
						jacobPoseToPix(jPoseToPix.ptr, pose, cam, obj);

						for (int r = 0; r < 2; r++){
							for (int c = 0; c < 9; c++){
								J(cnt * 2 + r, c) = jCamToPix(r, c);
							}
							for (int c = 0; c < 6; c++){
								J(cnt * 2 + r, c + 9 + i * 6) = jPoseToPix(r, c);
							}
						}

						errs[cnt] = normVec(err);
						cnt++;
					}
				}

				const double mean = meanVal(errs);
				const double median = medianVal(errs);
				const double rms = sqrt(meanSq(errs));

				Mat delta;
				if (solveEq(delta, J, E, errs) == false) return ret;

				{
					cam.fx += delta[0];
					cam.fy += delta[1];
					cam.cx += delta[2];
					cam.cy += delta[3];

					cam.k1 += delta[4];
					cam.k2 += delta[5];
					cam.k3 += delta[6];
					cam.p1 += delta[7];
					cam.p2 += delta[8];
				}

				for (int n = 0; n < vposes.size(); n++){
					vposes[n] = updatePose(vposes[n], &delta[9 + n * 6]);
				}

				SP_PRINTD("i:%02d [mean: %9.6lf], [median: %9.6lf], [rms: %9.6lf]\n", it, mean, median, rms);
				ret = rms;
			}

			return ret;
		}


		SP_CPUFUNC bool initStereo(Pose &stereo, const CamParam &cam0, const CamParam &cam1,
			const Mem1<Mem1<Vec2> > &pixsList0, const Mem1<Mem1<Vec2> > &pixsList1, const Mem1<Mem1<Vec2> > &objsList){
	
			double minv = SP_INFINITY;

			for (int i = 0; i < objsList.size(); i++){

				Pose pose0, pose1;
				if (calcPose(pose0, cam0, pixsList0[i], objsList[i]) == false) continue;
				if (calcPose(pose1, cam1, pixsList1[i], objsList[i]) == false) continue;

				const Mem1<double> errs0 = errPose(pose0, cam0, pixsList0[i], objsList[i]);
				const Mem1<double> errs1 = errPose(pose1, cam1, pixsList1[i], objsList[i]);

				const double err = medianVal(errs0) + medianVal(errs1);
				if (err < minv){
					minv = err;
					stereo = pose1 * invPose(pose0);
				}
			}

			return (minv < SP_INFINITY) ? true : false;
		}

		SP_CPUFUNC double optStereo(Pose &stereo, const CamParam &cam0, const CamParam &cam1,
			const Mem1<Mem1<Vec2> > &pixsList0, const Mem1<Mem1<Vec2> > &pixsList1, const Mem1<Mem1<Vec2> > &objsList, int maxit = 20){
			Mem1<Pose> vposes;
			Mem1<Mem1<Vec2> > vpixsList0, vpixsList1;
			Mem1<Mem1<Vec2> > vobjsList;

			// set valid data
			for (int i = 0; i < objsList.size(); i++){
				Pose pose0, pose1;
				if (calcPose(pose0, cam0, pixsList0[i], objsList[i]) == false) continue;
				if (calcPose(pose1, cam1, pixsList1[i], objsList[i]) == false) continue;

				vposes.push(pose0);
				vpixsList0.push(pixsList0[i]);
				vpixsList1.push(pixsList1[i]);
				vobjsList.push(objsList[i]);
			}


			int pmax = 0;
			for (int i = 0; i < vposes.size(); i++){
				pmax += vpixsList0[i].size();
			}

			double rms = -1.0;

			// gauss-newton
			for (int it = 0; it < maxit; it++){

				// refine vpose
				if(it > 0){
					for (int i = 0; i < vposes.size(); i++){
						Mat J(4 * vpixsList0[i].size(), 6);
						Mat E(4 * vpixsList0[i].size(), 1);

						const Pose pose0 = vposes[i];
						const Pose pose1 = stereo * vposes[i];

						const Mat sR = getMat(stereo.rot);

						for (int p = 0; p < vobjsList[i].size(); p++){
							const Vec3 obj = getVec(vobjsList[i][p].x, vobjsList[i][p].y, 0.0);

							double jPoseToPos0[3 * 6] = { 0 };
							double jPoseToPos1[3 * 6] = { 0 };
							jacobPoseToPos(jPoseToPos0, pose0, obj);
							mulMat(jPoseToPos1, 3, 6, sR.ptr, 3, 3, jPoseToPos0, 3, 6);

							double jPosToPix0[2 * 3] = { 0 };
							double jPosToPix1[2 * 3] = { 0 };
							jacobPosToPix(jPosToPix0, cam0, pose0 * obj);
							jacobPosToPix(jPosToPix1, cam1, pose1 * obj);

							mulMat(&J(p * 4 + 0, 0), 2, 6, jPosToPix0, 2, 3, jPoseToPos0, 3, 6);
							mulMat(&J(p * 4 + 2, 0), 2, 6, jPosToPix1, 2, 3, jPoseToPos1, 3, 6);

							const Vec2 err0 = vpixsList0[i][p] - mulCamD(cam0, prjVec(pose0 * obj));
							const Vec2 err1 = vpixsList1[i][p] - mulCamD(cam1, prjVec(pose1 * obj));

							E(p * 4 + 0, 0) = err0.x;
							E(p * 4 + 1, 0) = err0.y;
							E(p * 4 + 2, 0) = err1.x;
							E(p * 4 + 3, 0) = err1.y;
						}
						Mat delta;
						if (solveEq(delta, J, E) == false) return false;

						vposes[i] = updatePose(vposes[i], delta.ptr);
					}
				}

				// refine stereo
				{
					Mat J(pmax * 2, 6);
					J.zero();

					Mat E(pmax * 2, 1);
					Mem1<double> errs(pmax);

					int cnt = 0;
					for (int i = 0; i < vposes.size(); i++){

						const Pose &pose = vposes[i];

						const Mem1<Vec2> &vpixs = vpixsList1[i];
						const Mem1<Vec2> &vobjs = vobjsList[i];
						for (int p = 0; p < vpixs.size(); p++){
							const Vec2 pix = vpixs[p];
							const Vec3 obj = getVec(vobjs[p].x, vobjs[p].y, 0.0);

							const Vec2 vec = mulCamD(cam1, prjVec(stereo * pose * obj));

							const Vec2 err = pix - vec;
							E(cnt * 2 + 0, 0) = err.x;
							E(cnt * 2 + 1, 0) = err.y;

							jacobPoseToPix(&J[2 * 6 * cnt], stereo, cam1, pose * obj);

							errs[cnt] = normVec(err);
							cnt++;
						}
					}

					Mat delta;
					if (solveEq(delta, J, E, errs) == false) return rms;

					stereo = updatePose(stereo, delta.ptr);
				}

				Mem1<double> errs;
				for (int i = 0; i < vposes.size(); i++){
					errs.push(errPose(vposes[i], cam0, vpixsList0[i], vobjsList[i]));
					errs.push(errPose(stereo * vposes[i], cam1, vpixsList1[i], vobjsList[i]));
				}

				const double mean = meanVal(errs);
				const double median = medianVal(errs);
				rms = sqrt(meanSq(errs));

				SP_PRINTD("i:%02d [mean: %9.6lf], [median: %9.6lf], [rms: %9.6lf]\n", it, mean, median, rms);
			}

			return rms;
		}

		SP_CPUFUNC bool initRobotCam(Pose &X, Pose &Z, const Mem1<Pose> &As, const Mem1<Pose> &Bs) {
			const int num = As.size();

			auto calcQ = [](const Rot &rot)-> Mat {
				double m[4 * 4] = {
					+rot.qw, -rot.qx, -rot.qy, -rot.qz,
					+rot.qx, +rot.qw, -rot.qz, +rot.qy,
					+rot.qy, +rot.qz, +rot.qw, -rot.qx,
					+rot.qz, -rot.qy, +rot.qx, +rot.qw
				};
				return Mat(4, 4, m);
			};
			auto calcW = [](const Rot &rot)-> Mat {
				double m[4 * 4] = {
					+rot.qw, -rot.qx, -rot.qy, -rot.qz,
					+rot.qx, +rot.qw, +rot.qz, -rot.qy,
					+rot.qy, -rot.qz, +rot.qw, +rot.qx,
					+rot.qz, +rot.qy, -rot.qx, +rot.qw
				};
				return Mat(4, 4, m);
			};

			Mat C = zeroMat(4, 4);
			for (int i = 0; i < num; i++) {
				const Mat Q = calcQ(As[i].rot);
				const Mat W = calcW(Bs[i].rot);

				C += trnMat(Q) * W;
			}

			Mat eigVec, eigVal;
			if(eigMat(eigVec, eigVal, covMat(C)) == false) return false;

			int id = -1;
			double minv = SP_INFINITY;

			for (int i = 0; i < 4; i++) {
				const double lambda1 = num - sqrt(eigVal(i, i));
				const double lambda2 = num + sqrt(eigVal(i, i));

				const double v = (lambda1 > SP_SMALL) ? lambda1 : lambda2;
				if (v < minv) {
					minv = v;
					id = i;
				}
			}
			if (id < 0) return false;

			const double q[4] = { eigVec(0, id), eigVec(1, id), eigVec(2, id), eigVec(3, id) };

			const Mat zmat(4, 1, q);
			const Rot zrot = getRot(zmat[1], zmat[2], zmat[3], zmat[0]);

			const Mat xmat = C * zmat / (minv - num);
			const Rot xrot = getRot(xmat[1], xmat[2], xmat[3], xmat[0]);

			Mat mat0(num * 3, 1);
			Mat mat1(num * 3, 6);

			for (int i = 0; i < num; i++) {
				const Mat tmat = getMat(zrot * Bs[i].trn - As[i].trn);

				const Mat rmat = getMat(As[i].rot);
				const Mat imat = eyeMat(3, 3);

				for (int r = 0; r < 3; r++) {
					mat0(i * 3 + r, 0) = tmat(r, 0);

					for (int c = 0; c < 3; c++) {
						mat1(i * 3 + r, c + 0) = rmat(r, c);
						mat1(i * 3 + r, c + 3) = imat(r, c) * -1.0;
					}
				}
			}
			const Mat txy = invMat(trnMat(mat1) * mat1) * trnMat(mat1) * mat0;

			X = getPose(xrot, getVec(txy[0], txy[1], txy[2]));
			Z = getPose(zrot, getVec(txy[3], txy[4], txy[5]));

			return true;
		}

		SP_CPUFUNC double optRobotCam(Pose &X, Pose &Z, const Mem1<Pose> &Bs, const CamParam &cam, const Mem1<Mem1<Vec2> > &pixsList, const Mem1<Mem1<Vec2> > &objsList, int maxit = 20) {
			const int num = Bs.size();

			Pose iZ = invPose(Z);

			int pmax = 0;
			for (int i = 0; i < num; i++) {
				pmax += pixsList[i].size();
			}

			double rms = -1.0;

			for (int it = 0; it < maxit; it++) {
			
				Mat J(2 * pmax, 6 + 6);
				Mat E(2 * pmax, 1);
				Mem1<double> errs(pmax);

				int cnt = 0;

				for (int i = 0; i < num; i++) {
					const Pose iB = invPose(Bs[i]);

					const Mem1<Vec2> &pixs = pixsList[i];
					const Mem1<Vec2> &objs = objsList[i];

					Mat J0(2, 6);
					Mat J1(2, 6);
					for (int j = 0; j < objs.size(); j++) {
						{
							jacobPoseToPix(J0.ptr, X, cam, (iB * iZ) * objs[j]);
						}
						{
							Mat J1_6D3D(3, 6);
							jacobPoseToPos(J1_6D3D.ptr, iZ, getVec(objs[j], 0.0));

							Mat J1_3D2D(2, 3);
							jacobPosToPix(J1_3D2D.ptr, cam, iZ * objs[j]);

							J1 = J1_3D2D * getMat((X * iB).rot) * J1_6D3D;
						}

						for (int p = 0; p < 6; p++) {
							J(cnt * 2 + 0, p + 0) = J0(0, p);
							J(cnt * 2 + 1, p + 0) = J0(1, p);

							J(cnt * 2 + 0, p + 6) = J1(0, p);
							J(cnt * 2 + 1, p + 6) = J1(1, p);
						}

						const Vec2 err = pixs[j] - mulCamD(cam, prjVec((X * iB * iZ) * objs[j]));
						E(cnt * 2 + 0, 0) = err.x;
						E(cnt * 2 + 1, 0) = err.y;
						errs[cnt] = normVec(err);

						cnt++;
					}
				}

				Mat delta;
				if (solveEq(delta, J, E, errs) == false) return false;

				X = updatePose(X, &delta[0]);
				iZ = updatePose(iZ, &delta[6]);
				Z = invPose(iZ);

				const double mean = meanVal(errs);
				const double median = medianVal(errs);
				rms = sqrt(meanSq(errs));

				SP_PRINTD("i:%02d [mean: %9.6lf], [median: %9.6lf], [rms: %9.6lf]\n", it, mean, median, rms);
			}

			return rms;
		}

	}

	using namespace _calibration;

	//--------------------------------------------------------------------------------
	// calibrate camera parameter
	//--------------------------------------------------------------------------------

	SP_CPUFUNC double calibCam(CamParam &cam, const int dsize0, const int dsize1,
		const Mem1<Mem1<Vec2> > &pixsList, const Mem1<Mem1<Vec2> > &objsList, const int maxit = 20){
		double rms = -1.0;

		try{
			if (pixsList.size() < 3 || pixsList.size() != objsList.size()) throw "data size";

			const int dsize[2] = { dsize0, dsize1 };
			if (initCam(cam, dsize, pixsList, objsList) == false) throw "initCam";

			if ((rms = optCam(cam, dsize, pixsList, objsList, maxit)) < 0.0) throw "optCam";
		}
		catch (const char *str){
			SP_PRINTD("calibCam [%s]\n", str);
		}

		return rms;
	}


	//--------------------------------------------------------------------------------
	// calibrate stereo pose
	//--------------------------------------------------------------------------------

	SP_CPUFUNC double calibStereo(Pose &stereo, const CamParam &cam0, const CamParam &cam1,
		const Mem1<Mem1<Vec2> > &pixsList0, const Mem1<Mem1<Vec2> > &pixsList1, const Mem1<Mem1<Vec2> > &objsList, const int maxit = 20){

		double rms = -1.0;

		try{
			if (pixsList0.size() < 3 || pixsList0.size() != pixsList1.size() || pixsList0.size() != objsList.size()) throw "data size";

			if (initStereo(stereo, cam0, cam1, pixsList0, pixsList1, objsList) == false) throw "initStereo";
			
			if ((rms = optStereo(stereo, cam0, cam1, pixsList0, pixsList1, objsList, maxit)) < 0.0) throw "optStereo";
		}
		catch (const char *str){
			SP_PRINTD("calibStereo [%s]\n", str);
		}

		return rms;
	}
	
	SP_CPUFUNC double calibStereo(Pose &stereo, const CamParam &cam0, const CamParam &cam1,
		const Mem1<Mem1<Vec2> > &pixsList0, const Mem1<Mem1<Vec2> > &pixsList1, const Mem1<Mem1<Vec2> > &objsList0, const Mem1<Mem1<Vec2> > &objsList1, const int maxit = 20){

		Mem1<Mem1<Vec2> > cpixsList0, cpixsList1, cobjsList;
		for (int i = 0; i < pixsList0.size(); i++){

			Mem1<Vec2> cpixs0, cpixs1, cobjs;
			for (int j = 0; j < pixsList0[i].size(); j++){
				for (int k = 0; k < pixsList1[i].size(); k++){
					if (cmpVec(objsList0[i][j], objsList1[i][k]) == true){
						cpixs0.push(pixsList0[i][j]);
						cpixs1.push(pixsList1[i][k]);
						cobjs.push(objsList0[i][j]);
						break;
					}
				}
			}
			cpixsList0.push(cpixs0);
			cpixsList1.push(cpixs1);
			cobjsList.push(cobjs);
		}

		return calibStereo(stereo, cam0, cam1, cpixsList0, cpixsList1, cobjsList, maxit);
	}


	//--------------------------------------------------------------------------------
	// calibrate robot to cam
	//--------------------------------------------------------------------------------

	// 

	SP_CPUFUNC double calibRobotCam(Pose &X, Pose &Z, const Mem1<Pose> &Bs, const CamParam &cam, const Mem1<Mem1<Vec2> > &pixsList, const Mem1<Mem1<Vec2> > &objsList) {

		Mem1<Pose> As;

		{
			const int num = Bs.size();

			for (int i = 0; i < num; i++) {
				Pose mrk2camPose;
				calcPose(mrk2camPose, cam, pixsList[i], objsList[i]);
				As.push(invPose(mrk2camPose));
			}
		}

		double rms = -1.0;

		try {
			if (Bs.size() < 5 || Bs.size() != pixsList.size() || Bs.size() != objsList.size()) throw "data size";

			if (initRobotCam(X, Z, As, Bs) == false) throw "initRobotCam";
			
			if ((rms = optRobotCam(X, Z, Bs, cam, pixsList, objsList)) < 0.0) throw "optRobotCam";
		}
		catch (const char *str) {
			SP_PRINTD("calibRobotCam [%s]\n", str);
		}

		return rms;
	}


	//--------------------------------------------------------------------------------
	// rectify
	//--------------------------------------------------------------------------------

	struct RectParam{
		CamParam cam;
		CamParam pre;
		Rot rot;
	};
	
	SP_CPUFUNC void rectify(RectParam &rect0, RectParam &rect1, const CamParam &cam0, const CamParam &cam1, const Pose &stereo){
		SP_ASSERT(cmpSize(2, cam0.dsize, cam1.dsize));

		// pre parameter
		{
			rect0.pre = cam0;
			rect1.pre = cam1;
		}

		// rot
		{
			Vec3 vecx, vecy, vecz;
			vecx = unitVec((invRot(stereo.rot) * stereo.trn) * -1.0);

			const Vec3 uz = getVec(0.0, 0.0, 1.0);
			const Vec3 mz = unitVec(uz + invRot(stereo.rot) * uz);

			vecy = unitVec(crsVec(mz, vecx));
			vecz = unitVec(crsVec(vecx, vecy));

			const Rot rot = getRotAxis(vecx, vecy, vecz);

			rect0.rot = invRot(rot);
			rect1.rot = invRot(stereo.rot * rot);
		}

		// cam parameter
		{
			const int dsize[2] = { cam0.dsize[0], cam0.dsize[1] };
			
			const double f = (cam0.fx + cam0.fy + cam1.fx + cam1.fy) / 4.0;
			const Vec2 cent = getVec(dsize[0] - 1, dsize[1] - 1) * 0.5;

			RectParam *pRect[2] = { &rect0, &rect1 };
		
			for (int i = 0; i < 2; i++){
				pRect[i]->cam = getCamParam(dsize[0], dsize[1], f, f, cent.x, cent.y);

				const Vec2 npx = invCamD(pRect[i]->pre, cent);
				const Vec2 wrp = prjVec(pRect[i]->rot * prjVec(npx));

				const Vec2 pix = mulCamD(pRect[i]->cam, wrp);

				const Vec2 dif = cent - pix;
				pRect[i]->cam.cx += dif.x;
				pRect[i]->cam.cy += dif.y;
			}

			const double cy = (rect0.cam.cy + rect1.cam.cy) / 2.0;
			rect0.cam.cy = cy;
			rect1.cam.cy = cy;
		}
	}


	//--------------------------------------------------------------------------------
	// remap
	//--------------------------------------------------------------------------------

	SP_CPUFUNC void makeRemapTable(Mem2<Vec2> &table, const CamParam &cam){
		table.resize(cam.dsize);

		for (int v = 0; v < table.dsize[1]; v++){
			for (int u = 0; u < table.dsize[0]; u++){
				const Vec2 src = getVec(u, v);
				const Vec2 dst = pixDist(cam, src);
				table(u, v) = dst - src;
			}
		}

	}

	SP_CPUFUNC void makeRemapTable(Mem2<Vec2> &table, const RectParam &rect){
		table.resize(rect.cam.dsize);

		const Rot rot = invRot(rect.rot);
		for (int v = 0; v < table.dsize[1]; v++){
			for (int u = 0; u < table.dsize[0]; u++){
				const Vec2 src = getVec(u, v);
				const Vec2 npx = invCamD(rect.cam, src);
				const Vec2 wrp = prjVec(rot * prjVec(npx));

				const Vec2 dst = mulCamD(rect.pre, wrp);
				table(u, v) = dst - src;
			}
		}

	}



}
#endif
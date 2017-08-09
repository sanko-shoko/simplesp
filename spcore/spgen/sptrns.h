//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_TRNS_H__
#define __SP_TRNS_H__

#include "spcore/spcom.h"
#include "spcore/spwrap.h"
#include "spcore/spgen/spbase.h"
#include "spcore/spgen/spvec.h"
#include "spcore/spgen/spmath.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// jacob
	//--------------------------------------------------------------------------------

	SP_GENCALL void jacobPoseToPos(double *jacob, const Pose &pose, const Vec3 &pos){
		double rmat[3 * 3];
		cnvRotToMat(rmat,3, 3, pose.rot);
		const Vec3 v = mulMat(rmat, 3, 3, pos);
		jacob[0 * 6 + 0] = +0.0; jacob[0 * 6 + 1] = +v.z; jacob[0 * 6 + 2] = -v.y;
		jacob[1 * 6 + 0] = -v.z; jacob[1 * 6 + 1] = +0.0; jacob[1 * 6 + 2] = +v.x;
		jacob[2 * 6 + 0] = +v.y; jacob[2 * 6 + 1] = -v.x; jacob[2 * 6 + 2] = +0.0;

		jacob[0 * 6 + 3] = 1.0; jacob[0 * 6 + 4] = 0.0; jacob[0 * 6 + 5] = 0.0;
		jacob[1 * 6 + 3] = 0.0; jacob[1 * 6 + 4] = 1.0; jacob[1 * 6 + 5] = 0.0;
		jacob[2 * 6 + 3] = 0.0; jacob[2 * 6 + 4] = 0.0; jacob[2 * 6 + 5] = 1.0;

	}

	SP_GENCALL void jacobCamToPix(double *jacob, const CamParam &cam, const Vec2 &npx){
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

	SP_GENCALL void jacobNpxToDist(double *jacob, const CamParam &cam, const Vec2 &npx){
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

	SP_GENCALL void jacobNpxToPix(double *jacob, const CamParam &cam, const Vec2 &npx){

		double jNpxToDist[2 * 2] = { 0 };
		jacobNpxToDist(jNpxToDist, cam, npx);

		double jDistToPix[2 * 2] = { 0 };
		jDistToPix[0 * 2 + 0] = cam.fx;
		jDistToPix[1 * 2 + 1] = cam.fy;

		mulMat(jacob, 2, 2, jDistToPix, 2, 2, jNpxToDist, 2, 2);
	}

	SP_GENCALL void jacobPosToNpx(double *jacob, const Vec3 &pos){
		double divz = (pos.z != 0) ? 1.0 / pos.z : 0.0;

		jacob[0 * 3 + 0] = divz; jacob[0 * 3 + 1] = 0.0; jacob[0 * 3 + 2] = -pos.x * divz * divz;
		jacob[1 * 3 + 0] = 0.0; jacob[1 * 3 + 1] = divz; jacob[1 * 3 + 2] = -pos.y * divz * divz;
	}

	SP_GENCALL void jacobPosToPix(double *jacob, const CamParam &cam, const Vec3 &pos){

		double jPosToNpz[2 * 3] = { 0 };
		jacobPosToNpx(jPosToNpz, pos);

		double jNpxToPix[2 * 2];
		jacobNpxToPix(jNpxToPix, cam, prjVec(pos));

		mulMat(jacob, 2, 3, jNpxToPix, 2, 2, jPosToNpz, 2, 3);
	}

	SP_GENCALL void jacobPoseToNpx(double *jacob, const Pose &pose, const Vec3 &pos){
		double pmat[3 * 4];
		cnvPoseToMat(pmat, 3, 4, pose);

		double jPoseToPos[3 * 6] = { 0 };
		jacobPoseToPos(jPoseToPos, pose, pos);

		double jPosToNpx[2 * 3] = { 0 };
		jacobPosToNpx(jPosToNpx, mulMat(pmat, 3, 4, pos));

		mulMat(jacob, 2, 6, jPosToNpx, 2, 3, jPoseToPos, 3, 6);
	}


	SP_GENCALL void jacobPoseToPix(double *jacob, const Pose &pose, const CamParam &cam, const Vec3 &pos){
		double pmat[3 * 4];
		cnvPoseToMat(pmat, 3, 4, pose);

		double jPoseToNpx[2 * 6] = { 0 };
		jacobPoseToNpx(jPoseToNpx, pose, pos);

		double jNpxToPix[2 * 2];
		jacobNpxToPix(jNpxToPix, cam, prjVec(mulMat(pmat, 3, 4, pos)));

		mulMat(jacob, 2, 6, jNpxToPix, 2, 2, jPoseToNpx, 2, 6);
	}

	SP_GENCALL void jacobCamPoseToPix(double *jacob, const Pose &pose, const CamParam &cam, const Vec3 &pos){
		double pmat[3 * 4];
		cnvPoseToMat(pmat, 3, 4, pose);

		double jPoseToPix[2 * 6] = { 0 };
		jacobPoseToPix(jPoseToPix, pose, cam, pos);

		double jCamToPix[2 * 9] = { 0 };
		jacobCamToPix(jCamToPix, cam, prjVec(mulMat(pmat, 3, 4, pos)));

		for (int r = 0; r < 2; r++){
			const double f = (r == 0) ? cam.fx : cam.fy;
			for (int c = 0; c < 9; c++){
				jacob[r * 15 + (0 + c)] = jCamToPix[r * 9 + c];
			}
			for (int c = 0; c < 6; c++){
				jacob[r * 15 + (9 + c)] = jPoseToPix[r * 6 + c];
			}
		}
	}


	//--------------------------------------------------------------------------------
	// rotation
	//--------------------------------------------------------------------------------

	SP_GENCALL Rot nrmRot(const Rot &rot){
		Rot dst;

		const double div = sqrt(rot.qx * rot.qx + rot.qy * rot.qy + rot.qz * rot.qz + rot.qw * rot.qw);
		if (div > SP_SMALL){
			dst.qx = rot.qx / div;
			dst.qy = rot.qy / div;
			dst.qz = rot.qz / div;
			dst.qw = rot.qw / div;
		}
		else{
			dst.qx = 0.0;
			dst.qy = 0.0;
			dst.qz = 0.0;
			dst.qw = 1.0;
		}
		return dst;
	}

	SP_GENCALL Rot getRot(const double qx, const double qy, const double qz, const double qw){
		Rot dst;
		dst.qx = qx; 
		dst.qy = qy;
		dst.qz = qz;
		dst.qw = qw;
		return nrmRot(dst);
	}

	SP_GENCALL Rot getRot(const double *mat){
		Rot dst;
		dst.qx = sqrt(maxVal(0.0, 1 + mat[0 * 3 + 0] - mat[1 * 3 + 1] - mat[2 * 3 + 2])) / 2;
		dst.qy = sqrt(maxVal(0.0, 1 - mat[0 * 3 + 0] + mat[1 * 3 + 1] - mat[2 * 3 + 2])) / 2;
		dst.qz = sqrt(maxVal(0.0, 1 - mat[0 * 3 + 0] - mat[1 * 3 + 1] + mat[2 * 3 + 2])) / 2;
		dst.qw = sqrt(maxVal(0.0, 1 + mat[0 * 3 + 0] + mat[1 * 3 + 1] + mat[2 * 3 + 2])) / 2;

		dst.qx *= sign(dst.qx * (mat[2 * 3 + 1] - mat[1 * 3 + 2]));
		dst.qy *= sign(dst.qy * (mat[0 * 3 + 2] - mat[2 * 3 + 0]));
		dst.qz *= sign(dst.qz * (mat[1 * 3 + 0] - mat[0 * 3 + 1]));

		return nrmRot(dst);
	}

	SP_GENCALL Rot zeroRot(){
		return getRot(0.0, 0.0, 0.0, 1.0);
	}

	SP_GENCALL Rot invRot(const Rot &rot){
		return getRot(-rot.qx, -rot.qy, -rot.qz, rot.qw);
	}

	SP_GENCALL Rot mulRot(const Rot &rot0, const Rot &rot1){
		Rot dst;
		dst.qx = (rot0.qw * rot1.qx) + (rot0.qx * rot1.qw) + (rot0.qy * rot1.qz) - (rot0.qz * rot1.qy);
		dst.qy = (rot0.qw * rot1.qy) + (rot0.qy * rot1.qw) + (rot0.qz * rot1.qx) - (rot0.qx * rot1.qz);
		dst.qz = (rot0.qw * rot1.qz) + (rot0.qz * rot1.qw) + (rot0.qx * rot1.qy) - (rot0.qy * rot1.qx);

		dst.qw = (rot0.qw * rot1.qw) - (rot0.qx * rot1.qx) - (rot0.qy * rot1.qy) - (rot0.qz * rot1.qz);

		return nrmRot(dst);
	}

	SP_GENCALL Vec3 mulRot(const Rot &rot, const Vec3 &vec){
		double rotMat[3 * 3];
		cnvRotToMat(rotMat, 3, 3, rot);

		return mulMat(rotMat, 3, 3, vec);
	}

	SP_GENCALL Vec3 mulRot(const Rot &rot, const Vec2 &vec){
		return mulRot(rot, getVec(vec.x, vec.y, 0.0));
	}


	//--------------------------------------------------------------------------------
	// rotation operator
	//--------------------------------------------------------------------------------

	SP_GENCALL Vec3 operator * (const Rot &rot, const Vec3 &vec){
		return mulRot(rot, vec);
	}

	SP_GENCALL Vec3 operator * (const Rot &rot, const Vec2 &vec){
		return mulRot(rot, vec);
	}

	SP_GENCALL Rot operator * (const Rot &rot0, const Rot &rot1){
		return mulRot(rot0, rot1);
	}


	//--------------------------------------------------------------------------------
	// rotation util
	//--------------------------------------------------------------------------------

	SP_GENCALL Rot getRotAxis(const Vec3 &x, const Vec3 &y, const Vec3 &z) {
		const Vec3 nx = unitVec(x);
		const Vec3 ny = unitVec(y);
		const Vec3 nz = unitVec(z);
		double mat[3 * 3];
		mat[0 * 3 + 0] = nx.x; mat[0 * 3 + 1] = ny.x; mat[0 * 3 + 2] = nz.x;
		mat[1 * 3 + 0] = nx.y; mat[1 * 3 + 1] = ny.y; mat[1 * 3 + 2] = nz.y;
		mat[2 * 3 + 0] = nx.z; mat[2 * 3 + 1] = ny.z; mat[2 * 3 + 2] = nz.z;

		return getRot(mat);
	}

	SP_GENCALL Rot getRotAngle(const Vec3 &vec) {
		const double angle = normVec(vec);
		if (angle > SP_SMALL) {
			const double s = sin(angle * 0.5);
			const double c = cos(angle * 0.5);
			return getRot(s * vec.x / angle, s * vec.y / angle, s * vec.z / angle, c);
		}
		else {
			return zeroRot();
		}
	}

	SP_GENCALL Rot getRotAngle(const Vec3 &vec, const double angle) {
		return getRotAngle(unitVec(vec) * angle);
	}

	SP_GENCALL Rot getRotAngleX(const double angle) {
		return getRotAngle(getVec(1.0, 0.0, 0.0), angle);
	}

	SP_GENCALL Rot getRotAngleY(const double angle) {
		return getRotAngle(getVec(0.0, 1.0, 0.0), angle);
	}

	SP_GENCALL Rot getRotAngleZ(const double angle) {
		return getRotAngle(getVec(0.0, 0.0, 1.0), angle);
	}

	SP_GENCALL Vec3 getAngle(const Rot &rot) {
		Vec3 vec = getVec(0.0, 0.0, 0.0);

		if (cmpRot(rot, getRot(0.0, 0.0, 0.0, 1.0)) == false) {
			const double angle = acos(rot.qw) * 2.0;

			if (cmpVal(angle, 0.0) == false) {
				const double s = sin(angle * 0.5);
				vec.x = rot.qx / s * angle;
				vec.y = rot.qy / s * angle;
				vec.z = rot.qz / s * angle;
			}
		}
		return vec;
	}

	SP_GENCALL Rot getRotDirection(const Vec3 &vec){
		const Vec3 nrm = unitVec(vec);

		if (fabs(nrm.z) == 1.0){
			const double angle = (nrm.z > 0) ? 0.0 : SP_PI;
			return getRotAngleX(angle);
		}
		else{
			const Vec3 v0 = crsVec(getVec(0.0, 1.0, 0.0), getVec(nrm.x, nrm.y, 0.0));
			const double a0 = acos(nrm.y / sqrt(nrm.x * nrm.x + nrm.y * nrm.y));
			const Rot rot0 = getRotAngle(v0, a0);

			const Vec3 v1 = crsVec(getVec(0.0, 0.0, 1.0), nrm);
			const double a1 = acos(nrm.z);
			const Rot rot1 = getRotAngle(v1, a1);

			return invRot(rot1 * rot0);
		}
	}

	// zyx eulter
	SP_GENCALL Rot getRotEuler(const Vec3 &euler){
		const Rot rotx = getRotAngleX(euler.x);
		const Rot roty = getRotAngleY(euler.y);
		const Rot rotz = getRotAngleZ(euler.z);
		return rotz * roty * rotx;
	}

	// zyx eulter
	SP_GENCALL Vec3 getEuler(const Rot &rot){
		double mat[3 * 3];
		cnvRotToMat(mat, 3, 3, rot);
		
		Vec3 euler;
		euler.y = asin(-mat[2 * 3 + 0]);

		if (fabs(euler.y) < SP_PI / 2.0){
			euler.z = atan2(mat[1 * 3 + 0], mat[0 * 3 + 0]);
			euler.x = atan2(mat[2 * 3 + 1], mat[2 * 3 + 2]);
		}
		else{
			euler.z = atan2(-mat[1 * 3 + 2], mat[0 * 3 + 2]);
			euler.x = 0.0;
		}

		return euler;
	}

	// random uniform
	SP_CPUCALL Rot randRotUnif(const double max){
		return getRotAngle(randVecUnif(1.0, 1.0, 1.0), randValUnif() * max);
	}

	// random gauss
	SP_CPUCALL Rot randRotGauss(const double sig){
		return getRotAngle(randVecUnif(1.0, 1.0, 1.0), randValGauss() * sig);
	}

	// update
	SP_GENCALL Rot updateRot(const Rot &rot, const double *delta){
		return getRotAngle(getVec(delta[0], delta[1], delta[2])) * rot;
	}


	//--------------------------------------------------------------------------------
	// pose
	//--------------------------------------------------------------------------------

	SP_GENCALL Pose getPose(const Rot &rot, const Vec3 &trn){
		Pose dst;
		dst.rot = nrmRot(rot);
		dst.trn = trn;

		return dst;
	}

	SP_GENCALL Pose getPose(const Rot &rot){
		return getPose(rot, getVec(0.0, 0.0, 0.0));
	}

	SP_GENCALL Pose getPose(const Vec3 &trn){
		return getPose(zeroRot(), trn);
	}

	SP_GENCALL Pose getPose(const double *mat){
		Pose dst;
		double rotMat[3 * 3];
		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				rotMat[r * 3 + c] = mat[r * 4 + c];
			}
		}
		dst.rot = getRot(mat);
		dst.trn = getVec(mat[0 * 4 + 3], mat[1 * 4 + 3], mat[2 * 4 + 3]);
		return dst;
	}

	SP_GENCALL Pose zeroPose(){
		return getPose(zeroRot(), getVec(0.0, 0.0, 0.0));
	}

	SP_GENCALL Pose invPose(const Pose &pose){
		Pose dst;
		dst.rot = invRot(pose.rot);
		dst.trn = mulVec(mulRot(dst.rot, pose.trn), -1.0);

		return dst;
	}

	SP_GENCALL Pose mulPose(const Pose &pose0, const Pose &pose1){
		Pose dst;
		dst.rot = mulRot(pose0.rot, pose1.rot);
		dst.trn = addVec(mulRot(pose0.rot, pose1.trn), pose0.trn);

		return dst;
	}

	SP_GENCALL Vec3 mulPose(const Pose &pose, const Vec3 &vec){
		double poseMat[3 * 4];
		cnvPoseToMat(poseMat, 3, 4, pose);

		return mulMat(poseMat, 3, 4, vec);
	}

	SP_GENCALL Vec3 mulPose(const Pose &pose, const Vec2 &vec){
		return mulPose(pose, getVec(vec.x, vec.y, 0.0));
	}

	SP_GENCALL VecVN3 mulPose(const Pose &pose, const VecVN3 &vec){
		double poseMat[3 * 4];
		cnvPoseToMat(poseMat, 3, 4, pose);

		return mulMat(poseMat, 3, 4, vec);
	}

	SP_GENCALL Mesh mulPose(const Pose &pose, const Mesh &mesh){
		double poseMat[3 * 4];
		cnvPoseToMat(poseMat, 3, 4, pose);

		return mulMat(poseMat, 3, 4, mesh);
	}


	//--------------------------------------------------------------------------------
	// pose operator
	//--------------------------------------------------------------------------------

	SP_GENCALL Vec3 operator * (const Pose &pose, const Vec3 &vec){
		return mulPose(pose, vec);
	}

	SP_GENCALL Vec3 operator * (const Pose &pose, const Vec2 &vec){
		return mulPose(pose, vec);
	}

	SP_GENCALL VecVN3 operator * (const Pose &pose, const VecVN3 &vec){
		return mulPose(pose, vec);
	}

	SP_GENCALL Mesh operator * (const Pose &pose, const Mesh &mesh){
		return mulPose(pose, mesh);
	}

	SP_GENCALL Pose operator * (const Pose &pose0, const Pose &pose1){
		return mulPose(pose0, pose1);
	}

	//--------------------------------------------------------------------------------
	// pose util
	//--------------------------------------------------------------------------------

	// random uniform
	SP_CPUCALL Pose randPoseUnif(const double rmax, const double tmax){
		return getPose(randRotUnif(rmax), randVecUnif(tmax, tmax, tmax));
	}

	// random gauss
	SP_CPUCALL Pose randPoseGauss(const double rsig, const double tsig){
		return getPose(randRotGauss(rsig), randVecGauss(tsig, tsig, tsig));
	}

	// update
	SP_GENCALL Pose updatePose(const Pose &pose, const double *delta){
		Pose dst;
		dst.rot = updateRot(pose.rot, &delta[0]);
		dst.trn = addVec(pose.trn, getVec(delta[3], delta[4], delta[5]));
		return dst;
	}

	// geom pose
	SP_GENCALL Pose getGeomPose(const int div, const int id, const double distance) {
		const Vec3 v = getMeshPos(getGeodesicMesh(div, id)) * -1.0;
		
		Pose pose;
		pose.rot = getRotDirection(v);
		pose.trn = getVec(0.0, 0.0, distance);
		return pose;
	}

	//--------------------------------------------------------------------------------
	// rotation / pose operator
	//--------------------------------------------------------------------------------

	SP_GENCALL Pose operator * (const Rot &rot, const Pose &pose){
		return mulPose(getPose(rot), pose);
	}

	SP_GENCALL Pose operator * (const Pose &pose, const Rot &rot){
		return mulPose(pose, getPose(rot));
	}



	//--------------------------------------------------------------------------------
	// camera parameter
	//--------------------------------------------------------------------------------

	SP_GENCALL CamParam getCamParam(const int dsize0, const int dsize1, const double fx, const double fy, const double cx, const double cy){
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

	SP_GENCALL CamParam getCamParam(const int dsize0, const int dsize1) {
		// groundless camera parameter, but in many cases empirically, no big difference
		const double f = 0.8 * (dsize0 + dsize1);
		return getCamParam(dsize0, dsize1, f, f, (dsize0 - 1) * 0.5, (dsize1 - 1) * 0.5);
	}

	SP_GENCALL CamParam getCamParam(const int *dsize) {
		return getCamParam(dsize[0], dsize[1]);
	}


	//--------------------------------------------------------------------------------
	// camera util
	//--------------------------------------------------------------------------------

	SP_GENCALL Vec2 mulCam(const CamParam &cam, const Vec2 &npx) {
		Vec2 pix;
		pix.x = npx.x * cam.fx + cam.cx;
		pix.y = npx.y * cam.fy + cam.cy;
		return pix;
	}

	SP_GENCALL Vec2 invCam(const CamParam &cam, const Vec2 &pix) {
		Vec2 npx;
		npx.x = (pix.x - cam.cx) / cam.fx;
		npx.y = (pix.y - cam.cy) / cam.fy;
		return npx;
	}

	// distiortion
	SP_GENCALL Vec2 npxDist(const CamParam &cam, const Vec2 &npx) {
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
	SP_GENCALL Vec2 pixDist(const CamParam &cam, const Vec2 &pix) {
		return mulCam(cam, npxDist(cam, invCam(cam, pix)));
	}

	// undistortion
	SP_GENCALL Vec2 npxUndist(const CamParam &cam, const Vec2 &npx) {
		const int maxit = 10;

		Vec2 undist = npx;
		for (int it = 0; it < maxit; it++){
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
	SP_GENCALL Vec2 pixUndist(const CamParam &cam, const Vec2 &pix) {
		return mulCam(cam, npxUndist(cam, invCam(cam, pix)));
	}

}


#endif
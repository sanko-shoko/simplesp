//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_POSE_H__
#define __SP_POSE_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spbase.h"
#include "spcore/spgen/spmath.h"
#include "spcore/spgen/spvec.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // rotation
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rot nrmRot(const Rot &rot){
        Rot dst;

        const double div = sqrt(rot.qx * rot.qx + rot.qy * rot.qy + rot.qz * rot.qz + rot.qw * rot.qw);
        if (div > SP_SMALL){
            const double s = (sign(rot.qw) >= 0.0) ? +1 : -1;

            dst.qx = rot.qx / div * s;
            dst.qy = rot.qy / div * s;
            dst.qz = rot.qz / div * s;
            dst.qw = rot.qw / div * s;
        }
        else{
            dst.qx = 0.0;
            dst.qy = 0.0;
            dst.qz = 0.0;
            dst.qw = 1.0;
        }
        return dst;
    }

    SP_GENFUNC Rot getRot(const SP_REAL qx, const SP_REAL qy, const SP_REAL qz, const SP_REAL qw){
        Rot dst;
        dst.qx = qx; 
        dst.qy = qy;
        dst.qz = qz;
        dst.qw = qw;
        return nrmRot(dst);
    }

    SP_GENFUNC Rot getRot(const SP_REAL *mat, const int rows, const int cols){
        Rot dst;
        dst.qx = sqrt(maxval(0.0, 1 + mat[0 * cols + 0] - mat[1 * cols + 1] - mat[2 * cols + 2])) / 2;
        dst.qy = sqrt(maxval(0.0, 1 - mat[0 * cols + 0] + mat[1 * cols + 1] - mat[2 * cols + 2])) / 2;
        dst.qz = sqrt(maxval(0.0, 1 - mat[0 * cols + 0] - mat[1 * cols + 1] + mat[2 * cols + 2])) / 2;
        dst.qw = sqrt(maxval(0.0, 1 + mat[0 * cols + 0] + mat[1 * cols + 1] + mat[2 * cols + 2])) / 2;

        dst.qx *= sign(dst.qx * (mat[2 * cols + 1] - mat[1 * cols + 2]));
        dst.qy *= sign(dst.qy * (mat[0 * cols + 2] - mat[2 * cols + 0]));
        dst.qz *= sign(dst.qz * (mat[1 * cols + 0] - mat[0 * cols + 1]));

        return nrmRot(dst);
    }

    SP_GENFUNC void getMat(SP_REAL *dst, const int rows, const int cols, const Rot &rot) {
        {
            const double qx2 = rot.qx * rot.qx;
            const double qy2 = rot.qy * rot.qy;
            const double qz2 = rot.qz * rot.qz;
            const double qw2 = rot.qw * rot.qw;

            dst[0 * cols + 0] = SP_CAST(qw2 + qx2 - qy2 - qz2);
            dst[1 * cols + 1] = SP_CAST(qw2 - qx2 + qy2 - qz2);
            dst[2 * cols + 2] = SP_CAST(qw2 - qx2 - qy2 + qz2);
        }
        {
            const double qxy = rot.qx * rot.qy;
            const double qzw = rot.qz * rot.qw;
            dst[0 * cols + 1] = SP_CAST(2 * (qxy - qzw));
            dst[1 * cols + 0] = SP_CAST(2 * (qxy + qzw));

            const double qxz = rot.qx * rot.qz;
            const double qyw = rot.qy * rot.qw;
            dst[0 * cols + 2] = SP_CAST(2 * (qxz + qyw));
            dst[2 * cols + 0] = SP_CAST(2 * (qxz - qyw));

            const double qyz = rot.qy * rot.qz;
            const double qxw = rot.qx * rot.qw;
            dst[1 * cols + 2] = SP_CAST(2 * (qyz - qxw));
            dst[2 * cols + 1] = SP_CAST(2 * (qyz + qxw));
        }
    }

    SP_GENFUNC Rot zeroRot(){
        return getRot(0.0, 0.0, 0.0, 1.0);
    }

    SP_GENFUNC Rot invRot(const Rot &rot){
        return getRot(-rot.qx, -rot.qy, -rot.qz, rot.qw);
    }

    SP_GENFUNC Rot mulRot(const Rot &rot0, const Rot &rot1){
        Rot dst;
        dst.qx = SP_CAST((rot0.qw * rot1.qx) + (rot0.qx * rot1.qw) + (rot0.qy * rot1.qz) - (rot0.qz * rot1.qy));
        dst.qy = SP_CAST((rot0.qw * rot1.qy) + (rot0.qy * rot1.qw) + (rot0.qz * rot1.qx) - (rot0.qx * rot1.qz));
        dst.qz = SP_CAST((rot0.qw * rot1.qz) + (rot0.qz * rot1.qw) + (rot0.qx * rot1.qy) - (rot0.qy * rot1.qx));

        dst.qw = SP_CAST((rot0.qw * rot1.qw) - (rot0.qx * rot1.qx) - (rot0.qy * rot1.qy) - (rot0.qz * rot1.qz));

        return nrmRot(dst);
    }

    SP_GENFUNC Vec3 mulRot(const Rot &rot, const Vec3 &vec){
        SP_REAL rotMat[3 * 3];
        getMat(rotMat, 3, 3, rot);

        return mulMat(rotMat, 3, 3, vec);
    }

    SP_GENFUNC Vec3 mulRot(const Rot &rot, const Vec2 &vec){
        return mulRot(rot, getVec3(vec.x, vec.y, 0.0));
    }

    SP_GENFUNC bool cmpRot(const Rot &rot0, const Rot &rot1, const SP_REAL t = SP_SMALL) {
        bool ret = true;
        ret &= cmpVal(rot0.qx * sign(rot0.qw), rot1.qx * sign(rot1.qw), t);
        ret &= cmpVal(rot0.qy * sign(rot0.qw), rot1.qy * sign(rot1.qw), t);
        ret &= cmpVal(rot0.qz * sign(rot0.qw), rot1.qz * sign(rot1.qw), t);
        ret &= cmpVal(rot0.qw * sign(rot0.qw), rot1.qw * sign(rot1.qw), t);
        return ret;
    }


    //--------------------------------------------------------------------------------
    // rotation operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec3 operator * (const Rot &rot, const Vec3 &vec){
        return mulRot(rot, vec);
    }

    SP_GENFUNC Vec3 operator * (const Rot &rot, const Vec2 &vec){
        return mulRot(rot, vec);
    }

    SP_GENFUNC Rot operator * (const Rot &rot0, const Rot &rot1){
        return mulRot(rot0, rot1);
    }

    SP_GENFUNC void operator *= (Rot &rot0, const Rot &rot1) {
        rot0 = mulRot(rot0, rot1);
    }

    SP_GENFUNC bool operator == (const Rot &rot0, const Rot &rot1) {
        return cmpRot(rot0, rot1);
    }

    SP_GENFUNC bool operator != (const Rot &rot0, const Rot &rot1) {
        return !cmpRot(rot0, rot1);
    }

    //--------------------------------------------------------------------------------
    // rotation util
    //--------------------------------------------------------------------------------

    SP_GENFUNC void getMatAngleX(SP_REAL *dst, const int rows, const int cols, const SP_REAL angle) {
        dst[0 * cols + 0] = 1.0;
        dst[0 * cols + 1] = 0.0;
        dst[0 * cols + 2] = 0.0;

        dst[1 * cols + 0] = 0.0;
        dst[1 * cols + 1] = +cos(angle);
        dst[1 * cols + 2] = -sin(angle);

        dst[2 * cols + 0] = 0.0;
        dst[2 * cols + 1] = +sin(angle);
        dst[2 * cols + 2] = +cos(angle);
    }

    SP_GENFUNC void getMatAngleY(SP_REAL *dst, const int rows, const int cols, const SP_REAL angle) {
        dst[0 * cols + 0] = +cos(angle);
        dst[0 * cols + 1] = 0.0;
        dst[0 * cols + 2] = +sin(angle);

        dst[1 * cols + 0] = 0.0;
        dst[1 * cols + 1] = 1.0;
        dst[1 * cols + 2] = 0.0;

        dst[2 * cols + 0] = -sin(angle);
        dst[2 * cols + 1] = 0.0;
        dst[2 * cols + 2] = +cos(angle);
    }

    SP_GENFUNC void getMatAngleZ(SP_REAL *dst, const int rows, const int cols, const SP_REAL angle) {
        dst[0 * cols + 0] = +cos(angle);
        dst[0 * cols + 1] = -sin(angle);
        dst[0 * cols + 2] = 0.0;

        dst[1 * cols + 0] = +sin(angle);
        dst[1 * cols + 1] = +cos(angle);
        dst[1 * cols + 2] = 0.0;

        dst[2 * cols + 0] = 0.0;
        dst[2 * cols + 1] = 0.0;
        dst[2 * cols + 2] = 1.0;
    }

    SP_GENFUNC void getMatRodrigues(SP_REAL *dst, const int rows, const int cols, const Vec3 &vec) {
        const SP_REAL angle = normVec(vec);
        const Vec3 nrm = unitVec(vec);

        const SP_REAL c = cos(angle);
        const SP_REAL s = sin(angle);

        dst[0 * 3 + 0] = SP_CAST(nrm.x * nrm.x * (1.0 - c) + c);
        dst[0 * 3 + 1] = SP_CAST(nrm.x * nrm.y * (1.0 - c) - nrm.z * s);
        dst[0 * 3 + 2] = SP_CAST(nrm.x * nrm.z * (1.0 - c) + nrm.y * s);

        dst[1 * 3 + 0] = SP_CAST(nrm.y * nrm.x * (1.0 - c) + nrm.z * s);
        dst[1 * 3 + 1] = SP_CAST(nrm.y * nrm.y * (1.0 - c) + c);
        dst[1 * 3 + 2] = SP_CAST(nrm.y * nrm.z * (1.0 - c) - nrm.x * s);

        dst[2 * 3 + 0] = SP_CAST(nrm.z * nrm.x * (1.0 - c) - nrm.y * s);
        dst[2 * 3 + 1] = SP_CAST(nrm.z * nrm.y * (1.0 - c) + nrm.x * s);
        dst[2 * 3 + 2] = SP_CAST(nrm.z * nrm.z * (1.0 - c) + c);
    }

    SP_GENFUNC void getMatRodrigues(SP_REAL *dst, const int rows, const int cols, const Vec3 &vec, const SP_REAL angle) {
        getMatRodrigues(dst, rows, cols, unitVec(vec) * angle);
    }

    SP_GENFUNC Rot getRotAxis(const Vec3 &x, const Vec3 &y, const Vec3 &z) {
        const Vec3 nx = unitVec(x);
        const Vec3 ny = unitVec(y);
        const Vec3 nz = unitVec(z);
        SP_REAL mat[3 * 3];
        mat[0 * 3 + 0] = nx.x; mat[0 * 3 + 1] = ny.x; mat[0 * 3 + 2] = nz.x;
        mat[1 * 3 + 0] = nx.y; mat[1 * 3 + 1] = ny.y; mat[1 * 3 + 2] = nz.y;
        mat[2 * 3 + 0] = nx.z; mat[2 * 3 + 1] = ny.z; mat[2 * 3 + 2] = nz.z;

        return getRot(mat, 3, 3);
    }

    SP_GENFUNC Rot getRotAngle(const Vec3 &vec) {
        const SP_REAL angle = normVec(vec);
        const Vec3 nrm = unitVec(vec);

        if (angle > SP_SMALL) {
            const SP_REAL s = sin(angle * 0.5);
            const SP_REAL c = cos(angle * 0.5);
            return getRot(s * nrm.x, s * nrm.y, s * nrm.z, c);
        }
        else {
            return zeroRot();
        }
    }

    SP_GENFUNC Rot getRotAngle(const Vec3 &vec, const double angle) {
        return getRotAngle(unitVec(vec) * angle);
    }

    SP_GENFUNC Rot getRotAngleX(const double angle) {
        return getRotAngle(getVec3(1.0, 0.0, 0.0), angle);
    }

    SP_GENFUNC Rot getRotAngleY(const double angle) {
        return getRotAngle(getVec3(0.0, 1.0, 0.0), angle);
    }

    SP_GENFUNC Rot getRotAngleZ(const double angle) {
        return getRotAngle(getVec3(0.0, 0.0, 1.0), angle);
    }

    SP_GENFUNC Vec3 getAngle(const Rot &rot) {
        Vec3 vec = getVec3(0.0, 0.0, 0.0);

        if (cmpRot(rot, getRot(0.0, 0.0, 0.0, 1.0)) == false) {
            const SP_REAL angle = acos(rot.qw) * 2.0;

            if (cmpVal(angle, 0.0) == false) {
                const SP_REAL s = sin(angle * 0.5);
                vec.x = rot.qx / s * angle;
                vec.y = rot.qy / s * angle;
                vec.z = rot.qz / s * angle;
            }
        }
        return vec;
    }

    SP_GENFUNC SP_REAL getAngle(const Rot &rot, const int axis) {
        SP_ASSERT(axis >= 0 && axis < 3);

        const Vec3 v0 = getVec3(axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0, axis == 2 ? 1.0 : 0.0);
        const Vec3 v1 = rot * v0;
        const SP_REAL angle = acos(dotVec(v0, v1));
        return angle;
    }

    SP_GENFUNC Rot getRotDirection(const Vec3 &vec){
        const Vec3 nrm = unitVec(vec);

        if (fabs(nrm.z) == 1.0){
            const SP_REAL angle = (nrm.z > 0) ? 0.0 : SP_PI;
            return getRotAngleX(angle);
        }
        else{
            const Vec3 v0 = crsVec(getVec3(0.0, 1.0, 0.0), getVec3(nrm.x, nrm.y, 0.0));
            const SP_REAL a0 = acos(nrm.y / sqrt(nrm.x * nrm.x + nrm.y * nrm.y));
            const Rot rot0 = getRotAngle(v0, a0);

            const Vec3 v1 = crsVec(getVec3(0.0, 0.0, 1.0), nrm);
            const SP_REAL a1 = acos(nrm.z);
            const Rot rot1 = getRotAngle(v1, a1);

            return invRot(rot1 * rot0);
        }
    }

    // zyx eulter
    SP_GENFUNC Rot getRotEuler(const Vec3 &euler){
        const Rot rotx = getRotAngleX(euler.x);
        const Rot roty = getRotAngleY(euler.y);
        const Rot rotz = getRotAngleZ(euler.z);
        return rotz * roty * rotx;
    }

    // zyx eulter
    SP_GENFUNC Vec3 getEuler(const SP_REAL *mat, const int rows, const int cols) {

        Vec3 euler;
        euler.y = asin(-mat[2 * 3 + 0]);

        if (fabs(euler.y) < SP_PI / 2.0) {
            euler.z = atan2(mat[1 * 3 + 0], mat[0 * 3 + 0]);
            euler.x = atan2(mat[2 * 3 + 1], mat[2 * 3 + 2]);
        }
        else {
            euler.z = atan2(-mat[1 * 3 + 2], mat[0 * 3 + 2]);
            euler.x = 0.0;
        }

        return euler;
    }

    // zyx eulter
    SP_GENFUNC Vec3 getEuler(const Rot &rot){
        SP_REAL mat[3 * 3];
        getMat(mat, 3, 3, rot);
        
        return getEuler(mat, 3, 3);
    }

    // random unif
    SP_CPUFUNC Rot randuRot(const double max) {
        return getRotAngle(randuVec3(1.0, 1.0, 1.0), randu() * max);
    }

    // random gauss
    SP_CPUFUNC Rot randgRot(const double max){
        return getRotAngle(randuVec3(1.0, 1.0, 1.0), randg() * max);
    }

    // update
    SP_GENFUNC Rot updateRot(const Rot &rot, const SP_REAL *delta){
        return getRotAngle(getVec3(delta[0], delta[1], delta[2])) * rot;
    }

    // dif
    SP_GENFUNC SP_REAL difRot(const Rot &rot0, const Rot &rot1) {
        return normVec(getAngle(rot0 * invRot(rot1)));
    }

    // dif
    SP_GENFUNC SP_REAL difRot(const Rot &rot0, const Rot &rot1, const int axis) {
        return getAngle(rot0 * invRot(rot1), axis);
    }


    //--------------------------------------------------------------------------------
    // pose
    //--------------------------------------------------------------------------------

    SP_GENFUNC Pose getPose(const Rot &rot, const Vec3 &trn){
        Pose dst;
        dst.rot = nrmRot(rot);
        dst.trn = trn;

        return dst;
    }

    SP_GENFUNC Pose getPose(const Rot &rot){
        return getPose(rot, getVec3(0.0, 0.0, 0.0));
    }

    SP_GENFUNC Pose getPose(const Vec3 &trn){
        return getPose(zeroRot(), trn);
    }

    SP_GENFUNC Pose getPose(const SP_REAL *mat, const int rows, const int cols){
        Pose dst;
        if ((rows == 3 || rows == 4) && cols == 4) {
            dst.rot = getRot(mat, rows, cols);
            dst.trn = getVec3(mat[0 * cols + 3], mat[1 * cols + 3], mat[2 * cols + 3]);
        }
        if ((rows == 6 && cols == 1) || (rows == 1 && cols == 6)) {
            Vec3 euler = getVec3(mat[0], mat[1], mat[2]);
            dst.rot = getRotEuler(euler);
            dst.trn = getVec3(mat[3], mat[4], mat[5]);
        }
        return dst;
    }

    SP_GENFUNC void getMat(SP_REAL *dst, const int rows, const int cols, const Pose &pose) {
        if ((rows == 3 || rows == 4) && cols == 4) {
            eyeMat(dst, rows, cols);
            getMat(dst, rows, cols, pose.rot);

            dst[0 * cols + 3] = pose.trn.x;
            dst[1 * cols + 3] = pose.trn.y;
            dst[2 * cols + 3] = pose.trn.z;
        }
        if ((rows == 6 && cols == 1) || (rows == 1 && cols == 6)) {
            const Vec3 euler = getEuler(pose.rot);
            dst[0] = euler.x;
            dst[1] = euler.y;
            dst[2] = euler.z;
            dst[3] = pose.trn.x;
            dst[4] = pose.trn.y;
            dst[5] = pose.trn.z;
        }
    }

    SP_GENFUNC Pose zeroPose(){
        return getPose(zeroRot(), getVec3(0.0, 0.0, 0.0));
    }

    SP_GENFUNC Pose invPose(const Pose &pose){
        Pose dst;
        dst.rot = invRot(pose.rot);
        dst.trn = mulVec(mulRot(dst.rot, pose.trn), -1.0);

        return dst;
    }

    SP_GENFUNC Pose mulPose(const Pose &pose0, const Pose &pose1){
        Pose dst;
        dst.rot = mulRot(pose0.rot, pose1.rot);
        dst.trn = addVec(mulRot(pose0.rot, pose1.trn), pose0.trn);

        return dst;
    }

    SP_GENFUNC Vec3 mulPose(const Pose &pose, const Vec3 &vec){
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, vec);
    }

    SP_GENFUNC Vec3 mulPose(const Pose &pose, const Vec2 &vec){
        return mulPose(pose, getVec3(vec.x, vec.y, 0.0));
    }

    SP_GENFUNC VecPN3 mulPose(const Pose &pose, const VecPN3 &vec){
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, vec);
    }

    SP_GENFUNC Line3 mulPose(const Pose &pose, const Line3 &line) {
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, line);
    }

    SP_GENFUNC Mesh3 mulPose(const Pose &pose, const Mesh3 &mesh){
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, mesh);
    }

    SP_GENFUNC bool cmpPose(const Pose &pose0, const Pose &pose1, const SP_REAL tr = SP_SMALL, const SP_REAL tt = SP_SMALL) {
        bool ret = true;
        ret &= cmpRot(pose0.rot, pose1.rot, tr);
        ret &= cmpVec(pose0.trn, pose1.trn, tt);
        return ret;
    }


    //--------------------------------------------------------------------------------
    // pose operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec3 operator * (const Pose &pose, const Vec3 &vec){
        return mulPose(pose, vec);
    }

    SP_GENFUNC Vec3 operator * (const Pose &pose, const Vec2 &vec){
        return mulPose(pose, vec);
    }

    SP_GENFUNC VecPN3 operator * (const Pose &pose, const VecPN3 &vec){
        return mulPose(pose, vec);
    }

    SP_GENFUNC Line3 operator * (const Pose &pose, const Line3 &line) {
        return mulPose(pose, line);
    }
    
    SP_GENFUNC Mesh3 operator * (const Pose &pose, const Mesh3 &mesh){
        return mulPose(pose, mesh);
    }

    SP_GENFUNC Pose operator * (const Pose &pose0, const Pose &pose1){
        return mulPose(pose0, pose1);
    }

    SP_GENFUNC void operator *= (Pose &pose0, const Pose &pose1) {
        pose0 = mulPose(pose0, pose1);
    }

    SP_GENFUNC bool operator == (const Pose &pose0, const Pose &pose1) {
        return cmpPose(pose0, pose1);
    }

    SP_GENFUNC bool operator != (const Pose &pose0, const Pose &pose1) {
        return !cmpPose(pose0, pose1);
    }
    
    //--------------------------------------------------------------------------------
    // pose util
    //--------------------------------------------------------------------------------

    // random unif
    SP_CPUFUNC Pose randuPose(const double rmax, const double tmax) {
        return getPose(randuRot(rmax), randuVec3(tmax, tmax, tmax));
    }
    
    // random gauss
    SP_CPUFUNC Pose randgPose(const double rmax, const double tmax){
        return getPose(randgRot(rmax), randgVec3(tmax, tmax, tmax));
    }

    // update
    SP_GENFUNC Pose updatePose(const Pose &pose, const SP_REAL *delta){
        Pose dst;
        dst.rot = updateRot(pose.rot, &delta[0]);
        dst.trn = pose.trn + getVec3(delta[3], delta[4], delta[5]);
        return dst;
    }


    SP_GENFUNC Pose getGeodesicPose(const int level, const int id, const SP_REAL distance = 0.0) {
        const Vec3 v = getMeshPos(getGeodesicMesh(level, id)) * (-1.0);
        const Pose pose = getPose(getRotDirection(v), getVec3(0.0, 0.0, distance));
        return pose;
    }

    //--------------------------------------------------------------------------------
    // rotation / pose operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Pose operator * (const Rot &rot, const Pose &pose){
        return mulPose(getPose(rot), pose);
    }

    SP_GENFUNC Pose operator * (const Pose &pose, const Rot &rot){
        return mulPose(pose, getPose(rot));
    }

    SP_GENFUNC void operator *= (Pose &pose, const Rot &rot) {
        pose = mulPose(pose, getPose(rot));
    }

    //--------------------------------------------------------------------------------
    // jacob
    //--------------------------------------------------------------------------------

    SP_GENFUNC void jacobPosToNpx(SP_REAL *jacob, const Vec3 &pos) {
        SP_REAL divz = (pos.z != 0) ? 1.0 / pos.z : 0.0;

        jacob[0 * 3 + 0] = divz; jacob[0 * 3 + 1] = 0.0; jacob[0 * 3 + 2] = -pos.x * divz * divz;
        jacob[1 * 3 + 0] = 0.0; jacob[1 * 3 + 1] = divz; jacob[1 * 3 + 2] = -pos.y * divz * divz;
    }

    SP_GENFUNC void jacobPosToPix(SP_REAL *jacob, const CamParam &cam, const Vec3 &pos) {

        SP_REAL jPosToNpz[2 * 3] = { 0 };
        jacobPosToNpx(jPosToNpz, pos);

        SP_REAL jNpxToPix[2 * 2];
        jacobNpxToPix(jNpxToPix, cam, prjVec(pos));

        mulMat(jacob, 2, 3, jNpxToPix, 2, 2, jPosToNpz, 2, 3);
    }

    SP_GENFUNC void jacobPoseToPos(SP_REAL *jacob, const Pose &pose, const Vec3 &pos) {
        SP_REAL rmat[3 * 3];
        getMat(rmat, 3, 3, pose.rot);
        const Vec3 v = mulMat(rmat, 3, 3, pos);
        jacob[0 * 6 + 0] = +0.0; jacob[0 * 6 + 1] = +v.z; jacob[0 * 6 + 2] = -v.y;
        jacob[1 * 6 + 0] = -v.z; jacob[1 * 6 + 1] = +0.0; jacob[1 * 6 + 2] = +v.x;
        jacob[2 * 6 + 0] = +v.y; jacob[2 * 6 + 1] = -v.x; jacob[2 * 6 + 2] = +0.0;

        jacob[0 * 6 + 3] = 1.0; jacob[0 * 6 + 4] = 0.0; jacob[0 * 6 + 5] = 0.0;
        jacob[1 * 6 + 3] = 0.0; jacob[1 * 6 + 4] = 1.0; jacob[1 * 6 + 5] = 0.0;
        jacob[2 * 6 + 3] = 0.0; jacob[2 * 6 + 4] = 0.0; jacob[2 * 6 + 5] = 1.0;

    }

    SP_GENFUNC void jacobPoseToNpx(SP_REAL *jacob, const Pose &pose, const Vec3 &pos) {
        SP_REAL pmat[3 * 4];
        getMat(pmat, 3, 4, pose);

        SP_REAL jPoseToPos[3 * 6] = { 0 };
        jacobPoseToPos(jPoseToPos, pose, pos);

        SP_REAL jPosToNpx[2 * 3] = { 0 };
        jacobPosToNpx(jPosToNpx, mulMat(pmat, 3, 4, pos));

        mulMat(jacob, 2, 6, jPosToNpx, 2, 3, jPoseToPos, 3, 6);
    }

    SP_GENFUNC void jacobPoseToPix(SP_REAL *jacob, const CamParam &cam, const Pose &pose, const Vec3 &pos) {
        SP_REAL pmat[3 * 4];
        getMat(pmat, 3, 4, pose);

        SP_REAL jPoseToPos[3 * 6] = { 0 };
        jacobPoseToPos(jPoseToPos, pose, pos);

        SP_REAL jPosToPix[2 * 3];
        jacobPosToPix(jPosToPix, cam, mulMat(pmat, 3, 4, pos));

        mulMat(jacob, 2, 6, jPosToPix, 2, 3, jPoseToPos, 3, 6);
    }

}


#endif
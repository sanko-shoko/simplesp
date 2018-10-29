//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GEOMETRY_H__
#define __SP_GEOMETRY_H__

#include "spcore/spcore.h"
#include "spapp/spgeom/spxmat.h"

namespace sp {

    //--------------------------------------------------------------------------------
    // projection error
    //--------------------------------------------------------------------------------

    SP_CPUFUNC double calcPrjErr(const Pose &pose, const CamParam &cam, const Vec2 &pix, const Vec3 &obj) {
        const Vec3 vec = pose * obj;

        double ret = 0.0;
        if (vec.z > 0) {
            ret = normVec(pix - mulCamD(cam, prjVec(vec)));
        }
        else {
            ret = SP_INFINITY;
        }

        return ret;
    }

    SP_CPUFUNC double calcPrjErr(const Pose &pose, const CamParam &cam, const Vec2 &pix, const Vec2 &obj) {
        return calcPrjErr(pose, cam, pix, getVec(obj, 0.0));
    }

    SP_CPUFUNC Mem1<double> calcPrjErr(const Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs) {
        SP_ASSERT(pixs.size() == objs.size());

        Mem1<double> errs(pixs.size());
        for (int i = 0; i < pixs.size(); i++) {
            errs[i] = calcPrjErr(pose, cam, pixs[i], objs[i]);
        }
        return errs;
    }


    SP_CPUFUNC Mem1<double> calcPrjErr(const Mem1<Pose> &poses, const Mem1<CamParam> &cams, const Mem1<Vec2> &pixs, const Vec3 &obj) {
        SP_ASSERT(poses.size() == cams.size() && poses.size() == pixs.size());

        Mem1<double> errs(poses.size());
        for (int i = 0; i < poses.size(); i++) {
            errs[i] = calcPrjErr(poses[i], cams[i], pixs[i], obj);
        }
        return errs;
    }

    //--------------------------------------------------------------------------------
    // triangulation
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool refinePnt3d(Vec3 &pnt, const Mem1<Pose> &poses, const Mem1<Vec2> &npxs, const int maxit = 1) {
        SP_ASSERT(poses.size() == npxs.size());

        const int unit = 2;
        if (npxs.size() < unit) return false;

        Mat J(poses.size() * 2, 3);
        Mat E(poses.size() * 2, 1);
        Mem1<double> errs(poses.size());

        Mat jacob(2, 3);
        for (int it = 0; it < maxit; it++) {
            for (int i = 0; i < poses.size(); i++) {
                const Vec3 pos = poses[i] * pnt;

                jacobPosToNpx(jacob.ptr, pos);
                const Mat R = getMat(poses[i].rot);
                jacob = jacob * R;
                memcpy(&J(i * 2, 0), jacob.ptr, jacob.size() * sizeof(double));

                const Vec2 err = npxs[i] - prjVec(pos);
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

    SP_CPUFUNC bool refinePnt3d(Vec3 &pnt, const Mem1<Pose> &poses, const Mem1<CamParam> &cams, const Mem1<Vec2> &pixs, const int maxit = 1) {
        SP_ASSERT(poses.size() == cams.size() && poses.size() == pixs.size());

        Mem1<Vec2> npxs(pixs.size());
        for (int i = 0; i < npxs.size(); i++) {
            npxs[i] = invCamD(cams[i], pixs[i]);
        }
        return refinePnt3d(pnt, poses, npxs, maxit);
    }


    SP_CPUFUNC bool calcPnt3d(Vec3 &pnt, const Mem1<Pose> &poses, const Mem1<Vec2> &npxs) {
        SP_ASSERT(poses.size() == npxs.size());

        Mat M(poses.size() * 2, 3);
        Mat V(poses.size() * 2, 1);

        for (int i = 0; i < poses.size(); i++) {
            const Vec2 &npx = npxs[i];

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

    SP_CPUFUNC bool calcPnt3d(Vec3 &pnt, const Mem1<Pose> &poses, const Mem1<CamParam> &cams, const Mem1<Vec2> &pixs) {
        SP_ASSERT(poses.size() == cams.size() && poses.size() == pixs.size());

        Mem1<Vec2> npxs(pixs.size());
        for (int i = 0; i < npxs.size(); i++) {
            npxs[i] = invCamD(cams[i], pixs[i]);
        }
        return calcPnt3d(pnt, poses, npxs);
    }

    SP_CPUFUNC bool calcPnt3d(Vec3 &pnt, const Pose &pose0, const Vec2 &npx0, const Pose &pose1, const Vec2 &npx1) {
        const Pose _poses[2] = { pose0, pose1 };
        const Vec2 _npxs[2] = { npx0, npx1 };

        const Mem1<Pose> poses(2, _poses);
        const Mem1<Vec2> npxs(2, _npxs);

        return calcPnt3d(pnt, poses, npxs);
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

    SP_CPUFUNC bool calcPnt3d(Mem1<Vec3> &pnts, Mem1<bool> &mask, const Pose &pose0, const CamParam &cam0, const Mem1<Vec2> &pixs0, const Pose &pose1, const CamParam &cam1, const Mem1<Vec2> &pixs1) {
        SP_ASSERT(pixs0.size() == pixs1.size());

        pnts.resize(pixs0.size());
        mask.resize(pixs0.size());
        pnts.zero();
        mask.zero();

        for (int i = 0; i < pixs0.size(); i++) {
            Vec3 pnt;
            if (calcPnt3d(pnt, pose0, cam0, pixs0[i], pose1, cam1, pixs1[i]) == false) continue;

            pnts[i] = pnt;
            mask[i] = true;
        }
        return true;
    }

    SP_CPUFUNC bool calcPnt3dX(Vec3 &pnt, const Pose &pose0, const CamParam &cam0, const Vec2 &pix0, const Pose &pose1, const CamParam &cam1, const double pix1x) {
        const Vec2 &npx0 = invCamD(cam0, pix0);

        const Pose stereo = pose1 * invPose(pose0);
        const Mat mat = getMat(stereo);

        const Mat E = skewMat(stereo.trn) * getMat(stereo.rot);

        const Vec3 epi = E * getVec(npx0.x, npx0.y, 1.0);
        const Vec2 npx1 = npxUndistX(cam1, epi, (pix1x - cam1.cx) / cam1.fx);

        const double div
            = (mat(0, 0) - npx1.x * mat(2, 0)) * npx0.x
            + (mat(0, 1) - npx1.x * mat(2, 1)) * npx0.y
            + (mat(0, 2) - npx1.x * mat(2, 2));

        if (fabs(div) < SP_SMALL) return false;

        const double depth = (npx1.x * mat(2, 3) - mat(0, 3)) / div;

        pnt = pose0 * (getVec(npx0.x, npx0.y, 1.0) * depth);
        return true;
    }

    SP_CPUFUNC bool calcPnt3dY(Vec3 &pnt, const Pose &pose0, const CamParam &cam0, const Vec2 &pix0, const Pose &pose1, const CamParam &cam1, const double pix1y) {
        const Vec2 &npx0 = invCamD(cam0, pix0);

        const Pose stereo = pose1 * invPose(pose0);
        const Mat mat = getMat(stereo);

        const Mat E = skewMat(stereo.trn) * getMat(stereo.rot);

        const Vec3 epi = E * getVec(npx0.x, npx0.y, 1.0);
        const Vec2 npx1 = npxUndistY(cam1, epi, (pix1y - cam1.cy) / cam1.fy);

        const double div
            = (mat(1, 0) - npx1.y * mat(2, 0)) * npx0.x
            + (mat(1, 1) - npx1.y * mat(2, 1)) * npx0.y
            + (mat(1, 2) - npx1.y * mat(2, 2));

        if (fabs(div) < SP_SMALL) return false;

        const double depth = (npx1.y * mat(2, 3) - mat(1, 3)) / div;
        pnt = pose0 * (getVec(npx0.x, npx0.y, 1.0) * depth);
        return true;
    }


    //--------------------------------------------------------------------------------
    // matrix to pose
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC bool dcmpEMat(Pose &pose, const Mat &E, const Mem1<Vec2> &npxs0, const Mem1<Vec2> &npxs1) {
        SP_ASSERT(npxs0.size() == npxs1.size());

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

        int maxv = 0;
        for (int i = 0; i < 4; i++) {
            const Pose test = getPose(R[i % 2], T[i / 2]);

            Mem1<Vec3> pnts(npxs0.size());

            int cnt = 0;
            for (int i = 0; i < npxs0.size(); i++) {
                if (calcPnt3d(pnts[i], test, npxs0[i], zeroPose(), npxs1[i]) == false) continue;
                cnt++;
            }

            if (cnt > maxv) {
                maxv = cnt;
                pose = test;
            }
        }

        return true;
    }

    SP_CPUFUNC bool dcmpFMat(Pose &pose, const Mat &F, const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const Mem1<Vec2> &pixs1) {
        SP_ASSERT(pixs0.size() == pixs1.size());

        const Mat E = trnMat(getMat(cam0)) * F * getMat(cam1);

        const Mem1<Vec2> npxs0 = invCamD(cam0, pixs0);
        const Mem1<Vec2> npxs1 = invCamD(cam1, pixs1);

        if (dcmpEMat(pose, E, npxs0, npxs1) == false) return false;

        return true;
    }

    //--------------------------------------------------------------------------------
    // pose (refine)
    //--------------------------------------------------------------------------------
  
    // 3D-3D pose
    SP_CPUFUNC bool refinePose(Pose &pose, const Mem1<Vec3> &objs0, const Mem1<Vec3> &objs1, const int maxit = 10) {
        SP_ASSERT(objs0.size() == objs1.size());

        const int num = objs0.size();

        const int unit = 3;
        if (num < unit) return false;
        
        Mat J(3 * num, 6);
        Mat E(3 * num, 1);
        Mem1<double> errs(num);

        for (int it = 0; it < maxit; it++) {
            for (int i = 0; i < num; i++) {
                jacobPoseToPos(&J(i * 3, 0), pose, objs1[i]);

                const Vec3 err = objs0[i] - pose * objs1[i];
                E(i * 3 + 0, 0) = err.x;
                E(i * 3 + 1, 0) = err.y;
                E(i * 3 + 2, 0) = err.z;

                errs[i] = normVec(err);
            }
            Mat delta;
            if (solveEq(delta, J, E, errs) == false) return false;

            pose = updatePose(pose, delta.ptr);
        }

        return true;
    }

    // 2D-3D pose
    SP_CPUFUNC bool refinePose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs, const int maxit = 10) {
        SP_ASSERT(pixs.size() == objs.size());

        const int num = pixs.size();

        const int unit = 3;
        if (num < unit) return false;

        Mat J(2 * num, 6);
        Mat E(2 * num, 1);
        Mem1<double> errs(num);

        for (int it = 0; it < maxit; it++) {
            for (int i = 0; i < num; i++) {
                jacobPoseToPix(&J(i * 2, 0), cam, pose, objs[i]);

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

    // 2D-2D pose (planar object)
    SP_CPUFUNC bool refinePose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec2> &objs, const int maxit = 10) {
        return refinePose(pose, cam, pixs, getVec(objs, 0.0), maxit);
    }

    // 2D-2D pose (stereo camera)
    SP_CPUFUNC bool refinePose(Pose &pose, const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const Mem1<Vec2> &pixs1, const int maxit = 10) {
        SP_ASSERT(pixs0.size() == pixs1.size());

        const int num = pixs0.size();

        const int unit = 3;
        if (num < unit) return false;

        for (int it = 0; it < maxit; it++) {
            Mem1<Vec2> pixs;
            Mem1<Vec3> objs;

            for (int i = 0; i < num; i++) {
                Vec3 obj;
                if (calcPnt3d(obj, pose, cam0, pixs0[i], zeroPose(), cam1, pixs1[i]) == false) continue;
              
                pixs.push(pixs0[i]);
                objs.push(obj);
            }

            if (refinePose(pose, cam0, pixs, objs, 1) == false) return false;
            
            pose.trn /= normVec(pose.trn);
        }

        return true;
    }


    //--------------------------------------------------------------------------------
    // pose (basic)
    //--------------------------------------------------------------------------------

    // 3D-3D pose
    SP_CPUFUNC bool calcPose(Pose &pose, const Mem1<Vec3> &objs0, const Mem1<Vec3> &objs1, const int maxit = 10) {
        SP_ASSERT(objs0.size() == objs1.size());

        const int num = objs0.size();

        const int unit = 3;
        if (num < unit) return false;

        const Vec3 cent0 = meanVec(objs0);
        const Vec3 cent1 = meanVec(objs1);

        const Mem1<Vec3> mobjs0 = objs0 - cent0;
        const Mem1<Vec3> mobjs1 = objs1 - cent1;

        {
            Mat mat = zeroMat(3, 3);
            for (int i = 0; i < num; i++) {
                const double *y = reinterpret_cast<const double*>(&mobjs0[i]);
                const double *x = reinterpret_cast<const double*>(&mobjs1[i]);

                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        mat(r, c) += x[r] * y[c];
                    }
                }
            }
            Mat U, S, V;
            svdMat(U, S, V, mat, false);

            Mat H = eyeMat(3, 3);
            H(2, 2) = detMat(V * trnMat(U));

            pose.rot = getRot(V * H * trnMat(U));

            pose.trn = cent0 - pose.rot * cent1;
        }

        if (maxit - 1 > 0) {
            if (refinePose(pose, objs0, objs1, maxit - 1) == false) return false;
        }

        return true;
    }

    // 2D-3D pose P3P
    SP_CPUFUNC bool calcPoseP3P(Mem1<Pose> &poses, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs) {

        SP_ASSERT(pixs.size() >= 3 && pixs.size() == objs.size());

        Mem1<Vec3> nrms;
        for (int i = 0; i < 3; i++) {
            const Vec2 npx = invCamD(cam, pixs[i]);
            const Vec3 vec = getVec(npx, 1.0);
            const Vec3 nrm = vec / normVec(vec);
            nrms.push(nrm);
        }

        const double a = normVec(objs[1] - objs[0]);
        const double b = normVec(objs[2] - objs[1]);
        const double c = normVec(objs[0] - objs[2]);
        if (a < SP_SMALL || b < SP_SMALL || c < SP_SMALL) return false;

        const double cos_a = dotVec(nrms[1], nrms[0]);
        const double cos_b = dotVec(nrms[2], nrms[1]);
        const double cos_c = dotVec(nrms[0], nrms[2]);

        const double cos2_a = cos_a * cos_a;
        const double cos2_b = cos_b * cos_b;
        const double cos2_c = cos_c * cos_c;
        
        const double na = square(a / b);
        const double nc = square(c / b);

        const double s = na + nc;
        const double t = na - nc;

        const double A4 = square(t - 1.0) - 4.0 * nc * cos2_a;
        const double A3 = 4.0 * (t * (1.0 - t) * cos_b - (1.0 - s) * cos_a * cos_c + 2.0 * nc * cos2_a * cos_b);
        const double A2 = 2.0 * (t * t - 1.0 + 2.0 * t * t * cos2_b + 2.0 * (1.0 - nc) * cos2_a - 4.0 * s * cos_a * cos_b * cos_c + 2.0 * (1.0 - na) * cos2_c);
        const double A1 = 4.0 * (-t * (1.0 + t) * cos_b + 2.0 * na * cos2_c * cos_b - (1.0 - s) * cos_a * cos_c);
        const double A0 = square(1.0 + t) - 4.0 * na * cos2_c;

        Cmp vs[4];
        const int n = eq4(vs, A4, A3, A2, A1, A0);

        poses.clear();
        for (int i = 0; i < n; i++) {
            if (fabs(vs[i].im) > SP_SMALL) continue;

            const double v = vs[i].re;
            const double u = ((-1.0 + t) * v * v - 2.0 * t * cos_b * v + 1.0 + t) / (2.0 * (cos_c - v * cos_a));

            const double x = a * a / (u * u + v * v - 2.0 * u * v * cos_a);
            const double y = b * b / (1 + v * v - 2.0 * v * cos_b);
            const double z = c * c / (1 + u * u - 2.0 * u * cos_c);
            if (x < 0.0) continue;

            const double s1 = sqrt(x);
            const double s2 = u * s1;
            const double s3 = v * s1;

            Mem1<Vec3> pnts;
            pnts.push(nrms[0] * s2);
            pnts.push(nrms[1] * s3);
            pnts.push(nrms[2] * s1);

            Pose pose;
            if (calcPose(pose, pnts, objs) == true) {
                poses.push(pose);
            }
        }

        return true;
    }

    // 2D-3D pose P4P
    SP_CPUFUNC bool calcPoseP4P(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs) {

        SP_ASSERT(pixs.size() >= 4 && pixs.size() == objs.size());

        Mem1<Pose> poses;
        if(calcPoseP3P(poses, cam, pixs.slice(0, 0, 3), objs.slice(0, 0, 3)) == false) return false;

        double mine = SP_INFINITY;
        for (int i = 0; i < poses.size(); i++) {
            const Pose test = poses[i];
            const double err = sumVal(calcPrjErr(test, cam, pixs, objs));
            if (err < mine) {
                mine = err;
                pose = test;
            }
        }
        return true;
    }

    // 2D-3D pose DLT
    SP_CPUFUNC bool calcPoseDLT(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs) {
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
   
    // 2D-3D pose PNP
    SP_CPUFUNC bool calcPose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs, const int maxit = 10) {
        SP_ASSERT(pixs.size() == objs.size());

        const int unit = 4;
        if (pixs.size() < unit) return false;

        if (pixs.size() < 6) {
            if (calcPoseP4P(pose, cam, pixs.slice(0, 0, 4), objs.slice(0, 0, 4)) == false) return false;
        }
        else {
            if (calcPoseDLT(pose, cam, pixs, objs) == false) return false;
        }

        if (maxit - 1 > 0) {
            if (refinePose(pose, cam, pixs, objs, maxit - 1) == false) return false;
        }

        return true;
    }

    // 2D-2D pose (planar object)
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

    // 2D-2D pose (planar object)
    SP_CPUFUNC bool calcPose(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec2> &objs, const int maxit = 10) {
        SP_ASSERT(pixs.size() == objs.size());

        const int unit = 4;
        if (pixs.size() < unit) return false;

        const Mem1<Vec2> udpixs = pixUndist(cam, pixs);

        Mat hom;
        if (calcHMat(hom, udpixs, objs) == false) return false;

        if (calcPose(pose, cam, hom) == false) return false;

        if (maxit - 1 > 0) {
            if (refinePose(pose, cam, pixs, getVec(objs, 0.0), maxit) == false) return false;
        }

        return true;
    }

    // 2D-2D pose (stereo camera)
    SP_CPUFUNC bool calcPose(Pose &pose, const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const Mem1<Vec2> &pixs1, const int maxit = 10) {
        SP_ASSERT(pixs0.size() == pixs1.size());

        const Mem1<Vec2> npxs0 = invCamD(cam0, pixs0);
        const Mem1<Vec2> npxs1 = invCamD(cam1, pixs1);

        Mat E;
        if (calcEMat(E, npxs0, npxs1) == false) return false;

        if (dcmpEMat(pose, E, npxs0, npxs1) == false) return false;

        if (maxit - 1 > 0) {
            if (refinePose(pose, cam0, pixs0, cam1, pixs1) == false) return false;
        }

        return true;
    }


    //--------------------------------------------------------------------------------
    // pose (RANSAC + refine)
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool calcPnt3dRANSAC(Vec3 &pos, const Mem1<Pose> &poses, const Mem1<CamParam> &cams, const Mem1<Vec2> &pixs, const double thresh = 3.0) {
        SP_ASSERT(poses.size() == cams.size() && poses.size() == pixs.size());

        const int num = poses.size();
        const int unit = 2;

        if (num < unit) return false;
        if (pixs.size() < unit * SP_RANSAC_MINRATE) {
            return calcPnt3d(pos, poses, cams, pixs);
        }

        int maxit = adaptiveStop(SP_RANSAC_MINEVAL, unit);

        Mem1<Pose> sposes, rposes;
        Mem1<CamParam> scams, rcams;
        Mem1<Vec2> spixs, rpixs;

        double maxe = 0.0;
        int it = 0;
        for (it = 0; it < maxit; it++) {
            const int p = it % (pixs.size() - unit);
            if (p == 0) {
                sposes = shuffle(poses, it);
                scams = shuffle(cams, it);
                spixs = shuffle(pixs, it);
            }

            rposes.resize(unit, &sposes[p]);
            rcams.resize(unit, &scams[p]);
            rpixs.resize(unit, &spixs[p]);

            Vec3 test;
            if (calcPnt3d(test, rposes, rcams, rpixs) == false) continue;

            const Mem1<double> errs = calcPrjErr(poses, cams, pixs, test);
            const double eval = evalErr(errs, thresh);

            if (eval > maxe) {
                //SP_PRINTD("eval %lf\n", eval);
                maxe = eval;
                maxit = adaptiveStop(eval, unit);

                pos = test;
            }
        }
        //SP_PRINTD("RANSAC iteration %d rate %.2lf\n", it, maxe);
        if (maxe < SP_RANSAC_MINEVAL || maxe * num < unit * SP_RANSAC_MINRATE) return false;

        // refine
        {
            const Mem1<double> errs = calcPrjErr(poses, cams, pixs, pos);
            const Mem1<Pose> dposes = denoise(poses, errs, thresh);
            const Mem1<CamParam> dcams = denoise(cams, errs, thresh);
            const Mem1<Vec2> dpixs = denoise(pixs, errs, thresh);

            if (refinePnt3d(pos, dposes, dcams, dpixs) == false) return false;
        }
        return true;
    }

    // 3D-3D pose
    SP_CPUFUNC bool calcPoseRANSAC(Pose &pose, const Mem1<Vec3> &objs0, const Mem1<Vec3> &objs1, const double thresh = 10.0) {
        SP_ASSERT(objs0.size() == objs1.size());
      
        const int num = objs0.size();
        const int unit = 3;

        if (num < unit) return false;
        if (num < unit * SP_RANSAC_MINRATE) {
            return calcPose(pose, objs0, objs1);
        }

        int maxit = adaptiveStop(SP_RANSAC_MINEVAL, unit);

        Mem1<Vec3> sobjs0, robjs0;
        Mem1<Vec3> sobjs1, robjs1;

        double maxe = 0.0;
        int it = 0;
        for (it = 0; it < maxit; it++) {
            const int p = it % (num - unit);
            if (p == 0) {
                sobjs0 = shuffle(objs0, it);
                sobjs1 = shuffle(objs1, it);
            }
            robjs0.resize(unit, &sobjs0[p]);
            robjs1.resize(unit, &sobjs1[p]);

            Pose test;
            if (calcPose(test, robjs0, robjs1, 1) == false) continue;

            Mem1<double> errs;
            for (int i = 0; i < num; i++) {
                errs.push(normVec(objs0[i] - test * objs1[i]));
            }
            const double eval = evalErr(errs, thresh);

            if (eval > maxe) {
                //SP_PRINTD("eval %lf\n", eval);
                maxe = eval;
                maxit = adaptiveStop(eval, unit);

                pose = test;
            }
        }
        //SP_PRINTD("RANSAC iteration %d rate %.2lf\n", it, maxe);
        if (maxe < SP_RANSAC_MINEVAL || maxe * num < unit * SP_RANSAC_MINRATE) return false;

        // refine
        {
            Mem1<double> errs;
            for (int i = 0; i < num; i++) {
                errs.push(normVec(objs0[i] - pose * objs1[i]));
            }
            const Mem1<Vec3> dobjs0 = denoise(objs0, errs, thresh);
            const Mem1<Vec3> dobjs1 = denoise(objs1, errs, thresh);

            if (refinePose(pose, dobjs0, dobjs1) == false) return false;
        }

        return true;
    }

    // 2D-3D pose
    SP_CPUFUNC bool calcPoseRANSAC(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs, const double thresh = 5.0) {
        SP_ASSERT(pixs.size() == objs.size());
      
        const int num = pixs.size();
        const int unit = 3;

        if (num < unit) return false;
        if (num < unit * SP_RANSAC_MINRATE) {
            return calcPose(pose, cam, pixs, objs);
        }

        int maxit = adaptiveStop(SP_RANSAC_MINEVAL, unit);

        Mem1<Vec2> spixs, rpixs;
        Mem1<Vec3> sobjs, robjs;

        double maxe = 0.0;
        int it = 0;
        for (it = 0; it < maxit; it++) {
            const int p = it % (num - unit);
            if (p == 0) {
                spixs = shuffle(pixs, it);
                sobjs = shuffle(objs, it);
            }
            rpixs.resize(unit, &spixs[p]);
            robjs.resize(unit, &sobjs[p]);

            Mem1<Pose> tests;
            if (calcPoseP3P(tests, cam, rpixs, robjs) == false) continue;

            for (int i = 0; i < tests.size(); i++) {

                const Mem1<double> errs = calcPrjErr(tests[i], cam, pixs, objs);
                const double eval = evalErr(errs, thresh);

                if (eval > maxe) {
                    //SP_PRINTD("eval %lf\n", eval);
                    maxe = eval;
                    maxit = adaptiveStop(eval, unit);

                    pose = tests[i];
                }
            }
        }
        //SP_PRINTD("RANSAC iteration %d rate %.2lf\n", it, maxe);
        if (maxe < SP_RANSAC_MINEVAL || maxe * num < unit * SP_RANSAC_MINRATE) return false;

        // refine
        {
            const Mem1<double> errs = calcPrjErr(pose, cam, pixs, objs);
            const Mem1<Vec2> dpixs = denoise(pixs, errs, thresh);
            const Mem1<Vec3> dobjs = denoise(objs, errs, thresh);

            if (refinePose(pose, cam, dpixs, dobjs) == false) return false;
        }

        return true;
    }

    // 2D-2D pose (planar object)
    SP_CPUFUNC bool calcPoseRANSAC(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec2> &objs, const double thresh = 5.0) {
        SP_ASSERT(pixs.size() == objs.size());

        const int num = pixs.size();
        const int unit = 4;

        if (num < unit) return false;
        if (num < unit * SP_RANSAC_MINRATE) {
            return calcPose(pose, cam, pixs, objs);
        }

        int maxit = adaptiveStop(SP_RANSAC_MINEVAL, unit);

        Mem1<Vec2> spixs, rpixs;
        Mem1<Vec2> sobjs, robjs;

        double maxe = 0.0;
        int it = 0;
        for (it = 0; it < maxit; it++) {
            const int p = it % (num - unit);
            if (p == 0) {
                spixs = shuffle(pixs, it);
                sobjs = shuffle(objs, it);
            }
            rpixs.resize(unit, &spixs[p]);
            robjs.resize(unit, &sobjs[p]);

            Pose test;
            if (calcPose(test, cam, rpixs, robjs, 1) == false) continue;

            const Mem1<double> errs = calcPrjErr(test, cam, pixs, getVec(objs, 0.0));
            const double eval = evalErr(errs, thresh);

            if (eval > maxe) {
                //SP_PRINTD("eval %lf\n", eval);
                maxe = eval;
                maxit = adaptiveStop(eval, unit);

                pose = test;
            }
        }
        //SP_PRINTD("RANSAC iteration %d rate %.2lf\n", it, maxe);
        if (maxe < SP_RANSAC_MINEVAL || maxe * num < unit * SP_RANSAC_MINRATE) return false;

        // refine
        {
            const Mem1<double> errs = calcPrjErr(pose, cam, pixs, getVec(objs, 0.0));
            const Mem1<Vec2> dpixs = denoise(pixs, errs, thresh);
            const Mem1<Vec2> dobjs = denoise(objs, errs, thresh);

            if (refinePose(pose, cam, dpixs, dobjs) == false) return false;
        }

        return true;
    }

    // 2D-2D pose (stereo camera)
    SP_CPUFUNC bool calcPoseRANSAC(Pose &pose, const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const Mem1<Vec2> &pixs1, const double thresh = 5.0) {
        SP_ASSERT(pixs0.size() == pixs1.size());

        const Mem1<Vec2> npxs0 = invCamD(cam0, pixs0);
        const Mem1<Vec2> npxs1 = invCamD(cam1, pixs1);

        const double nth = thresh / ((cam0.fx + cam0.fy + cam1.fx + cam1.fy) / 4.0);

        Mat E;
        if (calcEMatRANSAC(E, npxs0, npxs1, nth) == false) return false;

        const Mem1<double> errs = errMatType2(E, npxs0, npxs1);

        const Mem1<Vec2> dnpxs0 = denoise(npxs0, errs, nth);
        const Mem1<Vec2> dnpxs1 = denoise(npxs1, errs, nth);
        if (dcmpEMat(pose, E, dnpxs0, dnpxs1) == false) return false;

        const Mem1<Vec2> dpixs0 = denoise(pixs0, errs, nth);
        const Mem1<Vec2> dpixs1 = denoise(pixs1, errs, nth);
        if (refinePose(pose, cam0, dpixs0, cam1, dpixs1) == false) return false;

        return true;
    }


    //--------------------------------------------------------------------------------
    // stereo
    //--------------------------------------------------------------------------------
 
    SP_CPUFUNC double evalStereo(const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const Mem1<Vec2> &pixs1, const double minAngle = 3.0 * SP_PI / 180.0) {

        Pose pose = zeroPose();
        if (calcPoseRANSAC(pose, cam0, pixs0, cam1, pixs1) == false) return 0.0;

        const double dist = normVec(pose.trn);
        if (dist < SP_SMALL) return 0.0;

        pose.trn /= dist;

        Mem1<Vec3> pnts;
        Mem1<bool> mask;
        if (calcPnt3d(pnts, mask, pose, cam0, pixs0, zeroPose(), cam1, pixs1) == false) return 0.0;

        Mem1<double> zlist;
        for (int i = 0; i < pnts.size(); i++) {
            if (mask[i] == false) continue;

            const Pose base = zeroPose();

            const Vec3 vec0 = unitVec(base.trn - pnts[i]);
            const Vec3 vec1 = unitVec(pose.trn - pnts[i]);
            const double angle = acos(dotVec(vec0, vec1));

            if (angle > minAngle) {
                zlist.push(pnts[i].z);
            }
        }
        if (zlist.size() == 0) return 0.0;

        const double pnum = minVal(10.0, log2(zlist.size()));
        const double eval = pnum / 10.0;
        return eval;
    }
}
#endif
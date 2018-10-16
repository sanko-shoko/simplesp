//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_XMAT_H__
#define __SP_XMAT_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // jacob
    //--------------------------------------------------------------------------------

    // s * (vec0_3) = (M_3x3) * (vec1_3) - > (J_2x9) * (M_9) = 0
    SP_GENFUNC void jacobMatType0(double *jacob, const double *vec0, const double *vec1){
        jacob[0 * 9 + 0] = +vec0[2] * vec1[0];
        jacob[0 * 9 + 1] = +vec0[2] * vec1[1];
        jacob[0 * 9 + 2] = +vec0[2] * vec1[2];
        jacob[0 * 9 + 3] = 0.0;
        jacob[0 * 9 + 4] = 0.0;
        jacob[0 * 9 + 5] = 0.0;
        jacob[0 * 9 + 6] = -vec0[0] * vec1[0];
        jacob[0 * 9 + 7] = -vec0[0] * vec1[1];
        jacob[0 * 9 + 8] = -vec0[0] * vec1[2];

        jacob[1 * 9 + 0] = 0.0;
        jacob[1 * 9 + 1] = 0.0;
        jacob[1 * 9 + 2] = 0.0;
        jacob[1 * 9 + 3] = +vec0[2] * vec1[0];
        jacob[1 * 9 + 4] = +vec0[2] * vec1[1];
        jacob[1 * 9 + 5] = +vec0[2] * vec1[2];
        jacob[1 * 9 + 6] = -vec0[1] * vec1[0];
        jacob[1 * 9 + 7] = -vec0[1] * vec1[1];
        jacob[1 * 9 + 8] = -vec0[1] * vec1[2];
    }

    // s * (vec0_3) = (M_3x4) * (vec1_3) - > (J_2x12) * (M_12) = 0
    SP_GENFUNC void jacobMatType1(double *jacob, const double *vec0, const double *vec1) {
        jacob[0 * 12 +  0] = +vec0[2] * vec1[0];
        jacob[0 * 12 +  1] = +vec0[2] * vec1[1];
        jacob[0 * 12 +  2] = +vec0[2] * vec1[2];
        jacob[0 * 12 +  3] = +vec0[2] * 1.0;
        jacob[0 * 12 +  4] = 0.0;
        jacob[0 * 12 +  5] = 0.0;
        jacob[0 * 12 +  6] = 0.0;
        jacob[0 * 12 +  7] = 0.0;
        jacob[0 * 12 +  8] = -vec0[0] * vec1[0];
        jacob[0 * 12 +  9] = -vec0[0] * vec1[1];
        jacob[0 * 12 + 10] = -vec0[0] * vec1[2];
        jacob[0 * 12 + 11] = -vec0[0] * 1.0;

        jacob[1 * 12 +  0] = 0.0;
        jacob[1 * 12 +  1] = 0.0;
        jacob[1 * 12 +  2] = 0.0;
        jacob[1 * 12 +  3] = 0.0;
        jacob[1 * 12 +  4] = +vec0[2] * vec1[0];
        jacob[1 * 12 +  5] = +vec0[2] * vec1[1];
        jacob[1 * 12 +  6] = +vec0[2] * vec1[2];
        jacob[1 * 12 +  7] = +vec0[2] * 1.0;
        jacob[1 * 12 +  8] = -vec0[1] * vec1[0];
        jacob[1 * 12 +  9] = -vec0[1] * vec1[1];
        jacob[1 * 12 + 10] = -vec0[1] * vec1[2];
        jacob[1 * 12 + 11] = -vec0[1] * 1.0;
    }

    // (vec1_3)^T * (M_3x3) * (vec0_3) = 0 -> (J_1x9) * (M_9) = 0
    SP_GENFUNC void jacobMatType2(double *jacob, const double *vec0, const double *vec1) {
        jacob[0] = vec0[0] * vec1[0];
        jacob[1] = vec0[1] * vec1[0];
        jacob[2] = vec0[2] * vec1[0];
        jacob[3] = vec0[0] * vec1[1];
        jacob[4] = vec0[1] * vec1[1];
        jacob[5] = vec0[2] * vec1[1];
        jacob[6] = vec0[0] * vec1[2];
        jacob[7] = vec0[1] * vec1[2];
        jacob[8] = vec0[2] * vec1[2];
    }

    SP_GENFUNC void jacobHMat(double *jacob, const double *img, const double *obj) {
        double pimg[3] = { img[0], img[1], 1.0 };
        double pobj[3] = { obj[0], obj[1], 1.0 };
        jacobMatType0(jacob, pimg, pobj);
    }

    SP_GENFUNC void jacobHMat(double *jacob, const Vec2 &img, const Vec2 &obj){
        double pimg[3] = { img.x, img.y, 1.0 };
        double pobj[3] = { obj.x, obj.y, 1.0 };
        jacobMatType0(jacob, pimg, pobj);
    }

    SP_GENFUNC void jacobPMat(double *jacob, const Vec2 &img, const Vec3 &obj) {
        double pimg[3] = { img.x, img.y, 1.0 };
        double pobj[3] = { obj.x, obj.y, obj.z };
        jacobMatType1(jacob, pimg, pobj);
    }

    SP_GENFUNC void jacobFMat(double *jacob, const double *pix0, const double *pix1) {
        double ppix0[3] = { pix0[0], pix0[1], 1.0 };
        double ppix1[3] = { pix1[0], pix1[1], 1.0 };
        jacobMatType2(jacob, ppix0, ppix1);
    }

    SP_GENFUNC void jacobFMat(double *jacob, const Vec2 &pix0, const Vec2 &pix1){
        double ppix0[3] = { pix0.x, pix0.y, 1.0 };
        double ppix1[3] = { pix1.x, pix1.y, 1.0 };
        jacobMatType2(jacob, ppix0, ppix1);
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
        if (pixs.size() < unit * SP_RANSAC_NUM) {
            return calcHMat(H, pixs, objs);
        }

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
        if (upixs0.size() < unit * SP_RANSAC_NUM) {
            return calcFMat(F, upixs0, upixs1);
        }

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
            if (calcFMat(test, rpixs0, rpixs1, 1) == false) continue;

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



}
#endif
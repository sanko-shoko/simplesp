//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FIT_H__
#define __SP_FIT_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spmodel.h"
#include "spapp/spgeom/spicp.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // fit 2d
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool fit2D(Pose &pose, const Mem2<Byte> &img, const CamParam &cam, const Mem1<Vec3> &objs, const Mem1<Vec3> &drcs, const int searchLng = 10, const int maxit = 10){
        const Rect rect = getRect2(img.dsize);

        for (int it = 0; it < maxit; it++){

            Mem1<Vec3> cobjs;
            Mem1<Vec2> cnrms;
            Mem1<Vec2> detects;
            Mem1<double> vlist(2 * searchLng + 1);

            for (int i = 0; i < objs.size(); i++){
                const Vec3 obj = pose * objs[i];
                const Vec3 drc = pose.rot * drcs[i];
                const Vec2 pix = mulCamD(cam, prjVec(obj));

                double jNpxToDist[2 * 2];
                jacobNpxToDist(jNpxToDist, cam, prjVec(obj));

                const Vec2 drc2 = mulMat(jNpxToDist, 2, 2, getVec(drc.x, drc.y));
                const Vec2 nrm = unitVec(getVec(-drc2.y, drc2.x));

                if (isInRect2(rect, pix.x, pix.y) == false) continue;

                double maxv = 0.0;
                double minv = SP_BYTEMAX;
                for (int l = -searchLng; l <= searchLng; l++){
                    const Vec2 p = pix + nrm * l;
                    const double val = acs2(img, p.x, p.y);
                    vlist[l + searchLng] = val;
                    maxv = maxVal(maxv, val);
                    minv = minVal(minv, val);
                }

                const double thresh = (maxv + minv) / 2.0;

                double minl = searchLng;
                for (int j = 0; j < 2 * searchLng; j++){
                    const double a = vlist[j] - thresh;
                    const double b = vlist[j + 1] - thresh;

                    if (a * b <= 0 && fabs(a - b) > 0){
                        const double l = j + a / (a - b) - searchLng;
                        if (fabs(l) < fabs(minl)){
                            minl = l;
                        }
                    }
                }

                if (minl < searchLng){
                    cobjs.push(objs[i]);
                    cnrms.push(nrm);
                    detects.push(pix + nrm * minl);
                }
            }
            if (cobjs.size() == 0) return false;

            {
                Mat J(cobjs.size(), 6);
                Mat E(cobjs.size(), 1);
                Mem1<double> errs(cobjs.size());

                for (int i = 0; i < cobjs.size(); i++){
                    const Vec3 obj = pose * cobjs[i];
                    const Vec2 pix = mulCamD(cam, prjVec(obj));

                    const Vec2 nrm = cnrms[i];

                    double jacob[2 * 6];
                    jacobPoseToPix(jacob, cam, pose, cobjs[i]);

                    mulMat(&J(i, 0), 1, 6, (const double*)&nrm, 1, 2, jacob, 2, 6);

                    const Vec2 err = detects[i] - pix;
                    E(i, 0) = dotVec(err, nrm);
                    errs[i] = normVec(err);
                }
                
                Mat delta;
                if (solver::solveAX_B(delta, J, E, solver::calcW(errs)) == false) return false;

                pose = updatePose(pose, delta.ptr);
            }
        }

        return true;
    }

    SP_CPUFUNC bool fit2D(Pose &pose, const Mem2<Byte> &img, const CamParam &cam, const Mem1<Vec2> &objs, const Mem1<Vec2> &drcs, const int searchLng = 10, const int maxit = 10){
        return fit2D(pose, img, cam, getVec(objs, 0.0), getVec(drcs, 0.0), searchLng, maxit);
    }

    SP_CPUFUNC bool fit2D(Pose &pose, const Mem2<Byte> &img, const CamParam &cam, const Mem1<PoseModel> &pmodels, const int searchLng = 10, const int maxit = 10) {

        bool ret = false;
        for (int i = 0; i < maxit; i++) {
            const int id = findPoseModel(pmodels, pose);

            Mem1<Vec3> objs, drcs;
            for (int i = 0; i < pmodels[id].edges.size(); i++) {
                objs.push(pmodels[id].edges[i].pos);
                drcs.push(pmodels[id].edges[i].drc);
            }

            ret = fit2D(pose, img, cam, objs, drcs, searchLng, 1);
            if (ret == false) break;
        }

        return ret;
    }


    //--------------------------------------------------------------------------------
    // fit 3d
    //--------------------------------------------------------------------------------

    template<typename VEC>
    SP_CPUFUNC bool fit3D(Pose &pose, const Mem2<VEC> &map, const CamParam &cam, const Mem1<PoseModel> &pmodels, const int maxit = 10) {

        bool ret = false;
        for (int i = 0; i < maxit; i++) {
            const int id = findPoseModel(pmodels, pose);

            ret = calcICP(pose, cam, map, pmodels[id].pnts, 1);
            if (ret == false) break;
        }

        return ret;
    }

}
#endif
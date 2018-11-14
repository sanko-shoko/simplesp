//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_ICP_H__
#define __SP_ICP_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // icp
    //--------------------------------------------------------------------------------

#define SP_ICP_MIN_CRSP 10

    namespace _icp{

        //--------------------------------------------------------------------------------
        // check outlier
        //--------------------------------------------------------------------------------

        SP_CPUFUNC bool checkOutlier(const VecPN3 &vec0, const VecPN3 &vec1){
            return (dotVec(vec0.nrm, vec1.nrm) > 0.5) ? true : false;
        }

        SP_CPUFUNC bool checkOutlier(const VecPN3 &vec0, const Vec3 &vec1){
            return true;
        }

        SP_CPUFUNC bool checkOutlier(const Vec3 &vec0, const VecPN3 &vec1){
            return true;
        }

        SP_CPUFUNC bool checkOutlier(const Vec3 &vec0, const Vec3 &vec1){
            return true;
        }


        //--------------------------------------------------------------------------------
        // position
        //--------------------------------------------------------------------------------

        SP_CPUFUNC Vec3 getPos(const VecPN3 &vec){
            return vec.pos;
        }

        SP_CPUFUNC Vec3 getPos(const Vec3 &vec){
            return vec;
        }


        //--------------------------------------------------------------------------------
        // update direction
        //--------------------------------------------------------------------------------

        SP_CPUFUNC Vec3 getDrc(const VecPN3 &vec0, const VecPN3 &vec1){
            return vec0.nrm;
        }

        SP_CPUFUNC Vec3 getDrc(const VecPN3 &vec0, const Vec3 &vec1){
            return vec0.nrm;
        }

        SP_CPUFUNC Vec3 getDrc(const Vec3 &vec0, const VecPN3 &vec1){
            return vec1.nrm;
        }

        SP_CPUFUNC Vec3 getDrc(const Vec3 &vec0, const Vec3 &vec1){
            return unitVec(vec1 - vec0);
        }


        template <typename TYEP0, typename TYEP1>
        SP_CPUFUNC void crsp(Mem1<TYEP0> &cpnts0, Mem1<TYEP1> &cpnts1, const Pose &pose, const Mem<TYEP0> &pnts0, const Mem<TYEP1> &pnts1) {
            cpnts0.clear();
            cpnts1.clear();

            cpnts0.reserve(pnts1.size());
            cpnts1.reserve(pnts1.size());

            for (int i = 0; i < pnts1.size(); i++){
                const TYEP1 vec = pose * pnts1[i];

                int c = -1;
                double minNorm = SP_INFINITY;
                for (int j = 0; j < pnts0.size(); j++){
                    const double norm = normVec(getPos(pnts0[j]) - getPos(vec));
                    if (norm < minNorm){
                        minNorm = norm;
                        c = j;
                    }
                }
                if (c < 0 || checkOutlier(pnts1[i], pnts0[c]) == false) continue;

                cpnts0.push(pnts0[c]);
                cpnts1.push(pnts1[i]);
            }
        }

        template <typename TYEP0, typename TYEP1>
        SP_CPUFUNC void crsp(Mem1<TYEP0> &cpnts0, Mem1<TYEP1> &cpnts1, const Pose &pose, const CamParam &cam, const Mem<TYEP0> &pnts0, const Mem<TYEP1> &pnts1){
            cpnts0.clear();
            cpnts1.clear();

            cpnts0.reserve(pnts1.size());
            cpnts1.reserve(pnts1.size());

            for (int i = 0; i < pnts1.size(); i++){
                const TYEP1 vec = pose * pnts1[i];

                const Vec2 pix = mulCam(cam, prjVec(getPos(vec)));
                if (isInRect2(pnts0.dsize, pix.x, pix.y) == false) continue;

                const int c = acsid2(pnts0.dsize, round(pix.x), round(pix.y));
                if (checkOutlier(vec, pnts0[c]) == false) continue;

                if (getPos(pnts0[c]).z == 0.0) continue;
                cpnts0.push(pnts0[c]);
                cpnts1.push(pnts1[i]);
            }
        }


        template <typename TYEP0, typename TYEP1>
        SP_CPUFUNC bool update(Pose &pose, const Mem1<TYEP0> &cpnts0, const Mem1<TYEP1> &cpnts1){
            Mat J(1 * cpnts0.size(), 6);
            Mat E(1 * cpnts0.size(), 1);
            Mem1<double> errs(cpnts0.size());

            Vec3 mvec = getVec(0.0, 0.0, 0.0);
            for (int i = 0; i < cpnts1.size(); i++) {
                mvec += getPos(cpnts1[i]);
            }
            mvec /= cpnts1.size();

            Pose mpose = pose * getPose(mvec);

            for (int i = 0; i < cpnts1.size(); i++){
                const TYEP1 vec = pose * cpnts1[i];

                const Vec3 err = getPos(cpnts0[i]) - getPos(vec);
                const Vec3 drc = getDrc(vec, cpnts0[i]);

                double jPoseToPos[3 * 6] = { 0 };
                jacobPoseToPos(jPoseToPos, mpose, getPos(cpnts1[i]) - mvec);

                double jDrc[3] = { drc.x, drc.y, drc.z };
                mulMat(&J(i, 0), 1, 6, jDrc, 1, 3, jPoseToPos, 3, 6);

                E(i, 0) = dotVec(err, drc);
                errs[i] = fabs(E(i, 0));
            }

            Mat delta;
            const bool ret = solveEq(delta, J, E, errs);
            if (ret == true) {
                pose = updatePose(mpose, delta.ptr) * getPose(-mvec);
            }
            return ret;
        }
    }

    // pnts0 <- pnts1 pose
    template<typename TYEP0, typename TYEP1>
    SP_CPUFUNC bool calcICP(Pose &pose, const Mem<TYEP0> &pnts0, const Mem<TYEP1> &pnts1, const int maxit = 10){
        SP_ASSERT(pnts0.dim == 1 && pnts1.dim == 1);

        for (int it = 0; it < maxit; it++){
            Mem1<TYEP0> cpnts0;
            Mem1<TYEP1> cpnts1;    
            
            _icp::crsp(cpnts0, cpnts1, pose, pnts0, pnts1);
            if (cpnts0.size() < SP_ICP_MIN_CRSP) return false;

            if (_icp::update(pose, cpnts0, cpnts1) == false) return false;

        }

        return true;
    }

    // pnts0 <- pnts1 pose
    template<typename TYEP0, typename TYEP1>
    SP_CPUFUNC bool calcICP(Pose &pose, const CamParam &cam, const Mem<TYEP0> &pnts0, const Mem<TYEP1> &pnts1, const int maxit = 10){
        SP_ASSERT(pnts0.dim == 2 && pnts1.dim == 1);

        for (int it = 0; it < maxit; it++){
            Mem1<TYEP0> cpnts0;
            Mem1<TYEP1> cpnts1;

            _icp::crsp(cpnts0, cpnts1, pose, cam, pnts0, pnts1);
            if (cpnts0.size() < SP_ICP_MIN_CRSP) return false;

            if (_icp::update(pose, cpnts0, cpnts1) == false) return false;
        }

        return true;
    }
}
#endif
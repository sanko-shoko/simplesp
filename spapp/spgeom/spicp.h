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
        SP_CPUFUNC void getCrsp(Mem1<TYEP0> &csrc, Mem1<TYEP1> &cref, const Pose &pose, const Mem<TYEP0> &src, const Mem<TYEP1> &ref){
            csrc.clear();
            cref.clear();

            csrc.reserve(src.size());
            cref.reserve(src.size());

            for (int i = 0; i < src.size(); i++){
                const TYEP0 vec = pose * src[i];

                int c = -1;
                double minNorm = SP_INFINITY;
                for (int j = 0; j < ref.size(); j++){
                    const double norm = normVec(getPos(ref[j]) - getPos(vec));
                    if (norm < minNorm){
                        minNorm = norm;
                        c = j;
                    }
                }
                if (c < 0 || checkOutlier(src[i], ref[c]) == false) continue;

                csrc.push(src[i]);
                cref.push(ref[c]);
            }
        }

        template <typename TYEP0, typename TYEP1>
        SP_CPUFUNC void getCrsp(Mem1<TYEP0> &csrc, Mem1<TYEP1> &cref, const Pose &pose, const CamParam &cam, const Mem<TYEP0> &src, const Mem<TYEP1> &ref){
            csrc.clear();
            cref.clear();

            csrc.reserve(src.size());
            cref.reserve(src.size());

            for (int i = 0; i < src.size(); i++){
                const TYEP0 vec = pose * src[i];

                const Vec2 pix = mulCam(cam, prjVec(getPos(vec)));
                if (isInRect2(ref.dsize, pix.x, pix.y) == false) continue;

                const int c = acsid2(ref.dsize, round(pix.x), round(pix.y));
                if (checkOutlier(vec, ref[c]) == false) continue;

                if (getPos(ref[c]).z == 0.0) continue;

                csrc.push(src[i]);
                cref.push(ref[c]);
            }
        }


        template <typename TYEP0, typename TYEP1>
        SP_CPUFUNC bool solve(Mat &result, const Pose &pose, const Mem1<TYEP0> &csrc, const Mem1<TYEP1> &cref){
            Mat J(1 * csrc.size(), 6);
            Mat E(1 * csrc.size(), 1);
            Mem1<double> errs(csrc.size());

            for (int i = 0; i < csrc.size(); i++){
                const TYEP0 vec = pose * csrc[i];

                const Vec3 err = getPos(cref[i]) - getPos(vec);
                const Vec3 drc = getDrc(vec, cref[i]);

                double jPoseToPos[3 * 6] = { 0 };
                jacobPoseToPos(jPoseToPos, pose, getPos(csrc[i]));

                double jDrc[3] = { drc.x, drc.y, drc.z };
                mulMat(&J(i, 0), 1, 6, jDrc, 1, 3, jPoseToPos, 3, 6);

                E(i, 0) = dotVec(err, drc);
                errs[i] = fabs(E(i, 0));
            }

            return solveEq(result, J, E, errs);
        }
    }

    template<typename TYEP0, typename TYEP1>
    SP_CPUFUNC bool calcICP(Pose &pose, const Mem<TYEP0> &src, const Mem<TYEP1> &ref, const int maxit = 10){
        SP_ASSERT(src.dim == 1 && ref.dim == 1);

        Mem1<TYEP0> csrc;
        Mem1<TYEP1> cref;

        for (int it = 0; it < maxit; it++){
            _icp::getCrsp(csrc, cref, pose, src, ref);
            if (csrc.size() < SP_ICP_MIN_CRSP) return false;

            Mat delta;
            if (_icp::solve(delta, pose, csrc, cref) == false) return false;


            pose = updatePose(pose, delta.ptr);
        }

        return true;
    }

    template<typename TYEP0, typename TYEP1>
    SP_CPUFUNC bool calcICP(Pose &pose, const CamParam &cam, const Mem<TYEP0> &src, const Mem<TYEP1> &ref, const int maxit = 10){
        SP_ASSERT(src.dim == 1 && ref.dim == 2);

        Mem1<TYEP0> csrc;
        Mem1<TYEP1> cref;

        for (int it = 0; it < maxit; it++){
            _icp::getCrsp(csrc, cref, pose, cam, src, ref);
            if (csrc.size() < SP_ICP_MIN_CRSP) return false;

            Mat delta;
            if (_icp::solve(delta, pose, csrc, cref) == false) return false;

            pose = updatePose(pose, delta.ptr);
        }

        return true;
    }
}
#endif
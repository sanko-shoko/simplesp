//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VOXEL_H__
#define __SP_VOXEL_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spmodel.h"

namespace sp{

    class Voxel {

    public:
        // voxel size
        int size;

        // voxel unit length
        double unit;

        // voxel cneter;
        Vec3 cent;

        // voxel map
        Mem3<char> vmap;

    public:
        Voxel() {
            size = 0;
            unit = 0.0;
        }

        void init(const int size, const double unit) {
            this->size = size;
            this->unit = unit;

            vmap.resize(size, size, size);
            vmap.zero();
        }

        Vec3 getCenter() const {
            return getVec(vmap.dsize[0] - 1, vmap.dsize[1] - 1, vmap.dsize[2] - 1) * 0.5;
        }
    };

    SP_CPUFUNC bool cnvVoxel(Voxel &voxel, const Mem1<Mesh> &model, const double unit = 1.0) {

        const int size = ceil(getModelRadius(model) / unit) * 2;
        voxel.init(size, unit);

        const CamParam cam = getCamParam(size * 2, size * 2);
        const double distance = sqrt(3.0) * getModelDistance(model, cam);

        const int level = 0;
        for (int i = 0; i < getGeodesicMeshNum(level); i++) {
            const Pose pose = getGeodesicPose(level, i, distance);

            Mem2<VecPN3> map;
            renderVecPN(map, cam, pose, model);

            const Vec3 cent = voxel.getCenter();

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int z = 0; z < voxel.vmap.dsize[2]; z++) {
                for (int y = 0; y < voxel.vmap.dsize[1]; y++) {
                    for (int x = 0; x < voxel.vmap.dsize[0]; x++) {
                        const Vec3 mpos = getVec(x, y, z);
                        const Vec3 cpos = pose * ((mpos - cent) * unit);

                        const Vec2 pix = mulCam(cam, prjVec(cpos));
                        if (isInRect2(map.dsize, pix.x, pix.y) == false) continue;

                        char &val = voxel.vmap(x, y, z);

                        const Vec3 &pos = map(round(pix.x), round(pix.y)).pos;
                        const Vec3 &nrm = map(round(pix.x), round(pix.y)).nrm;

                        if (pos.z == 0.0) {
                            cnvVal(val, maxVal(val - 1, -100));
                        }
                        else {
                            if (dotVec(cpos, nrm) >= 0) continue;

                            const double dist = minVal(pos.z - cpos.z, unit) / unit;

                            if (dist > -1.0 && dist < 0.0) {
                                cnvVal(val, minVal(val + 1, +100));
                            }
                            else if (dist > 0.0){
                                cnvVal(val, maxVal(val - 1, -100));
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < voxel.vmap.size(); i++) {
            voxel.vmap[i] = (voxel.vmap[i] >= 0) ? 1 : 0;
        }

    }

    //--------------------------------------------------------------------------------
    // truncated signed distance function
    //--------------------------------------------------------------------------------

#define SP_TSDF_QUANT 127.0
#define SP_TSDF_MU 5.0
#define SP_TSDF_WMAX 20

    struct TSDF {
        char val;
        char weight;
    };

    SP_CPUFUNC Vec3 getSDFNrm(const Mem3<TSDF> &tsdfmap, const int x, const int y, const int z) {

        const double vx = tsdfmap(x + 1, y, z).val - tsdfmap(x - 1, y, z).val;
        const double vy = tsdfmap(x, y + 1, z).val - tsdfmap(x, y - 1, z).val;
        const double vz = tsdfmap(x, y, z + 1).val - tsdfmap(x, y, z - 1).val;
        return unitVec(getVec(vx, vy, vz));
    }

    SP_CPUFUNC void updateTSDF(Mem3<TSDF> &tsdfmap, const double unit, const CamParam &cam, const Pose &pose, const Mem2<double> &depth) {
        SP_ASSERT(isValid(3, tsdfmap));

        const Vec3 cent = getVec(tsdfmap.dsize[0] - 1, tsdfmap.dsize[1] - 1, tsdfmap.dsize[2] - 1) * 0.5;
        const double mu = SP_TSDF_MU * unit;

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int z = 0; z < tsdfmap.dsize[2]; z++) {
            for (int y = 0; y < tsdfmap.dsize[1]; y++) {
                for (int x = 0; x < tsdfmap.dsize[0]; x++) {
                    const Vec3 mpos = getVec(x, y, z);
                    const Vec3 cpos = pose * ((mpos - cent) * unit);

                    const Vec2 pix = mulCam(cam, prjVec(cpos));
                    if (isInRect2(depth.dsize, pix.x, pix.y) == false) continue;

                    const double d = depth(round(pix.x), round(pix.y));
                    if (d == 0.0) continue;

                    TSDF &tsdf = tsdfmap(x, y, z);

                    const double dist = minVal(d - cpos.z, mu) / mu;
                    if (dist < -1.0) continue;

                    cnvVal(tsdf.val, (tsdf.val * tsdf.weight + SP_TSDF_QUANT * dist) / (tsdf.weight + 1.0));
                    tsdf.weight = minVal(tsdf.weight + 1, SP_TSDF_WMAX);
                }
            }
        }
    }


    SP_CPUFUNC void rayCasting(Mem2<VecPN3> &pnmap, const CamParam &cam, const Pose &pose, const Mem3<TSDF> &tsdfmap, const double unit) {
        pnmap.resize(cam.dsize);
        pnmap.zero();

        const Vec3 cent = getVec(tsdfmap.dsize[0] - 1, tsdfmap.dsize[1] - 1, tsdfmap.dsize[2] - 1) * 0.5;
        const double mu = SP_TSDF_MU * unit;

        const Pose ipose = invPose(pose);
        const double radius = normVec(cent * unit);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < pnmap.dsize[1]; v++) {
            for (int u = 0; u < pnmap.dsize[0]; u++) {
                const Vec3 cvec = prjVec(invCam(cam, getVec(u, v)));

                double maxv;
                double minv;
                {
                    const double a = sqVec(cvec);
                    const double b = -2.0 * dotVec(cvec, pose.trn);
                    const double c = sqVec(pose.trn) - square(radius);

                    const double D = b * b - 4 * a * c;
                    if (D <= 0) continue;

                    maxv = (-b + sqrt(D)) / (2.0 * a);
                    minv = (-b - sqrt(D)) / (2.0 * a);

                    if (maxv <= 0) continue;
                    minv = maxVal(minv, 0.0);
                }

                const Vec3 mvec = ipose.rot * cvec;

                double step = unit;
                double pre = -1.0;

                double detect = minv;
                for (double d = minv; d < maxv; d += step) {
                    const Vec3 mpos = (ipose.trn + mvec * d) / unit + cent;

                    if (isInRect3(tsdfmap.dsize, mpos.x, mpos.y, mpos.z) == false) continue;

                    const TSDF &tsdf = tsdfmap(round(mpos.x), round(mpos.y), round(mpos.z));
                    const double val = tsdf.val / SP_TSDF_QUANT;

                    const double thresh = 0.9;
                    if (step == mu) {
                        if (val < thresh && tsdf.weight > 0) {
                            d -= mu;
                            step = unit;
                        }
                    }
                    else {
                        if (val > thresh || tsdf.weight == 0) {
                            step = mu;
                        }
                        if (val <= 0 && pre > 0) {
                            detect = d + step * val / (pre - val);
                            break;
                        }
                    }
                    pre = val;
                }

                if (detect > minv) {
                    const Vec3 mpos = (ipose.trn + mvec * detect) / unit + cent;
                    const Vec3 mnrm = getSDFNrm(tsdfmap, round(mpos.x), round(mpos.y), round(mpos.z));

                    const Vec3 cpos = cvec * detect;
                    const Vec3 cnrm = pose.rot * mnrm;

                    pnmap(u, v) = getVecPN(cpos, cnrm);
                }
            }
        }
    }

    //    SP_CPUFUNC void rayCastingFast(Mem2<VecPN3> &pnmap, const CamParam &cam, const Pose &pose, const Mem3<TSDF> &tsdfmap, const double unit){
    //        SP_ASSERT(isValid(pnmap, 2));
    //        pnmap.zero();
    //
    //        const Vec3 cent = getVec(tsdfmap.dsize[0] - 1, tsdfmap.dsize[1] - 1, tsdfmap.dsize[2] - 1) * 0.5 * unit;
    //        const double mu = SP_TSDF_MU * unit;
    //
    //        const Pose ipose = invPose(pose);
    //        const double radius = normVec(cent);
    //
    //
    //        Mem2<double> depth(pnmap.dsize);
    //        for (int v = 0; v < depth.dsize[1]; v++){
    //            for (int u = 0; u < depth.dsize[0]; u++){
    //                depth(u, v) = radius * randValUnif() + pose.trn.z;
    //            }
    //        }
    //
    //        Mem2<double> eval(pnmap.dsize);
    //        setElm(eval, 1.0);
    //
    //        for (int it = 0; it < 2; it++){
    //            // eval
    //
    //#if SP_USE_OMP
    //#pragma omp parallel for
    //#endif
    //            for (int v = 0; v < depth.dsize[1]; v++){
    //                for (int u = 0; u < depth.dsize[0]; u++){
    //
    //                    const double delta = eval(u, v) * mu;
    //
    //                    const Vec2 npx = invCam(cam, getVec(u, v));
    //                    const Vec3 cvec = getVec(npx.x, npx.y, 1.0);
    //
    //                    const Vec3 mvec = ipose.rot * cvec;
    //                    const Vec3 mpos = (ipose.trn + mvec * (depth(u, v) + delta) + cent) / unit;
    //                    
    //                    if (isInRect3(tsdfmap.dsize, mpos.x, mpos.y, mpos.z) == false) continue;
    //
    //                    const TSDF &tsdf = tsdfmap(round(mpos.x), round(mpos.y), round(mpos.z));
    //
    //                    if (tsdf.weight == 0) continue;
    //                    const double e = tsdf.val / SP_TSDF_QUANT;
    //
    //                    if (fabs(e) < fabs(eval(u, v))){
    //                        eval(u, v) = e;
    //                        depth(u, v) += delta;
    //                    }
    //                }
    //            }
    //
    //            // propagate
    //            const int prop = (it % 2 == 0) ? +1 : -1;
    //            
    //#if SP_USE_OMP
    //#pragma omp parallel for
    //#endif
    //            for (int v = 0; v < depth.dsize[1]; v++){
    //                for (int iu = 0; iu < depth.dsize[0]; iu++){
    //                    const int u = (prop > 0) ? iu : depth.dsize[0] - iu;
    //                        
    //                    const Vec2 npx = invCam(cam, getVec(u, v));
    //                    const Vec3 cvec = getVec(npx.x, npx.y, 1.0);
    //
    //                    const Vec3 mvec = ipose.rot * cvec;
    //
    //                    const double ce = eval(u, v);
    //                    const double cd = depth(u, v);
    //
    //                    const int x = u + prop;
    //                    const int y = v;
    //                    if (isInRect2(depth.dsize, x, y) == false) continue;
    //
    //                    const double re = eval(x, y);
    //                    const double rd = depth(x, y);
    //
    //                    if (re < 0.5 && rd < cd - mu){
    //                        const Vec3 mpos = (ipose.trn + mvec * rd + cent) / unit;
    //
    //                        if (isInRect3(tsdfmap.dsize, mpos.x, mpos.y, mpos.z) == false) continue;
    //
    //                        const TSDF &tsdf = tsdfmap(round(mpos.x), round(mpos.y), round(mpos.z));
    //
    //                        if (tsdf.weight == 0) continue;
    //
    //                        depth(u, v) = rd;
    //                        eval(u, v) = tsdf.val / SP_TSDF_QUANT;
    //                    }
    //                }
    //            }
    //#if SP_USE_OMP
    //#pragma omp parallel for
    //#endif
    //            for (int u = 0; u < depth.dsize[0]; u++){
    //                for (int iv = 0; iv < depth.dsize[1]; iv++){
    //                    const int v = (prop > 0) ? iv : depth.dsize[1] - iv;
    //
    //                    const Vec2 npx = invCam(cam, getVec(u, v));
    //                    const Vec3 cvec = getVec(npx.x, npx.y, 1.0);
    //
    //                    const Vec3 mvec = ipose.rot * cvec;
    //
    //                    const double ce = eval(u, v);
    //                    const double cd = depth(u, v);
    //
    //                    const int x = u;
    //                    const int y = v + prop;
    //                    if (isInRect2(depth.dsize, x, y) == false) continue;
    //
    //                    const double re = eval(x, y);
    //                    const double rd = depth(x, y);
    //
    //                    if (re < 0.5 && rd < cd - mu){
    //                        const Vec3 mpos = (ipose.trn + mvec * rd + cent) / unit;
    //
    //                        if (isInRect3(tsdfmap.dsize, mpos.x, mpos.y, mpos.z) == false) continue;
    //
    //                        const TSDF &tsdf = tsdfmap(round(mpos.x), round(mpos.y), round(mpos.z));
    //
    //                        if (tsdf.weight == 0) continue;
    //
    //                        depth(u, v) = rd;
    //                        eval(u, v) = tsdf.val / SP_TSDF_QUANT;
    //                    }
    //                }
    //            }
    //
    //        }
    //
    //        for (int v = 0; v < depth.dsize[1]; v++){
    //            for (int u = 0; u < depth.dsize[0]; u++){
    //                if (fabs(eval(u, v)) < 1.0){
    //                    const Vec2 npx = invCam(cam, getVec(u, v));
    //                    const Vec3 cvec = getVec(npx.x, npx.y, 1.0);
    //
    //                    const Vec3 mvec = ipose.rot * cvec;
    //
    //                    const double detect = depth(u, v);
    //                    const Vec3 mpos = (ipose.trn + mvec * detect + cent) / unit;
    //                    const Vec3 mnrm = getSDFNrm(tsdfmap, round(mpos.x), round(mpos.y), round(mpos.z));
    //
    //                    const Vec3 cpos = cvec * detect;
    //                    const Vec3 cnrm = pose.rot * mnrm;
    //
    //                    pnmap(u, v) = getVecPN(cpos, cnrm);
    //                }
    //            }
    //        }
    //    }
}

#endif
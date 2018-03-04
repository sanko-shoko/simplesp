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
        int dsize[3];

        // voxel unit length
        double unit;

        // voxel map
        Mem3<char> vmap;
 
        // weight map
        Mem3<char> wmap;

    public:
        Voxel() {
            dsize[0] = 0;
            dsize[1] = 0;
            dsize[2] = 0;
            unit = 0.0;
        }

        void init(const int size, const double unit) {
            dsize[0] = size;
            dsize[1] = size;
            dsize[2] = size;
            this->unit = unit;

            vmap.resize(dsize);
            wmap.resize(dsize);
            zero();
        }

        void zero() {
            vmap.zero();
            wmap.zero();
        }

        Vec3 center() const {
            return getVec(dsize[0] - 1, dsize[1] - 1, dsize[2] - 1) * 0.5;
        }
    };


    SP_CPUFUNC bool cnvModelToVoxel(Voxel &voxel, const Mem1<Mesh> &model, const double unit = 1.0) {

        const int size = (ceil(getModelRadius(model) / unit) + 1) * 2;
        voxel.init(size, unit);

        const CamParam cam = getCamParam(size * 2, size * 2);
        const double distance = sqrt(3.0) * getModelDistance(model, cam);

        const Vec3 cent = voxel.center();

        const int level = 0;

        for (int i = 0; i < getGeodesicMeshNum(level); i++) {
            const Pose pose = getGeodesicPose(level, i, distance);

            Mem2<VecPN3> map;
            renderVecPN(map, cam, pose, model);

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int z = 0; z < voxel.dsize[2]; z++) {
                for (int y = 0; y < voxel.dsize[1]; y++) {
                    for (int x = 0; x < voxel.dsize[0]; x++) {
                        const Vec3 mpos = getVec(x, y, z);
                        const Vec3 cpos = pose * ((mpos - cent) * unit);

                        const Vec2 pix = mulCam(cam, prjVec(cpos));
                        if (isInRect2(map.dsize, pix.x, pix.y) == false) continue;

                        char &val = voxel.vmap(x, y, z);

                        const Vec3 &pos = map(round(pix.x), round(pix.y)).pos;
                        const Vec3 &nrm = map(round(pix.x), round(pix.y)).nrm;

                        if (pos.z == 0.0) {
                            cnvVal(val, minVal(val + 1, +100));
                        }
                        else {
                            if (dotVec(cpos, nrm) >= 0) continue;

                            const double dist = minVal(pos.z - cpos.z, unit) / unit;

                            if (dist > -1.0 && dist < 0.0) {
                                cnvVal(val, maxVal(val - 1, -100));
                            }
                            else if (dist > 0.0){
                                cnvVal(val, minVal(val + 1, +100));
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < voxel.vmap.size(); i++) {
            voxel.vmap[i] = (voxel.vmap[i] > 0) ? +1 : -1;
        }

    }

    // Marching cubes
    SP_CPUFUNC bool cnvVoxelToModel(Mem1<Mesh> &model, const Voxel &voxel) {

        model.clear();

        typedef MemA<int, 3> Mem3i;
        typedef MemA<int, 8> Mem8i;

        Mem1<Mem1<Mem3i> > orders;
        {
            // vertex orders
            for (int z = 0; z < 2; z++) {
                for (int y = 0; y < 2; y++) {
                    for (int x = 0; x < 2; x++) {
                        const int _x[2] = { x, 1 - x };
                        const int _y[2] = { y, 1 - y };
                        const int _z[2] = { z, 1 - z };
                        const int m = (x + y + z) % 2 ? -1 : +1;
                        for(int i = 0; i < 3; i++){
                            Mem1<Mem3i> order;
                            for (int j = 0; j < 8; j++) {
                                const int ix = (j & (1 << ((6 - i + m * 0) % 3))) ? 1 : 0;
                                const int iy = (j & (1 << ((6 - i + m * 1) % 3))) ? 1 : 0;
                                const int iz = (j & (1 << ((6 - i + m * 2) % 3))) ? 1 : 0;
                                order.push(Mem3i(_x[ix], _y[iy], _z[iz]));
                            }
                            orders.push(order);
                        }
                     }
                }
            }
        }

        Mem1<Mem8i> patterns;
        {
            // 15 patterns
            patterns.push(Mem8i(+1, +1, +1, +1, +1, +1, +1, +1));
            patterns.push(Mem8i(+1, +1, +1, +1, +1, +1, -1, +1));
            patterns.push(Mem8i(+1, +1, +1, +1, +1, +1, -1, -1));
            patterns.push(Mem8i(+1, +1, +1, -1, +1, +1, -1, +1));
            patterns.push(Mem8i(+1, +1, +1, +1, -1, -1, +1, -1));

            patterns.push(Mem8i(+1, +1, +1, +1, -1, -1, -1, -1));
            patterns.push(Mem8i(+1, +1, -1, +1, -1, -1, +1, -1));
            patterns.push(Mem8i(-1, +1, +1, -1, +1, -1, -1, +1));
            patterns.push(Mem8i(-1, +1, +1, +1, -1, -1, -1, +1));
            patterns.push(Mem8i(-1, +1, +1, +1, -1, -1, +1, -1));

            patterns.push(Mem8i(+1, -1, +1, +1, +1, +1, -1, +1));
            patterns.push(Mem8i(+1, -1, +1, +1, +1, +1, -1, -1));
            patterns.push(Mem8i(+1, -1, -1, +1, +1, +1, +1, -1));
            patterns.push(Mem8i(+1, -1, -1, +1, +1, -1, -1, +1));
            patterns.push(Mem8i(+1, -1, +1, +1, -1, -1, -1, +1));
        }

        Mem3<Mem1<Mesh> > map(voxel.dsize[0] - 1, voxel.dsize[1] - 1, voxel.dsize[2] - 1);

        for (int z = 0; z < voxel.dsize[2] - 1; z++) {
            for (int y = 0; y < voxel.dsize[1] - 1; y++) {
                for (int x = 0; x < voxel.dsize[0] - 1; x++) {

                    // order id,  pattern id
                    int oid = 0, pid = 0;

                    int reverse = 0;

                    // matching
                    for (int i = 0; i < orders.size(); i++) {
                        const Mem1<Mem3i> &order = orders[i];

                        for (int j = 0; j < patterns.size(); j++) {
                            const Mem8i &pattern = patterns[j];

                            int score = 0;
                            for (int k = 0; k < 8; k++) {
                                const char &val = voxel.vmap(x + order[k][0], y + order[k][1], z + order[k][2]);
                                
                                if ((val - 0.5) * pattern[k] > 0) {
                                    score++;
                                }
                                else {
                                    score--;
                                }
                            }

                            if (abs(score) == 8) {
                                oid = i;
                                pid = j;
                                reverse = sign(score);
                                goto _match;
                            }
                        }
                    }
                _match:;

                    if (pid == 0) continue;

                    Vec3 p[8];
                    double v[8];
                    for (int i = 0; i < 8; i++) {
                        p[i] = getVec(x, y, z) + getVec(orders[oid][i]);
                        v[i] = fabs(voxel.vmap(round(p[i].x), round(p[i].y), round(p[i].z)));
                    }

                    Mem1<Mesh> tmps;

                    switch (pid) {
                    default:
                        break;
                    case 1:
                    {
                        tmps.push(getMesh(p[6] + p[2], p[6] + p[7], p[6] + p[4]) / 2.0);
                        break;
                    }
                    case 2:
                    {
                        tmps.push(getMesh(p[6] + p[2], p[7] + p[5], p[6] + p[4]) / 2.0);
                        tmps.push(getMesh(p[7] + p[3], p[7] + p[5], p[6] + p[2]) / 2.0);
                        break;
                    }
                    case 3:
                    {
                        tmps.push(getMesh(p[6] + p[2], p[6] + p[7], p[6] + p[4]) / 2.0);
                        tmps.push(getMesh(p[3] + p[1], p[3] + p[7], p[3] + p[2]) / 2.0);
                        break;
                    }
                    case 4:
                    {
                        tmps.push(getMesh(p[4] + p[0], p[7] + p[3], p[5] + p[1]) / 2.0);
                        tmps.push(getMesh(p[4] + p[0], p[4] + p[6], p[7] + p[3]) / 2.0);
                        tmps.push(getMesh(p[7] + p[3], p[4] + p[6], p[7] + p[6]) / 2.0);
                        break;
                    }

                    case 5:
                    {
                        tmps.push(getMesh(p[4] + p[0], p[6] + p[2], p[5] + p[1]) / 2.0);
                        tmps.push(getMesh(p[5] + p[1], p[6] + p[2], p[7] + p[3]) / 2.0);
                        break;
                    }

                    case 6:
                    {
                        tmps.push(getMesh(p[2] + p[0], p[2] + p[3], p[2] + p[6]) / 2.0);
                        tmps.push(getMesh(p[4] + p[0], p[7] + p[3], p[5] + p[1]) / 2.0);
                        tmps.push(getMesh(p[4] + p[0], p[4] + p[6], p[7] + p[3]) / 2.0);
                        tmps.push(getMesh(p[7] + p[3], p[4] + p[6], p[7] + p[6]) / 2.0);
                        break;
                    }
                    case 7:
                    {
                        tmps.push(getMesh(p[0] + p[1], p[0] + p[2], p[0] + p[4]) / 2.0);
                        tmps.push(getMesh(p[3] + p[1], p[3] + p[7], p[3] + p[2]) / 2.0);
                        tmps.push(getMesh(p[5] + p[1], p[5] + p[7], p[5] + p[4]) / 2.0);
                        tmps.push(getMesh(p[6] + p[2], p[6] + p[7], p[6] + p[4]) / 2.0);
                        break;
                    }
                    case 8:
                    {
                        tmps.push(getMesh(p[0] + p[1], p[0] + p[2], p[6] + p[2]) / 2.0);
                        tmps.push(getMesh(p[0] + p[1], p[6] + p[2], p[6] + p[7]) / 2.0);
                        tmps.push(getMesh(p[0] + p[1], p[6] + p[7], p[5] + p[1]) / 2.0);
                        tmps.push(getMesh(p[5] + p[1], p[6] + p[7], p[5] + p[7]) / 2.0);
                        break;
                    }
                    case 9:
                    {
                        tmps.push(getMesh(p[0] + p[1], p[0] + p[2], p[5] + p[1]) / 2.0);
                        tmps.push(getMesh(p[5] + p[1], p[0] + p[2], p[6] + p[7]) / 2.0);
                        tmps.push(getMesh(p[0] + p[2], p[4] + p[6], p[6] + p[7]) / 2.0);
                        tmps.push(getMesh(p[5] + p[1], p[7] + p[6], p[7] + p[3]) / 2.0);
                        break;
                    }
                    case 10:
                    {
                        tmps.push(getMesh(p[6] + p[2], p[6] + p[7], p[6] + p[4]) / 2.0);
                        tmps.push(getMesh(p[1] + p[0], p[1] + p[5], p[1] + p[3]) / 2.0);
                        break;
                    }
                    case 11:
                    {
                        tmps.push(getMesh(p[1] + p[0], p[1] + p[5], p[1] + p[3]) / 2.0);
                        tmps.push(getMesh(p[6] + p[2], p[7] + p[5], p[6] + p[4]) / 2.0);
                        tmps.push(getMesh(p[7] + p[3], p[7] + p[5], p[6] + p[2]) / 2.0);
                        break;
                    }
                    case 12:
                    {
                        tmps.push(getMesh(p[1] + p[0], p[1] + p[5], p[1] + p[3]) / 2.0);
                        tmps.push(getMesh(p[2] + p[0], p[2] + p[3], p[2] + p[6]) / 2.0);
                        tmps.push(getMesh(p[7] + p[3], p[7] + p[5], p[7] + p[6]) / 2.0);
                        break;
                    }
                    case 13:
                    {
                        tmps.push(getMesh(p[1] + p[0], p[5] + p[7], p[1] + p[3]) / 2.0);
                        tmps.push(getMesh(p[1] + p[0], p[5] + p[4], p[5] + p[7]) / 2.0);
                        tmps.push(getMesh(p[2] + p[0], p[2] + p[3], p[6] + p[7]) / 2.0);
                        tmps.push(getMesh(p[2] + p[0], p[6] + p[7], p[6] + p[4]) / 2.0);
                        break;
                    }
                    case 14:
                    {
                        tmps.push(getMesh(p[1] + p[0], p[4] + p[0], p[1] + p[3]) / 2.0);
                        tmps.push(getMesh(p[1] + p[3], p[4] + p[0], p[6] + p[7]) / 2.0);
                        tmps.push(getMesh(p[4] + p[0], p[6] + p[2], p[6] + p[7]) / 2.0);
                        tmps.push(getMesh(p[5] + p[7], p[1] + p[3], p[6] + p[7]) / 2.0);
                        break;
                    }

                    }
                    if (reverse < 0) {
                        for (int i = 0; i < tmps.size(); i++) {
                            swap(tmps[i].pos[1], tmps[i].pos[2]);
                        }
                    }

                    model.push(tmps);
                    map(x, y, z).push(tmps);
                }
            }
        }

        // hole filling
        for (int z = 0; z < voxel.dsize[2] - 1; z++) {
            for (int y = 0; y < voxel.dsize[1] - 1; y++) {
                for (int x = 0; x < voxel.dsize[0] - 1; x++) {
                    for(int n = 0; n < 3; n++){
                        if (n == 0 && x + 1 >= voxel.dsize[0] - 1) continue;
                        if (n == 1 && y + 1 >= voxel.dsize[1] - 1) continue;
                        if (n == 2 && z + 1 >= voxel.dsize[2] - 1) continue;

                        Mem1<Mesh> tmps;
                        tmps.push(map(x, y, z));

                        if (n == 0) tmps.push(map(x + 1, y, z));
                        if (n == 1) tmps.push(map(x, y + 1, z));
                        if (n == 2) tmps.push(map(x, y, z + 1));

                        if (tmps.size() < 4) continue;

                        Mem1<Vec3> vecs;
                        for (int i = 0; i < tmps.size(); i++) {
                            bool check = false;
                            for (int j = 0; j < 3 && check == false; j++) {
                                const Vec3 &a = tmps[i].pos[(j + 0) % 3];
                                const Vec3 &b = tmps[i].pos[(j + 1) % 3];
                                const Vec3 v = b - a;
                                if (n == 0 && cmpVal(a.x, x + 1.0) == true && cmpVal(v.x, 0.0) == true) {
                                    check = true;
                                    vecs.push(a);
                                    vecs.push(b);
                                }
                                if (n == 1 && cmpVal(a.y, y + 1.0) == true && cmpVal(v.y, 0.0) == true) {
                                    check = true;
                                    vecs.push(a);
                                    vecs.push(b);
                                }
                                if (n == 2 && cmpVal(a.z, z + 1.0) == true && cmpVal(v.z, 0.0) == true) {
                                    check = true;
                                    vecs.push(a);
                                    vecs.push(b);
                                }
                            }
                            if (check == false) {
                                tmps.del(i--);
                            }
                        }
                        if (tmps.size() != 4) continue;

                        if (cmpVec(vecs[0] + vecs[1], vecs[4] + vecs[5]) == true) continue;
                        if (cmpVec(vecs[0] + vecs[1], vecs[6] + vecs[7]) == true) continue;
                        if (cmpVec(vecs[2] + vecs[3], vecs[4] + vecs[5]) == true) continue;
                        if (cmpVec(vecs[2] + vecs[3], vecs[6] + vecs[7]) == true) continue;

                        model.push(getMesh(vecs[0], vecs[3], vecs[1]));
                        model.push(getMesh(vecs[1], vecs[3], vecs[2]));
                    }
                }
            }
        }

        model = (model - voxel.center()) * voxel.unit;
        return true;
    }

    //--------------------------------------------------------------------------------
    // truncated signed distance function
    //--------------------------------------------------------------------------------

#define SP_TSDF_QUANT 127.0
#define SP_TSDF_MU 5.0
#define SP_TSDF_WMAX 20

    SP_CPUFUNC Vec3 getSDFNrm(const Mem3<char> &tsdfmap, const int x, const int y, const int z) {

        const double vx = tsdfmap(x + 1, y, z) - tsdfmap(x - 1, y, z);
        const double vy = tsdfmap(x, y + 1, z) - tsdfmap(x, y - 1, z);
        const double vz = tsdfmap(x, y, z + 1) - tsdfmap(x, y, z - 1);
        return unitVec(getVec(vx, vy, vz));
    }

    SP_CPUFUNC void updateTSDF(Voxel &tsdf, const CamParam &cam, const Pose &pose, const Mem2<double> &depth) {

        const Vec3 cent = tsdf.center();
        const double mu = SP_TSDF_MU * tsdf.unit;

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int z = 0; z < tsdf.dsize[2]; z++) {
            for (int y = 0; y < tsdf.dsize[1]; y++) {
                for (int x = 0; x < tsdf.dsize[0]; x++) {
                    const Vec3 mpos = getVec(x, y, z);
                    const Vec3 cpos = pose * ((mpos - cent) * tsdf.unit);

                    const Vec2 pix = mulCam(cam, prjVec(cpos));
                    if (isInRect2(depth.dsize, pix.x, pix.y) == false) continue;

                    const double d = depth(round(pix.x), round(pix.y));
                    if (d == 0.0) continue;

                    char &val = tsdf.vmap(x, y, z);
                    char &weight = tsdf.wmap(x, y, z);

                    const double dist = minVal(d - cpos.z, mu) / mu;
                    if (dist < -1.0) continue;

                    cnvVal(val, (val * weight + SP_TSDF_QUANT * dist) / (weight + 1.0));
                    weight = minVal(weight + 1, SP_TSDF_WMAX);
                }
            }
        }
    }


    SP_CPUFUNC void rayCasting(Mem2<VecPN3> &map, const CamParam &cam, const Pose &pose, const Voxel &tsdf) {
       
        map.resize(cam.dsize);
        map.zero();

        const Vec3 cent = tsdf.center();
        const double mu = SP_TSDF_MU * tsdf.unit;

        const Pose ipose = invPose(pose);
        const double radius = normVec(cent * tsdf.unit);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < map.dsize[1]; v++) {
            for (int u = 0; u < map.dsize[0]; u++) {
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

                double step = tsdf.unit;
                double pre = -1.0;

                double detect = minv;
                for (double d = minv; d < maxv; d += step) {
                    const Vec3 mpos = (ipose.trn + mvec * d) / tsdf.unit + cent;

                    if (isInRect3(tsdf.dsize, mpos.x, mpos.y, mpos.z) == false) continue;

                    const double val = tsdf.vmap(round(mpos.x), round(mpos.y), round(mpos.z)) / SP_TSDF_QUANT;
                    const char &weight = tsdf.wmap(round(mpos.x), round(mpos.y), round(mpos.z));

                    const double thresh = 0.9;
                    if (step == mu) {
                        if (val < thresh && weight > 0) {
                            d -= mu;
                            step = tsdf.unit;
                        }
                    }
                    else {
                        if (val > thresh || weight == 0) {
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
                    const Vec3 mpos = (ipose.trn + mvec * detect) / tsdf.unit + cent;
                    const Vec3 mnrm = getSDFNrm(tsdf.vmap, round(mpos.x), round(mpos.y), round(mpos.z));

                    const Vec3 cpos = cvec * detect;
                    const Vec3 cnrm = pose.rot * mnrm;

                    map(u, v) = getVecPN(cpos, cnrm);
                }
            }
        }
    }

//    SP_CPUFUNC void rayCastingFast(Mem2<VecPN3> &map, const CamParam &cam, const Pose &pose, const Voxel &tsdf) {
//  
//        map.resize(cam.dsize);
//        map.zero();
//
//        const Vec3 cent = tsdf.center();
//        const double mu = SP_TSDF_MU * tsdf.unit;
//
//        const Pose ipose = invPose(pose);
//        const double radius = normVec(cent * tsdf.unit);
//
//        Mem2<double> depth(cam.dsize);
//        for (int v = 0; v < depth.dsize[1]; v++) {
//            for (int u = 0; u < depth.dsize[0]; u++) {
//                depth(u, v) = radius * randValUnif() + pose.trn.z;
//            }
//        }
//
//        Mem2<double> eval(map.dsize);
//        setElm(eval, 1.0);
//
//        for (int it = 0; it < 100; it++) {
//            // eval
//
//#if SP_USE_OMP
//#pragma omp parallel for
//#endif
//            for (int v = 0; v < depth.dsize[1]; v++) {
//                for (int u = 0; u < depth.dsize[0]; u++) {
//
//                    const double delta = eval(u, v) * mu;
//
//                    const Vec2 npx = invCam(cam, getVec(u, v));
//                    const Vec3 cvec = getVec(npx.x, npx.y, 1.0);
//
//                    const Vec3 mvec = ipose.rot * cvec;
//                    const Vec3 mpos = (ipose.trn + mvec * (depth(u, v) + delta)) / tsdf.unit + cent;
//
//                    if (isInRect3(tsdf.dsize, mpos.x, mpos.y, mpos.z) == false) continue;
//
//                    const char &val = tsdf.vmap(round(mpos.x), round(mpos.y), round(mpos.z));
//                    const char &weight = tsdf.wmap(round(mpos.x), round(mpos.y), round(mpos.z));
//
//                    if (weight == 0) {
//                        depth(u, v) -= delta;
//                        continue;
//                    }
//                    const double e = val / SP_TSDF_QUANT;
//
//                    if (fabs(e) < fabs(eval(u, v))) {
//                        eval(u, v) = e;
//                        depth(u, v) += delta;
//                    }
//                }
//            }
//            continue;
//            // propagate
//            const int prop = (it % 2 == 0) ? +1 : -1;
//
//#if SP_USE_OMP
//#pragma omp parallel for
//#endif
//            for (int v = 0; v < depth.dsize[1]; v++) {
//                for (int iu = 0; iu < depth.dsize[0]; iu++) {
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
//                    if (re < 0.5 && rd < cd - mu) {
//                        const Vec3 mpos = (ipose.trn + mvec * rd + cent) / tsdf.unit;
//
//                        if (isInRect3(tsdf.dsize, mpos.x, mpos.y, mpos.z) == false) continue;
//
//                        const char &val = tsdf.vmap(round(mpos.x), round(mpos.y), round(mpos.z));
//                        const char &weight = tsdf.wmap(round(mpos.x), round(mpos.y), round(mpos.z));
//
//                        if (weight == 0) continue;
//
//                        depth(u, v) = rd;
//                        eval(u, v) = val / SP_TSDF_QUANT;
//                    }
//                }
//            }
//#if SP_USE_OMP
//#pragma omp parallel for
//#endif
//            for (int u = 0; u < depth.dsize[0]; u++) {
//                for (int iv = 0; iv < depth.dsize[1]; iv++) {
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
//                    if (re < 0.5 && rd < cd - mu) {
//                        const Vec3 mpos = (ipose.trn + mvec * rd + cent) / tsdf.unit;
//
//                        if (isInRect3(tsdf.dsize, mpos.x, mpos.y, mpos.z) == false) continue;
//
//                        const char &val = tsdf.vmap(round(mpos.x), round(mpos.y), round(mpos.z));
//                        const char &weight = tsdf.wmap(round(mpos.x), round(mpos.y), round(mpos.z));
//
//                        if (weight == 0) continue;
//
//                        depth(u, v) = rd;
//                        eval(u, v) = val / SP_TSDF_QUANT;
//                    }
//                }
//            }
//
//        }
//
//        for (int v = 0; v < depth.dsize[1]; v++) {
//            for (int u = 0; u < depth.dsize[0]; u++) {
//                if (fabs(eval(u, v)) < 1.0) {
//                    const Vec2 npx = invCam(cam, getVec(u, v));
//                    const Vec3 cvec = getVec(npx.x, npx.y, 1.0);
//
//                    const Vec3 mvec = ipose.rot * cvec;
//
//                    const double detect = depth(u, v);
//                    const Vec3 mpos = (ipose.trn + mvec * detect + cent) / tsdf.unit;
//                    const Vec3 mnrm = getSDFNrm(tsdf.vmap, round(mpos.x), round(mpos.y), round(mpos.z));
//
//                    const Vec3 cpos = cvec * detect;
//                    const Vec3 cnrm = pose.rot * mnrm;
//
//                    map(u, v) = getVecPN(cpos, cnrm);
//                }
//            }
//        }
//    }
}

#endif
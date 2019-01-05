//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VOXEL_H__
#define __SP_VOXEL_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spmodel.h"

namespace sp {

#define SP_VOXEL_VMAX 127
#define SP_VOXEL_WMAX 20
#define SP_VOXEL_NULL -128

    class Voxel {

    public:
        // voxel size
        int dsize[3];

        // voxel unit length
        double unit;

        // value map
        Mem3<char> vmap;

        // weight map
        Mem3<char> wmap;

        // color map
        Mem3<Col4> cmap;

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
            cmap.resize(dsize);

            zero();
        }

        void zero() {
            setElm(vmap, SP_VOXEL_NULL);
            wmap.zero();
            cmap.zero();
        }

        void update(const int x, const int y, const int z, const double src) {
            if (src > +1.0) return;

            char &val = vmap(x, y, z);
            char &wei = wmap(x, y, z);

            cnvVal(val, (val * wei + SP_VOXEL_VMAX * src) / (wei + 1.0));
            wei = minVal(wei + 1, SP_VOXEL_WMAX);
        }

        char getv(const int x, const int y, const int z) const {
            char val = -1;

            if (inRect3(dsize, x, y, z) == true) {
                val = vmap(x, y, z);
            }
            return val;
        }

        char getw(const int x, const int y, const int z) const {
            char wei = 0;

            if (inRect3(dsize, x, y, z) == true) {
                wei = wmap(x, y, z);
            }
            return wei;
        }

        Vec3 getn(const int x, const int y, const int z) const {
            const double vx = vmap(x + 1, y, z) - vmap(x - 1, y, z);
            const double vy = vmap(x, y + 1, z) - vmap(x, y - 1, z);
            const double vz = vmap(x, y, z + 1) - vmap(x, y, z - 1);
            return unitVec(getVec(-vx, -vy, -vz));
        }

        Vec3 center() const {
            return getVec(dsize[0] - 1, dsize[1] - 1, dsize[2] - 1) * 0.5;
        }
    };


    SP_CPUFUNC bool cnvModelToVoxel(Voxel &voxel, const Mem1<Mesh3> &model, const double unit = 1.0) {

        const int size = (ceil(getModelRadius(model) / unit) + 2) * 2;
        SP_PRINTD("voxel size %d\n", size);

        voxel.init(size, unit);
        setElm(voxel.vmap, +1);

        const double step = sqrt(3.0) * unit;

        const CamParam cam = getCamParam(size * 2, size * 2);
        const double distance = sqrt(3.0) * getModelDistance(model, cam);

        const Vec3 cent = voxel.center();

        const int level = 0;

        for (int i = 0; i < getGeodesicMeshNum(level); i++) {
            const Pose pose = getGeodesicPose(level, i, distance);

            Mem2<VecPN3> pnmap;
            renderVecPN(pnmap, cam, pose, model);

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int z = 0; z < voxel.dsize[2]; z++) {
                for (int y = 0; y < voxel.dsize[1]; y++) {
                    for (int x = 0; x < voxel.dsize[0]; x++) {
                        const Vec3 mpos = getVec(x, y, z);
                        const Vec3 cpos = pose * ((mpos - cent) * unit);

                        const Vec2 pix = mulCam(cam, prjVec(cpos));
                        if (inRect2(pnmap.dsize, pix.x, pix.y) == false) continue;

                        const Vec3 &pos = pnmap(round(pix.x), round(pix.y)).pos;
                        const Vec3 &nrm = pnmap(round(pix.x), round(pix.y)).nrm;

                        if (pos.z == 0.0) {
                            voxel.update(x, y, z, -0.1);
                        }
                        else {
                            if (dotVec(cpos, nrm) >= 0) continue;

                            const double dist = maxVal(cpos.z - pos.z, -step) / step;
                            voxel.update(x, y, z, dist);
                        }
                    }
                }
            }
        }

        for (int i = 0; i < voxel.vmap.size(); i++) {
            voxel.vmap[i] = (voxel.vmap[i] >= 0) ? +1 : -1;
        }
        return true;
    }

    // Marching cubes
    SP_CPUFUNC bool cnvVoxelToModel(Mem1<Mesh3> &model, const Voxel &voxel, const Rect *prect = NULL) {

        model.clear();

        typedef MemA<int, 3> Mem3i;
        typedef MemA<int, 8> Mem8i;

        Mem1<Mem1<Mem3i> > orders;

        // vertex orders
        for (int z = 0; z < 2; z++) {
            for (int y = 0; y < 2; y++) {
                for (int x = 0; x < 2; x++) {
                    const int m = (x + y + z) % 2 ? -1 : +1;

                    for (int i = 0; i < 3; i++) {
                        Mem1<Mem3i> order;
                        for (int j = 0; j < 8; j++) {
                            const int ix = (j & (1 << ((6 - i + m * 0) % 3))) ? 1 - x : x;
                            const int iy = (j & (1 << ((6 - i + m * 1) % 3))) ? 1 - y : y;
                            const int iz = (j & (1 << ((6 - i + m * 2) % 3))) ? 1 - z : z;
                            order.push(Mem3i(ix, iy, iz));
                        }
                        orders.push(order);
                    }
                }
            }
        }

        Mem1<Mem8i> patterns;
        Mem1<int> pcnts;
        {
            // 15 patterns
            patterns.push(Mem8i(+1, +1, +1, +1, +1, +1, +1, +1)); pcnts.push(0);
            patterns.push(Mem8i(+1, +1, +1, +1, +1, +1, -1, +1)); pcnts.push(1);
            patterns.push(Mem8i(+1, +1, +1, +1, +1, +1, -1, -1)); pcnts.push(2);
            patterns.push(Mem8i(+1, +1, +1, -1, +1, +1, -1, +1)); pcnts.push(2);
            patterns.push(Mem8i(+1, +1, +1, +1, -1, -1, +1, -1)); pcnts.push(3);

            patterns.push(Mem8i(+1, +1, +1, +1, -1, -1, -1, -1)); pcnts.push(4);
            patterns.push(Mem8i(+1, +1, -1, +1, -1, -1, +1, -1)); pcnts.push(4);
            patterns.push(Mem8i(-1, +1, +1, -1, +1, -1, -1, +1)); pcnts.push(4);
            patterns.push(Mem8i(-1, +1, +1, +1, -1, -1, -1, +1)); pcnts.push(4);
            patterns.push(Mem8i(-1, +1, +1, +1, -1, -1, +1, -1)); pcnts.push(4);

            patterns.push(Mem8i(+1, -1, +1, +1, +1, +1, -1, +1)); pcnts.push(2);
            patterns.push(Mem8i(+1, -1, +1, +1, +1, +1, -1, -1)); pcnts.push(3);
            patterns.push(Mem8i(+1, -1, -1, +1, +1, +1, +1, -1)); pcnts.push(3);
            patterns.push(Mem8i(+1, -1, -1, +1, +1, -1, -1, +1)); pcnts.push(4);
            patterns.push(Mem8i(+1, -1, +1, +1, -1, -1, -1, +1)); pcnts.push(4);
        }

        Rect crect = (prect == NULL) ? getRect3(voxel.dsize) : andRect(getRect3(voxel.dsize), *prect);
        Rect mrect = crect + 1;

        for (int i = 0; i < 3; i++) {
            crect.dsize[i] -= 1;
            mrect.dsize[i] -= 1;
        }

        Mem3<Mem1<Mesh3> > map(mrect.dsize);

        for (int z = mrect.dbase[2]; z < mrect.dbase[2] + mrect.dsize[2]; z++) {
            for (int y = mrect.dbase[1]; y < mrect.dbase[1] + mrect.dsize[1]; y++) {
                for (int x = mrect.dbase[0]; x < mrect.dbase[0] + mrect.dsize[0]; x++) {

                    // pattern id
                    int pid = -1;
                    
                    Vec3 p[8];
                    char v[8];

                    int reverse = 0;

                    {
                        Mem3<char> vmap(2, 2, 2);
                        for (int zz = 0; zz < 2; zz++) {
                            for (int yy = 0; yy < 2; yy++) {
                                for (int xx = 0; xx < 2; xx++) {
                                    vmap(xx, yy, zz) = voxel.getv(x + xx, y + yy, z + zz);
                                }
                            }
                        }

                        int pcnt = 0;
                        for (int i = 0; i < vmap.size(); i++) {
                            if (vmap[i] >= 0) pcnt++;
                        }
                        pcnt = minVal(pcnt, 8 - pcnt);

                        // matching
                        for (int i = 0; i < patterns.size(); i++) {
                            if (pcnt != pcnts[i]) continue;
                            const Mem8i &pattern = patterns[i];

                            for (int j = 0; j < orders.size(); j++) {
                                const Mem1<Mem3i> &order = orders[j];

                                int score = 0;
                                for (int k = 0; k < 8; k++) {
                                    p[k] = getVec(x, y, z) + getVec(order[k]);
                                    v[k] = vmap(order[k][0], order[k][1], order[k][2]);

                                    score += ((v[k] + 0.5) * pattern[k] > 0) ? +1 : -1;
                                }

                                if (abs(score) == 8) {
                                    pid = i;
                                    reverse = sign(score);
                                    goto _match;
                                }
                            }
                        }

                    _match:;
                    }

                    if (pid < 1) continue;

                    Mem1<Mesh3> tmps;

                    auto m = [p, v](const int i, const int j)-> Vec3 {
                        return (abs(v[j]) * p[i] + abs(v[i]) * p[j]) / (abs(v[i]) + abs(v[j]));
                    };

                    switch (pid) {
                    default:
                        break;
                    case 1:
                    {
                        tmps.push(getMesh(m(6, 2), m(6, 4), m(6, 7)));
                        break;
                    }
                    case 2:
                    {
                        tmps.push(getMesh(m(6, 2), m(6, 4), m(7, 5)));
                        tmps.push(getMesh(m(7, 3), m(6, 2), m(7, 5)));
                        break;
                    }
                    case 3:
                    {
                        tmps.push(getMesh(m(6, 2), m(6, 4), m(6, 7)));
                        tmps.push(getMesh(m(3, 1), m(3, 2), m(3, 7)));
                        break;
                    }
                    case 4:
                    {
                        tmps.push(getMesh(m(4, 0), m(5, 1), m(7, 3)));
                        tmps.push(getMesh(m(4, 0), m(7, 3), m(4, 6)));
                        tmps.push(getMesh(m(7, 3), m(7, 6), m(4, 6)));
                        break;
                    }

                    case 5:
                    {
                        tmps.push(getMesh(m(4, 0), m(5, 1), m(6, 2)));
                        tmps.push(getMesh(m(5, 1), m(7, 3), m(6, 2)));
                        break;
                    }

                    case 6:
                    {
                        tmps.push(getMesh(m(2, 0), m(2, 6), m(2, 3)));
                        tmps.push(getMesh(m(4, 0), m(5, 1), m(7, 3)));
                        tmps.push(getMesh(m(4, 0), m(7, 3), m(4, 6)));
                        tmps.push(getMesh(m(7, 3), m(7, 6), m(4, 6)));
                        break;
                    }
                    case 7:
                    {
                        tmps.push(getMesh(m(0, 1), m(0, 4), m(0, 2)));
                        tmps.push(getMesh(m(3, 1), m(3, 2), m(3, 7)));
                        tmps.push(getMesh(m(5, 1), m(5, 7), m(5, 4)));
                        tmps.push(getMesh(m(6, 2), m(6, 4), m(6, 7)));
                        break;
                    }
                    case 8:
                    {
                        tmps.push(getMesh(m(0, 1), m(6, 2), m(0, 2)));
                        tmps.push(getMesh(m(0, 1), m(6, 7), m(6, 2)));
                        tmps.push(getMesh(m(0, 1), m(5, 1), m(6, 7)));
                        tmps.push(getMesh(m(5, 1), m(5, 7), m(6, 7)));
                        break;
                    }
                    case 9:
                    {
                        tmps.push(getMesh(m(0, 1), m(5, 1), m(0, 2)));
                        tmps.push(getMesh(m(5, 1), m(6, 7), m(0, 2)));
                        tmps.push(getMesh(m(0, 2), m(6, 7), m(4, 6)));
                        tmps.push(getMesh(m(5, 1), m(7, 3), m(7, 6)));
                        break;
                    }
                    case 10:
                    {
                        tmps.push(getMesh(m(6, 2), m(6, 4), m(6, 7)));
                        tmps.push(getMesh(m(1, 0), m(1, 3), m(1, 5)));
                        break;
                    }
                    case 11:
                    {
                        tmps.push(getMesh(m(1, 0), m(1, 3), m(1, 5)));
                        tmps.push(getMesh(m(6, 2), m(6, 4), m(7, 5)));
                        tmps.push(getMesh(m(7, 3), m(6, 2), m(7, 5)));
                        break;
                    }
                    case 12:
                    {
                        tmps.push(getMesh(m(1, 0), m(1, 3), m(1, 5)));
                        tmps.push(getMesh(m(2, 0), m(2, 6), m(2, 3)));
                        tmps.push(getMesh(m(7, 3), m(7, 6), m(7, 5)));
                        break;
                    }
                    case 13:
                    {
                        tmps.push(getMesh(m(1, 0), m(1, 3), m(5, 7)));
                        tmps.push(getMesh(m(1, 0), m(5, 7), m(5, 4)));
                        tmps.push(getMesh(m(2, 0), m(6, 7), m(2, 3)));
                        tmps.push(getMesh(m(2, 0), m(6, 4), m(6, 7)));
                        break;
                    }
                    case 14:
                    {
                        tmps.push(getMesh(m(1, 0), m(1, 3), m(4, 0)));
                        tmps.push(getMesh(m(1, 3), m(6, 7), m(4, 0)));
                        tmps.push(getMesh(m(4, 0), m(6, 7), m(6, 2)));
                        tmps.push(getMesh(m(5, 7), m(6, 7), m(1, 3)));
                        break;
                    }

                    }

                    if (reverse < 0) {
                        for (int i = 0; i < tmps.size(); i++) {
                            swap(tmps[i].pos[1], tmps[i].pos[2]);
                        }
                    }

                    model.push(tmps);
                    map(x - mrect.dbase[0], y - mrect.dbase[1], z - mrect.dbase[2]).push(tmps);
                }
            }
        }

        // hole filling
        for (int z = mrect.dbase[2]; z < mrect.dbase[2] + mrect.dsize[2]; z++) {
            for (int y = mrect.dbase[1]; y < mrect.dbase[1] + mrect.dsize[1]; y++) {
                for (int x = mrect.dbase[0]; x < mrect.dbase[0] + mrect.dsize[0]; x++) {
                    for (int n = 0; n < 3; n++) {

                        if (n == 0 && x + 1 >= voxel.dsize[0]) continue;
                        if (n == 1 && y + 1 >= voxel.dsize[1]) continue;
                        if (n == 2 && z + 1 >= voxel.dsize[2]) continue;

                        Mem1<Mesh3> tmps;
                        tmps.push(map(x - mrect.dbase[0], y - mrect.dbase[1], z - mrect.dbase[2]));

                        if (n == 0) tmps.push(map(x - mrect.dbase[0] + 1, y - mrect.dbase[1], z - mrect.dbase[2]));
                        if (n == 1) tmps.push(map(x - mrect.dbase[0], y - mrect.dbase[1] + 1, z - mrect.dbase[2]));
                        if (n == 2) tmps.push(map(x - mrect.dbase[0], y - mrect.dbase[1], z - mrect.dbase[2] + 1));

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

    // voxel range mask
    SP_CPUFUNC void calcVoxelMask(Mem2<Vec2> &rmsk, const CamParam &cam, const Pose &pose, const Voxel &voxel) {

        rmsk.resize(cam.dsize);

        const double radius = (voxel.dsize[0] - 1) * voxel.unit * 0.5;

        struct VoxelPlane {
            Vec3 pos, axis[3];
        };
        Mem1<VoxelPlane> vps;
        {
            const Vec3 _axis[3] = { getVec(1.0, 0.0, 0.0), getVec(0.0, 1.0, 0.0), getVec(0.0, 0.0, 1.0) };
            
            for (int i = 0; i < 3; i++) {
                for (int s = -1; s <= +1; s += 2) {
                    VoxelPlane vp;
                    vp.pos = pose * (_axis[(i + 0) % 3] * s * radius);
                    for (int j = 0; j < 3; j++) {
                        vp.axis[j] = pose.rot * (_axis[(i + j + 1) % 3] * s);
                    }
                    vps.push(vp);
                }
            }
        }

        for (int v = 0; v < rmsk.dsize[1]; v++) {
            for (int u = 0; u < rmsk.dsize[0]; u++) {
                Vec2 &range = rmsk(u, v);
                range = getVec(0.0, SP_INFINITY);

                const Vec3 vec = prjVec(invCam(cam, getVec(u, v)));
                for (int i = 0; i < vps.size(); i++) {
                    const Vec3 &pos = vps[i].pos;
                    const Vec3 &X = vps[i].axis[0];
                    const Vec3 &Y = vps[i].axis[1];
                    const Vec3 &Z = vps[i].axis[2];

                    const double n = dotVec(vec, Z);
                    if (fabs(n) < SP_SMALL) continue;

                    const Vec3 crs = vec * (dotVec(pos, Z) / n);
                    if (crs.z <= SP_SMALL) continue;

                    const double x = dotVec(crs - pos, X);
                    const double y = dotVec(crs - pos, Y);

                    if (n < 0) {
                        if (maxVal(fabs(x), fabs(y)) > radius * 1.00) continue;
                        range.x = maxVal(range.x, crs.z);
                    }
                    else {
                        if (maxVal(fabs(x), fabs(y)) > radius * 1.01) continue;
                        range.y = minVal(range.y, crs.z);
                    }
                }
   
            }
        }

    }


    //--------------------------------------------------------------------------------
    // visual hull
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool visualHull(Voxel &voxel, const Mem1<Mem2<Byte> > &imgs, const Mem1<CamParam> &cams, const Mem1<Pose> &poses, const double unit = 1.0) {

        double meanDist = 0.0;
        {
            for (int i = 0; i < poses.size(); i++) {
                meanDist += poses[i].trn.z;
            }
            meanDist /= poses.size();
        }

        const int size = static_cast<int>(meanDist / 2.0 / unit);
        voxel.init(size, unit);
        setElm(voxel.vmap, +1);

        const Vec3 cent = voxel.center();

        for (int i = 0; i < imgs.size(); i++) {
            const Mem2<Byte> &img = imgs[i];
            const CamParam &cam = cams[i];
            const Pose &pose = poses[i];


#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int z = 0; z < voxel.dsize[2]; z++) {
                for (int y = 0; y < voxel.dsize[1]; y++) {
                    for (int x = 0; x < voxel.dsize[0]; x++) {
                        const Vec3 mpos = getVec(x, y, z);
                        const Vec3 cpos = pose * ((mpos - cent) * unit);

                        const Vec2 pix = mulCam(cam, prjVec(cpos));
                        if (inRect2(img.dsize, pix.x, pix.y) == false) continue;

                        const Byte &val = img(round(pix.x), round(pix.y));

                        if (val == 0) {
                            voxel.update(x, y, z, -1.0);
                        }
                    }
                }
            }
        }

        for (int i = 0; i < voxel.vmap.size(); i++) {
            voxel.vmap[i] = (voxel.vmap[i] > 0) ? +1 : -1;
        }
        return true;
    }

    //--------------------------------------------------------------------------------
    // truncated signed distance function
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void updateTSDF(Voxel &voxel, const CamParam &cam, const Pose &pose, const Mem2<double> &depth, const double mu = 5.0) {

        const Vec3 cent = voxel.center();
        const double step = mu * voxel.unit;

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int z = 0; z < voxel.dsize[2]; z++) {
            for (int y = 0; y < voxel.dsize[1]; y++) {
                for (int x = 0; x < voxel.dsize[0]; x++) {
                    const Vec3 mpos = getVec(x, y, z);
                    const Vec3 cpos = pose * ((mpos - cent) * voxel.unit);

                    const Vec2 pix = mulCam(cam, prjVec(cpos));
                    if (inRect2(depth.dsize, pix.x, pix.y) == false) continue;

                    const double d = depth(round(pix.x), round(pix.y));
                    if (d == 0.0) continue;

                    const double dist = maxVal(cpos.z - d, -step) / step;
                    voxel.update(x, y, z, dist);
                }
            }
        }
    }

    SP_CPUFUNC void rayCasting(Mem2<VecPN3> &map, const CamParam &cam, const Pose &pose, const Voxel &voxel, const double mu = 5.0) {

        map.resize(cam.dsize);
        map.zero();

        const Vec3 cent = voxel.center();
        const Pose ipose = invPose(pose);

        Mem2<Vec2> rmsk;
        calcVoxelMask(rmsk, cam, pose, voxel);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < map.dsize[1]; v++) {
            for (int u = 0; u < map.dsize[0]; u++) {

                const double minv = rmsk(u, v).x;
                const double maxv = rmsk(u, v).y;
                if (minv <= SP_SMALL) continue;

                const Vec3 cvec = prjVec(invCam(cam, getVec(u, v)));
                const Vec3 mvec = ipose.rot * cvec;

                double detect = minv;

                char pre = 0;
                double step = mu;

                for (double d = minv; d < maxv; d += step * voxel.unit) {
                    const Vec3 mpos = (ipose.trn + mvec * d) / voxel.unit + cent;
                    const int x = round(mpos.x);
                    const int y = round(mpos.y);
                    const int z = round(mpos.z);

                    if (inRect3(voxel.dsize, x, y, z) == false) continue;

                    const char val = voxel.vmap(x, y, z);
                    const char wei = voxel.wmap(x, y, z);

                    if (wei == 0) {
                        pre = 0;
                        step = mu * voxel.unit;
                        continue;
                    }

                    if (val >= 0 && pre < 0) {
                        if (step == 1.0){
                            detect = d - step * val / (val - pre);
                            break;
                        }
                        else {
                            d -= step;
                        }
                    }
                    else {
                        pre = val;
                    }

                    step = (val > -0.9 * SP_VOXEL_VMAX) ? 1.0 : mu;
                }

                if (detect > minv) {
                    const Vec3 mpos = (ipose.trn + mvec * detect) / voxel.unit + cent;
                    const Vec3 mnrm = voxel.getn(round(mpos.x), round(mpos.y), round(mpos.z));

                    const Vec3 cpos = cvec * detect;
                    const Vec3 cnrm = pose.rot * mnrm;

                    map(u, v) = getVecPN(cpos, cnrm);
                }
            }
        }
    }

}

#endif
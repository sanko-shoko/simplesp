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
#define SP_VOXEL_NULL -127

    template<typename CELM = char>
    class Voxel {

    public:
        // voxel size
        int dsize[3];

        // voxel unit length
        SP_REAL unit;

        // weight map
        Mem3<char> wmap;

        // value map
        Mem3<char> vmap;

        // color map
        Mem3<CELM> cmap;
    public:

        Voxel() {
            dsize[0] = 0;
            dsize[1] = 0;
            dsize[2] = 0;
            unit = 0.0;
        }

        Voxel(const Voxel<> &voxel) {
            *this = voxel;
        }

        Voxel& operator = (const Voxel<> &voxel) {
            dsize[0] = voxel.dsize[0];
            dsize[1] = voxel.dsize[1];
            dsize[2] = voxel.dsize[2];
            unit = voxel.unit;

            vmap = voxel.vmap;
            wmap = voxel.wmap;
            cmap = voxel.cmap;
            return *this;
        }

        void init(const int *dsize, const double unit) {
            this->dsize[0] = dsize[0];
            this->dsize[1] = dsize[1];
            this->dsize[2] = dsize[2];
            this->unit = unit;

            vmap.resize(dsize);
            wmap.resize(dsize);
            cmap.resize(dsize);
            zero();
        }
        
        void init(const int size, const double unit) {
            const int dsize[3] = { size, size, size };
            init(dsize, unit);
        }

 
        void zero() {
            setElm(vmap, -SP_VOXEL_VMAX);
            wmap.zero();
            cmap.zero();
        }

        void update(const int x, const int y, const int z, const double srcv) {
            if (srcv > +1.0) return;

            char &val = vmap(x, y, z);
            char &wei = wmap(x, y, z);

            cnvVal(val, (val * wei + SP_VOXEL_VMAX * srcv) / (wei + 1.0));
            wei = minval(wei + 1, SP_VOXEL_WMAX);
        }

        char getv(const int x, const int y, const int z) const {
            char val = SP_VOXEL_NULL;

            if (inRect(dsize, x, y, z) == true) {
                val = vmap(x, y, z);
            }
            return val;
        }

        char getw(const int x, const int y, const int z) const {
            char wei = 0;

            if (inRect(dsize, x, y, z) == true) {
                wei = wmap(x, y, z);
            }
            return wei;
        }

        Vec3 getn(const int x, const int y, const int z) const {
            const double vx = vmap(x + 1, y, z) - vmap(x - 1, y, z);
            const double vy = vmap(x, y + 1, z) - vmap(x, y - 1, z);
            const double vz = vmap(x, y, z + 1) - vmap(x, y, z - 1);
            return unitVec(getVec3(-vx, -vy, -vz));
        }

        Vec3 center() const {
            return getVec3(dsize[0] - 1, dsize[1] - 1, dsize[2] - 1) * 0.5;
        }
    };


    SP_CPUFUNC bool cnvMeshToVoxel(Voxel<> &voxel, const Mem1<Mesh3> &meshes, const double unit = 1.0) {

        const int size = (ceil(getModelRadius(meshes) / unit) + 2) * 2;
        SP_PRINTD("voxel size %d\n", size);

        voxel.init(size, unit);
        setElm(voxel.vmap, +1);

        const SP_REAL step = sqrt(3.0) * unit;

        const CamParam cam = getCamParam(size * 2, size * 2);
        const SP_REAL distance = sqrt(3.0) * getModelDistance(meshes, cam);

        const Vec3 cent = voxel.center();

        const int level = 0;

        for (int i = 0; i < getGeodesicMeshNum(level); i++) {
            const Pose pose = getGeodesicPose(level, i, distance);

            Mem2<VecPN3> pnmap;
            renderVecPN(pnmap, cam, pose, meshes);

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int z = 0; z < voxel.dsize[2]; z++) {
                for (int y = 0; y < voxel.dsize[1]; y++) {
                    for (int x = 0; x < voxel.dsize[0]; x++) {
                        const Vec3 mpos = getVec3(x, y, z);
                        const Vec3 cpos = pose * ((mpos - cent) * unit);

                        const Vec2 pix = mulCam(cam, prjVec(cpos));
                        if (inRect(pnmap.dsize, pix.x, pix.y) == false) continue;

                        const Vec3 &pos = pnmap(round(pix.x), round(pix.y)).pos;
                        const Vec3 &nrm = pnmap(round(pix.x), round(pix.y)).nrm;

                        if (pos.z == 0.0) {
                            voxel.update(x, y, z, -0.1);
                        }
                        else {
                            if (dotVec(cpos, nrm) >= 0) continue;

                            const SP_REAL dist = maxval(cpos.z - pos.z, -step) / step;
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

    
    //--------------------------------------------------------------------------------
    // Marching cubes
    //--------------------------------------------------------------------------------
  
    namespace _mc {
        SP_CPUFUNC Mem1<Mem1<Vec3> > getVertexOrder() {
            Mem1<Mem1<Vec3> > orders;

            for (int z = 0; z < 2; z++) {
                for (int y = 0; y < 2; y++) {
                    for (int x = 0; x < 2; x++) {
                        const int m = (x + y + z) % 2 ? -1 : +1;

                        for (int i = 0; i < 3; i++) {
                            Mem1<Vec3> order;
                            for (int j = 0; j < 8; j++) {
                                Vec3 vi;
                                vi.x = (j & (1 << ((6 - i + m * 0) % 3))) ? 1 - x : x;
                                vi.y = (j & (1 << ((6 - i + m * 1) % 3))) ? 1 - y : y;
                                vi.z = (j & (1 << ((6 - i + m * 2) % 3))) ? 1 - z : z;
                                order.push(vi);
                            }
                            orders.push(order);
                        }
                    }
                }
            }
            return orders;
        }

        SP_CPUFUNC Mem1<Mem1<Byte> > getPattern() {
            Mem1<Mem1<Byte> > ptns;
            Mem1<Mem1<Vec3> > orders = getVertexOrder();
            
            // 15 pattern
            const Byte list[][8] = {
                { +0, +0, +0, +0, +0, +0, +0, +0 },{ +0, +0, +0, +0, +0, +0, +1, +0 },{ +0, +0, +0, +0, +0, +0, +1, +1 },
                { +0, +0, +0, +1, +0, +0, +1, +0 },{ +0, +0, +0, +0, +1, +1, +0, +1 },{ +0, +0, +0, +0, +1, +1, +1, +1 },
                { +0, +0, +1, +0, +1, +1, +0, +1 },{ +1, +0, +0, +1, +0, +1, +1, +0 },{ +1, +0, +0, +0, +1, +1, +1, +0 },
                { +1, +0, +0, +0, +1, +1, +0, +1 },{ +0, +1, +0, +0, +0, +0, +1, +0 },{ +0, +1, +0, +0, +0, +0, +1, +1 },
                { +0, +1, +1, +0, +0, +0, +0, +1 },{ +0, +1, +1, +0, +0, +1, +1, +0 },{ +0, +1, +0, +0, +1, +1, +1, +0 }
            };

            for (int p = 0; p < 15; p++) {
                Mem1<Byte> tmps;
                for (int o = 0; o < orders.size(); o++) {

                    Byte b = 0;
                    for (int i = 0; i < 8; i++) {
                        setBit(&b, round(orders[o][i].z * 4 + orders[o][i].y * 2 + orders[o][i].x), list[p][i]);
                    }
                    tmps.push(b);
                }
                ptns.push(tmps);
            }
            return ptns;
        }

        SP_CPUFUNC Vec3 mvec(const char *v, const Vec3 *p, const int i, const int j) {
            return (v == NULL) ? ((p[i] + p[j]) / (2.0)) : (abs(v[j]) * p[i] + abs(v[i]) * p[j]) / (abs(v[i]) + abs(v[j]));
        };

        SP_CPUFUNC Mesh3 mmesh(const Vec3 &a, const Vec3 &b, const Vec3 &c, const int pid) {
            return (pid > 0) ? getMesh3(a, b, c) : getMesh3(a, c, b);
        };

        SP_CPUFUNC Mesh3 mmesh(const char *v, const Vec3 *p, const int a0, const int a1, const int b0, const int b1, const int c0, const int c1, const int pid) {
            return mmesh(mvec(v, p, a0, a1), mvec(v, p, b0, b1), mvec(v, p, c0, c1), pid);
        };

        SP_CPUFUNC Mem1<Mesh3> div3a(const Vec3 &a, const Vec3 &b, const Vec3 &c, const int pid) {
            const Vec3 s = (a + b + c) / 3.0;
            const Vec3 ab = (a + b) / 2.0;
            const Vec3 bc = (b + c) / 2.0;
            const Vec3 ca = (c + a) / 2.0;

            Mem1<Mesh3> ret;
            ret.push(mmesh(a, ab, s, pid));
            ret.push(mmesh(a, s, ca, pid));
            ret.push(mmesh(b, bc, s, pid));
            ret.push(mmesh(b, s, ab, pid));
            ret.push(mmesh(c, ca, s, pid));
            ret.push(mmesh(c, s, bc, pid));
            return ret;
        };
        SP_CPUFUNC Mem1<Mesh3> div3b(const Vec3 &a, const Vec3 &b, const Vec3 &c, const int pid) {

            const Vec3 s = (a + b + c) / 3.0;
            const Vec3 ab = (a + b) / 2.0;
            const Vec3 bc = (b + c) / 2.0;
            const Vec3 ca = (c + a) / 2.0;

            Mem1<Mesh3> ret;
            ret.push(mmesh(a, ab, ca, pid));
            ret.push(mmesh(b, bc, ab, pid));
            ret.push(mmesh(c, ca, bc, pid));
            ret.push(mmesh(ab, bc, ca, pid));
            return ret;
        };
        SP_CPUFUNC Mem1<Mesh3> div4(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d, const int pid) {
            const Vec3 s = (a + b + c + d) / 4.0;
            const Vec3 ab = (a + b) / 2.0;
            const Vec3 bc = (b + c) / 2.0;
            const Vec3 cd = (c + d) / 2.0;
            const Vec3 da = (d + a) / 2.0;
           
            Mem1<Mesh3> ret;
            ret.push(mmesh(a, ab, da, pid));
            ret.push(mmesh(s, da, ab, pid));
            ret.push(mmesh(b, bc, ab, pid));
            ret.push(mmesh(s, ab, bc, pid));
            ret.push(mmesh(c, cd, bc, pid));
            ret.push(mmesh(s, bc, cd, pid));
            ret.push(mmesh(d, da, cd, pid));
            ret.push(mmesh(s, cd, da, pid));
            return ret;
        };
        SP_CPUFUNC Mem1<Mesh3> div6(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d, const Vec3 &e, const Vec3 &f, const int pid) {
            const Vec3 s = (a + b + c + d + e + f) / 6.0;
            const Vec3 ab = (a + b) / 2.0;
            const Vec3 bc = (b + c) / 2.0;
            const Vec3 cd = (c + d) / 2.0;
            const Vec3 de = (d + e) / 2.0;
            const Vec3 ef = (e + f) / 2.0;
            const Vec3 fa = (f + a) / 2.0;
            Mem1<Mesh3> ret;
            ret.push(mmesh(a, ab, s, pid));
            ret.push(mmesh(a, s, fa, pid));
            ret.push(mmesh(b, bc, s, pid));
            ret.push(mmesh(b, s, ab, pid));
            ret.push(mmesh(c, cd, s, pid));
            ret.push(mmesh(c, s, bc, pid));
            ret.push(mmesh(d, de, s, pid));
            ret.push(mmesh(d, s, cd, pid));
            ret.push(mmesh(e, ef, s, pid));
            ret.push(mmesh(e, s, de, pid));
            ret.push(mmesh(f, fa, s, pid));
            ret.push(mmesh(f, s, ef, pid));
            return ret;
        };
    }

    SP_CPUFUNC bool cnvVoxelToMesh(Mem1<Mesh3> &meshes, const Voxel<> &voxel) {

        meshes.clear();

        const Mem1<Mem1<Byte> > ptns = _mc::getPattern();
        const Mem1<Mem1<Vec3> > orders = _mc::getVertexOrder();

        Rect3 vrect = getRect3(voxel.dsize);
        for (int i = 0; i < 3; i++) {
            vrect.dbase[i] -= 1;
            vrect.dsize[i] += 1;
        }

        Mem1<Mem1<Mesh3> > zms(vrect.dsize[2]);
        Mem1<Mem1<char> > dpids(vrect.dsize[2]);

        {
#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int z = vrect.dbase[2]; z < vrect.dbase[2] + vrect.dsize[2]; z++) {
                for (int y = vrect.dbase[1]; y < vrect.dbase[1] + vrect.dsize[1]; y++) {
                    for (int x = vrect.dbase[0]; x < vrect.dbase[0] + vrect.dsize[0]; x++) {

                        Mem3<char> vmap(2, 2, 2);
                        for (int zz = 0; zz < 2; zz++) {
                            for (int yy = 0; yy < 2; yy++) {
                                for (int xx = 0; xx < 2; xx++) {
                                    vmap(xx, yy, zz) = voxel.getv(x + xx, y + yy, z + zz);
                                }
                            }
                        }
  
                        Byte bb = 0;
                        cnvBit<char>(&bb, 1, vmap.ptr, 8, 0);

                        const int bcnt = cntBit(bb);
                        if (bcnt == 8 || bcnt == 0) continue;

                        if (bcnt > 4) {
                            bb = (Byte)~bb;
                        }

                        // pattern id
                        int pid = 0;
                        int ord = 0;


                        // matching
                        for (int p = 1; p < ptns.size(); p++) {
                            if (bcnt != cntBit(ptns[p][0]) && 8-bcnt != cntBit(ptns[p][0])) continue;

                            const Byte *b = ptns[p].ptr;
                            for (int o = 0; o < 24; o++) {
                                if (b[o] != bb) continue;

                                pid = p * ((bcnt <= 4) ? +1 : -1);
                                ord = o;
                                goto _match;
                            }
                        }
                        continue;
                    _match:;

                        const int mx = x - vrect.dbase[0];
                        const int my = y - vrect.dbase[1];
                        const int mz = z - vrect.dbase[2];
                        Mem1<Mesh3> &ms = zms[mz];

                        Vec3 p[8];
                        char v[8];

                        for (int k = 0; k < 8; k++) {
                            const Vec3 &vi = orders[ord][k];
                            p[k] = getVec3(x + vi.x, y + vi.y, z + vi.z);
                            v[k] = vmap(round(vi.x), round(vi.y), round(vi.z));
                        }

                        auto h = [&](const int a0, const int a1, const int b0, const int b1, const int c0, const int c1) {
                            ms.push(_mc::mmesh(v, p, a0, a1, b0, b1, c0, c1, pid));
                        };

                        switch (abs(pid)) {
                        default:
                            break;
                        case 1:
                            h(6, 2, 6, 7, 6, 4);
                            break;
                        case 2:
                            h(6, 2, 7, 3, 7, 5);
                            h(6, 2, 7, 5, 6, 4);
                            break;
                        case 3:
                            h(3, 1, 3, 7, 3, 2);
                            h(6, 2, 6, 7, 6, 4);
                            break;
                        case 4:
                            h(4, 0, 7, 3, 5, 1);
                            h(4, 0, 4, 6, 7, 3);
                            h(4, 6, 7, 6, 7, 3);
                            break;
                        case 5:
                            h(4, 0, 6, 2, 5, 1);
                            h(5, 1, 6, 2, 7, 3);
                            break;
                        case 6:
                            h(2, 0, 2, 3, 2, 6);
                            h(4, 0, 7, 3, 5, 1);
                            h(4, 0, 4, 6, 7, 3);
                            h(4, 6, 7, 6, 7, 3);
                            break;
                        case 7:
                            h(0, 1, 0, 2, 0, 4);
                            h(3, 1, 3, 7, 3, 2);
                            h(5, 1, 5, 4, 5, 7);
                            h(6, 2, 6, 7, 6, 4);
                            break;
                        case 8:
                            h(0, 1, 0, 2, 6, 2);
                            h(0, 1, 6, 2, 6, 7);
                            h(0, 1, 6, 7, 5, 1);
                            h(5, 1, 6, 7, 5, 7);
                            break;
                        case 9:
                            h(0, 1, 0, 2, 5, 1);
                            h(0, 2, 6, 7, 5, 1);
                            h(0, 2, 4, 6, 7, 6);
                            h(5, 1, 7, 6, 7, 3);
                            break;
                        case 10:
                            h(1, 0, 1, 5, 1, 3);
                            h(6, 2, 6, 7, 6, 4);
                            break;
                        case 11:
                            h(1, 0, 1, 5, 1, 3);
                            h(6, 2, 7, 3, 7, 5);
                            h(6, 2, 7, 5, 6, 4);
                            break;
                        case 12:
                            h(1, 0, 1, 5, 1, 3);
                            h(2, 0, 2, 3, 2, 6);
                            h(7, 3, 7, 5, 7, 6);
                            break;
                        case 13:
                            h(1, 0, 5, 7, 1, 3);
                            h(1, 0, 5, 4, 5, 7);
                            h(2, 0, 2, 3, 6, 7);
                            h(2, 0, 6, 7, 6, 4);
                            break;
                        case 14:
                            h(1, 0, 4, 0, 1, 3);
                            h(1, 3, 4, 0, 6, 7);
                            h(4, 0, 6, 2, 6, 7);
                            h(5, 7, 1, 3, 6, 7);
                            break;
                        }

                        dpids[mz].push(pid);

                        // hole filling
                        {
                            auto fill = [&](const int a, const int b, const int c, const int d, const Vec3 &n) {
                                const Vec3 p0 = getVec3(x, y, z) + getVec3(orders[ord][a].x, orders[ord][a].y, orders[ord][a].z) + n;
                                const Vec3 p1 = getVec3(x, y, z) + getVec3(orders[ord][b].x, orders[ord][b].y, orders[ord][b].z) + n;
                                const Vec3 p2 = getVec3(x, y, z) + getVec3(orders[ord][c].x, orders[ord][c].y, orders[ord][c].z) + n;
                                const Vec3 p3 = getVec3(x, y, z) + getVec3(orders[ord][d].x, orders[ord][d].y, orders[ord][d].z) + n;

                                int cnt = 0;
                                cnt += (voxel.getv(round(p0.x), round(p0.y), round(p0.z)) >= 0) ? 1 : 0;
                                cnt += (voxel.getv(round(p1.x), round(p1.y), round(p1.z)) >= 0) ? 1 : 0;
                                cnt += (voxel.getv(round(p2.x), round(p2.y), round(p2.z)) >= 0) ? 1 : 0;
                                cnt += (voxel.getv(round(p3.x), round(p3.y), round(p3.z)) >= 0) ? 1 : 0;

                                if (cnt >= 3) {
                                    zms[mz].push(getMesh3(_mc::mvec(v, p, a, b), _mc::mvec(v, p, b, d), _mc::mvec(v, p, a, c)));
                                    zms[mz].push(getMesh3(_mc::mvec(v, p, a, c), _mc::mvec(v, p, b, d), _mc::mvec(v, p, c, d)));
                                    dpids[mz].push(15);
                                }
                            };

                            // +dx
                            if (pid == 7 || pid == 11 || pid == 12) {
                                const Vec3 nx = getVec3(orders[ord][1].x, orders[ord][1].y, orders[ord][1].z) - getVec3(orders[ord][0].x, orders[ord][0].y, orders[ord][0].z);
                                fill(3, 1, 7, 5, +nx);
                            }
                            // -dx
                            if (pid == 6 || pid == 7) {
                                const Vec3 nx = getVec3(orders[ord][1].x, orders[ord][1].y, orders[ord][1].z) - getVec3(orders[ord][0].x, orders[ord][0].y, orders[ord][0].z);
                                fill(0, 2, 4, 6, -nx);
                            }
                            // +dy
                            if (pid == 3 || pid == 6 || pid == 7 || pid == 12) {
                                const Vec3 ny = getVec3(orders[ord][2].x, orders[ord][2].y, orders[ord][2].z) - getVec3(orders[ord][0].x, orders[ord][0].y, orders[ord][0].z);
                                fill(2, 3, 6, 7, +ny);
                            }
                            // -dy
                            if (pid == 7) {
                                const Vec3 ny = getVec3(orders[ord][2].x, orders[ord][2].y, orders[ord][2].z) - getVec3(orders[ord][0].x, orders[ord][0].y, orders[ord][0].z);
                                fill(1, 0, 5, 4, -ny);
                            }
                            // +dz
                            if (pid == 7 || pid == 13) {
                                const Vec3 nz = getVec3(orders[ord][4].x, orders[ord][4].y, orders[ord][4].z) - getVec3(orders[ord][0].x, orders[ord][0].y, orders[ord][0].z);
                                fill(6, 7, 4, 5, +nz);
                            }
                            // -dz
                            if (pid == 7 || pid == 12 || pid == 13) {
                                const Vec3 nz = getVec3(orders[ord][4].x, orders[ord][4].y, orders[ord][4].z) - getVec3(orders[ord][0].x, orders[ord][0].y, orders[ord][0].z);
                                fill(0, 1, 2, 3, -nz);
                            }
                        }

                    }
                }
            }
        
}
        {

#if 0

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int z = vrect.dbase[2]; z < vrect.dbase[2] + vrect.dsize[2]; z++) {
                const int mz = z - vrect.dbase[2];

                Mem1<Mesh3> tmps;
                int cnt = 0;
                for (int i = 0; i < dpids[mz].size(); i++) {
                    const int pid = dpids[mz][i];
                    auto h = [&](const Vec3 &a, const Vec3 &b, const Vec3 &c) -> Mesh3 {
                        return (pid < 0) ? getMesh3(a, b, c) : getMesh3(a, c, b);
                    };
                    auto div3 = [&](const Vec3 &a, const Vec3 &b, const Vec3 &c, const int type) {
                        const Vec3 s = (a + b + c) / 3.0;
                        const Vec3 ab = (a + b) / 2.0;
                        const Vec3 bc = (b + c) / 2.0;
                        const Vec3 ca = (c + a) / 2.0;

                        const SP_REAL A = normVec(a - b);
                        const SP_REAL B = normVec(b - c);
                        const SP_REAL C = normVec(c - a);
                        if (A == 0.0 || B == 0.0 || C == 0.0) return;

                        const SP_REAL s0 = fabs(A - B) / A;
                        const SP_REAL s1 = fabs(B - C) / B;
                        const SP_REAL s2 = fabs(C - A) / C;
                        switch (type) {
                        case 0:
                            tmps.push(h(a, s, ab));
                            tmps.push(h(a, ca, s));
                            tmps.push(h(b, s, bc));
                            tmps.push(h(b, ab, s));
                            tmps.push(h(c, s, ca));
                            tmps.push(h(c, bc, s));
                            break;
                        case 1:
                            tmps.push(h(a, ca, ab));
                            tmps.push(h(b, ab, bc));
                            tmps.push(h(c, bc, ca));
                            tmps.push(h(ab, ca, bc));
                            break;
                        }
                    };
                    auto div4 = [&](const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d) {
                        const Vec3 s = (a + b + c + d) / 4.0;
                        const Vec3 ab = (a + b) / 2.0;
                        const Vec3 bc = (b + c) / 2.0;
                        const Vec3 cd = (c + d) / 2.0;
                        const Vec3 da = (d + a) / 2.0;
                        tmps.push(h(a, da, ab));
                        tmps.push(h(s, ab, da));
                        tmps.push(h(b, ab, bc));
                        tmps.push(h(s, bc, ab));
                        tmps.push(h(c, bc, cd));
                        tmps.push(h(s, cd, bc));
                        tmps.push(h(d, cd, da));
                        tmps.push(h(s, da, cd));
                    };
                    auto div6 = [&](const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d, const Vec3 &e, const Vec3 &f) {
                        const Vec3 s = (a + b + c + d + e + f) / 6.0;
                        const Vec3 ab = (a + b) / 2.0;
                        const Vec3 bc = (b + c) / 2.0;
                        const Vec3 cd = (c + d) / 2.0;
                        const Vec3 de = (d + e) / 2.0;
                        const Vec3 ef = (e + f) / 2.0;
                        const Vec3 fa = (f + a) / 2.0;
                        tmps.push(h(a, s, ab));
                        tmps.push(h(a, fa, s));
                        tmps.push(h(b, s, bc));
                        tmps.push(h(b, ab, s));
                        tmps.push(h(c, s, cd));
                        tmps.push(h(c, bc, s));
                        tmps.push(h(d, s, de));
                        tmps.push(h(d, cd, s));
                        tmps.push(h(e, s, ef));
                        tmps.push(h(e, de, s));
                        tmps.push(h(f, s, fa));
                        tmps.push(h(f, ef, s));
                    };
                    const int i1 = (pid > 0) ? 1 : 2;
                    const int i2 = (pid > 0) ? 2 : 1;
                    switch (abs(pid)) {
                    case 1:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 0);
                        cnt += 1;
                        break;
                    case 2:
                        div4(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], zms[mz][cnt + 1].pos[i2]);
                        cnt += 2;
                        break;
                    case 3:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 0);
                        div3(zms[mz][cnt + 1].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 1].pos[i2], 0);
                        cnt += 2;
                        break;
                    case 4:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 1);
                        div4(zms[mz][cnt + 1].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 2].pos[i1], zms[mz][cnt + 1].pos[i2]);
                        cnt += 3;
                        break;
                    case 5:
                        div4(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 1].pos[i2], zms[mz][cnt + 0].pos[i2]);
                        cnt += 2;
                        break;
                    case 6:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 0);
                        div3(zms[mz][cnt + 1].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 1].pos[i2], 1);
                        div4(zms[mz][cnt + 2].pos[0], zms[mz][cnt + 2].pos[i1], zms[mz][cnt + 3].pos[i1], zms[mz][cnt + 2].pos[i2]);
                        cnt += 4;
                        break;
                    case 7:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 0);
                        div3(zms[mz][cnt + 1].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 1].pos[i2], 0);
                        div3(zms[mz][cnt + 2].pos[0], zms[mz][cnt + 2].pos[i1], zms[mz][cnt + 2].pos[i2], 0);
                        div3(zms[mz][cnt + 3].pos[0], zms[mz][cnt + 3].pos[i1], zms[mz][cnt + 3].pos[i2], 0);
                        cnt += 4;
                        break;
                    case 8:
                        div6(
                            zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2],
                            zms[mz][cnt + 1].pos[i2], zms[mz][cnt + 3].pos[i2], zms[mz][cnt + 3].pos[0]);
                        cnt += 4;
                        break;
                    case 9:
                        div6(
                            zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 2].pos[i1],
                            zms[mz][cnt + 2].pos[i2], zms[mz][cnt + 3].pos[i2], zms[mz][cnt + 3].pos[0]);
                        cnt += 4;
                        break;
                    case 10:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 0);
                        div3(zms[mz][cnt + 1].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 1].pos[i2], 0);
                        cnt += 2;
                        break;
                    case 11:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 0);
                        div4(zms[mz][cnt + 1].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 1].pos[i2], zms[mz][cnt + 2].pos[i2]);
                        cnt += 3;
                        break;
                    case 12:
                        div3(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 0].pos[i2], 0);
                        div3(zms[mz][cnt + 1].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 1].pos[i2], 0);
                        div3(zms[mz][cnt + 2].pos[0], zms[mz][cnt + 2].pos[i1], zms[mz][cnt + 2].pos[i2], 0);
                        cnt += 3;
                        break;
                    case 13:
                        div4(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 1].pos[i1], zms[mz][cnt + 1].pos[i2], zms[mz][cnt + 0].pos[i2]);
                        div4(zms[mz][cnt + 2].pos[0], zms[mz][cnt + 2].pos[i1], zms[mz][cnt + 2].pos[i2], zms[mz][cnt + 3].pos[i2]);
                        cnt += 4;
                        break;
                    case 14:
                        div6(
                            zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 2].pos[i1],
                            zms[mz][cnt + 2].pos[i2], zms[mz][cnt + 3].pos[0], zms[mz][cnt + 3].pos[i1]);
                        cnt += 4;
                        break;
                    case 15:
                        div4(zms[mz][cnt + 0].pos[0], zms[mz][cnt + 0].pos[i1], zms[mz][cnt + 1].pos[i2], zms[mz][cnt + 0].pos[i2]);
                        cnt += 2;
                        break;
                    }
                }

                zms[mz] = tmps;
            }
#endif
            {
                meshes.reserve(voxel.vmap.size() * 10);

                for (int z = vrect.dbase[2]; z < vrect.dbase[2] + vrect.dsize[2]; z++) {
                    const int mz = z - vrect.dbase[2];

                    for (int i = 0; i < zms[mz].size(); i++) {
                        const Mesh3 &m = zms[mz][i];
                        if (normVec(crsVec(m.pos[1] - m.pos[0], m.pos[2] - m.pos[0])) > SP_SMALL) {
                            meshes.push(m);
                        }
                    }
                }
            }

            meshes = (meshes - voxel.center()) * voxel.unit;
        }
        return true;
    }


    // voxel range mask
    SP_CPUFUNC void calcVoxelMask(Mem2<Vec2> &rmsk, const CamParam &cam, const Pose &pose, const Voxel<> &voxel) {

        rmsk.resize(cam.dsize);

        const SP_REAL radius = (voxel.dsize[0] - 1) * voxel.unit * 0.5;

        struct VoxelPlane {
            Vec3 pos, axis[3];
        };
        Mem1<VoxelPlane> vps;
        {
            const Vec3 _axis[3] = { getVec3(1.0, 0.0, 0.0), getVec3(0.0, 1.0, 0.0), getVec3(0.0, 0.0, 1.0) };
            
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
                range = getVec2(0.0, SP_INFINITY);

                const Vec3 vec = getVec3(invCam(cam, getVec2(u, v)), 1.0);
                for (int i = 0; i < vps.size(); i++) {
                    const Vec3 &pos = vps[i].pos;
                    const Vec3 &X = vps[i].axis[0];
                    const Vec3 &Y = vps[i].axis[1];
                    const Vec3 &Z = vps[i].axis[2];

                    const SP_REAL n = dotVec(vec, Z);
                    if (fabs(n) < SP_SMALL) continue;

                    const Vec3 crs = vec * (dotVec(pos, Z) / n);
                    if (crs.z <= SP_SMALL) continue;

                    const SP_REAL x = dotVec(crs - pos, X);
                    const SP_REAL y = dotVec(crs - pos, Y);

                    if (n < 0) {
                        if (maxval(fabs(x), fabs(y)) > radius * 1.00) continue;
                        range.x = maxval(range.x, crs.z);
                    }
                    else {
                        if (maxval(fabs(x), fabs(y)) > radius * 1.01) continue;
                        range.y = minval(range.y, crs.z);
                    }
                }
   
            }
        }

    }


    //--------------------------------------------------------------------------------
    // visual hull
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool visualHull(Voxel<> &voxel, const Mem1<Mem2<Byte> > &imgs, const Mem1<CamParam> &cams, const Mem1<Pose> &poses, const SP_REAL unit = 1.0) {

        SP_REAL meanDist = 0.0;
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
                        const Vec3 mpos = getVec3(x, y, z);
                        const Vec3 cpos = pose * ((mpos - cent) * unit);

                        const Vec2 pix = mulCam(cam, prjVec(cpos));
                        if (inRect(img.dsize, pix.x, pix.y) == false) continue;

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

    SP_CPUFUNC void updateTSDF(Voxel<> &voxel, const CamParam &cam, const Pose &pose, const Mem2<SP_REAL> &depth, const SP_REAL mu = 5.0) {

        const Vec3 cent = voxel.center();
        const SP_REAL step = mu * voxel.unit;

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int z = 0; z < voxel.dsize[2]; z++) {
            for (int y = 0; y < voxel.dsize[1]; y++) {
                for (int x = 0; x < voxel.dsize[0]; x++) {
                    const Vec3 mpos = getVec3(x, y, z);
                    const Vec3 cpos = pose * ((mpos - cent) * voxel.unit);

                    const Vec2 pix = mulCam(cam, prjVec(cpos));
                    if (inRect(depth.dsize, pix.x, pix.y) == false) continue;

                    const SP_REAL d = depth(round(pix.x), round(pix.y));
                    if (d == 0.0) continue;

                    const SP_REAL dist = maxval(cpos.z - d, -step) / step;
                    voxel.update(x, y, z, dist);
                }
            }
        }
    }

    SP_CPUFUNC void rayCasting(Mem2<VecPN3> &map, const CamParam &cam, const Pose &pose, const Voxel<> &voxel, const SP_REAL mu = 5.0) {

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

                const SP_REAL minv = rmsk(u, v).x;
                const SP_REAL maxv = rmsk(u, v).y;
                if (minv <= SP_SMALL) continue;

                const Vec3 cvec = getVec3(invCam(cam, getVec2(u, v)), 1.0);
                const Vec3 mvec = ipose.rot * cvec;

                SP_REAL detect = minv;

                char pre = 0;
                SP_REAL step = mu;

                for (SP_REAL d = minv; d < maxv; d += step * voxel.unit) {
                    const Vec3 mpos = (ipose.trn + mvec * d) / voxel.unit + cent;
                    const int x = round(mpos.x);
                    const int y = round(mpos.y);
                    const int z = round(mpos.z);

                    if (inRect(voxel.dsize, x, y, z) == false) continue;

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

                    map(u, v) = getVecPN3(cpos, cnrm);
                }
            }
        }
    }

}

#endif
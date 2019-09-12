//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RAY_H__
#define __SP_RAY_H__

#include "spcore/spcore.h"

namespace sp {

    SP_CPUFUNC bool traceMesh(SP_REAL *result, const Mesh3 &mesh, const VecPD3 &ray, const double minv, const double maxv) {
        const Vec3 base = mesh.pos[0] - ray.pos;
        const Vec3 A = mesh.pos[1] - mesh.pos[0];
        const Vec3 B = mesh.pos[2] - mesh.pos[0];

        SP_REAL mat[3 * 3] = { ray.drc.x, -A.x, -B.x, ray.drc.y, -A.y, -B.y, ray.drc.z, -A.z, -B.z };
        SP_REAL inv[3 * 3];
        if (invMat33(inv, mat) == false) return false;

        SP_REAL val[3] = { base.x, base.y, base.z };
        mulMat(result, 3, 1, inv, 3, 3, val, 3, 1);

        if (result[0] < minv || result[0] > maxv) return false;
        if (result[1] < -SP_SMALL || result[2] < -SP_SMALL || result[1] + result[2] > 1.0 + SP_SMALL) return false;
        return true;
    }


    SP_CPUFUNC bool checkHit(const Box3 &box, const VecPD3 &ray, const double minv, const double maxv) {
        double find[2] = { minv, maxv };
        for (int i = 0; i < 3; i++) {
            const double v = acsv(ray.drc, i);
            if (fabs(v) < SP_SMALL) continue;

            const Vec3 &a = (v >= 0.0) ? box.pos[0] : box.pos[1];
            const Vec3 &b = (v >= 0.0) ? box.pos[1] : box.pos[0];

            const double f0 = (acsv(a, i) - acsv(ray.pos, i)) / v;
            const double f1 = (acsv(b, i) - acsv(ray.pos, i)) / v;
            find[0] = maxval(f0, find[0]);
            find[1] = minval(f1, find[1]);
            if (find[1] < find[0]) {
                return false;
            }
        }
        return true;
    }

    //--------------------------------------------------------------------------------
    // bounding volume hierarchy
    //--------------------------------------------------------------------------------
   
    class BVH {

    public:
        struct Node {
            int level;

            int base;
            int size;

            Box3 box;
            Node *n0, *n1;
        };

        struct Index {
            int acnt;

            const Mesh3* pmesh;
            const Material* pmat;
        };

    private:
        int m_acnt;

        Mem1<Node> m_nodes;

        Mem1<Index> m_idxs;

    public:
        BVH() {
            clear();
        }

        Mem1<const Node*> getNodes(const int level) const {
            Mem1<const Node*> nodes;

            for (int i = 0; i < m_nodes.size(); i++) {
                if (m_nodes[i].level == level) {
                    nodes.push(&m_nodes[i]);
                }
            }
            return nodes;
        }

        void clear() {
            m_acnt = 0;
            m_nodes.clear();
            m_idxs.clear();
        }
        
        void add(const Mem1<Mesh3> &meshes, const Mem1<Material*> &pmats = Mem1<Material*>()) {
            const int offset = m_idxs.size();
            const int size = meshes.size();

            const bool f = (pmats.size() > 0) ? true : false;

            m_idxs.extend(size);
            for (int i = 0; i < size; i++) {
                m_idxs[offset + i].acnt = m_acnt;
                m_idxs[offset + i].pmesh = &meshes[i];
                m_idxs[offset + i].pmat = (f == true) ? pmats[i] : NULL;
            }
            m_acnt++;
        }

        void build() {

            // mesh centers (for sort)
            struct IndexEx : public Index {
                Vec3 cent;
            };

            Mem1<IndexEx> idxs(m_idxs.size());
            for (int i = 0; i < idxs.size(); i++) {
                idxs[i].acnt = m_idxs[i].acnt;
                idxs[i].pmesh = m_idxs[i].pmesh;
                idxs[i].pmat = m_idxs[i].pmat;
                idxs[i].cent = getMeshCent(*m_idxs[i].pmesh);
            }

            m_nodes.clear();
            m_nodes.reserve(2 * idxs.size() - 1);
            auto initn = [&](const int level, const int base, const int size) -> Node*{
                Node *n = m_nodes.extend();
                n->level = level;
                n->base = base;
                n->size = size;
                n->n0 = NULL;
                n->n1 = NULL;

                n->box = nullBox3();
                for (int i = base; i < base + size; i++) {
                    n->box = orBox(n->box, *idxs[i].pmesh);
                }
                return n;
            };

            auto sorti = [&](Node &n, const int ax) {
                typedef int(*CMP)(const void*, const void*);
                CMP cmp[3];

                cmp[0] = [](const void* i1, const void* i2) -> int {
                    return (((IndexEx*)i1)->cent.x < ((IndexEx*)i2)->cent.x) ? +1 : -1;
                };
                cmp[1] = [](const void* i1, const void* i2) -> int {
                    return (((IndexEx*)i1)->cent.y < ((IndexEx*)i2)->cent.y) ? +1 : -1;
                };
                cmp[2] = [](const void* i1, const void* i2) -> int {
                    return (((IndexEx*)i1)->cent.z < ((IndexEx*)i2)->cent.z) ? +1 : -1;
                };

                sort(&idxs[n.base], n.size, cmp[ax]);
            };

            initn(0, 0, idxs.size());

            Mem2<SP_REAL> map(idxs.size(), 2);
            
            for(int ni = 0; ni < m_nodes.size(); ni++){
                Node& n = m_nodes[ni];
                if (n.size == 1) continue;

                int di = 0;
                int da = 0;
                {
                    double mina = SP_INFINITY;
                    for (int a = 0; a < 3; a++) {
                        sorti(n, a);
                        Box3 bl = nullBox3();
                        Box3 br = nullBox3();

                        for (int i = 1; i < n.size; i++) {
                            const int l = i;
                            const int r = n.size - i;
                            bl = orBox(bl, *idxs[n.base + l - 1].pmesh);
                            br = orBox(br, *idxs[n.base + r].pmesh);
                            map(l, 0) = getBoxArea(bl) * i;
                            map(r, 1) = getBoxArea(br) * i;
                        }

                        for (int i = 1; i < n.size; i++) {
                            const double area = 1.0 + (map(i, 0) + map(i, 1)) / getBoxArea(n.box);
                            if (area < mina) {
                                mina = area;
                                di = i;
                                da = a;
                            }
                        }
                    }
                }

                sorti(n, da);
                n.n0 = initn(n.level + 1, n.base, di);
                n.n1 = initn(n.level + 1, n.base + di, n.size - di);
            }

            for (int i = 0; i < m_idxs.size(); i++) {
                m_idxs[i] = idxs[i];
            }
        }

        struct Hit : Index{
            VecPD3 vec;
            SP_REAL scl;
        };

        bool trace(Hit &hit, const VecPD3 &ray, const double minv, const double maxv) const {
            int minid = -1;

            Mem1<const Node*> que;
            que.reserve(20);
            que.push(&m_nodes[0]);

            double lmaxv = maxv;
            int acnt = -1;

            while (que.size() > 0) {
                const Node *n = *que.last();
                que.pop();
                if (checkHit(n->box, ray, minv, lmaxv) == false) {
                    continue;
                }
                if (n->n0 != NULL && n->n1 != NULL) {
                    que.push(n->n0);
                    que.push(n->n1);
                    continue;
                }
                for (int i = n->base; i < n->base + n->size; i++) {
                    SP_REAL result[3] = { 0 };
                    if (traceMesh(result, *m_idxs[i].pmesh, ray, minv, lmaxv + SP_SMALL) == true) {
                        const Vec3 nrm = getMeshNrm(*m_idxs[i].pmesh);

                        const bool f = (dotVec(nrm, ray.drc) < 0.0);
                        if (result[0] < lmaxv - SP_SMALL || m_idxs[i].acnt < acnt) {
                            lmaxv = result[0];
                            minid = i;
                            acnt = m_idxs[i].acnt;
                        }
                    }
                }
            }
            if (minid < 0) {
                return false;
            }

            const Index &idx = m_idxs[minid];
            hit.acnt = idx.acnt;
            hit.pmat = idx.pmat;
            hit.pmesh = idx.pmesh;
            hit.scl = lmaxv;
            hit.vec.pos = ray.pos + ray.drc * lmaxv;
            hit.vec.drc = getMeshNrm(*m_idxs[minid].pmesh);

            return true;
        }
    };

    
    //--------------------------------------------------------------------------------
    // path trace
    //--------------------------------------------------------------------------------

    class PathTrace {

    public:
        static const int maxlt = 4;

        Pose m_pose;
        CamParam m_cam;
        bool m_pers;

        struct Img {
            Col4f amb;
            Col4f dif[maxlt + 1];
            float z;
        };
        Mem2<Img> m_img;
        Mem2<Byte> m_amask;

        struct ImgCnt {
            int amb;
            int dif;
            int z;
        };

        Mem2<ImgCnt> m_imgcnt;

        struct Limit {
            int amb;
            int dif;
        };

        Limit m_limit;

        Mem1<Vec3> m_lights;

        BVH m_bvh;

        Col4f m_bgcol;

        int m_upcnt;

       
    public:

        PathTrace() {
            m_bgcol = getCol4f(1.0, 1.0, 1.0, 0.0);
            m_bgcol = getCol4f(0.0, 0.0, 0.0, 0.0);
            setLimit(500, 20);
        }

        void clear() {
            m_bvh.clear();
            zero();
        }

        void zero() {
            m_upcnt = 0;
            m_img.zero();
            m_imgcnt.zero();
            m_amask.zero();
        }

        int upcnt() {
            return m_upcnt;
        }

        float prog() {
            return static_cast<float>(m_upcnt) / (m_limit.amb + m_limit.dif);
        }

        void setLimit(const int amb, const int dif) {
            m_limit.amb = amb;
            m_limit.dif = dif;
        }

        void setLights(const Mem1<Vec3> &lights) {
            SP_ASSERT(lights.size() <= maxlt);
            m_lights = lights;
            zero();
        }

        void setCam(const CamParam cam, const bool pers) {
            m_cam = cam;
            m_pers = pers;
            m_img.resize(m_cam.dsize);
            m_imgcnt.resize(m_cam.dsize);
            m_amask.resize(m_cam.dsize);

            zero();
        }

        void setPose(const Pose &pose) {
            m_pose = pose;
            zero();
        }
        const Pose& getPose() const {
            return m_pose;
        }

        void add(const Mem1<Mesh3> &meshes, const Mem1<Material*> &pmats) {
            m_bvh.add(meshes, pmats);
        }

        void build() {
            m_bvh.build();
        }

        void update() {
            if (m_upcnt > m_limit.amb + m_limit.dif) return;
            const Pose ipose = invPose(m_pose);
            const Mat irmat = getMat(ipose.rot);

            const int w = m_img.dsize[0];
            const int h = m_img.dsize[1];
#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int v = 0; v < h; v++) {
                for (int u = 0; u < w; u++) {
                    if (m_upcnt > 10 && m_img(u, v).z == SP_RCAST(0.0)) continue;

                    Vec2 pix = getVec2(u, v);
                    const unsigned int seed = (v * w + u + m_upcnt);
                    if (m_upcnt > 0) pix += randuVec2(0.5, 0.5, seed);

                    const Vec3 drc = unitVec(prjVec(invCam(m_cam, pix), 1.0, m_pers));
                    trace(m_img(u, v), m_imgcnt(u, v), getVecPD3(ipose.trn, irmat * drc), seed);
                }
            }

            m_upcnt++;
        }


        void bilateralFilter(const double sigma_s, const double sigma_c) {

            Mem2<Col4f> dif(m_img.dsize);
            Mem2<Col4f> amb(m_img.dsize);
            for (int i = 0; i < m_img.size(); i++) {
                dif[i] = m_img[i].dif[0];
                amb[i] = m_img[i].amb;
            }

            const int half = maxval(1, round((sigma_s - 0.8) / 0.3 + 1));

            Mem2<SP_REAL> kernel(2 * half + 1, 2 * half + 1);
            for (int y = -half; y <= +half; y++) {
                for (int x = -half; x <= +half; x++) {
                    const SP_REAL r2 = sq(x) + sq(y);
                    kernel(x + half, y + half) = exp(-r2 / (2.0 * sq(sigma_s)));
                }
            }

            const SP_REAL expscale = 10.0;
            Mem1<SP_REAL> exptable(100);
            for (int i = 0; i < exptable.size(); i++) {
                const double r = sq(i / expscale);
                const double v = exp(-r / 2.0);
                exptable[i] = SP_RCAST(v);
            }

            const Rect2 rect = getRect2(m_img.dsize);


//#if SP_USE_OMP
//#pragma omp parallel for
//#endif
//            for (int v = 0; v < dst.dsize[1]; v++) {
//                for (int u = 0; u < dst.dsize[0]; u++) {
//
//                    for (int c = 0; c < ch; c++) {
//                        const ELEM base = acs2<TYPE, ELEM>(tmp, u, v, c);
//
//                        SP_REAL sum = 0.0, div = 0.0;
//                        for (int ky = -half; ky < +half; ky++) {
//                            for (int kx = -half; kx < +half; kx++) {
//                                if (inRect(rect, u + kx, v + ky) == false) continue;
//
//                                const ELEM &val = acs2<TYPE, ELEM>(tmp, u + kx, v + ky, c);
//
//                                const SP_REAL a = kernel(kx + half, ky + half);
//                                const SP_REAL b = exptable(round(fabs(val - base) * expscale / sigma_c));
//
//                                sum += a * b * val;
//                                div += a * b;
//                            }
//                        }
//
//                        acs2<TYPE, ELEM>(dst, u, v, c) = cast<ELEM>(sum / div);
//                    }
//                }
//            }
        }
        void makeImg(Mem2<Col4> &img, const double amb, const double *difs) {
            img.resize(m_cam.dsize);
            img.zero();

            for (int i = 0; i < img.size(); i++) {
                Col4f amb = getCol4f(0.0, 0.0, 0.0, 0.0);
                Col4f dif = getCol4f(0.0, 0.0, 0.0, 0.0);
                if (m_imgcnt[i].dif > 0) {
                    for (int l = 0; l < m_lights.size(); l++) {
                        dif += m_img[i].dif[l + 1];
                    }
                    dif *= 1.0 / (m_lights.size());
                }
                if (m_imgcnt[i].amb > 0) {
                    amb = m_img[i].amb;
                }
                Col4f col = (amb + dif) * 0.5;
                img[i] = cast<Col4>(col);

            }
        }

    private:

        void trace(Img &img, ImgCnt &imgcnt, const VecPD3 &ray, const unsigned int seed) {
            const int maxamb = 500;
            const int maxdif = 20;

            BVH::Hit hit;
            if (m_bvh.trace(hit, ray, 0.0, SP_INFINITY) == false) {
                if (imgcnt.dif < maxdif) {
                    img.dif[0] = meanCol(img.dif[0], imgcnt.dif, m_bgcol, 1.0);
                    for (int i = 0; i < m_lights.size(); i++) {
                        img.dif[i + 1] = meanCol(img.dif[i + 1], imgcnt.dif, m_bgcol, 1.0);
                        //const float a0 = img.dif[i + 1].a;
                        //const float a1 = m_bgcol.a;

                        //if (a1 > 0.0f) {
                        //    img.dif[i + 1].r = (img.dif[i + 1].r * imgcnt.dif * a0 + m_bgcol.r * a1) / (imgcnt.dif + (a0 + a1));
                        //    img.dif[i + 1].g = (img.dif[i + 1].g * imgcnt.dif * a0 + m_bgcol.g * a1) / (imgcnt.dif + (a0 + a1));
                        //    img.dif[i + 1].b = (img.dif[i + 1].b * imgcnt.dif * a0 + m_bgcol.b * a1) / (imgcnt.dif + (a0 + a1));
                        //}

                        //img.dif[i + 1].a = (img.dif[i + 1].a * imgcnt.dif + m_bgcol.a) / (imgcnt.dif + 1.0);
                    }
                    imgcnt.dif++;
                }
                if (imgcnt.amb < maxdif) {
                    const float a0 = img.amb.a;
                    const float a1 = m_bgcol.a;
                    img.amb = meanCol(img.amb, imgcnt.amb, m_bgcol, 1.0);

                    //if (a1 > 0.0f) {
                    //    img.amb.r = (img.amb.r * imgcnt.amb * a0 + m_bgcol.r * a1) / (imgcnt.amb + (a0 + a1));
                    //    img.amb.g = (img.amb.g * imgcnt.amb * a0 + m_bgcol.g * a1) / (imgcnt.amb + (a0 + a1));
                    //    img.amb.b = (img.amb.b * imgcnt.amb * a0 + m_bgcol.b * a1) / (imgcnt.amb + (a0 + a1));
                    //}

                    //img.amb.a = (img.amb.a * imgcnt.amb + m_bgcol.a) / (imgcnt.amb + 1.0);
                    //img.amb = (img.amb * imgcnt.amb + m_bgcol) / (imgcnt.amb + 1.0);
                    imgcnt.amb++;
                }
            }
            else {
                if (imgcnt.amb > 0 && imgcnt.amb < maxamb) {
                    img.amb = meanCol(img.amb, imgcnt.amb, trace_amb(ray, hit, 0, seed), 1.0);
                    imgcnt.amb++;
                }
                else {
                    img.amb = hit.pmat->amb;
                    imgcnt.amb++;
                }

                if(imgcnt.dif >= 0 && imgcnt.dif < maxdif) {
                    img.dif[0] = meanCol(img.dif[0], imgcnt.dif, hit.pmat->dif, 1.0);
                    for (int i = 0; i < m_lights.size(); i++) {
                        img.dif[i + 1] = meanCol(img.dif[i + 1], imgcnt.dif, trace_dif(ray, hit, m_lights[i] + randgVec3(1.0, 1.0, 1.0, seed) * 1.0, 0, seed), 1.0);
                    }
                    imgcnt.dif++;
                }
                else {

                }

                {
                    img.z = (img.z * imgcnt.z + hit.scl) / (imgcnt.z + 1.0);
                    imgcnt.z++;
                }
            }

        }

        Col4f trace_dif(const VecPD3 &ray, const BVH::Hit base, const Vec3 lpos, const int level, const unsigned int seed) {
            const SP_REAL delta = 0.001;

            Col4f col = getCol4f(0.0, 0.0, 0.0, 0.0);
            const Vec3 nrm = getMeshNrm(*base.pmesh);

            VecPD3 next;
            next.pos = base.vec.pos + getMeshNrm(*base.pmesh) * delta;
            next.drc = unitVec(lpos - base.vec.pos);

            const SP_REAL d = dotVec(base.vec.drc, next.drc);
            BVH::Hit hit;
            if (d > 0.0 && m_bvh.trace(hit, next, 0.0, SP_INFINITY) == false) {
                col = base.pmat->dif * d;
                col.a = base.pmat->dif.a;
            }

            return col;
        }


        Col4f trace_amb(const VecPD3 &ray, const BVH::Hit base, const int level, const unsigned int seed) {
            const SP_REAL delta = 0.001;

            Col4f col = getCol4f(0.0, 0.0, 0.0, 0.0);
            const Vec3 nrm = getMeshNrm(*base.pmesh);

            Vec3 drc;
            {
                drc = unitVec(randuVec3(1.0, 1.0, 1.0, seed));

                if (dotVec(drc, nrm) < 0.0) {
                    drc *= -1.0;
                }
            }

            VecPD3 next;
            next.pos = base.vec.pos + nrm * delta;;
            next.drc = (drc + nrm) * 0.5;

            BVH::Hit hit;
            if (1 || m_bvh.trace(hit, next, 0.0, SP_INFINITY) == false) {
                col = base.pmat->amb;
                col.a = base.pmat->amb.a;
            }

            return col;
        }

    };
}
#endif
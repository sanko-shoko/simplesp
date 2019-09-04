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

        Mem1<const Node*> getNode(const int level) const {
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

        bool trace(Index *idx, SP_REAL *norm, Vec3 *nrm, const VecPD3 &ray, const double minv, const double maxv) const {
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
                        if (result[0] < lmaxv - SP_SMALL || m_idxs[i].acnt < acnt) {
                            lmaxv = result[0];
                            //if (dotVec(getMeshNrm(*m_idxs[i].pmesh), ray.drc) < 0.0) 
                            {
                                minid = i;
                                acnt = m_idxs[i].acnt;
                            }
                        }
                    }
                }
            }
            if (minid < 0) {
                return false;
            }

            if (idx != NULL) *idx = m_idxs[minid];
            if (norm != NULL) *norm = lmaxv;
            if (nrm != NULL) *nrm = getMeshNrm(*m_idxs[minid].pmesh);
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
            Vec4 ambrnd;
            Vec4 ltdrc[maxlt];
            Vec4 ltrnd[maxlt];
        };
        Mem2<Img> m_img;

        struct ImgCnt {
            int ambrnd;
            int ltdrc[maxlt];
            int ltrnd[maxlt];
        };
        Mem2<ImgCnt> m_imgcnt;

        Mem1<Vec3> m_lights;

        BVH m_bvh;
       
    public:
        void clear() {
            m_bvh.clear();
        }

        void set(const Mem1<Vec3> &lights) {
            SP_ASSERT(lights.size() <= maxlt);
            m_lights = lights;
            m_img.zero();
            m_imgcnt.zero();
        }

        void set(const CamParam cam, const Pose &pose, const bool pers) {
            m_cam = cam;
            m_pose = pose;
            m_pers = pers;

            m_img.resize(m_cam.dsize);
            m_img.zero();

            m_imgcnt.resize(m_cam.dsize);
            m_imgcnt.zero();
        }

        void add(const Mem1<Mesh3> &meshes, const Mem1<Material*> &pmats) {
            m_bvh.add(meshes, pmats);
        }

        void build() {
            m_bvh.build();
        }


        void update() {
            const Pose ipose = invPose(m_pose);
            const Mat mrot = getMat(ipose.rot);

            const int w = m_img.dsize[0];
            const int h = m_img.dsize[1];
//#if SP_USE_OMP
//#pragma omp parallel for
//#endif
#pragma omp parallel for
            for (int v = 0; v < h; v++) {
                for (int u = 0; u < w; u++) {
                    const Vec3 p = prjVec(invCam(m_cam, getVec2(u, v)), 1.0, m_pers);
                    trace(m_img(u, v), m_imgcnt(u, v), getVecPD3(ipose.trn, mrot * p));
                }
            }
        }

        void makeImg(Mem2<Col4> &img, const double amb, const double *lts) {
            img.resize(m_cam.dsize);
            img.zero();

            for (int i = 0; i < img.size(); i++) {
                img[i] = cast<Col4>(m_img[i].ltdrc[0]);
            }
        }

    private:


        void trace(Img &img, ImgCnt &imgcnt, const VecPD3 &ray) {
            const int maxltdif = 10;
            const int maxltrnd = 100;

            const int maxambrnd = 100;

            int minid = -1;

            // diffuse
            {
                int mincnt = maxltdif + 1;
                for (int i = 0; i < m_lights.size(); i++) {
                    if (imgcnt.ltdrc[i] < mincnt) {
                        minid = i;
                        mincnt = imgcnt.ltdrc[i];
                    }
                }
                if (minid >= 0) {
                    imgcnt.ltdrc[minid]++;
                }
            }

            if (minid < 0) {

            }
            else {
                trace_ltdrc(img, imgcnt, ray, minid, 0);

            }

        }

        bool trace_ltdrc(Img &img, ImgCnt &imgcnt, const VecPD3 &ray, const int lid, const int level) {
            Vec4 &v = img.ltdrc[lid];
            int &c = imgcnt.ltdrc[lid];

            const Vec3 lpos = m_lights[lid];
            const SP_REAL delta = 0.001;

            if (level == 0) {
                v = getVec4(0.0, 0.0, 0.0, 0.0);

                BVH::Index index[2];
                SP_REAL norm[2];
                Vec3 nrm[2];
                if (m_bvh.trace(&index[0], &norm[0], &nrm[0], ray, 0, 10000.0) == true) {
                    //v = getVec4(0.2, 0.2, 0.2, 0.2);
                    VecPD3 next;
                    next.pos = ray.pos + ray.drc * norm[0];
                    next.drc = unitVec(lpos - next.pos);

                    const SP_REAL d = dotVec(nrm[0], next.drc);
                    //if (d > 0.0) {
                    //    Vec4 vv = cast<Vec4>(index[0].pmat->dif) * d;
                    //    v = vv;
                    //}
                    if (d > 0.0 && m_bvh.trace(&index[1], &norm[1], &nrm[1], next, delta, 10000.0) == false) {
                        Vec4 vv = cast<Vec4>(index[0].pmat->dif) * d;
                        v = vv;
                    }
                }
                else {

                }
                c++;
            }
            else {

            }
            return true;
        }

        bool traceRnd(Vec4 &amb, const VecPD3 &ray, const int level) {

        }
    };
}
#endif
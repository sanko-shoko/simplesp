//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RAY_H__
#define __SP_RAY_H__

#include "spcore/spcore.h"

namespace sp {

    SP_CPUFUNC bool traceMesh(SP_REAL *result, const Mesh3 &mesh, const VecPD3 &ray, const double minv, const double maxv) {

        //{
        //    const Vec3 base = mesh.pos[0] - ray.pos;
        //    const Vec3 A = mesh.pos[1] - mesh.pos[0];
        //    const Vec3 B = mesh.pos[2] - mesh.pos[0];
        //    SP_REAL mat[3 * 3] = { ray.drc.x, -A.x, -B.x, ray.drc.y, -A.y, -B.y, ray.drc.z, -A.z, -B.z };
        //    SP_REAL inv[3 * 3];
        //    if (invMat33(inv, mat) == false) return false;

        //    SP_REAL val[3] = { base.x, base.y, base.z };
        //    mulMat(result, 3, 1, inv, 3, 3, val, 3, 1);
        //    if (result[0] < minv || result[0] > maxv) return false;
        //    if (result[1] < -SP_SMALL || result[2] < -SP_SMALL || result[1] + result[2] > 1.0 + SP_SMALL) return false;
        //}
        {
            const Vec3 A = mesh.pos[1] - mesh.pos[0];
            const Vec3 B = mesh.pos[2] - mesh.pos[0];
            SP_REAL mat[3 * 3] = { ray.drc.x, -A.x, -B.x, ray.drc.y, -A.y, -B.y, ray.drc.z, -A.z, -B.z };
            const double v0 = mat[0 * 3 + 0] * (mat[1 * 3 + 1] * mat[2 * 3 + 2] - mat[2 * 3 + 1] * mat[1 * 3 + 2]);
            const double v1 = mat[0 * 3 + 1] * (mat[1 * 3 + 0] * mat[2 * 3 + 2] - mat[2 * 3 + 0] * mat[1 * 3 + 2]);
            const double v2 = mat[0 * 3 + 2] * (mat[1 * 3 + 0] * mat[2 * 3 + 1] - mat[2 * 3 + 0] * mat[1 * 3 + 1]);
            const double det = v0 - v1 + v2;
            if (fabs(det) < SP_SMALL) return false;

            const double mx = (mesh.pos[0].x - ray.pos.x) / det;
            const double my = (mesh.pos[0].y - ray.pos.y) / det;
            const double mz = (mesh.pos[0].z - ray.pos.z) / det;
            result[1] =
                - (mat[1 * 3 + 0] * mat[2 * 3 + 2] - mat[1 * 3 + 2] * mat[2 * 3 + 0]) * mx
                + (mat[0 * 3 + 0] * mat[2 * 3 + 2] - mat[0 * 3 + 2] * mat[2 * 3 + 0]) * my
                - (mat[0 * 3 + 0] * mat[1 * 3 + 2] - mat[0 * 3 + 2] * mat[1 * 3 + 0]) * mz;
            if (result[1] < -SP_SMALL) return false;

            result[2] =
                + (mat[1 * 3 + 0] * mat[2 * 3 + 1] - mat[1 * 3 + 1] * mat[2 * 3 + 0]) * mx
                - (mat[0 * 3 + 0] * mat[2 * 3 + 1] - mat[0 * 3 + 1] * mat[2 * 3 + 0]) * my
                + (mat[0 * 3 + 0] * mat[1 * 3 + 1] - mat[0 * 3 + 1] * mat[1 * 3 + 0]) * mz;
            if (result[2] < -SP_SMALL) return false;
            if (result[1] + result[2] > 1.0 + SP_SMALL) return false;

            result[0] =
                + (mat[1 * 3 + 1] * mat[2 * 3 + 2] - mat[1 * 3 + 2] * mat[2 * 3 + 1]) * mx
                - (mat[0 * 3 + 1] * mat[2 * 3 + 2] - mat[0 * 3 + 2] * mat[2 * 3 + 1]) * my
                + (mat[0 * 3 + 1] * mat[1 * 3 + 2] - mat[0 * 3 + 2] * mat[1 * 3 + 1]) * mz;
            if (result[0] < minv || result[0] > maxv) return false;
        }
        return true;
    }


    SP_CPUFUNC bool checkHit(const Box3 &box, const VecPD3 &ray, const double minv, const double maxv) {
        double n = minv;
        double f = maxv;
        for (int i = 0; i < 3; i++) {
            const double v = acsv(ray.drc, i);
            if (fabs(v) < SP_SMALL) {
                if (acsv(ray.pos, i) < acsv(box.pos[0], i)) return false;
                if (acsv(ray.pos, i) > acsv(box.pos[1], i)) return false;
                continue;
            }

            const Vec3 &a = (v >= 0.0) ? box.pos[0] : box.pos[1];
            const Vec3 &b = (v >= 0.0) ? box.pos[1] : box.pos[0];

            const double tn = (acsv(a, i) - acsv(ray.pos, i)) / v;
            const double tf = (acsv(b, i) - acsv(ray.pos, i)) / v;
            n = maxval(n, tn);
            f = minval(f, tf);
            if (f < n) {
                return false;
            }
        }
        return true;
    }

    //--------------------------------------------------------------------------------
    // light
    //--------------------------------------------------------------------------------
  
    class Light {
    public:
        Col4f col;
        float val;
        float sdw;

        Light() {
            col = getCol4f(1.0, 1.0, 1.0, 1.0);
            val = 0.5f;
            sdw = 1.0f;
        }
        Light(const Light &light) {
            *this = light;
        }
        Light(const Col4f &col, const double val, const double sdw) {
            this->col = col;
            this->val = val;
            this->sdw = sdw;
        }

        Light& operator = (const Light &light) {
            col = light.col;
            val = light.val;
            sdw = light.sdw;
            return *this;
        }
    };

    class PntLight : public Light{
    public:
        Vec3 pnt;

        PntLight() : Light(){
            pnt = getVec3(1.0, 0.0, 0.0);
        }
        PntLight(const PntLight &light) {
            *this = light;
        }
        PntLight(const Col4f &col, const double val, const double sdw, const Vec3 &pnt) {
            this->col = col;
            this->val = val;
            this->sdw = sdw;
            this->pnt = pnt;
        }
        PntLight& operator = (const PntLight &light) {
            col = light.col;
            val = light.val;
            sdw = light.sdw;
            pnt = light.pnt;
            return *this;
        }
    };


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

        MemP<Mem1<Mesh3>> m_buffs;

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
            m_buffs.clear();
            m_nodes.clear();
            m_idxs.clear();
        }
        
        void addModel(const Mem1<Mesh3> &meshes, const Mem1<Material*> &pmats = Mem1<Material*>()) {

            Mem1<Mesh3> &buff = *m_buffs.malloc();
            buff = meshes;

            const int offset = m_idxs.size();
            const bool pm = (pmats.size() > 0) ? true : false;

            m_idxs.extend(meshes.size());
            for (int i = 0; i < meshes.size(); i++) {
                m_idxs[offset + i].acnt = m_acnt;
                m_idxs[offset + i].pmesh = &buff[i];
                m_idxs[offset + i].pmat = (pm == true) ? pmats[i] : NULL;
            }
            m_acnt++;
        }

        void addModel(const Mem1<Mesh3*> &pmeshes, const Mem1<Material*> &pmats = Mem1<Material*>()) {

            const int offset = m_idxs.size();
            const bool pm = (pmats.size() > 0) ? true : false;

            m_idxs.extend(pmeshes.size());
            for (int i = 0; i < pmeshes.size(); i++) {
                m_idxs[offset + i].acnt = m_acnt;
                m_idxs[offset + i].pmesh = pmeshes[i];
                m_idxs[offset + i].pmat = (pm == true) ? pmats[i] : NULL;
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
            SP_REAL norm;
        };

        bool trace(Hit &hit, const VecPD3 &ray, const double minv, const double maxv) const {
            int minid = -1;

            int stack = 0;
            const int MAXN = 100;
            const Node* que[MAXN];
            que[stack++] = &m_nodes[0];

            double lmaxv = maxv;
            int acnt = -1;

            while (stack > 0) {
                const Node *n = que[--stack];
                if (checkHit(n->box, ray, minv, lmaxv) == false) {
                    continue;
                }
                if (n->n0 != NULL && n->n1 != NULL && stack < MAXN - 2) {
                    que[stack++] = n->n0;
                    que[stack++] = n->n1;
                    continue;
                }
                {
                    const int i = n->base;
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
            hit.norm = lmaxv;
            hit.vec.pos = ray.pos + ray.drc * lmaxv;
            hit.vec.drc = getMeshNrm(*m_idxs[minid].pmesh);

            return true;
        }
    };

    
    //--------------------------------------------------------------------------------
    // path trace
    //--------------------------------------------------------------------------------

    class PathTrace {
    private:
        static const int maxlt = 4;

        struct Img {
            Col4f ambv;
            Col4f difv[maxlt];

            Col4f ambs;
            Col4f difs[maxlt];

            float msk;
        };
        struct Cnt {
            int amb;
            int dif[maxlt];
            int msk;
        };
        struct Lim {
            int amb;
            int dif;
            int msk;
        };

        BVH m_bvh;

    public:

        Pose m_pose;
        CamParam m_cam;

        Mem2<Img> m_img;
        Cnt m_cnt;
        Lim m_lim;

        Light m_ambient;
        Mem1<PntLight> m_plights;

        int m_upcnt;

    public:

        PathTrace() {
            memset(&m_lim, 0, sizeof(Cnt));
            reset();
        }

        void clear() {
            m_bvh.clear();
            reset();
        }

        void reset() {
            m_img.zero();
            memset(&m_cnt, 0, sizeof(Cnt));
            m_lim.msk = 100;
        }

        float prog() {
            int cnt = 0;
            int lim = 0;
            cnt += m_cnt.amb;
            lim += m_lim.amb;
            for (int i = 0; i < m_plights.size(); i++) {
                cnt += m_cnt.dif[i];
                lim += m_lim.dif;
            }
            if (cnt > 0) {
                return (cnt == lim) ? 1.0f : static_cast<float>(cnt) / (lim);
            }
            else {
                return (m_cnt.msk == m_lim.msk) ? 1.0f : static_cast<float>(m_cnt.msk) / (m_lim.msk);
            }
        }

        void setCam(const CamParam cam) {
            if (m_cam != cam) {
                m_cam = cam;
                m_img.resize(m_cam.dsize);
                reset();
            }
        }
        const CamParam& getCam() const {
            return m_cam;
        }

        void setAmbient(const Light &light, const int lim = 100) {
            m_ambient = light;
            if (m_lim.amb != lim) {
                m_cnt.amb = 0;
                m_cnt.msk = 0;
            }
            m_lim.amb = lim;
        }
        const Light& getAmbient() {
            return m_ambient;
        }

        void setPntLights(const Mem1<PntLight> &lights, const int lim = 30) {
            SP_ASSERT(lights.size() <= maxlt);

            bool changed = false;
            for (int i = 0; i < lights.size(); i++) {
                if (i >= m_plights.size() || lights[i].pnt != m_plights[i].pnt) {
                    m_cnt.dif[i] = 0;
                    m_cnt.msk = 0;
                    changed = true;
                }
            }
            if (changed == true || m_plights.size() != lights.size()) {
                m_upcnt = 0;
            }
            m_plights = lights;
            m_lim.dif = lim;
        }

        const Mem1<PntLight>& setPntLights() const {
            return m_plights;
        }

        void setPose(const Pose &pose) {
            if (m_pose != pose) {
                m_pose = pose;
                reset();
            }
        }
        const Pose& getPose() const {
            return m_pose;
        }

        void addModel(const Mem1<Mesh3> &meshes, const Mem1<Material*> &pmats) {
            m_bvh.addModel(meshes, pmats);
        }
        void addModel(const Mem1<Mesh3*> &pmeshes, const Mem1<Material*> &pmats) {
            m_bvh.addModel(pmeshes, pmats);
        }

        void build() {
            m_bvh.build();
        }

        void update() {
            if (prog() == 1.0f) return;

            const Pose ipose = invPose(m_pose);
            const Mat irmat = getMat(ipose.rot);

            const int w = m_cam.dsize[0];
            const int h = m_cam.dsize[1];
#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int v = 0; v < h; v++) {
                for (int u = 0; u < w; u++) {
                    if (m_cnt.msk >= m_lim.msk && m_img(u, v).msk == SP_RCAST(0.0)) continue;

                    Vec2 pix = getVec2(u, v);
                    const unsigned int seed = m_upcnt;
                    if (m_upcnt > 0) pix += randuVec2(0.5, 0.5, seed);

                    Vec3 pos, drc;
                    const Vec2 p2d = invCam(m_cam, pix);
                    if (m_cam.type == CamParam_Pers) {
                        pos = ipose.trn;
                        drc = unitVec(prjVec(invCam(m_cam, pix), 1.0, true));
                    }
                    else {
                        pos = ipose.trn + irmat * getVec3(p2d.x, p2d.y, 0.0);
                        drc = getVec3(0.0, 0.0, 1.0);
                    }

                    trace(m_img(u, v), m_cnt, m_lim, getVecPD3(pos, irmat * drc), m_plights);
                }
            }
            {
                m_cnt.amb = minval(m_lim.amb, m_cnt.amb + 1);
                for (int i = 0; i < m_plights.size(); i++) {
                    m_cnt.dif[i] = minval(m_lim.dif, m_cnt.dif[i] + 1);
                }
                m_cnt.msk = minval(m_lim.msk, m_cnt.msk + 1);
            }
            m_upcnt++;
        }

        void render(Mem2<Col4> &img, const Col4f &bgcol = getCol4f(1.0, 1.0, 1.0, 1.0), const double vig = 0.0) {
            img.resize(m_cam.dsize);

            const Vec2 cent = getVec2(img.dsize[0] - 1, img.dsize[1] - 1) * 0.5;

            auto blend = [](Col4f &dst, const Col4f &srcs, const Col4f &srcv, const Light &light) {
                dst.r += (srcs.r * light.sdw + srcv.r * (1.0f - light.sdw)) * light.val * light.col.r;
                dst.g += (srcs.g * light.sdw + srcv.g * (1.0f - light.sdw)) * light.val * light.col.g;
                dst.b += (srcs.b * light.sdw + srcv.b * (1.0f - light.sdw)) * light.val * light.col.b;
                dst.a += (srcs.a * light.sdw + srcv.a * (1.0f - light.sdw)) * light.val;
            };
            for (int v = 0; v < img.dsize[1]; v++) {
                for (int u = 0; u < img.dsize[0]; u++) {
                    
                    Img &im = m_img(u, v);

                    Col4f col = getCol4f(0.0, 0.0, 0.0, 0.0);
                    float sum = 0.0f;
                    for (int l = 0; l < m_plights.size(); l++) {
                        if (m_cnt.dif[l] > 0) {
                            blend(col, im.difs[l], im.difv[l], m_plights[l]);
                            sum += m_plights[l].val;
                        }
                    }
                    {
                        if (m_cnt.amb > 0) {
                            blend(col, im.ambs, im.ambv, m_ambient);
                            sum += m_ambient.val;
                        }
                    }
                    if (sum > 0.0f) {
                        col.a /= sum;
                    }
                    else {
                        col = getCol4f(0.0, 0.0, 0.0, im.msk);
                    }

                    col = meanCol(col, im.msk, bgcol, 1.0f - im.msk);

                    Vec2 vv = getVec2(u, v) - cent;
                    vv.x /= cent.x;
                    vv.y /= cent.y;
                    const double a = sqVec(vv) * vig;
                    col = meanCol(col, 1.0 - a, getCol4f(0.0, 0.0, 0.0, 1.0), a);

                    img(u, v) = cast<Col4>(col);
                }
            }
        }

    private:

        void trace(Img &img, Cnt &cnt, Lim &lim, const VecPD3 &ray, Mem1<PntLight> &lights) {

            BVH::Hit hit;
            const bool ret = m_bvh.trace(hit, ray, 0.0, SP_INFINITY);

            if (ret == true) {
                if (cnt.amb < lim.amb) {
                    if (cnt.amb == 0) {
                        img.ambs = getCol4f(0.0, 0.0, 0.0, 0.0);
                    }
                    {
                        Col4f colv, cols;
                        trace_amb(colv, cols, ray, hit, 0, cnt.amb);
                        img.ambs = meanCol(img.ambs, cnt.amb, cols, 1.0);
                        img.ambv = meanCol(img.ambv, cnt.amb, colv, 1.0);
                    }
                }
                for (int i = 0; i < lights.size(); i++) {
                    if (cnt.dif[i] == 0) {
                        img.difs[i] = getCol4f(0.0, 0.0, 0.0, 0.0);
                    }
                    if (cnt.dif[i] < lim.dif) {
                        Col4f colv, cols;
                        trace_dif(colv, cols, ray, hit, lights[i].pnt + randgVec3(1.0, 1.0, 1.0, cnt.dif[i]) * 1.0, 0, cnt.dif[i]);
                        img.difs[i] = meanCol(img.difs[i], cnt.dif[i], cols, 1.0);
                        img.difv[i] = meanCol(img.difv[i], cnt.dif[i], colv, 1.0);
                    }
                }
            }
            if (cnt.msk < lim.msk) {
                if (cnt.msk == 0) {
                    img.msk = 0.0f;
                }
                img.msk = (img.msk * cnt.msk + (ret ? 1.0f : 0.0f)) / (cnt.msk + 1.0f);
            }
        }

        void trace_dif(Col4f &colv, Col4f &cols, const VecPD3 &ray, const BVH::Hit base, const Vec3 lpos, const int level, const unsigned int seed) {
            const SP_REAL delta = 0.001;

            const Vec3 nrm = getMeshNrm(*base.pmesh);

            VecPD3 next;
            next.pos = base.vec.pos + getMeshNrm(*base.pmesh) * delta;
            next.drc = unitVec(lpos - base.vec.pos);

            const SP_REAL d = dotVec(base.vec.drc, next.drc);
            BVH::Hit hit;
            if (d > 0.0 && m_bvh.trace(hit, next, 0.0, SP_INFINITY) == false) {
                cols = base.pmat->dif * d;
                cols.a = base.pmat->dif.a;
            }
            else {
                cols = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
            if (d > 0.0) {
                colv = base.pmat->dif * d;
                colv.a = base.pmat->dif.a;
            }
            else {
                colv = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
        }


        void trace_amb(Col4f &colv, Col4f &cols, const VecPD3 &ray, const BVH::Hit base, const int level, const unsigned int seed) {
            const SP_REAL delta = 0.001;

            const Vec3 nrm = getMeshNrm(*base.pmesh);

            Vec3 drc;
            {
                drc = unitVec(nrm * 0.5 + randuVec3(1.0, 1.0, 1.0, seed));

                if (dotVec(drc, nrm) < 0.0) {
                    drc *= -1.0;
                }
            }

            VecPD3 next;
            next.pos = base.vec.pos + nrm * delta;;
            next.drc = drc;// (drc + nrm) * 0.5;

            BVH::Hit hit;
            if (m_bvh.trace(hit, next, 0.0, SP_INFINITY) == false) {
                cols = base.pmat->amb;
            }
            else {
                cols = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
            {
                colv = base.pmat->amb;
            }
        }

    };
}
#endif
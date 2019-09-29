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

    SP_CPUFUNC bool tracePlane(SP_REAL *result, const VecPD3 &plane, const VecPD3 &ray, const double minv, const double maxv) {

        const Vec3 p = dotVec(plane.drc, plane.pos - ray.pos) * plane.drc;
        const Vec3 n = unitVec(p);
        if (dotVec(n, ray.drc) > 0.0 && normVec(p) > 0.0) {
            const double s = normVec(p) / dotVec(ray.drc, n);
            if (s < minv || s > maxv) return false;
            result[0] = s;
            return true;
        }
        
        return false;
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

            const Mesh3 *pmesh;
            const Material *pmat;
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
            auto initn = [&](const int level, const int base, const int size) -> Node* {
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

            Mem1<Mem1<Node*>> tnodes;
            {
                tnodes.reserve(20);

                const int lim = 5000;
                const int mul = idxs.size() / lim;
                const int level = (mul > 0) ? round(log2(mul)) : 0;

                for (int ni = 0; ni < m_nodes.size(); ni++) {
                    Node& n = m_nodes[ni];
                    const int di = n.size / 2;

                    if (n.level < level) {
                        n.n0 = initn(n.level + 1, n.base, di);
                        n.n1 = initn(n.level + 1, n.base + di, n.size - di);
                    }
                    else {
                        const int t = tnodes.size();
                        Mem1<Node*> &nodes = *tnodes.extend();
                        nodes.reserve(2 * n.size - 1);
                        nodes.push(&n);
                    }
                }
            }
         
            Mem1<SP_REAL> map(idxs.size());

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int i = 0; i < tnodes.size(); i++) {
                for (int ni = 0; ni < tnodes[i].size(); ni++) {
                    Node& n = *tnodes[i][ni];
                    if (n.size == 1) continue;

                    int di = 0;
                    int da = 0;

                    double mina = SP_INFINITY;
                    for (int a = 0; a < 3; a++) {
                        sorti(n, a);
                        Box3 bl = nullBox3();
                        Box3 br = nullBox3();

                        for (int i = 1; i < n.size; i++) {
                            const int l = i;
                            bl = orBox(bl, *idxs[n.base + l - 1].pmesh);
                            map[n.base + l] = 0;
                            map[n.base + l] += getBoxArea(bl) * i;
                        }
                        for (int i = 1; i < n.size; i++) {
                            const int r = n.size - i;
                            br = orBox(br, *idxs[n.base + r].pmesh);
                            map[n.base + r] += getBoxArea(br) * i;
                        }

                        for (int i = 1; i < n.size; i++) {
                            //const double area = 1.0 + (map(i, 0) + map(i, 1)) / getBoxArea(n.box);
                            if (map[n.base + i] < mina) {
                                mina = map[n.base + i];
                                di = i;
                                da = a;
                            }
                        }
                    }
                    sorti(n, da);
#if SP_USE_OMP
#pragma omp critical
#endif
                    {
                        n.n0 = initn(n.level + 1, n.base, di);
                        n.n1 = initn(n.level + 1, n.base + di, n.size - di);
                    }
                    tnodes[i].push(n.n0);
                    tnodes[i].push(n.n1);
                }
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
    public:

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
                memcpy(this, &light, sizeof(Light));
                return *this;
            }
        };

        class PntLight : public Light {
        public:
            Vec3 pos;

            PntLight() : Light() {
                pos = getVec3(0.0, 0.0, 0.0);
            }
            PntLight(const PntLight &light) {
                *this = light;
            }
            PntLight(const Col4f &col, const double val, const double sdw, const Vec3 &pos) : Light(col, val, sdw) {
                this->pos = pos;
            }
            PntLight& operator = (const PntLight &light) {
                memcpy(this, &light, sizeof(PntLight));
                return *this;
            }
        };
        class Plane {
        public:
            VecPD3 vec;
            Material mat;
        };


    private:
        static const int maxlt = 4;

        struct Data {
            Col4f val;
            Col4f sdw;
        };
        struct Img {
            Data amb;
            Data dif[maxlt];
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

        CamParam m_cam;
        Pose m_pose;

        Mem2<Img> m_img;
        Cnt m_cnt;
        Lim m_lim;

        Mem1<PntLight> m_plights;

        Mem1<Light> m_ambient;
        Mem1<Plane> m_ground;

        int m_upcnt;

    public:

        PathTrace() {
            m_upcnt = 0;

            m_cam = getCamParam(640, 480);
            m_pose = sp::getPose(getVec3(0.0, 0.0, 1000.0));

            m_lim.amb = 0;
            m_lim.dif = 0;
            m_lim.msk = 64;

            reset();
        }

        void clear() {
            m_bvh.clear();
            reset();
        }

        void reset() {
            m_img.resize(m_cam.dsize);
            m_img.zero();
            memset(&m_cnt, 0, sizeof(Cnt));
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
            cnt += m_cnt.msk;
            lim += m_lim.msk;
         
            return (cnt == lim) ? 1.0f : static_cast<float>(cnt) / (lim + 1);
        }

        void setCam(const CamParam cam) {
            if (m_cam != cam) {
                m_cam = cam;
                reset();
            }
        }
        const CamParam& getCam() const {
            return m_cam;
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

        void setAmbient(const Light *light, const int lim = 100) {
            if (light == NULL) {
                m_ambient.clear();
                m_cnt.amb = 0;
                m_lim.amb = 0;
            }
            else {
                m_ambient.resize(1);
                m_ambient[0] = *light;
                m_lim.amb = lim;
            }
        }
        const Light* getAmbient() {
            return (m_ambient.size() > 0) ? &m_ambient[0] : NULL;
        }

        void setGround(const Plane *ground) {
            if (ground == NULL) {
                if (m_ground.size() > 0) {
                    reset();
                }
                m_ground.clear();
            }
            else {
                if (m_ground.size() == 0 || ground->vec != m_ground[0].vec || ground->mat != m_ground[0].mat) {
                    reset();
                }
                m_ground.resize(1);
                m_ground[0] = *ground;
            }
        }

        const Plane* getGround() {
            return (m_ground.size() > 0) ? &m_ground[0] : NULL;
        }

        void setPntLights(const Mem1<PntLight> &lights, const int lim = 30) {
            SP_ASSERT(lights.size() <= maxlt);

            for (int i = 0; i < maxlt; i++) {
                if (i >= minval(m_plights.size(), lights.size()) || lights[i].pos != m_plights[i].pos) {
                    m_cnt.dif[i] = 0;
                }
            }
            if (lights.size() > 0) {
                m_lim.dif = lim;
            }
            else {
                m_lim.dif = 0;
            }
            m_plights = lights;
        }

        const Mem1<PntLight>& setPntLights() const {
            return m_plights;
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

                    calc(m_img(u, v), m_cnt, m_lim, getVecPD3(pos, irmat * drc), m_plights);
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

        void render(Mem2<Col4> &img, const Col4f &bgcol = getCol4f(1.0, 1.0, 1.0, 1.0)) {
            img.resize(m_cam.dsize);

            auto blend = [](Col4f &dst, const Data &data, const Light &light) {
                dst.r += (data.sdw.r * light.sdw + data.val.r * (1.0f - light.sdw)) * light.val * light.col.r;
                dst.g += (data.sdw.g * light.sdw + data.val.g * (1.0f - light.sdw)) * light.val * light.col.g;
                dst.b += (data.sdw.b * light.sdw + data.val.b * (1.0f - light.sdw)) * light.val * light.col.b;
                dst.a += (data.sdw.a * light.sdw + data.val.a * (1.0f - light.sdw)) * light.val;
            };
#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int v = 0; v < img.dsize[1]; v++) {
                for (int u = 0; u < img.dsize[0]; u++) {

                    Img &im = m_img(u, v);

                    Col4f col = getCol4f(0.0, 0.0, 0.0, 0.0);
                    float sum = 0.0f;
                    for (int l = 0; l < m_plights.size(); l++) {
                        if (m_cnt.dif[l] > 0) {
                            blend(col, im.dif[l], m_plights[l]);
                            sum += m_plights[l].val;
                        }
                    }
                    if (m_ambient.size() > 0) {
                        if (m_cnt.amb > 0) {
                            blend(col, im.amb, m_ambient[0]);
                            sum += m_ambient[0].val;
                        }
                    }
                    if (sum > 0.0f) {
                        col.a /= sum;
                    }
                    else {
                        col = getCol4f(0.0, 0.0, 0.0, im.msk);
                    }

                    col = meanCol(col, im.msk, bgcol, 1.0f - im.msk);

                    img(u, v) = cast<Col4>(col);
                }
            }
        }

    private:

        bool trace(BVH::Hit &hit, const VecPD3 &ray, const double minv, const double maxv){
        
            bool ret = false;
            double norm = maxv;
            {
                ret = m_bvh.trace(hit, ray, minv, norm);
                if (ret == true) {
                    norm = hit.norm;
                }
            }
            if (m_ground.size() > 0) {
                SP_REAL result[3];
                if (tracePlane(result, m_ground[0].vec, ray, 0, norm)) {
                    hit.acnt = 0;
                    hit.norm = result[0];
                    hit.pmat = &m_ground[0].mat;
                    hit.pmesh = NULL;
                    hit.vec.pos = ray.pos + ray.drc * result[0];
                    hit.vec.drc = m_ground[0].vec.drc;
                    ret = true;
                }
            }

            return ret;
        }

        void calc(Img &img, Cnt &cnt, Lim &lim, const VecPD3 &ray, Mem1<PntLight> &lights) {

            BVH::Hit hit;
            const bool ret = trace(hit, ray, 0.0, SP_INFINITY);

            if (ret == true) {
                if (cnt.amb < lim.amb) {
                    if (cnt.amb == 0) {
                        img.amb.sdw = getCol4f(0.0, 0.0, 0.0, 0.0);
                        img.amb.val = getCol4f(0.0, 0.0, 0.0, 0.0);
                    }
                    {
                        Data data;
                        calc_amb(data, ray, hit, 0, cnt.amb);
                        img.amb.sdw = meanCol(img.amb.sdw, cnt.amb, data.sdw, 1.0);
                        img.amb.val = meanCol(img.amb.val, cnt.amb, data.val, 1.0);
                    }
                }
                for (int i = 0; i < lights.size(); i++) {
                    if (cnt.dif[i] == 0) {
                        img.dif[i].sdw = getCol4f(0.0, 0.0, 0.0, 0.0);
                        img.dif[i].val = getCol4f(0.0, 0.0, 0.0, 0.0);
                    }
                    if (cnt.dif[i] < lim.dif) {
                        Data data;
                        calc_dif(data, ray, hit, lights[i].pos + randgVec3(1.0, 1.0, 1.0, cnt.dif[i]) * 1.0, 0, cnt.dif[i]);
                        img.dif[i].sdw = meanCol(img.dif[i].sdw, cnt.dif[i], data.sdw, 1.0);
                        img.dif[i].val = meanCol(img.dif[i].val, cnt.dif[i], data.val, 1.0);
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

        void calc_dif(Data &data, const VecPD3 &ray, const BVH::Hit base, const Vec3 lpos, const int level, const unsigned int seed) {
            const SP_REAL delta = 0.001;

            const Vec3 nrm = base.vec.drc;

            VecPD3 next;
            next.pos = base.vec.pos + base.vec.drc * delta;
            next.drc = unitVec(lpos - base.vec.pos);

            const SP_REAL d = dotVec(base.vec.drc, next.drc);
            BVH::Hit hit;
            if (d > 0.0 && trace(hit, next, 0.0, SP_INFINITY) == false) {
                data.sdw = base.pmat->dif * d;
                data.sdw.a = base.pmat->dif.a;
            }
            else {
                data.sdw = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
            if (d > 0.0) {
                data.val = base.pmat->dif * d;
                data.val.a = base.pmat->dif.a;
            }
            else {
                data.val = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
        }


        void calc_amb(Data &data, const VecPD3 &ray, const BVH::Hit base, const int level, const unsigned int seed) {
            const SP_REAL delta = 0.001;

            const Vec3 nrm = base.vec.drc;

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
            if (trace(hit, next, 0.0, SP_INFINITY) == false) {
                data.sdw = base.pmat->amb;
            }
            else {
                data.sdw = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
            {
                data.val = base.pmat->amb;
            }
        }

    };
}
#endif
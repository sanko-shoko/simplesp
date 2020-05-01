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
            if (result[1] > 1.0 + SP_SMALL) return false;

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
        if (dotVec(n, ray.drc) > 0.0 && normVec(p) > 0.0)
        {
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
        //for (int i = 0; i < 3; i++) {
        //    const double v = acsv(ray.drc, i);
        //    if (fabs(v) < SP_SMALL) {
        //        if (acsv(ray.pos, i) < acsv(box.pos[0], i)) return false;
        //        if (acsv(ray.pos, i) > acsv(box.pos[1], i)) return false;
        //        continue;
        //    }

        //    const Vec3 &a = (v >= 0.0) ? box.pos[0] : box.pos[1];
        //    const Vec3 &b = (v >= 0.0) ? box.pos[1] : box.pos[0];

        //    const double tn = (acsv(a, i) - acsv(ray.pos, i)) / v;
        //    const double tf = (acsv(b, i) - acsv(ray.pos, i)) / v;
        //    n = maxVal(n, tn);
        //    f = minVal(f, tf);
        //    if (f < n) {
        //        return false;
        //    }
        //}

        if (fabs(ray.drc.x) < SP_SMALL) {
            if (ray.pos.x < box.pos[0].x) return false;
            if (ray.pos.x > box.pos[1].x) return false;
        }
        else {
            const Vec3 &a = (ray.drc.x >= 0.0) ? box.pos[0] : box.pos[1];
            const Vec3 &b = (ray.drc.x >= 0.0) ? box.pos[1] : box.pos[0];

            const double tn = (a.x - ray.pos.x) / ray.drc.x;
            const double tf = (b.x - ray.pos.x) / ray.drc.x;
            n = maxVal(n, tn);
            f = minVal(f, tf);
            if (f < n) return false;
        }

        if (fabs(ray.drc.y) < SP_SMALL) {
            if (ray.pos.y < box.pos[0].y) return false;
            if (ray.pos.y > box.pos[1].y) return false;
        }
        else {
            const Vec3 &a = (ray.drc.y >= 0.0) ? box.pos[0] : box.pos[1];
            const Vec3 &b = (ray.drc.y >= 0.0) ? box.pos[1] : box.pos[0];

            const double tn = (a.y - ray.pos.y) / ray.drc.y;
            const double tf = (b.y - ray.pos.y) / ray.drc.y;
            n = maxVal(n, tn);
            f = minVal(f, tf);
            if (f < n) return false;
        }

        if (fabs(ray.drc.z) < SP_SMALL) {
            if (ray.pos.z < box.pos[0].z) return false;
            if (ray.pos.z > box.pos[1].z) return false;
        }
        else {
            const Vec3 &a = (ray.drc.z >= 0.0) ? box.pos[0] : box.pos[1];
            const Vec3 &b = (ray.drc.z >= 0.0) ? box.pos[1] : box.pos[0];

            const double tn = (a.z - ray.pos.z) / ray.drc.z;
            const double tf = (b.z - ray.pos.z) / ray.drc.z;
            n = maxVal(n, tn);
            f = minVal(f, tf);
            if (f < n) return false;
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
            int objid;

            const Mesh3 *pmesh;
            const Material *pmat;
        };

    private:
        int m_objid;

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
            m_objid = 0;
            m_buffs.clear();
            m_nodes.clear();
            m_idxs.clear();
        }


        void addModel(const Mem1<Mesh3> &meshes, const Mem1<Material*> &pmats = Mem1<Material*>()) {
            //SP_ASSERT(m_acnt < 100);

            Mem1<Mesh3> &buff = *m_buffs.malloc();
            buff = meshes;

            const int offset = m_idxs.size();

            m_idxs.extend(meshes.size());
            for (int i = 0; i < meshes.size(); i++) {
                m_idxs[offset + i].objid = m_objid;
                m_idxs[offset + i].pmesh = &buff[i];
                m_idxs[offset + i].pmat = (pmats.size() > 0) ? pmats[i] : NULL;
            }
            m_objid++;
        }

        void addModel(const Mem1<Mesh3*> &pmeshes, const Mem1<Material*> &pmats = Mem1<Material*>()) {

            const int offset = m_idxs.size();

            m_idxs.extend(pmeshes.size());
            for (int i = 0; i < pmeshes.size(); i++) {
                m_idxs[offset + i].objid = m_objid;
                m_idxs[offset + i].pmesh = pmeshes[i];
                m_idxs[offset + i].pmat = (pmats.size() > 0) ? pmats[i] : NULL;
            }
            m_objid++;
        }

        void build() {

            // mesh centers (for sort)
            struct IndexEx : public Index {
                Vec3 cent;
            };

            if (m_idxs.size() == 0) return;

            Mem1<IndexEx> idxs(m_idxs.size());
            for (int i = 0; i < idxs.size(); i++) {
                idxs[i].objid = m_idxs[i].objid;
                idxs[i].pmesh = m_idxs[i].pmesh;
                idxs[i].pmat = m_idxs[i].pmat;
                idxs[i].cent = getMeshCent(*m_idxs[i].pmesh);
            }

            m_nodes.clear();
            m_nodes.reserve(2 * idxs.size() - 1);

            auto initn = [&](const int level, const int base, const int size) -> Node* {
                Node *n = NULL;
#if SP_USE_OMP
#pragma omp critical
#endif
                {
                    n = m_nodes.extend();
                }
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

            static Mem1<SP_REAL> buff;
            buff.resize(idxs.size());
            auto sorti = [&](Node &n, Mem1<IndexEx> &idxs) -> int{
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
                
                int di = 0;
                int da = 0;

                double mina = SP_INFINITY;
                for (int a = 0; a < 3; a++) {
                    sort(&idxs[n.base], n.size, cmp[a]);
                    Box3 bl = nullBox3();
                    Box3 br = nullBox3();

                    for (int i = 1; i < n.size; i++) {
                        const int l = i;
                        bl = orBox(bl, *idxs[n.base + l - 1].pmesh);
                        buff[n.base + l] = 0;
                        buff[n.base + l] += getBoxArea(bl) * i;
                    }
                    for (int i = 1; i < n.size; i++) {
                        const int r = n.size - i;
                        br = orBox(br, *idxs[n.base + r].pmesh);
                        buff[n.base + r] += getBoxArea(br) * i;
                    }

                    for (int i = 1; i < n.size; i++) {
                        //const double area = 1.0 + (buff(i, 0) + buff(i, 1)) / getBoxArea(n.box);
                        if (buff[n.base + i] < mina) {
                            mina = buff[n.base + i];
                            di = i;
                            da = a;
                        }
                    }
                }
                if (da != 2) {
                    sort(&idxs[n.base], n.size, cmp[da]);
                }
                return di;
            };

            initn(0, 0, idxs.size());

            Mem1<Mem1<Node*> > tnodes;
            if (idxs.size() > 1000) {
                const int level = 5;

                tnodes.reserve(256);
                for (int ni = 0; ni < m_nodes.size(); ni++) {
                    Node& n = m_nodes[ni];
                    if (n.size == 1) continue;

                    if (n.level < level) {
                        const int di = sorti(n, idxs);
                        n.n0 = initn(n.level + 1, n.base, di);
                        n.n1 = initn(n.level + 1, n.base + di, n.size - di);
                    }
                    else {
                        Mem1<Node*> &nodes = *tnodes.extend();
                        nodes.reserve(2 * n.size - 1);
                        nodes.push(&n);
                    }
                }
            }
            else {
                Mem1<Node*> &nodes = *tnodes.extend();
                nodes.push(&m_nodes[0]);
            }

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int i = 0; i < tnodes.size(); i++) {
                for (int ni = 0; ni < tnodes[i].size(); ni++) {
                    Node& n = *tnodes[i][ni];
                    if (n.size == 1) continue;

                    const int di = sorti(n, idxs);
                    
                    n.n0 = initn(n.level + 1, n.base, di);
                    n.n1 = initn(n.level + 1, n.base + di, n.size - di);
                    tnodes[i].push(n.n0);
                    tnodes[i].push(n.n1);
                }
            }

            for (int i = 0; i < m_idxs.size(); i++) {
                m_idxs[i] = idxs[i];
            }
        }

        struct Hit : Index {
            VecPD3 vec;
            SP_REAL norm;
        };

        bool trace(Hit &hit, const VecPD3 &ray, const double minv, const double maxv) const {
            if (m_idxs.size() == 0) return false;

            int stack = 0;
            const int MAXN = 100;
            const Node* que[MAXN];
            que[stack++] = &m_nodes[0];

            double lmaxv = maxv;
            int minid = -1;
            int objid = -1;

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

                        if (result[0] < lmaxv - SP_SMALL || m_idxs[i].objid < objid) {
                            const Vec3 nrm = getMeshNrm(*m_idxs[i].pmesh);
                            const bool f = (dotVec(nrm, ray.drc) < 0.0);

                            lmaxv = result[0];
                            minid = i;
                            objid = m_idxs[i].objid;
                        }
                    }
                }
            }

            memset(&hit, 0, sizeof(Hit));

            if (minid < 0) {
                return false;
            }

            const Index &idx = m_idxs[minid];

            hit.objid = idx.objid;
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
        class Object {
        public:
            bool valid;
            Object() {
                valid = true;
            }
        };

        //--------------------------------------------------------------------------------
        // light
        //--------------------------------------------------------------------------------

        class Light : public Object{
        public:

            Col4f col;
            float val;
            float sdw;

            Light() : Object() {
                col = getCol4f(1.0, 1.0, 1.0, 1.0);
                val = 0.5f;
                sdw = 1.0f;
            }
            Light(const Light &light) : Object() {
                *this = light;
            }
            Light(const Col4f &col, const double val, const double sdw) : Object() {
                init(col, val, sdw);
            }
            void init(const Col4f &col, const double val, const double sdw) {
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
            PntLight(const PntLight &light) : Light() {
                *this = light;
            }
            PntLight(const Col4f &col, const double val, const double sdw, const Vec3 &pos) : Light() {
                init(col, val, sdw, pos);
            }

            void init(const Col4f &col, const double val, const double sdw, const Vec3 &pos) {
                Light::init(col, val, sdw);
                this->pos = pos;
            }
            PntLight& operator = (const PntLight &light) {
                memcpy(this, &light, sizeof(PntLight));
                return *this;
            }

        };

        class Plane : public Object {
        public:
            VecPD3 vec;
            Material mat;

            void init(const VecPD3 &vec, const Material &mat) {
                this->vec = vec;
                this->mat = mat;
            }
        };

        const static int SAMPLE_UNIT = 2;

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

        struct Hit : public BVH::Hit {
            bool valid;
            bool find;
        };

        BVH m_bvh;

    public:

        CamParam m_cam;
        Pose m_pose;

        Mem2<Img> m_img;
        Cnt m_cnt;
        Lim m_lim;

        Mem2<MemA<Hit, (SAMPLE_UNIT * SAMPLE_UNIT)> > m_raymap;

        // objects
        Light m_ambient;
        Mem1<PntLight> m_plights;
        Plane m_plane;

    public:

        PathTrace() {

            m_cam = getCamParam(640, 480);
            m_pose = sp::getPose(getVec3(0.0, 0.0, 1000.0));

            m_lim.amb = 0;
            m_lim.dif = 0;
            m_lim.msk = (SAMPLE_UNIT * SAMPLE_UNIT);

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

            m_raymap.resize(m_cam.dsize);
            m_raymap.zero();
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

            return (cnt == lim) ? 1.0f : static_cast<float>(cnt) / (lim);
        }

        int upcnt() {
            int cnt = 0;
            cnt += m_cnt.amb;
            for (int i = 0; i < m_plights.size(); i++) {
                cnt += m_cnt.dif[i];
            }
            cnt += m_cnt.msk;

            return cnt;
        }

        void setCam(const CamParam &cam) {
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

        void setAmbient(const Light &light, const int lim = 100) {
            m_ambient = light;
            m_lim.amb = lim;
            if (light.valid == false) {
                m_cnt.amb = 0;
                m_lim.amb = 0;
            }
        }

        void setPlane(const Plane &plane) {
            if (plane.valid == true) {
                if (m_plane.valid == false || plane.vec != m_plane.vec || plane.mat != m_plane.mat) {
                    reset();
                }
            }
            else {
                if (m_plane.valid == true) {
                    reset();
                }
            }
            m_plane = plane;
        }

        void setPntLights(const Mem1<PntLight> &lights, const int lim = 30) {
            SP_ASSERT(lights.size() <= maxlt);

            for (int i = 0; i < maxlt; i++) {
                if (i >= minVal(m_plights.size(), lights.size()) || lights[i].pos != m_plights[i].pos) {
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

        void addModel(const Mem1<Mesh3> &meshes, const Mem1<Material*> &pmats) {
            m_bvh.addModel(meshes, pmats);
        }
        void addModel(const Mem1<Mesh3*> &pmeshes, const Mem1<Material*> &pmats) {
            m_bvh.addModel(pmeshes, pmats);
        }

        void build() {
            m_bvh.build();
        }

        bool update() {
            if (prog() == 1.0f) return false;

            t_ipose = invPose(m_pose);
            t_irmat = getMat(t_ipose.rot);

            const int w = m_cam.dsize[0];
            const int h = m_cam.dsize[1];
#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int v = 0; v < h; v++) {
                for (int u = 0; u < w; u++) {
                    if (m_cnt.msk >= m_lim.msk && m_img(u, v).msk == SP_RCAST(0.0)) continue;

                    Vec2 pix = getVec2(u, v);

                    calc(m_img(u, v), m_cnt, m_lim, m_raymap(u, v), pix);
                }
            }
            {
                m_cnt.amb = minVal(m_lim.amb, m_cnt.amb + 1);
                for (int i = 0; i < m_plights.size(); i++) {
                    m_cnt.dif[i] = minVal(m_lim.dif, m_cnt.dif[i] + 1);
                }
                m_cnt.msk = minVal(m_lim.msk, m_cnt.msk + 1);
            }
            return true;
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
                    if (m_ambient.valid == true) {
                        if (m_cnt.amb > 0) {
                            blend(col, im.amb, m_ambient);
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

                    img(u, v) = cast<Col4>(col);
                }
            }
        }

    private:

        bool trace(BVH::Hit &hit, const VecPD3 &ray, const double minv, const double maxv) {

            bool ret = false;
            double norm = maxv;
            {
                ret = m_bvh.trace(hit, ray, minv, norm);
                if (ret == true) {
                    norm = hit.norm;
                }
            }
            if (m_plane.valid == true) {
                SP_REAL result[3];
                double min = (m_cam.type == CamParam_Pers) ? 0.0 : -SP_INFINITY;
                if (tracePlane(result, m_plane.vec, ray, min, norm)) {
                    hit.objid = 0;
                    hit.norm = result[0];
                    hit.pmat = &m_plane.mat;
                    hit.pmesh = NULL;
                    hit.vec.pos = ray.pos + ray.drc * result[0];
                    hit.vec.drc = m_plane.vec.drc;
                    ret = true;
                }
            }

            return ret;
        }

        Pose t_ipose;
        Mat t_irmat;
        VecPD3 getRay(const Vec2 &pix, const int i) {

            const double dx = ((i % (SAMPLE_UNIT * SAMPLE_UNIT)) / SAMPLE_UNIT) + 1;
            const double dy = ((i % (SAMPLE_UNIT * SAMPLE_UNIT)) % SAMPLE_UNIT) + 1;
            const double delta = 1.0 / (SAMPLE_UNIT + 1);

            const Vec2 dpix = pix + getVec2(dx * delta - 0.5, dy * delta - 0.5);

            Vec3 pos, drc;
            const Vec2 p2d = invCam(m_cam, dpix);
            if (m_cam.type == CamParam_Pers) {
                pos = t_ipose.trn;
                drc = unitVec(prjVec(invCam(m_cam, dpix), 1.0, true));
            }
            else {
                pos = t_ipose.trn + t_irmat * getVec3(p2d.x, p2d.y, -1000.0 * 10);
                drc = getVec3(0.0, 0.0, 1.0);
            }

            return getVecPD3(pos, t_irmat * drc);
        }

        void calc(Img &img, Cnt &cnt, Lim &lim, MemA<Hit, (SAMPLE_UNIT * SAMPLE_UNIT)> &rays, const Vec2 &pix) {

            auto init = [&](const Vec2 &pix, const int i) -> Hit{
                Hit &phit = rays[i % (SAMPLE_UNIT * SAMPLE_UNIT)];
                if (phit.valid == false) {
                    phit.valid = true;
                    const VecPD3 ray = getRay(pix, i);
                    phit.find = trace(phit, ray, 0.0, SP_INFINITY);
                }
                return phit;
            };

            {
                if (cnt.amb < lim.amb) {
                    Hit hit = init(pix, cnt.amb);
                    const VecPD3 ray = getRay(pix, cnt.amb);

                    if (cnt.amb == 0) {
                        img.amb.sdw = getCol4f(0.0, 0.0, 0.0, 0.0);
                        img.amb.val = getCol4f(0.0, 0.0, 0.0, 0.0);
                    }
                    if (hit.find == true) {
                        Data data;
                        calc_amb(data, ray, hit, 0, cnt.amb);
                        img.amb.sdw = meanCol(img.amb.sdw, cnt.amb, data.sdw, 1.0);
                        img.amb.val = meanCol(img.amb.val, cnt.amb, data.val, 1.0);
                    }
                }
            }
            for (int i = 0; i < m_plights.size(); i++) {
                if (cnt.dif[i] < lim.dif) {
                    Hit hit = init(pix, cnt.dif[i]);
                    const VecPD3 ray = getRay(pix, cnt.dif[i]);

                    if (cnt.dif[i] == 0) {
                        img.dif[i].sdw = getCol4f(0.0, 0.0, 0.0, 0.0);
                        img.dif[i].val = getCol4f(0.0, 0.0, 0.0, 0.0);
                    }
                    if (hit.find == true) {
                        Data data;
                        calc_dif(data, ray, hit, m_plights[i].pos + randgVec3(1.0, 1.0, 1.0, cnt.dif[i]) * 1.0, 0, cnt.dif[i]);
                        img.dif[i].sdw = meanCol(img.dif[i].sdw, cnt.dif[i], data.sdw, 1.0);
                        img.dif[i].val = meanCol(img.dif[i].val, cnt.dif[i], data.val, 1.0);
                    }
                }
            }
            {
                if (cnt.msk < lim.msk) {
                    Hit hit = init(pix, cnt.msk);

                    if (cnt.msk == 0) {
                        img.msk = 0.0f;
                    }
                    img.msk = (img.msk * cnt.msk + (hit.find ? 1.0f : 0.0f)) / (cnt.msk + 1.0f);
                }
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
                const Col4f col = cast<Col4f>(base.pmat->col);
                data.sdw = col * d;
                data.sdw.a = 1.0f;
            }
            else {
                data.sdw = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
            if (d > 0.0) {
                const Col4f col = cast<Col4f>(base.pmat->col);
                data.val = col * d;
                data.val.a = 1.0f;
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
                const Col4f col = cast<Col4f>(base.pmat->col);
                data.sdw = col;
            }
            else {
                data.sdw = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
            {
                const Col4f col = cast<Col4f>(base.pmat->col);
                data.val = col;
            }
        }

    };
}
#endif
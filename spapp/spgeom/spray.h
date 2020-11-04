//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
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

        struct Data {
            Mesh3 mesh;
            Material mat;
        };

        struct Index {
            int id;
        };

        struct Hit {
            bool calc;
            bool find;

            VecPD3 vec;
            Material mat;
        };

        struct Layout {
            int uid;
            Mat pose;
            Mat invp;
        };

        struct Unit {
            Mem1<Data> data;


            Mem1<Node> nodes;
            Mem1<Index> idxs;
        };
    private:

        Mem1<Layout> m_layouts;
        MemP<Unit> m_units;
        Mem1<Node> m_nodes;
        Mem1<Index> m_idxs;

    public:
        BVH() {
            clear();
        }

        Mem1<const Node*> getNodes(const int i, const int level) const {
            Mem1<const Node*> nodes;

            const Unit &unit = m_units[i];
            for (int i = 0; i < unit.nodes.size(); i++) {
                if (unit.nodes[i].level == level) {
                    nodes.push(&unit.nodes[i]);
                }
            }
            return nodes;
        }

        void clear() {
            m_layouts.clear();
            m_units.clear();
            m_nodes.clear();
            m_idxs.clear();
        }

        void addModel(const Mem1<Mesh3> &meshes, const Mem1<Material> &mats, const Mem1<Mat> &poses) {
            SP_ASSERT(meshes.size() == mats.size());
            if (meshes.size() == 0) return;

            for (int i = 0; i < poses.size(); i++) {
                Layout &layout = *m_layouts.extend();
                layout.uid = m_units.size();
                layout.pose = poses[i];
                layout.invp = invMat(poses[i]);
            }

            Unit &unit = *m_units.malloc();
            unit.data.resize(meshes.size());
            unit.idxs.resize(meshes.size());
            for (int i = 0; i < meshes.size(); i++) {
                unit.data[i].mesh = meshes[i];
                unit.data[i].mat = mats[i];
                unit.idxs[i].id = i;
            }
        }

        void build() {
            if (m_layouts.size() == 0) return;

            struct IndexEx : public Index {
                Box3 box;
                Vec3 cent;
            };

            typedef int(*CMP)(const void*, const void*);
            CMP cmp[3];
            cmp[0] = [](const void* i1, const void* i2) -> int { return (((IndexEx*)i1)->cent.x < ((IndexEx*)i2)->cent.x) ? +1 : -1; };
            cmp[1] = [](const void* i1, const void* i2) -> int { return (((IndexEx*)i1)->cent.y < ((IndexEx*)i2)->cent.y) ? +1 : -1; };
            cmp[2] = [](const void* i1, const void* i2) -> int { return (((IndexEx*)i1)->cent.z < ((IndexEx*)i2)->cent.z) ? +1 : -1; };
      
            auto initn = [&](Mem1<Node> &nodes, Mem1<IndexEx> &idxs, const int level, const int base, const int size) -> Node* {
                Node *n = NULL;
#if SP_USE_OMP
#pragma omp critical
#endif
                {
                    n = nodes.extend();
                }
                n->level = level;
                n->base = base;
                n->size = size;
                n->n0 = NULL;
                n->n1 = NULL;

                n->box = nullBox3();
                for (int i = base; i < base + size; i++) {
                    n->box = orBox(n->box, idxs[i].box);
                }
                return n;
            };
            auto sorti = [&](Node &n, Mem1<IndexEx> &idxs, Mem1<SP_REAL> &buff) -> int {
                int di = 0;
                int da = 0;

                double mina = SP_INFINITY;
                for (int a = 0; a < 3; a++) {
                    sort(&idxs[n.base], n.size, cmp[a]);
                    Box3 bl = nullBox3();
                    Box3 br = nullBox3();

                    for (int i = 1; i < n.size; i++) {
                        const int l = i;
                        bl = orBox(bl, idxs[n.base + l - 1].box);
                        buff[n.base + l] = 0;
                        buff[n.base + l] += getBoxArea(bl) * i;
                    }
                    for (int i = 1; i < n.size; i++) {
                        const int r = n.size - i;
                        br = orBox(br, idxs[n.base + r].box);
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
            
            {
                for (int i = 0; i < m_units.size(); i++) {
                    Unit &unit = m_units[i];
                    Mem1<IndexEx> idxs(unit.idxs.size());

                    for (int i = 0; i < idxs.size(); i++) {
                        idxs[i].id = unit.idxs[i].id;
                        idxs[i].box = getBox3(unit.data[idxs[i].id].mesh);
                        idxs[i].cent = getMeshCent(unit.data[idxs[i].id].mesh);
                    }

                    unit.nodes.clear();
                    unit.nodes.reserve(2 * idxs.size() - 1);

                    Mem1<SP_REAL> buff(idxs.size());
                    
                    initn(unit.nodes, idxs, 0, 0, idxs.size());

                    Mem1<Mem1<Node*> > tnodes;
                    if (idxs.size() > 1000) {
                        const int level = 5;

                        tnodes.reserve(256);
                        for (int ni = 0; ni < unit.nodes.size(); ni++) {
                            Node& n = unit.nodes[ni];
                            if (n.size == 1) continue;

                            if (n.level < level) {
                                const int di = sorti(n, idxs, buff);
                                n.n0 = initn(unit.nodes, idxs, n.level + 1, n.base, di);
                                n.n1 = initn(unit.nodes, idxs, n.level + 1, n.base + di, n.size - di);
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
                        nodes.push(&unit.nodes[0]);
                    }

#if SP_USE_OMP
#pragma omp parallel for
#endif
                    for (int i = 0; i < tnodes.size(); i++) {
                        for (int ni = 0; ni < tnodes[i].size(); ni++) {
                            Node& n = *tnodes[i][ni];
                            if (n.size == 1) continue;

                            const int di = sorti(n, idxs, buff);

                            n.n0 = initn(unit.nodes, idxs, n.level + 1, n.base, di);
                            n.n1 = initn(unit.nodes, idxs, n.level + 1, n.base + di, n.size - di);
                            tnodes[i].push(n.n0);
                            tnodes[i].push(n.n1);
                        }
                    }

                    for (int i = 0; i < idxs.size(); i++) {
                        unit.idxs[i].id = idxs[i].id;
                    }
                }
            }
            {

                Mem1<IndexEx> idxs(m_layouts.size());
                m_idxs.resize(m_layouts.size());

                for (int i = 0; i < idxs.size(); i++) {
                    Unit &unit = m_units[m_layouts[i].uid];
                    idxs[i].id = i;
                    idxs[i].box = nullBox3();
                    for (int j = 0; j < 8; j++) {
                        const int a = (j & 0x01) ? 1 : 0;
                        const int b = (j & 0x02) ? 1 : 0;
                        const int c = (j & 0x04) ? 1 : 0;
                        const Vec3 v = getVec3(unit.nodes[0].box.pos[a].x, unit.nodes[0].box.pos[b].y, unit.nodes[0].box.pos[c].z);
                        idxs[i].box = orBox(idxs[i].box, m_layouts[i].pose * v);
                    }
                    idxs[i].cent = getBoxCent(idxs[i].box);
                }

                m_nodes.clear();
                m_nodes.reserve(2 * idxs.size() - 1);

                Mem1<SP_REAL> buff(idxs.size());
                
                initn(m_nodes, idxs, 0, 0, idxs.size());

                Mem1<Node*> tnodes;
                {
                    tnodes.push(&m_nodes[0]);
                }

                for (int ni = 0; ni < tnodes.size(); ni++) {
                    Node& n = *tnodes[ni];
                    if (n.size == 1) continue;

                    const int di = sorti(n, idxs, buff);

                    n.n0 = initn(m_nodes, idxs, n.level + 1, n.base, di);
                    n.n1 = initn(m_nodes, idxs, n.level + 1, n.base + di, n.size - di);
                    tnodes.push(n.n0);
                    tnodes.push(n.n1);
                }

                for (int i = 0; i < idxs.size(); i++) {
                    m_idxs[i].id = idxs[i].id;
                }
            }
        }

        bool trace(Hit &hit, const VecPD3 &ray, const double minv, const double maxv) const {
            memset(&hit, 0, sizeof(Hit));
            hit.calc = true;

            const int QUE_MAX = 100;
            const Node* queA[QUE_MAX];
            const Node* queB[QUE_MAX];

            double lmaxv = maxv;
            if (m_nodes.size() > 0) {
                int stack = 0;
                queA[stack++] = &m_nodes[0];

                while (stack > 0) {
                    const Node *n = queA[--stack];
                    if (checkHit(n->box, ray, minv, lmaxv) == false) {
                        continue;
                    }
                    if (n->n0 != NULL && n->n1 != NULL && stack < QUE_MAX - 2) {
                        queA[stack++] = n->n0;
                        queA[stack++] = n->n1;
                        continue;
                    }

                    const int i = m_idxs[n->base].id;
                    {
                        const Mat &pose = m_layouts[i].pose;
                        const Mat &invp = m_layouts[i].invp;

                        const Unit &unit = m_units[m_layouts[i].uid];

                        const VecPD3 bray = invp * ray;

                        int stack = 0;
                        queB[stack++] = &unit.nodes[0];

                        int minid = -1;
                        int objid = -1;

                        const SP_REAL delta = 0.001;

                        while (stack > 0) {
                            const Node *n = queB[--stack];
                            if (checkHit(n->box, bray, minv, lmaxv) == false) {
                                continue;
                            }
                            if (n->n0 != NULL && n->n1 != NULL && stack < QUE_MAX - 2) {
                                queB[stack++] = n->n0;
                                queB[stack++] = n->n1;
                                continue;
                            }
                            {
                                const int id = n->base;
                                SP_REAL result[3] = { 0 };
                                if (traceMesh(result, unit.data[unit.idxs[id].id].mesh, bray, minv, lmaxv) == true) {

                                    const Vec3 nrm = getMeshNrm(unit.data[unit.idxs[id].id].mesh);
                                    const bool f = (dotVec(nrm, bray.drc) < 0.0);

                                    lmaxv = result[0];
                                    minid = id;
                                }
                            }
                        }

                        if (minid >= 0) {
                            hit.find = true;
                            hit.mat = unit.data[unit.idxs[minid].id].mat;

                            hit.vec.pos = pose * (bray.pos + bray.drc * lmaxv);
                            hit.vec.drc = unitVec(pose.part(0, 0, 3, 3)  * getMeshNrm(unit.data[unit.idxs[minid].id].mesh));
                        }
                    }
                }
            }
            return hit.find;
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
                val = 0.8f;
                sdw = 0.8f;
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

    private:
        const static int SAMPLE_UNIT = 3;
        const static int LEVEL_MAX = 5;
        const static int LIGHT_MAX = 4;

        struct Data {
            Col4f col;
            float sdw;
        };
        struct Img {
            Data amb;
            Data dif[LIGHT_MAX];
            float msk;
        };
        struct Cnt {
            int amb;
            int dif[LIGHT_MAX];
            int msk;
        };

        BVH m_bvh;

    public:

        CamParam m_cam;
        Pose m_pose;

        Mem2<Img> m_img;
        Cnt m_cnt;
        Cnt m_lim;

        Mem2<MemA<BVH::Hit, SAMPLE_UNIT * SAMPLE_UNIT> > m_hitmap;
        Mem2<MemA<VecPD3, SAMPLE_UNIT * SAMPLE_UNIT> > m_raymap;

        // objects
        Light m_ambient;
        Mem1<PntLight> m_plights;
        Plane m_plane;

    public:

        PathTrace() {
            memset(&m_lim, 0, sizeof(Cnt));
            m_lim.msk = SAMPLE_UNIT * SAMPLE_UNIT;

            setCam(getCamParam(640, 480), getPose(getVec3(0.0, 0.0, 1000.0)));
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
                lim += m_lim.dif[i];
            }
            cnt += m_cnt.msk;
            lim += m_lim.msk;

            return (cnt == lim) ? 1.0f : static_cast<float>(cnt) / (lim);
        }

        void setCam(const CamParam &cam, const Pose &pose) {

            if (m_cam == cam && m_pose == pose) return;

            m_cam = cam;
            m_pose = pose;

            m_hitmap.resize(m_cam.dsize);
            m_hitmap.zero();

            m_raymap.resize(m_cam.dsize);

            const Pose wpose = invPose(m_pose);
            const Mat wrot = getMat(wpose.rot);

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int v = 0; v < m_cam.dsize[1]; v++) {
                for (int u = 0; u < m_cam.dsize[0]; u++) {

                    for (int i = 0; i < (SAMPLE_UNIT * SAMPLE_UNIT); i++) {
                        const double delta = 1.0 / (SAMPLE_UNIT + 1);
                        const double du = ((i / SAMPLE_UNIT) + 1) * delta;
                        const double dv = ((i % SAMPLE_UNIT) + 1) * delta;

                        const Vec2 npx = invCam(m_cam, getVec2(u - 0.5 + du, v - 0.5 + dv));

                        VecPD3 &vec = m_raymap(u, v)[i];
                        if (m_cam.type == CamParam_Pers) {
                            vec.pos = wpose.pos;
                            vec.drc = wrot * unitVec(prjVec(npx, 1.0, true));
                        }
                        else {
                            vec.pos = wpose.pos + wrot * getVec3(npx.x, npx.y, -1000.0 * 10);
                            vec.drc = wrot * getVec3(0.0, 0.0, 1.0);
                        }
                    }

                }
            }

            reset();
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

        void setAmbient(const Light &light, const int lim = 100) {
            m_ambient = light;
            m_lim.amb = lim;
            if (light.valid == false) {
                m_cnt.amb = 0;
                m_lim.amb = 0;
            }
        }

        void setPntLights(const Mem1<PntLight> &lights, const int lim = 30) {
            SP_ASSERT(lights.size() <= LIGHT_MAX);

            for (int i = 0; i < LIGHT_MAX; i++) {
                if (i >= minVal(m_plights.size(), lights.size()) || lights[i].pos != m_plights[i].pos) {
                    m_cnt.dif[i] = 0;
                }
                
                m_lim.dif[i] = (lights.size() > 0) ? lim : 0;
            }

            m_plights = lights;
        }

        void addModel(const Mem1<Mesh3> &meshes, const Mem1<Material> &mats, const Mem1<Mat> &poses) {
            m_bvh.addModel(meshes, mats, poses);
        }

        void build() {
            m_bvh.build();
        }

        bool update() {
            if (prog() == 1.0f) return false;

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int v = 0; v < m_cam.dsize[1]; v++) {
                for (int u = 0; u < m_cam.dsize[0]; u++) {
                    if (m_cnt.msk >= m_lim.msk && m_img(u, v).msk == SP_CAST_REAL(0.0)) continue;

                    calc(m_img(u, v), m_hitmap(u, v), m_raymap(u, v));
                }
            }
            {
                m_cnt.amb = minVal(m_lim.amb, m_cnt.amb + 1);
                for (int i = 0; i < m_plights.size(); i++) {
                    m_cnt.dif[i] = minVal(m_lim.dif[i], m_cnt.dif[i] + 1);
                }
                m_cnt.msk = minVal(m_lim.msk, m_cnt.msk + 1);
            }
            return true;
        }

        void render(Mem2<Col4> &img, const Col4f &bgcol = getCol4f(1.0, 1.0, 1.0, 1.0)) {
            img.resize(m_cam.dsize);

            auto blend = [](Col4f &dst, const Data &data, const Light &light) {
                dst.r += ((1.0f - data.sdw * light.sdw) * data.col.r) * light.val * light.col.r;
                dst.g += ((1.0f - data.sdw * light.sdw) * data.col.g) * light.val * light.col.g;
                dst.b += ((1.0f - data.sdw * light.sdw) * data.col.b) * light.val * light.col.b;
                dst.a += data.col.a;
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

                    col = meanCol(col, col.a, bgcol, 1.0f - col.a);

                    img(u, v) = cast<Col4>(col);
                }
            }
        }

    private:

        bool trace(BVH::Hit &hit, const VecPD3 &ray, const double minv, const double maxv) {
            bool ret = false;
            double maxt = maxv;
            {
                ret = m_bvh.trace(hit, ray, minv, maxt);
                if (ret == true) {
                    maxt = normVec(hit.vec.pos - ray.pos);
                }
            }
            if (m_plane.valid == true) {
                SP_REAL result[3];
                double min = (m_cam.type == CamParam_Pers) ? 0.0 : -SP_INFINITY;
                if (tracePlane(result, m_plane.vec, ray, min, maxt)) {
                    hit.calc = true;
                    hit.find = true;
                    hit.mat = m_plane.mat;
                    hit.vec.pos = ray.pos + ray.drc * result[0];
                    hit.vec.drc = m_plane.vec.drc;
                    ret = true;
                }
            }
            return ret;
        }

        void calc(Img &img, MemA<BVH::Hit, SAMPLE_UNIT * SAMPLE_UNIT> &hits, const MemA<VecPD3, SAMPLE_UNIT * SAMPLE_UNIT> &rays) {

            auto precalc = [&](BVH::Hit &hit, VecPD3 &ray, const int i){
                ray = rays[i % (SAMPLE_UNIT * SAMPLE_UNIT)];
                hit = hits[i % (SAMPLE_UNIT * SAMPLE_UNIT)];
                if (hit.calc == false) {
                    trace(hit, ray, 0.0, SP_INFINITY);
                }
                return hit;
            };

            {
                if (m_cnt.amb < m_lim.amb) {
                    BVH::Hit hit;
                    VecPD3 ray;
                    precalc(hit, ray, m_cnt.amb);

                    if (m_cnt.amb == 0) {
                        img.amb.col = getCol4f(0.0, 0.0, 0.0, 0.0);
                        img.amb.sdw = 0.0f;
                    }
                    if (hit.find == true) {
                        Data data;
                        calc_amb(data, ray, hit, 0, (unsigned int)m_cnt.amb);

                        img.amb.col = meanCol(img.amb.col, m_cnt.amb, data.col, 1.0);
                        img.amb.sdw = (img.amb.sdw * m_cnt.amb + data.sdw) / (m_cnt.amb + 1);
                    }
                }
            }
            for (int i = 0; i < m_plights.size(); i++) {
                if (m_cnt.dif[i] < m_lim.dif[i]) {
                    BVH::Hit hit;
                    VecPD3 ray;
                    precalc(hit, ray, m_cnt.dif[i]);

                    if (m_cnt.dif[i] == 0) {
                        img.dif[i].col = getCol4f(0.0, 0.0, 0.0, 0.0);
                        img.dif[i].sdw = 0.0f;
                    }
                    if (hit.find == true) {
                        Data data;
                        calc_dif(data, ray, hit, m_plights[i].pos + randgVec3(1.0, 1.0, 1.0, m_cnt.dif[i]) * 1.0, 0, m_cnt.dif[i]);
                        img.dif[i].col = meanCol(img.dif[i].col, m_cnt.dif[i], data.col, 1.0);
                        img.dif[i].sdw = (img.dif[i].sdw * m_cnt.dif[i] + data.sdw) / (m_cnt.dif[i] + 1.0);
                    }
                }
            }
            {
                if (m_cnt.msk < m_lim.msk) {
                    BVH::Hit hit;
                    VecPD3 ray;
                    precalc(hit, ray, m_cnt.msk);

                    if (m_cnt.msk == 0) {
                        img.msk = 0.0f;
                    }
                    img.msk = (img.msk * m_cnt.msk + (hit.find ? 1.0f : 0.0f)) / (m_cnt.msk + 1.0f);
                }
            }
        }

        void calc_dif(Data &data, const VecPD3 &ray, const BVH::Hit &base, const Vec3 lpos, const int level, const unsigned int seed) {
            const SP_REAL delta = 0.001;

            const Vec3 nrm = base.vec.drc;

            VecPD3 next;
            next.pos = base.vec.pos + base.vec.drc * delta;
            next.drc = unitVec(lpos - base.vec.pos);

            const SP_REAL d = dotVec(base.vec.drc, next.drc);
            BVH::Hit hit;
            if (d > 0.0 && trace(hit, next, 0.0, SP_INFINITY) == false) {
                data.sdw = 0.0;
            }
            else {
                data.sdw = 1.0;
            }
            if (d > 0.0) {
                const Col4f col = base.mat.col;
                data.col = col * d;
                data.col.a = 1.0f;
            }
            else {
                data.col = getCol4f(0.0, 0.0, 0.0, 1.0);
            }
        }

        void calc_amb(Data &data, const VecPD3 &ray, const BVH::Hit &base, const int level, const unsigned int seed) {
            if (level >= LEVEL_MAX) {
                Data d;
                test_amb(d, ray, base, level, seed);
                data.col += d.col;
                data.sdw += d.sdw;
                return;
            }

            const SP_REAL delta = 0.001;

            const double rate0 = 1.0 - maxVal(base.mat.tr, base.mat.rf);
            const double rate1 = base.mat.tr * maxVal(base.mat.tr, base.mat.rf) / (base.mat.tr + base.mat.rf);
            const double rate2 = base.mat.rf * maxVal(base.mat.tr, base.mat.rf) / (base.mat.tr + base.mat.rf);

            data.col = getCol4f(0.0, 0.0, 0.0, 0.0);
            data.sdw = 0.0;

            if (rate0 > 0.0) {
                Data d;
                test_amb(d, ray, base, level, seed);
                data.col += d.col * rate0;
                data.sdw += d.sdw * rate0;
            }

            if (rate1 > 0.0) {
                VecPD3 next;
                next.pos = base.vec.pos + ray.drc * delta;
                next.drc = ray.drc;

                BVH::Hit hit;
                Data d;
                d.col = getCol4f(0.0, 0.0, 0.0, 0.0);
                d.sdw = 0.0;

                bool ret = false;
                for (int i = 0; i < LEVEL_MAX - level; i++) {
                    ret = trace(hit, next, 0.0, SP_INFINITY);
                    if (ret == true) {
                        next.pos = hit.vec.pos + next.drc * delta;
                        next.drc = next.drc;
                        if (dotVec(hit.vec.drc, next.drc) > 0.0) {
                            continue;
                        }
                        else {
                            calc_amb(d, next, hit, level + 1, seed + 1);
                            break;
                        }
                    }
                    else {
                        break;
                    }
                }

                data.col += d.col * rate1;
                data.sdw += d.sdw * rate1;
            }
        }


        void test_amb(Data &data, const VecPD3 &ray, const BVH::Hit &base, const int level, const unsigned int seed) {

            const SP_REAL delta = 0.001;

            VecPD3 next;
            next.pos = base.vec.pos + base.vec.drc * delta;
            next.drc = unitVec(base.vec.drc * (1.0 + delta) + randuVec3(1.0, 1.0, 1.0, seed));

            BVH::Hit hit;

            bool ret = false;
            data.col = base.mat.col;
            data.sdw = 0.0;
            float tmp = 1.0;

            if (1) {
                ret = trace(hit, next, 0.0, SP_INFINITY);
                if (ret) {
                    data.sdw = 1.0f;
                }
            }
            else {
                for (int i = 0; i < LEVEL_MAX - level; i++) {
                    ret = trace(hit, next, 0.0, SP_INFINITY);
                    if (ret == true) {
                        const double r = hit.mat.tr * maxVal(hit.mat.tr, hit.mat.rf) / (hit.mat.tr + hit.mat.rf);
                        if (r > 0.0) {
                            next.pos = hit.vec.pos + next.drc * delta;
                            next.drc = next.drc;

                            data.sdw += tmp * (1.0 - r);
                            if (dotVec(hit.vec.drc, next.drc) > 0.0) {
                                tmp *= r;
                            }
                            else {
                            }
                        }
                        else {
                            data.sdw += tmp;
                            break;
                        }
                    }
                    else {
                        break;
                    }
                }
            }
            data.col = base.mat.col;
        }
    };
}
#endif
//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RAY_H__
#define __SP_RAY_H__

#include "spcore/spcore.h"
#include <algorithm>
#include <vector>
#include <numeric>
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
        if (result[1] < 0.0 || result[2] < 0.0 || result[1] + result[2] > 1.0) return false;
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
            int a;
            int i;
            Vec3 c;

            Index() {
            }

            Index(const Index &idx) {
                *this = idx;
            }

            Index& operator = (const Index &idx) {
                a = idx.a;
                i = idx.i;
                c = idx.c;
                return *this;
            }
        };

    private:
        int m_acnt;

        Mem1<const Mesh3*> m_pmeshes;

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
            m_pmeshes.clear();
            m_nodes.clear();
            m_idxs.clear();
        }
        
        void add(const Mem1<Mesh3> &meshes) {
            const int offset = m_idxs.size();

            m_pmeshes.extend(meshes.size());
            for (int i = 0; i < m_pmeshes.size(); i++) {
                m_pmeshes[offset + i] = &meshes[i];
            }

            m_idxs.extend(meshes.size());
            for (int i = 0; i < m_idxs.size(); i++) {
                m_idxs[offset + i].a = m_acnt;
                m_idxs[offset + i].i = i;
                m_idxs[offset + i].c = getMeshCent(meshes[i]);
            }
            m_acnt++;
        }

        void build() {

            m_nodes.clear();
            m_nodes.reserve(2 * m_idxs.size() - 1);
            auto initn = [&](const int level, const int base, const int size) -> Node*{
                Node *n = m_nodes.extend();
                n->level = level;
                n->base = base;
                n->size = size;
                n->n0 = NULL;
                n->n1 = NULL;

                n->box = nullBox3();
                for (int i = base; i < base + size; i++) {
                    n->box = orBox(n->box, *m_pmeshes[m_idxs[i].i]);
                }
                return n;
            };

            auto sorti = [&](Node &n, const int ax) {
                typedef int(*CMP)(const void*, const void*);
                CMP cmp[3];

                cmp[0] = [](const void* i1, const void* i2) -> int {
                    return (((Index*)i1)->c.x < ((Index*)i2)->c.x) ? +1 : -1;
                };
                cmp[1] = [](const void* i1, const void* i2) -> int {
                    return (((Index*)i1)->c.y < ((Index*)i2)->c.y) ? +1 : -1;
                };
                cmp[2] = [](const void* i1, const void* i2) -> int {
                    return (((Index*)i1)->c.z < ((Index*)i2)->c.z) ? +1 : -1;
                };

                sort(&m_idxs[n.base], n.size, cmp[ax]);
            };

            initn(0, 0, m_idxs.size());

            Mem2<SP_REAL> map(m_idxs.size(), 2);
            
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
                            bl = orBox(bl, *m_pmeshes[m_idxs[n.base + l - 1].i]);
                            br = orBox(br, *m_pmeshes[m_idxs[n.base + r].i]);
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

        }

        bool trace(const Index **idx, SP_REAL *norm, const VecPD3 &ray, const double minv, const double maxv) const {
            int minid = -1;

            Mem1<const Node*> que;
            que.reserve(20);
            que.push(&m_nodes[0]);

            double lmaxv = maxv;
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
                    if (traceMesh(result, *m_pmeshes[m_idxs[i].i], ray, minv, lmaxv) == true) {
                        lmaxv = result[0];
                        minid = i;
                    }
                }
            }
            if (minid < 0) {
                return false;
            }

            if (idx != NULL) *idx = &m_idxs[minid];
            if (norm != NULL) *norm = lmaxv;
            return true;
        }
    };

    class PathTrace {

    public:
        int m_tcnt;

        Mem2<Vec3> m_amb;

        Mem1<Mem2<Vec3> > m_difs;
        Mem1<Mem2<Vec3> > m_spcs;

        Mem1<Vec3> m_lights;

    public:
        void init(const Mem1<Vec3> &lights) {
            m_lights = lights;
        }
        void clear() {

        }

        void trace() {

        }
    };
}
#endif
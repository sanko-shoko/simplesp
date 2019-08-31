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

    // position and direction
    struct Ray {
        Vec3 pos;
        Vec3 drc;
    };

    SP_CPUFUNC Ray getRay(const Vec3 &pos, const Vec3 &drc) {
        Ray ret;
        ret.pos = pos;
        ret.drc = drc;
        return ret;
    }

    SP_CPUFUNC bool traceMesh(SP_REAL *result, const Mesh3 &mesh, const Ray &ray, const double minv, const double maxv) {
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


    SP_CPUFUNC bool checkHit(const Box3 &box, const Ray &ray, const double minv, const double maxv) {
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
            int s, size;
            Box3 box;
            Node *n0, *n1;
        };

        struct Idx {
            int i;
            Vec3 c;
            const Mesh3 *m;

            Idx() {
            }

            Idx(const Idx &idx) {
                this->i = i;
                this->c = c;
            }

            Idx& operator = (const Idx &idx) {
                this->i = i;
                this->c = c;
                return *this;
            }
        };

    private:

        Mem1<Node> m_nodes;
        Mem1<Idx> m_idxs;

    public:

        Mem1<const Node*> getNode(const int level) const {
            Mem1<const Node*> nodes;

            const Node *s[32] = { 0 };
            s[0] = &m_nodes[0];

            int l[99] = { 0 };

            for (int si = 0; si >= 0; si--) {
                const Node& n = *s[si];
                const int ll = l[si];
                if (ll == level) {
                    nodes.push(&n);
                }
                if (ll >= level) continue;

                if (n.n0 != NULL && n.n1 != NULL) {
                    s[si + 0] = n.n0;
                    s[si + 1] = n.n1;
                    l[si + 0] = ll + 1;
                    l[si + 1] = ll + 1;
                    si += 2;
                }
            }
            return nodes;
        }

        void build(const Mem1<Mesh3> &meshes) {

            m_idxs.resize(meshes.size());
            for (int i = 0; i < m_idxs.size(); i++) {
                m_idxs[i].i = i;
                m_idxs[i].m = &meshes[i];
                m_idxs[i].c = getMeshCent(meshes[i]);
            }

            m_nodes.reserve(2 * meshes.size() - 1);
            
            Box3 lbox;
            lbox.pos[0] = getVec3(1.0, 1.0, 1.0) * (+SP_INFINITY);
            lbox.pos[1] = getVec3(1.0, 1.0, 1.0) * (-SP_INFINITY);

            auto initn = [&](const int s, const int size) -> Node*{
                Node &n = *m_nodes.extend();
                n.s = s;
                n.size = size;
                n.n0 = NULL;
                n.n1 = NULL;
                n.box = lbox;

                for (int i = s; i < s + size; i++) {
                    n.box = orBox(n.box, *m_idxs[i].m);
                }
                return &n;
            };

            initn(0, meshes.size());

            Mem2<SP_REAL> map(meshes.size(), 2);
            
            for(int ni = 0; ni < m_nodes.size(); ni++){
                Node& n = m_nodes[ni];

                auto bsort = [&](int ax) {
                    typedef int(*CMP)(const void*, const void*);

                    CMP cmp[3];
                    cmp[0] = [](const void* i1, const void* i2) -> int {
                        return (((Idx*)i1)->c.x < ((Idx*)i2)->c.x) ? +1 : -1;
                    };
                    cmp[1] = [](const void* i1, const void* i2) -> int {
                        return (((Idx*)i1)->c.y < ((Idx*)i2)->c.y) ? +1 : -1;
                    };
                    cmp[2] = [](const void* i1, const void* i2) -> int {
                        return (((Idx*)i1)->c.z < ((Idx*)i2)->c.z) ? +1 : -1;
                    };

                    sort(&m_idxs[n.s], n.size, cmp[ax]);
                };

                if (n.size < 2) {
                    continue;
                }

                SP_REAL b = SP_INFINITY;
                int bi, ba;
                for (int a = 0; a < 3; a++) {
                    bsort(a);
                    Box3 bl = lbox;
                    Box3 br = lbox;
                    for (int i = 0; i < n.size; i++) {
                        int j = n.size - i;
                        map(i, 0) = getBoxArea(bl) * i;
                        map(j, 1) = getBoxArea(br) * i;

                        bl = orBox(bl, *m_idxs[n.s + 0 + i].m);
                        br = orBox(br, *m_idxs[n.s - 1 + j].m);
                    }

                    for (int i = 1; i < n.size; i++) {
                        SP_REAL c = 1 + (map(i, 0) + map(i, 1)) / getBoxArea(n.box);
                        if (c < b) {
                            b = c;
                            bi = i;
                            ba = a;
                        }
                    }
                }

                if (b > n.size) {
                }
                else {
                    bsort(ba);

                    n.n0 = initn(n.s, bi);
                    n.n1 = initn(n.s + bi, n.size - bi);
                }
            }

        }

        struct Hit {
            int i;
            SP_REAL norm;
        };

        bool trace(Hit &hit, const Ray& ray, const double minv, const double maxv) const {
            int minid = -1;

            Mem1<const Node*> que;
            que.reserve(32);
            que.push(&m_nodes[0]);

            double norm = maxv;
            while (que.size() > 0) {
                const Node& n = **que.last();
                que.pop();
                {
                    if (checkHit(n.box, ray, minv, norm) == false) {
                        continue;
                    }
                }
                if (n.n0 != NULL && n.n1 != NULL) {
                    que.push(n.n0);
                    que.push(n.n1);
                    continue;
                }
                for (int i = n.s; i < n.s + n.size; i++) {
                    SP_REAL result[3] = { 0 };
                    if (traceMesh(result, *m_idxs[i].m, ray, minv, norm) == true) {
                        norm = result[0];
                        minid = i;
                    }
                }
            }
            if (minid < 0) {
                return false;
            }
            hit.i = m_idxs[minid].i;
            hit.norm = norm;
            return true;
        }
    };
}
#endif
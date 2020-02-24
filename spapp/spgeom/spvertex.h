//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VERTEX_H__
#define __SP_VERTEX_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // 2d
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC Mem<Mesh2> divPolygon(const Mem<Vec2> &vtxs) {
        Mem1<Mesh2> meshes;
        if (vtxs.size() < 3) return meshes;

        Mem1<Vec2> tmps = vtxs;

        while(tmps.size() > 3) {
            int p = -1;

            SP_REAL maxv = 0.0;
            for (int i = 0; i < tmps.size(); i++) {
                const SP_REAL norm = normVec(tmps[i]);
                if (norm > maxv) {
                    maxv = norm;
                    p = i;
                }
            }

            const SP_REAL drc = crsVec(tmps.lacs(p - 1) - tmps.lacs(p), tmps.lacs(p + 1) - tmps.lacs(p));

            for (int i = 0; i < tmps.size(); i++) {
                const int pi = p + i;
                const Vec2 A = tmps.lacs(pi - 1);
                const Vec2 B = tmps.lacs(pi + 0);
                const Vec2 C = tmps.lacs(pi + 1);

                const Vec2 X = A - B;
                const Vec2 Y = C - B;
                if (crsVec(X, Y) * drc <= 0) continue;

                Mat mat(2, 2);
                mat(0, 0) = X.x;
                mat(1, 0) = X.y;
                mat(0, 1) = Y.x;
                mat(1, 1) = Y.y;

                bool check = false;
                for (int j = 0; j < tmps.size() - 3; j++) {
                    const Vec2 V = tmps.lacs((pi + 2) + j) - B;

                    Mat vec(2, 1);
                    vec(0, 0) = V.x;
                    vec(1, 0) = V.y;

                    const Mat result = invMat(mat) * vec;
                    if (result.size() == 0) continue;

                    if (result[0] >= 0.0 && result[1] >= 0.0 && (result[0] + result[1]) <= 1.0) {
                        check = true;
                        break;
                    }
                }

                if (check == false) {
                    meshes.push(getMesh2(A, B, C));
                    p = pi % tmps.size();
                    break;
                }
            }

            tmps.del(p);
        }

        meshes.push(getMesh2(tmps[0], tmps[1], tmps[2]));

        return meshes;
    }

    //--------------------------------------------------------------------------------
    // 3d
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool calcVertexNormal(Mem1<Vec3> &nrms, const Mem1<Mesh3> &meshes) {
        nrms.resize(meshes.size() * 3);
        nrms.zero();

        KdTree<SP_REAL> kdtree(3);

        Mem1<Vec3> mnrms(meshes.size());
        for (int i = 0; i < meshes.size(); i++) {
            kdtree.addData(&meshes[i].pos[0]);
            kdtree.addData(&meshes[i].pos[1]);
            kdtree.addData(&meshes[i].pos[2]);
            mnrms[i] = getMeshNrm(meshes[i]);
        }
        kdtree.makeTree();

        for (int i = 0; i < meshes.size(); i++) {
            for (int j = 0; j < 3; j++) {
                const Mem1<int> index = kdtree.search(&meshes[i].pos[j], 0.1);

                Vec3 nrm = getVec3(0.0, 0.0, 0.0);
                for (int k = 0; k < index.size(); k++) {
                    const int s = index[k] / 3;
                    nrm += mnrms[s];
                }
                nrms[i * 3 + j] = unitVec(nrm);
            }
        }

        return true;
    }


    template <typename TYPE, typename ELEM>
    SP_CPUFUNC void indexing(Mem1<TYPE> &dst, Mem1<int> &idxs, const Mem1<TYPE> &src) {
        dst.clear();
        idxs.clear();
        if (src.size() == 0) return;

        const int dim = sizeof(TYPE) / sizeof(ELEM);
        const int num = src.size() * dim;

        KdTree<ELEM> kdtree(dim);

        for (int i = 0; i < src.size(); i++) {
            kdtree.addData(&src[i]);
        }
        kdtree.makeTree();

        dst.clear();
        idxs.resize(src.size());

        Mem1<int> refs(src.size());
        for (int i = 0; i < src.size(); i++) {
            refs[i] = -1;

            const Mem1<int> index = kdtree.search(&src[i], 0.1);
            int minv = num;
            for (int k = 0; k < index.size(); k++) {
                minv = minVal(minv, index[k]);
            }

            if (i == minv) {
                refs[i] = dst.size();
                dst.push(src[i]);
            }
            idxs[i] = refs[minv];
        }

    }

}
#endif
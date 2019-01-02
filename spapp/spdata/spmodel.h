//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MODEL_H__
#define __SP_MODEL_H__

#include "spcore/spcore.h"
#include "spapp/spimg/sprender.h"
#include "spapp/spalgo/spkdtree.h"

namespace sp{
    
    //--------------------------------------------------------------------------------
    // model edge
    //--------------------------------------------------------------------------------
   
    struct Edge {
        Vec3 pos, drc, nrm[2];
    };


    //--------------------------------------------------------------------------------
    // model util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Vec3 getModelCenter(const Mem1<Mesh3> &model){
        Vec3 sum = zero<Vec3>();
        for (int i = 0; i < model.size(); i++){
            sum += getMeshPos(model[i]);
        }

        return sum / model.size();
    }

    SP_CPUFUNC double getModelRadius(const Mem1<Mesh3> &model){
        Mem1<double> mem(model.size());
        for (int i = 0; i < mem.size(); i++){
            mem[i] = normVec(getMeshPos(model[i]));
        }

        return maxVal(mem);
    }

    SP_CPUFUNC double getModelDistance(const Mem1<Mesh3> &model, const CamParam &cam){

        const double radius = getModelRadius(model);
        const double distance = 1.2 * maxVal(cam.fx, cam.fy) * radius / (0.5 * minVal(cam.dsize[0], cam.dsize[1]));
    
        return distance;
    }

    SP_CPUFUNC Mem1<VecPN3> getModelPoint(const Mem1<Mesh3> &model, const int density = 50){
        const CamParam cam = getCamParam(density, density);
        const double distance = getModelDistance(model, cam);
        
        Mem1<VecPN3> tmp;
        const int num = getGeodesicMeshNum(0);
        for (int i = 0; i < num; i++){
            const Vec3 v = getMeshPos(getGeodesicMesh(0, i)) * (-1.0);
            const Pose pose = getPose(getRotDirection(v), getVec(0.0, 0.0, distance));
            Mem2<VecPN3> map;
            renderVecPN(map, cam, pose, model);

            const Mat mat = getMat(invPose(pose));
            for (int j = 0; j < map.size(); j++){
                if (map[j].pos.z > 0 && dotVec(map[j].pos, map[j].nrm) < 0){
                    tmp.push(mat * map[j]);
                }
            }
        }

        tmp = shuffle(tmp);

        Mem1<VecPN3> pnts;
        const double unit = 2 * distance / (cam.fx + cam.fy);

        for (int i = 0; i < tmp.size(); i++){
            bool check = true;
            for (int j = 0; j < pnts.size(); j++){
                if (dotVec(pnts[j].nrm, tmp[i].nrm) > 0.5 && normVec(pnts[j].pos - tmp[i].pos) < unit){
                    check = false;
                    break;
                }
            }
            if (check == true){
                pnts.push(tmp[i]);
            }
        }

        return pnts;
    }

    SP_CPUFUNC Mem1<Edge> getModelEdge(const Mem1<Mesh3> &model, const int density = 50) {

        KdTree<double> kdtree;
        kdtree.init(3);
        for (int i = 0; i < model.size(); i++) {
            for (int j = 0; j < 3; j++) {
                kdtree.addData(&model[i].pos[j]);
            }
        }
        kdtree.makeTree();

        Mem1<Edge> edges;

        const double radius = getModelRadius(model);
        const double unit = 2.0 * radius / density;

        for (int i = 0; i < model.size(); i++) {

            for (int j = 0; j < 3; j++) {
                const Vec3 A = model[i].pos[(j + 0) % 3];
                const Vec3 B = model[i].pos[(j + 1) % 3];
                const Vec3 V = unitVec(B - A);

                const Mem1<int> list = kdtree.search(&A, normVec(B - A) + 0.01);

                for (int k = 0; k < list.size(); k++) {
                    const int mid = list[k] / 3;
                    const int pid = list[k] % 3;
                    if (mid <= i) continue;

                    const Vec3 C = model[mid].pos[(pid + 0) % 3];
                    const Vec3 D = model[mid].pos[(pid + 1) % 3];

                    const Vec3 F = normVec(C - A) > normVec(D - A) ? C : D;

                    if (fabs(dotVec(V, unitVec(F - A))) < 0.99) continue;
                    if (fabs(dotVec(V, unitVec(D - C))) < 0.99) continue;

                    const Vec3 O = dotVec(V, C) < dotVec(V, D) ? C : D;
                    const Vec3 P = dotVec(V, C) < dotVec(V, D) ? D : C;

                    const Vec3 X = dotVec(V, A) > dotVec(V, O) ? A : O;
                    const Vec3 Y = dotVec(V, B) < dotVec(V, P) ? B : P;

                    if (normVec(Y - X) < 0.01) continue;

                    const int div = ceil(normVec(Y - X) / unit);
                    for (int d = 0; d < div; d++) {
                        Edge edge;
                        edge.pos = (Y - X) / (div + 1.0) * (d + 1.0) + X;
                        edge.drc = V;
                        edge.nrm[0] = getMeshNrm(model[i]);
                        edge.nrm[1] = getMeshNrm(model[mid]);

                        edges.push(edge);
                    }
                }

            }
        }

        return edges;
    }


    //--------------------------------------------------------------------------------
    // pose model
    //--------------------------------------------------------------------------------

    class PoseModel {
    public:

        Pose pose;
        Mem1<Edge> edges;
        Mem1<VecPN3> pnts;

    public:

        PoseModel() {
        }

        PoseModel(const PoseModel &pmodel) {
            *this = pmodel;
        }

        PoseModel& operator = (const PoseModel &pmodel) {
            pose = pmodel.pose;
            edges = pmodel.edges;
            pnts = pmodel.pnts;
            return *this;
        }
    };


    SP_CPUFUNC Mem1<PoseModel> getPoseModel(const Mem1<Mesh3> &model, const double distance, const int level = 2, const int density = 50) {

        const double radius = getModelRadius(model);
        const double unit = 2.0 * radius / density;

        const int size = 300;
        const double f = distance * size / (1.2 * 2.0 * radius);
        const CamParam cam = getCamParam(size, size, f, f);

        Mem1<PoseModel> pmodels;

        // pose
        {
            const int num = getGeodesicMeshNum(level);
            pmodels.resize(num);

            for (int i = 0; i < num; i++) {
                const Vec3 v = getMeshPos(getGeodesicMesh(level, i)) * (-1.0);
                const Pose pose = getPose(getRotDirection(v), getVec(0.0, 0.0, distance));
               
                pmodels[i].pose = pose;
            }
        }

        // contour edge
        {
            const Mem1<Edge> &edges = getModelEdge(model, density);
  
            KdTree<double> kdtree;
            kdtree.init(3);
            for (int i = 0; i < edges.size(); i++) {
                kdtree.addData(&edges[i].pos);
            }
            kdtree.makeTree();

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int i = 0; i < pmodels.size(); i++) {
                PoseModel &pmodel = pmodels[i];
                pmodel.edges.clear();

                const Pose &pose = pmodel.pose;

                Mem2<VecPN3> map;
                renderVecPN(map, cam, pose, model);

                const Mat pmat = getMat(pose);
                const Mat rmat = getMat(pose.rot);

                Mem1<bool> flags(edges.size());
                flags.zero();

                for (int j = 0; j < edges.size(); j++) {
                    const Vec3 pos = pmat * edges[j].pos;
                    const Vec2 pix = mulCamD(cam, prjVec(pos));
                    if (flags[j] == true) continue;

                    const int x = round(pix.x);
                    const int y = round(pix.y);
                    bool contour = false ;
                    for (int v = -1; v <= 1; v++) {
                        for (int u = -1; u <= 1; u++) {
                            const VecPN3 &vec = map(x + u, y + v);
                            if (vec.pos.z == 0.0) {
                                contour = true;
                                goto _exit0;
                            }
                        }
                    }
                _exit0:

                    const Vec3 nrm0 = rmat * edges[j].nrm[0];
                    const Vec3 nrm1 = rmat * edges[j].nrm[1];
                    
                    if (contour == true && dotVec(nrm0, pos) * dotVec(nrm1, pos) <= 0.0)
                    {
                        pmodel.edges.push(edges[j]);

                        const Mem1<int> list = kdtree.search(&edges[j].pos, unit);
                        for (int k = 0; k < list.size(); k++) {
                            flags[list[k]] = true;
                        }
                    }
                }
            }
        }

        // surface point
        {
            const Mem1<VecPN3> &pnts = getModelPoint(model, density);

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int i = 0; i < pmodels.size(); i++) {
                PoseModel &pmodel = pmodels[i];
                pmodel.pnts.clear();

                const Pose &pose = pmodel.pose;

                Mem2<VecPN3> map;
                renderVecPN(map, cam, pose, model);
                
                const Mat pmat = getMat(pose);
                const Mat rmat = getMat(pose.rot);

                for (int j = 0; j < pnts.size(); j++) {
                    const Vec3 pos = pmat * pnts[j].pos;
                    const Vec3 nrm = rmat * pnts[j].nrm;

                    const Vec2 pix = mulCamD(cam, prjVec(pos));

                    const int x = round(pix.x);
                    const int y = round(pix.y);
                    bool visible = false;
                    for (int v = -1; v <= 1; v++) {
                        for (int u = -1; u <= 1; u++) {
                            const VecPN3 &vec = map(x + u, y + v);
                            if (vec.pos.z > pos.z - SP_SMALL) {
                                visible = true;
                                goto _exit1;
                            }
                        }
                    }
                _exit1:

                    if (visible == true && dotVec(nrm, pos) <= 0.0) {
                        pmodel.pnts.push(pnts[j]);
                    }
                }
            }
        }

        return pmodels;
    }

    SP_CPUFUNC int findPoseModel(const Mem1<PoseModel> &pmodels, const Pose &pose) {
        int id = -1;
        double minv = SP_INFINITY;
        for (int i = 0; i < pmodels.size(); i++) {
            Vec3 vec0 = getEuler(pose.rot);
            Vec3 vec1 = getEuler(pmodels[i].pose.rot);
            vec0.z = 0.0;
            vec1.z = 0.0;
            const double dif = difRot(getRotEuler(vec0), getRotEuler(vec1));
            if (dif < minv) {
                minv = dif;
                id = i;
            }
        }
        return id;
    }

    //--------------------------------------------------------------------------------
    // sample model
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool loadBunny(const char *path, Mem1<Mesh3> &model) {
        if (loadPLY(path, model) == false) return false;

        Vec3 center = getModelCenter(model);
        for (int i = 0; i < model.size(); i++) {
            model[i] -= center;

            // m -> mm
            model[i] *= 1000.0;
        }
        return true;
    }

    SP_CPUFUNC void loadPlane(Mem1<Mesh3> &model, const double size) {
        model.clear();

        const double hs = size * 0.5;
        const Vec3 a = getVec(-hs, -hs, 0.0);
        const Vec3 b = getVec(+hs, -hs, 0.0);
        const Vec3 c = getVec(+hs, +hs, 0.0);
        const Vec3 d = getVec(-hs, +hs, 0.0);

        model.push(getMesh(a, b, c));
        model.push(getMesh(a, c, d));
    }

    SP_CPUFUNC void loadGeodesicDorm(Mem1<Mesh3> &model, const double size, const int div) {
        model.clear();

        const int num = getGeodesicMeshNum(div);
        for (int i = 0; i < num; i++) {
            model.push(getGeodesicMesh(div, i) * size);
        }

    }


}

#endif
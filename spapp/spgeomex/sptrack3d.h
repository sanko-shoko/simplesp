//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_TRACK3D_H__
#define __SP_TRACK3D_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spmodel.h"
#include "spapp/spdata/spbmp.h"
#include "spapp/spalgo/sprandomforest.h"

namespace sp{

    class Track3DRF {
    public:
        const int POINT_NUM = 20;
        const int SAMPLE_NUM = 1000;

        const double RAND_TRN = 10.0;
        const double RAND_ROT = 10.0 * SP_PI / 180.0;

        struct GeoNode{
            Pose pose;
            Mem1<Vec3> pnts;
            Mem1<Mem<double> > dataList;
            RandomForestReg rf[6];
        };

        Mem1<GeoNode> m_nodes;

        int m_div;

    public:
 
        Vec3 getDirect(const Pose &pose) {
            return unitVec((invRot(pose.rot) * pose.trn)) * -1.0;
        }

        void train(const Mem1<Mesh3> &model, const int div = 1) {
            srand(0);

            const int gnum = 1;// getGeodesicMeshNum(m_div);
            m_nodes.resize(gnum);

            CamParam cam = getCamParam(640, 480);

            const double distance = getModelDistance(model, cam);
            for (int i = 0; i < gnum; i++) {
                printf("%d\n", i);
                makeTree(m_nodes[i], model, cam, getGeomPose(m_div, i, distance));
            }
        }

        void makeTree(GeoNode &gnode, const Mem1<Mesh3> &model, const CamParam &cam, const Pose &pose) {
            gnode.pose = pose;

            Mem2<double> depth;
            {
                renderDepth(depth, cam, pose, model);
                //static int i = 0;
                //Mem2<Col3> tmp;
                //cnvDepthToImg(tmp, depth, 300, 1000);
                //saveBMP(tmp, strFormat("img%03d.bmp", i++).c_str());
            }
            
            {
                struct Tmp {
                    Vec3 point;
                    double order;

                    bool operator > (const Tmp &pd) const { return order > pd.order; }
                    bool operator < (const Tmp &pd) const { return order < pd.order; }
                };

                Mem1<Tmp> tmps;
                const double angle = randValUnif() * SP_PI;
                const Vec2 nl = getVec(cos(angle), sin(angle));

                for (int v = 0; v < cam.dsize[1]; v++) {
                    for (int u = 0; u < cam.dsize[0]; u++) {
                        const double d = depth(u, v);
                        if (d > 0.0) {
                            const Vec2 npx = invCam(cam, getVec(u, v));
                            
                            Tmp tmp;
                            tmp.point = getVec(npx.x, npx.y, 1.0) * d;
                            tmp.order = dotVec(nl, getVec(u, v));
                            tmps.push(tmp);
                        }
                    }
                }

                sort(tmps);

                const double rate = 1.0;// 0.3 * randValUnif() + 0.5;
                for (int p = 0; p < POINT_NUM; p++) {
                    const int i = rand() % round(rate * tmps.size());
                    const Vec3 vec = invPose(gnode.pose) * tmps[i].point;
                    gnode.pnts.push(vec);
                }
            }

            // make sample
            {
                const Vec3 Nv = getDirect(gnode.pose);

                Mem1<double> Ys[6];
                for (int i = 0; i < SAMPLE_NUM; i++) {
                    const Pose delta = randPoseUnif(RAND_ROT, RAND_TRN);
                    const Pose pose = gnode.pose * delta;

                    const Vec3 euler = getEuler(delta.rot);

                    Mem<double> data = Mem1<double>(POINT_NUM);

                    for (int p = 0; p < POINT_NUM; p++) {
                        const Vec2 npx = prjVec(pose * gnode.pnts[p]);
                        const Vec2 pix = mulCam(cam, npx);

                        const double d = depth(round(pix.x), round(pix.y));
                        if (d > 0.0) {
                            const Vec3 vec = getVec(npx.x, npx.y, 1.0) * d;
                            const Vec3 vec1 = invPose(pose) * vec;
                            const Vec3 vec2 = gnode.pnts[p];
                            const Vec3 dif = vec1 - vec2;
                            const double v = dotVec(Nv, invPose(pose) * vec - gnode.pnts[p]);
                            data[p] = dotVec(Nv, invPose(pose) * vec - gnode.pnts[p]);
                        }
                        else {
                            data[p] = 0.0;
                        }
                    }

                    gnode.dataList.push(data);
                    Ys[0].push(euler.x);
                    Ys[1].push(euler.y);
                    Ys[2].push(euler.z);

                    Ys[3].push(delta.trn.x);
                    Ys[4].push(delta.trn.y);
                    Ys[5].push(delta.trn.z);

                }
                gnode.rf[0].train(gnode.dataList, Ys[0]);
                gnode.rf[1].train(gnode.dataList, Ys[1]);
                gnode.rf[2].train(gnode.dataList, Ys[2]);
                gnode.rf[3].train(gnode.dataList, Ys[3]);
                gnode.rf[4].train(gnode.dataList, Ys[4]);
                gnode.rf[5].train(gnode.dataList, Ys[5]);


            }
        }


        double updatePose(Pose &pose, const CamParam &cam, const Mem2<double> &depth) {
            struct Result{
                double param, eval;
                bool operator > (const Result r) { return this->eval > r.eval; }
                bool operator < (const Result r) { return this->eval < r.eval; }
            };

            const double maxAngle = 35.0 * SP_PI / 180.0;

            Mem1<Result> list[6];
            Mem<double> data = Mem1<double>(POINT_NUM);

            Pose dst;
            //for (int i = 0; i < getGeodesicMeshNum(m_div); i++) {
            for (int i = 0; i < 1; i++) {
                GeoNode &node = m_nodes[i];

                const Vec3 ref = getDirect(node.pose);
                const Vec3 vec = getDirect(pose);

                const double angle = acos(dotVec(vec, ref));
                if (fabs(angle) < maxAngle) {
                    const Vec3 Nv = getDirect(node.pose);

                    for (int p = 0; p < POINT_NUM; p++) {
                        const Vec2 npx = prjVec(pose * node.pnts[p]);
                        const Vec2 pix = mulCam(cam, npx);

                        const double d = depth(round(pix.x), round(pix.y));
                        if (d > 0.0) {
                            const Vec3 v = getVec(npx.x, npx.y, 1.0) * d;
                            data[p] = dotVec(Nv, invPose(pose) * v - node.pnts[p]);
                        }
                        else {
                            data[p] = 0.0;
                        }
                    }
                    Pose delta = zeroPose();
                    Vec3 euler;
                    euler.x = node.rf[0].estimate(data)[0];
                    euler.y = node.rf[1].estimate(data)[0];
                    euler.z = node.rf[2].estimate(data)[0];
                    delta.rot = getRotEuler(euler);
                    delta.trn.x = node.rf[3].estimate(data)[0];
                    delta.trn.y = node.rf[4].estimate(data)[0];
                    delta.trn.z = node.rf[5].estimate(data)[0];
                    print(euler);
                    print(delta);

                    dst = pose * invPose(delta);
                }
            }
            pose = dst;

            return 0.0;
        }
    };

}
#endif
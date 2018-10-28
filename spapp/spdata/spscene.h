//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SCENE_H__
#define __SP_SCENE_H__

#include "spcore/spcore.h"
#include "spapp/spimgex/spfeature.h"

namespace sp{

    class View {
    public:
        bool valid;

        // camera parameter
        CamParam cam;

        // camera pose
        Pose pose;
 
        // captured image
        Mem2<Col3> img;

        // features
        Mem1<Feature> fts;
  
        View() {
            valid = false;

            cam = getCamParam(0, 0);
            pose = zeroPose();
        }

        View(const View &view) {
            *this = view;
        }

        View& operator = (const View &view) {
            valid = view.valid;

            cam = view.cam;
            pose = view.pose;

            img = view.img;
            fts = view.fts;

            return *this;
        }

    };

    class MapPnt : public VecPN3{
    public:
        // point color
        Col3 col;

        // projection err
        double err;

        // view index
        Mem1<View*> views;

        // feature index
        Mem1<Feature*> fts;

        MapPnt() {
            pos = getVec(0.0, 0.0, 0.0);
            nrm = getVec(0.0, 0.0, 0.0);
            col = getCol(0, 0, 0);
            err = SP_INFINITY;
        }

        MapPnt(const MapPnt &mpnt) {
            *this = mpnt;
        }

        MapPnt& operator = (const MapPnt &mpnt) {
            pos = mpnt.pos;
            nrm = mpnt.nrm;

            col = mpnt.col;
            err = mpnt.err;

            views = mpnt.views;
            fts = mpnt.fts;

            return *this;
        }

    public:

        void updatePrjErr() {

            Mem1<double> errs;

            const int num = views.size();
            for (int i = 0; i < num; i++) {
                const Pose &pose = views[i]->pose;
                const CamParam &cam = views[i]->cam;
                const Vec2 &pix = fts[i]->pix;

                errs.push(calcPrjErr(pose, cam, pix, pos));
            }

            err = (errs.size() > 0) ? medianVal(errs) : SP_INFINITY;
        }

        void updateCol() {

            Vec3 vec = getVec(0.0, 0.0, 0.0);

            const int num = views.size();
            for (int i = 0; i < num; i++) {
                const Vec2 &pix = fts[i]->pix;
                vec += getVec(acsc(views[i]->img, pix.x, pix.y)) / num;
            }

            col = getCol(vec);
        }
    };

}

#endif
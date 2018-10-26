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

        Col3 col;

        double err;

        // index -> [view, feature]
        Mem1<MemA<int, 2> > index;

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

            index = mpnt.index;
            return *this;
        }
    };

}

#endif
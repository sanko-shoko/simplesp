//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MAP_H__
#define __SP_MAP_H__

#include "spcore/spcore.h"
#include "spapp/spimgex/spfeature.h"

namespace sp{

    struct View {

        // camera parameter
        CamParam cam;

        // captured image
        Mem2<Col3> img;

        // features
        Mem1<Feature> fts;

        // pose state
        enum State {
            POSE_NULL = 0,
            POSE_VALID = 1
        };
        State state;

        // camera pose
        Pose pose;

        // view links
        Mem1<int> links;

        View() {
            cam = getCamParam(0, 0);

            state = POSE_NULL;
            pose = zeroPose();
        }

        View(const View &view) {
            *this = view;
        }

        View& operator = (const View &view) {
            cam = view.cam;
            img = view.img;
            fts = view.fts;

            state = view.state;
            pose = view.pose;

            links = view.links;
            return *this;
        }

    };

    struct MapPoint {

        Vec3 pos;

        Col3 col;

        double err;

        // index -> [view, feature]
        Mem1<MemA<int, 2> > index;

        MapPoint() {
            pos = getVec(0.0, 0.0, 0.0);
            col = getCol(0, 0, 0);
            err = SP_INFINITY;
        }

        MapPoint(const MapPoint &mpnt) {
            *this = mpnt;
        }

        MapPoint& operator = (const MapPoint &mpnt) {
            pos = mpnt.pos;
            col = mpnt.col;
            err = mpnt.err;
            index = mpnt.index;
            return *this;
        }
    };

}

#endif
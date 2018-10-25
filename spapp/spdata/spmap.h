//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MAP_H__
#define __SP_MAP_H__

#include "spcore/spcore.h"
#include "spapp/spimgex/spfeature.h"

namespace sp{

    struct ViewData {

        // camera parameter
        CamParam cam;

        // captured image
        Mem2<Col3> img;

        // features
        Mem1<Feature> fts;

        // camera pose is valid
        bool valid;

        // camera pose
        Pose pose;

        // pair links
        Mem1<int> links;

        ViewData() {
            cam = getCamParam(0, 0);

            valid = false;
            pose = zeroPose();
        }

        ViewData(const ViewData &view) {
            *this = view;
        }

        ViewData& operator = (const ViewData &view) {
            cam = view.cam;
            img = view.img;
            fts = view.fts;

            valid = view.valid;
            pose = view.pose;

            links = view.links;
            return *this;
        }

    };

    struct PairData {
        bool valid;

        // pair id
        int a, b;

        // match data
        Mem1<int> matches;

        // match features eval
        double eval;

        PairData() {
            valid = false;
            a = -1;
            b = -1;
            eval = -1.0;
        }

        PairData(const PairData &pair) {
            *this = pair;
        }

        PairData& operator = (const PairData &pair) {
            valid = pair.valid;
            matches = pair.matches;
            a = pair.a;
            b = pair.b;
            eval = pair.eval;
            return *this;
        }
    };

    struct MapData {

        Vec3 pos;

        Col3 col;

        double err;

        // index -> [view, feature]
        Mem1<MemA<int, 2> > index;

        MapData() {
            pos = getVec(0.0, 0.0, 0.0);
            col = getCol(0, 0, 0);
            err = SP_INFINITY;
        }

        MapData(const MapData &map) {
            *this = map;
        }

        MapData& operator = (const MapData &map) {
            pos = map.pos;
            col = map.col;
            err = map.err;
            index = map.index;
            return *this;
        }
    };

}

#endif
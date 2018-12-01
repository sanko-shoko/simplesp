//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FEATURE_H__
#define __SP_FEATURE_H__

#include "spcore/spcore.h"
#include "spapp/spgeom/spgeom.h"

namespace sp {

    class MapPnt;

    class Dsc {

    public:
        // dimension
        int dim;

        // real value
        Mem1<Byte> val;

        // binary
        Mem1<Byte> bin;

        // descripter type
        enum Type {
            DSC_NULL = 0,
            DSC_SIFT = 1
        };
        Type type;

        Dsc() {
            type = DSC_SIFT;
        }

        Dsc(const Dsc &dsc) {
            *this = dsc;
        }
        
        Dsc& operator = (const Dsc &dsc) {
            dim = dsc.dim;
            val = dsc.val;
            bin = dsc.bin;

            type = dsc.type;

            return *this;
        }
    };

    class Feature {

    public:

        // point
        Vec2 pix;

        // direct
        Vec2 drc;

        // scale
        double scl;

        // contrast
        double cst;

        // descripter
        Dsc dsc;

        // map point;
        MapPnt *mpnt;

    public:

        Feature() {
            pix = getVec(0.0, 0.0);
            drc = getVec(0.0, 0.0);
            scl = 0.0;
            mpnt = NULL;
        }

        Feature(const Feature &ft) {
            *this = ft;
        }

        Feature& operator = (const Feature &ft) {
            pix = ft.pix;
            drc = ft.drc;
            scl = ft.scl;
            cst = ft.cst;

            dsc = ft.dsc;
            mpnt = ft.mpnt;

            return *this;
        }

    };

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

    class MapPnt : public VecPN3 {
    public:
        bool valid;

        // point color
        Col3 col;

        // projection err
        double err;

        // view index
        Mem1<View*> views;

        // feature index
        Mem1<Feature*> fts;

        Mem1<double> errs;

        MapPnt() {
            valid = false;

            pos = getVec(0.0, 0.0, 0.0);
            nrm = getVec(0.0, 0.0, 0.0);
            col = getCol(0, 0, 0);
            err = SP_INFINITY;
        }

        MapPnt(const MapPnt &mpnt) {
            *this = mpnt;
        }

        MapPnt& operator = (const MapPnt &mpnt) {
            valid = mpnt.valid;

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
            if (valid == false) return;

            const int num = views.size();

            errs.resize(num);

            for (int i = 0; i < num; i++) {
                const Pose &pose = views[i]->pose;
                const CamParam &cam = views[i]->cam;
                const Vec2 &pix = fts[i]->pix;

                errs[i] = calcPrjErr(pose, cam, pix, pos);
            }

            err = (errs.size() > 0) ? medianVal(errs) : SP_INFINITY;
        }

        void updateCol() {
            if (valid == false) return;

            Vec3 vec = getVec(0.0, 0.0, 0.0);

            const int num = views.size();

            for (int i = 0; i < num; i++) {
                const Vec2 &pix = fts[i]->pix;
                vec += getVec(acsc(views[i]->img, pix.x, pix.y)) / num;
            }

            col = getCol(vec);
        }
    };


    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC int getMatchCnt(const Mem1<int> &matches) {
        int cnt = 0;
        for (int i = 0; i < matches.size(); i++) {
            if (matches[i] >= 0) cnt++;
        }
        return cnt;
    };

    SP_CPUFUNC double getMatchEval(const Mem1<int> &matches) {
        const int minv = 10;
        const int maxv = 100;

        const int n = minVal(maxv, matches.size());
        const int c = minVal(maxv, getMatchCnt(matches));

        const int v = c - minv;
        const int m = maxv - minv;
        if (v <= 0) return 0.0;

        // c: minn..maxv, scale: 0..1
        const double scale = 1.0 - 1.0 / v + v / (m * m);
        return scale * c / n;
    };

    SP_CPUFUNC Mem1<Vec2> getMatchPixs(const Mem1<Feature> &fts, const Mem1<int> &matches, const bool flag = true) {
        Mem1<Vec2> pixs;
        for (int i = 0; i < matches.size(); i++) {
            const int j = matches[i];
            if (j < 0) continue;
            const int f = (flag == true) ? i : j;
            pixs.push(fts[f].pix);
        }
        return pixs;
    }

    SP_CPUFUNC int findMatch(const Feature &ft, const Mem1<Feature> &fts) {

        int id = -1;

        if (ft.dsc.type == Dsc::Type::DSC_SIFT) {
            const double MIN_NCC = 0.9;
            const double MIN_BIN = 0.8;

            double maxv = MIN_NCC;

            // SIFT dim = 128
            const int dim = 128;

            for (int i = 0; i < fts.size(); i++) {
                if (ft.cst * fts[i].cst <= 0.0) continue;

                {
                    const double btest = static_cast<double>(cntBit(ft.dsc.bin.ptr, fts[i].dsc.bin.ptr, ft.dsc.bin.size())) / dim;
                    if (btest < MIN_BIN) continue;
                }

                const float *data0 = reinterpret_cast<float*>(ft.dsc.val.ptr);
                const float *data1 = reinterpret_cast<float*>(fts[i].dsc.val.ptr);

                double sum = 0.0;
                for (int d = 0; d < dim; d++) {
                    sum += (*data0++) * (*data1++);
                }

                if (sum > maxv) {
                    maxv = sum;
                    id = i;
                }
            }
        }

        return id;
    }

    SP_CPUFUNC Mem1<int> findMatch(const Mem1<Feature> &fts0, const Mem1<Feature> &fts1, const bool crossCheck = true) {
        Mem1<int> matches(fts0.size());

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int i = 0; i < fts0.size(); i++) {
            matches[i] = -1;

            int j, k;
            {
                j = findMatch(fts0[i], fts1);
                if (j < 0) continue;
            }

            // cross check
            if (crossCheck == true) {
                k = findMatch(fts1[j], fts0);
                if (k != i) continue;
            }

            matches[i] = j;
        }

        return matches;
    }

    SP_CPUFUNC void prepareMatch(Feature &ft) {

        if (ft.dsc.type == Dsc::Type::DSC_SIFT) {

            const int dim = 128;

            const float *data = reinterpret_cast<float*>(ft.dsc.val.ptr);

            const float thresh = static_cast<float>(1.0 / sqrt(dim));

            Mem1<Byte> *tmp = const_cast<Mem1<Byte>*>(&ft.dsc.bin);

            tmp->resize((dim + 8 - 1) / 8);
            cnvBit(tmp->ptr, tmp->size(), data, dim, thresh);
        }
    }

    SP_CPUFUNC void prepareMatch(Mem1<Feature> &fts) {
        for (int i = 0; i < fts.size(); i++) {
            prepareMatch(fts[i]);
        }
    }
}

#endif

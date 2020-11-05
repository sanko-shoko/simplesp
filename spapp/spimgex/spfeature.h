//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
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
            DSC_SIFT = 1,
            DSC_CFBlob = 2
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

    class Ftr {

    public:

        // point
        Vec2 pix;

        // direct
        Vec2 drc;

        // scale
        SP_REAL scl;

        // contrast
        SP_REAL cst;

        // descripter
        Dsc dsc;

        // map point;
        MapPnt *mpnt;

    public:

        Ftr() {
            pix = getVec2(0.0, 0.0);
            drc = getVec2(0.0, 0.0);
            scl = 0.0;
            mpnt = NULL;
        }

        Ftr(const Ftr &ftr) {
            *this = ftr;
        }

        Ftr& operator = (const Ftr &ftr) {
            pix = ftr.pix;
            drc = ftr.drc;
            scl = ftr.scl;
            cst = ftr.cst;

            dsc = ftr.dsc;
            mpnt = ftr.mpnt;

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
        Mem1<Ftr> ftrs;

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
            ftrs = view.ftrs;

            return *this;
        }

    };

    class MapPnt : public VecPD3 {

    public:

        bool valid;

        // point color
        Col3 col;

        // projection err
        SP_REAL err;

        // view index
        Mem1<View*> views;

        // feature index
        Mem1<Ftr*> ftrs;

        Mem1<SP_REAL> errs;

        MapPnt() {
            valid = false;

            pos = getVec3(0.0, 0.0, 0.0);
            drc = getVec3(0.0, 0.0, 0.0);
            col = getCol3(0, 0, 0);
            err = SP_INFINITY;
        }

        MapPnt(const MapPnt &mpnt) {
            *this = mpnt;
        }

        MapPnt& operator = (const MapPnt &mpnt) {
            valid = mpnt.valid;

            pos = mpnt.pos;
            drc = mpnt.drc;

            col = mpnt.col;
            err = mpnt.err;

            views = mpnt.views;
            ftrs = mpnt.ftrs;

            return *this;
        }

    public:

        void updateErr() {
            if (valid == false) return;

            const int num = views.size();

            errs.resize(num);

            for (int i = 0; i < num; i++) {
                const Pose &pose = views[i]->pose;
                const CamParam &cam = views[i]->cam;
                const Vec2 &pix = ftrs[i]->pix;

                errs[i] = calcPrjErr(pose, cam, pix, pos);
            }

            err = (errs.size() > 0) ? medianVal(errs) : SP_INFINITY;
        }

        void updateCol() {
            if (valid == false) return;

            Vec3 vec = getVec3(0.0, 0.0, 0.0);

            const int num = views.size();

            for (int i = 0; i < num; i++) {
                const Vec2 &pix = ftrs[i]->pix;
                vec += cast<Vec3>(acsc(views[i]->img, pix.x, pix.y));
            }

            col = cast<Col3>(vec / (num));
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

    SP_CPUFUNC SP_REAL getMatchEval(const Mem1<int> &matches) {
        const int minv = 10;
        const int maxv = 100;

        const int n = min(maxv, matches.size());
        const int c = min(maxv, getMatchCnt(matches));

        const int v = c - minv;
        const int m = maxv - minv;
        if (v <= 0) return 0.0;

        // c: minn..maxv, scale: 0..1
        const SP_REAL scale = 1.0 - 1.0 / v + v / (m * m);
        return scale * c / n;
    };

    SP_CPUFUNC Mem1<Vec2> getMatchPixs(const Mem1<Ftr> &ftrs, const Mem1<int> &matches, const bool flag = true) {
        Mem1<Vec2> pixs;
        pixs.reserve(matches.size());
        for (int i = 0; i < matches.size(); i++) {
            const int j = matches[i];
            if (j < 0) continue;
            const int f = (flag == true) ? i : j;
            pixs.push(ftrs[f].pix);
        }
        return pixs;
    }

    SP_CPUFUNC int findMatch(const Ftr &ftr, const Mem1<Ftr> &ftrs) {

        int id = -1;

        switch (ftr.dsc.type) {
        case Dsc::Type::DSC_SIFT:
        {
            const SP_REAL MIN_NCC = 0.9f;
            const SP_REAL MIN_BIN = MIN_NCC * 0.9f;

            SP_REAL maxv = MIN_NCC;

            // SIFT dim = 128
            const int dim = 128;

            for (int i = 0; i < ftrs.size(); i++) {
                if (ftr.cst * ftrs[i].cst <= 0.0) continue;

                {
                    const SP_REAL btest = static_cast<SP_REAL>(cntBit(ftr.dsc.bin.ptr, ftrs[i].dsc.bin.ptr, ftr.dsc.bin.size())) / dim;
                    if (btest < MIN_BIN) continue;
                }

                const float *data0 = reinterpret_cast<float*>(ftr.dsc.val.ptr);
                const float *data1 = reinterpret_cast<float*>(ftrs[i].dsc.val.ptr);

                SP_REAL sum = 0.0;
                for (int d = 0; d < dim; d++) {
                    sum += (*data0++) * (*data1++);
                }

                if (sum > maxv) {
                    maxv = sum;
                    id = i;
                }
            }
            break;
        }
        case Dsc::Type::DSC_CFBlob:
        {
            const SP_REAL MIN_NCC = 0.9f;
            const SP_REAL MIN_BIN = MIN_NCC * 0.9f;

            SP_REAL maxv = MIN_NCC;

            // SIFT dim = 128
            const int dim = 128;

            for (int i = 0; i < ftrs.size(); i++) {
                if (ftr.cst * ftrs[i].cst <= 0.0) continue;

                {
                    const SP_REAL btest = static_cast<SP_REAL>(cntBit(ftr.dsc.bin.ptr, ftrs[i].dsc.bin.ptr, ftr.dsc.bin.size())) / dim;
                    if (btest < MIN_BIN) continue;
                }

                const float *data0 = reinterpret_cast<float*>(ftr.dsc.val.ptr);
                const float *data1 = reinterpret_cast<float*>(ftrs[i].dsc.val.ptr);

                SP_REAL sum = 0.0;
                for (int d = 0; d < dim; d++) {
                    sum += (*data0++) * (*data1++);
                }

                if (sum > maxv) {
                    maxv = sum;
                    id = i;
                }
            }
            break;
        }
        }
        return id;
    }

    SP_CPUFUNC Mem1<int> findMatch(const Mem1<Ftr> &ftrs0, const Mem1<Ftr> &ftrs1, const bool crossCheck = true) {
        Mem1<int> matches(ftrs0.size());

//#if SP_USE_OMP
//#pragma omp parallel for
//#endif
        for (int i = 0; i < ftrs0.size(); i++) {
            matches[i] = -1;

            int j, k;
            {
                j = findMatch(ftrs0[i], ftrs1);
                if (j < 0) continue;
            }

            // cross check
            if (crossCheck == true) 
            {
                k = findMatch(ftrs1[j], ftrs0);
                if (k != i) continue;
            }

            matches[i] = j;
        }

        return matches;
    }

}

#endif

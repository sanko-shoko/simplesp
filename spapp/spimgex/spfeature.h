//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FEATURE_H__
#define __SP_FEATURE_H__

#include "spcore/spcore.h"
#include "spapp/spgeom/spgeom.h"

namespace sp {
    class MapPnt;

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
        Mem1<Byte> dsc;

        // descripter type
        enum Type {
            DSC_NULL = 0,
            DSC_SIFT = 1
        };
        Type type;

        // map point;
        MapPnt *mpnt;

    public:

        Feature() {
            pix = getVec(0.0, 0.0);
            drc = getVec(0.0, 0.0);
            scl = 0.0;
            mpnt = NULL;
            type = DSC_SIFT;
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
            type = ft.type;

            bin = ft.bin;
            return *this;
        }

    public:

        // binary
        Mem1<Byte> bin;
    };


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

        if (ft.type == Feature::Type::DSC_SIFT) {
            const double MIN_NCC = 0.9;
            const double MIN_BIN = 0.8;

            double maxv = MIN_NCC;

            // dim = 128
            const int dim = ft.dsc.size() / sizeof(float);

            for (int i = 0; i < fts.size(); i++) {
                if (ft.cst * fts[i].cst <= 0.0) continue;

                if (ft.bin.size() > 0 && fts[i].bin.size() > 0) {
                    const double btest = static_cast<double>(cntBit(ft.bin.ptr, fts[i].bin.ptr, ft.bin.size())) / dim;
                    if (btest < MIN_BIN) continue;
                }

                const float *data0 = reinterpret_cast<float*>(ft.dsc.ptr);
                const float *data1 = reinterpret_cast<float*>(fts[i].dsc.ptr);

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

        if (ft.type == Feature::Type::DSC_SIFT) {

            const int dim = ft.dsc.size() / sizeof(float);

            const float *data = reinterpret_cast<float*>(ft.dsc.ptr);

            const float thresh = static_cast<float>(1.0 / sqrt(dim));

            Mem1<Byte> *tmp = const_cast<Mem1<Byte>*>(&ft.bin);

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

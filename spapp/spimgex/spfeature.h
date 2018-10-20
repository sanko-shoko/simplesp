//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FEATURE_H__
#define __SP_FEATURE_H__

#include "spcore/spcore.h"
#include "spapp/spgeom/spgeometry.h"

namespace sp{

    class Feature{
    public:

        // feature point
        Vec2 pix;

        // feature direct
        Vec2 drc;

        // feature scale
        double scl;

        // feature descripter
        Mem1<Byte> dsc;

        // descripter type
        enum Type{
            DSC_32F = 0
        };
        Type type;

        // map id;
        int mid;

        Feature() {
            pix = getVec(0.0, 0.0);
            drc = getVec(0.0, 0.0);
            scl = 0.0;
            mid = -1;
            type = DSC_32F;
        }

        Feature(const Feature &ft) {
            *this = ft;
        }

        Feature& operator = (const Feature &ft) {
            pix = ft.pix;
            drc = ft.drc;
            scl = ft.scl;
            dsc = ft.dsc;
            mid = ft.mid;
            type = ft.type;
            return *this;
        }

    };

    SP_CPUFUNC int getMatchCnt(const Mem1<int> &matches) {
        int cnt = 0;
        for (int i = 0; i < matches.size(); i++) {
            if (matches[i] >= 0) cnt++;
        }
        return cnt;
    };

    SP_CPUFUNC double getMatchRate(const Mem1<int> &matches) {
        const int thresh = 10;
        if (matches.size() <= thresh) return 0.0;

        const double scale = 1.0 - 1.0 / (matches.size() - thresh);
        return getMatchCnt(matches) * scale / matches.size();
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

    SP_CPUFUNC int findMatch(const Feature &ft, const Mem1<Feature> &fts, const Mem1<bool> mask = Mem1<bool>()) {

        int id = -1;

        if (ft.type == Feature::Type::DSC_32F) {
            const double NCC_MIN = 0.9;
            double maxv = NCC_MIN;

            for (int i = 0; i < fts.size(); i++) {
                if (mask.size() != 0 && mask[i] == false) continue;

                const int dim = ft.dsc.size() / sizeof(float);
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

    SP_CPUFUNC Mem1<int> findMatchFMat(const Mem1<Feature> &fts0, const Mem1<Feature> &fts1, const Mat &F, const bool crossCheck = true) {
        Mem1<int> matches(fts0.size());

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int i = 0; i < fts0.size(); i++){
            matches[i] = -1;

            int j, k;
            {
                Mem1<bool> mask;
                for (int j = 0; j < fts1.size(); j++) {
                    const double err = errFMat(F, fts0[i].pix, fts1[j].pix);
                    mask.push(evalErr(err) > 0.0);
                }
                j = findMatch(fts0[i], fts1, mask);
                if (j < 0) continue;
            }

            // cross check
            if (crossCheck == true) {
                Mem1<bool> mask;
                for (int k = 0; k < fts0.size(); k++){
                    const double err = errFMat(F, fts0[k].pix, fts1[j].pix);
                    mask.push(evalErr(err) > 0.0);
                }

                k = findMatch(fts1[j], fts0, mask);
                if (k != i) continue;
            }

            matches[i] = j;
        }

        return matches;
    }

}

#endif
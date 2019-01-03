//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BIN_H__
#define __SP_BIN_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spfilter.h"

namespace sp{

    SP_CPUFUNC void binalize(Mem2<Byte> &dst, const Mem2<Byte> &src, const int thresh, const bool inv = false){
        dst.resize(src.dsize);
        const Mem2<Byte> &tmp = (&dst != &src) ? src : clone(src);

        if (inv == false) {
            for (int i = 0; i < dst.size(); i++) {
                dst[i] = (tmp[i] >= thresh) ? 255 : 0;
            }
        }
        else {
            for (int i = 0; i < dst.size(); i++) {
                dst[i] = (tmp[i] < thresh) ? 255 : 0;
            }
        }

    }

    SP_CPUFUNC void binalizeAdapt(Mem2<Byte> &dst, const Mem2<Byte> &src, const bool inv = false){

        int hist[256] = { 0 };
        for (int i = 0; i < src.size(); i++) {
            hist[src[i]]++;
        }

        int thresh = 0;
        double maxEval = 0.0;
        for (int t = 1; t < 256; t++) {
            double cnt0 = 0.0;
            double sum0 = 0.0;
            for (int i = 0; i < t; i++) {
                cnt0 += hist[i];
                sum0 += i * hist[i];
            }
            if (cnt0 == 0) continue;

            double cnt1 = 0.0;
            double sum1 = 0.0;
            for (int i = t; i < 256; i++) {
                cnt1 += hist[i];
                sum1 += i * hist[i];
            }
            if (cnt1 == 0) continue;

            const double mean0 = sum0 / cnt0;
            const double mean1 = sum1 / cnt1;

            const double eval = cnt0 * cnt1 * (mean0 - mean1) * (mean0 - mean1);
            if (eval > maxEval) {
                maxEval = eval;
                thresh = t;
            }
        }
        SP_PRINTD("binalizeAdapt thresh %d\n", thresh);
        binalize(dst, src, thresh, inv);
    }

    SP_CPUFUNC void binalizeBlock(Mem2<Byte> &dst, const Mem2<Byte> &src, const int blockSize, const bool inv = false){
        dst.resize(src.dsize);
        const Mem2<Byte> &tmp = (&dst != &src) ? src : clone(src);

        const int step = src.dsize[0];

        const Byte *pSrc = tmp.ptr;
        Byte *pDst = dst.ptr;

        for (int v = 0; v < dst.dsize[1]; v += blockSize){
            for (int u = 0; u < dst.dsize[0]; u += blockSize){
                const int sizeX = minVal(u + blockSize, src.dsize[0]);
                const int sizeY = minVal(v + blockSize, src.dsize[1]);

                Byte maxv = 0;
                Byte minv = SP_BYTEMAX;

                const int margin = blockSize / 2;
                for (int y = v - margin; y < sizeY + margin; y++){
                    for (int x = u - margin; x < sizeX + margin; x++){
                        const Byte val = tmp(x, y);
                        maxv = maxVal(maxv, val);
                        minv = minVal(minv, val);
                    }
                }

                const int thresh = (maxv + minv) / 2;
                if (inv == false) {
                    for (int y = v; y < sizeY; y++) {
                        for (int x = u; x < sizeX; x++) {
                            pDst[y * step + x] = (pSrc[y * step + x] >= thresh) ? 255 : 0;
                        }
                    }
                }
                else {
                    for (int y = v; y < sizeY; y++) {
                        for (int x = u; x < sizeX; x++) {
                            pDst[y * step + x] = (pSrc[y * step + x] < thresh) ? 255 : 0;
                        }
                    }
                }
            }
        }
    }


}

#endif
﻿//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FILTERF_H__
#define __SP_FILTERF_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // gaussian filter 
    //--------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------
    // box filter 
    //--------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------
    // max/min filter 
    //--------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------
    // laplacian filter 
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void laplacianFilter3x3Fast(Mem<float> &dst, const Mem<Byte> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
     
        const int step = src.dsize[0];

        const Byte *psrc = src.ptr;
        float *pdst = dst.ptr;

        for (int v = 0; v < dst.dsize[1]; v++) {
            const int va = (v == 0) ? v + 1 : v;
            const int vb = (v == dst.dsize[1] - 1) ? v - 1 : v;

            for (int u = 0; u < dst.dsize[0]; u++) {
                const int ua = (u == 0) ? u + 1 : u;
                const int ub = (u == dst.dsize[0] - 1) ? u - 1 : u;

                short sum = 0;
                const Byte a0 = psrc[(va - 1) * step + (ua - 1)];
                const Byte a1 = psrc[(va - 1) * step + (ua + 0)];
                const Byte a2 = psrc[(va - 1) * step + (ua + 1)];
                sum += a0 + a1 + a2;

                const Byte a3 = psrc[(v + 0) * step + (u - 1)];
                const Byte a4 = psrc[(v + 0) * step + (u + 0)];
                const Byte a5 = psrc[(v + 0) * step + (u + 1)];
                sum += a3 + a4 + a5;

                const Byte a6 = psrc[(vb + 1) * step + (ub - 1)];
                const Byte a7 = psrc[(vb + 1) * step + (ub + 0)];
                const Byte a8 = psrc[(vb + 1) * step + (ub + 1)];
                sum += a6 + a7 + a8;

                cnvVal(pdst[v * step + u], sum / 16.0);
            }
        }
    }

    SP_CPUFUNC void laplacianFilter3x3Fast2(Mem<float> &dst, const Mem<Byte> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        const int w = src.dsize[0];
        const int h = src.dsize[1];

        const Byte *psrc = src.ptr;
        float *pdst = dst.ptr;

        for (int v = 0; v < h; v++) {
            const int vs = (v == 0) ? v + 1 : v;
            const int vc = v;
            const int ve = (v == dst.dsize[1] - 1) ? v - 1 : v;
            
            short pre0 = 0;
            short pre1 = 0;

            const Byte *psrc0 = &psrc[(vs - 1) * w];
            const Byte *psrc1 = &psrc[(vc + 0) * w];
            const Byte *psrc2 = &psrc[(ve + 1) * w];
            {
                const Byte a0 = *psrc0;
                const Byte a1 = *psrc1;
                const Byte a2 = *psrc2;

                pre0 = a0 + a1 + a2;
                pre1 = a0 + a1 + a2;
            }

            for (int u = 0; u < w - 1; u++) {

                short sum = 9 * *psrc1;

                const Byte a0 = *(++psrc0);
                const Byte a1 = *(++psrc1);
                const Byte a2 = *(++psrc2);

                const short tmp = a0 + a1 + a2;
                sum -= pre0 + pre1 + tmp;

                pre0 = pre1;
                pre1 = tmp;

                cnvVal(*pdst++, sum / 16.0);
            }
            {
                const int u = w - 1;
                short sum = 9 * *psrc1;

                const Byte a0 = *(psrc0);
                const Byte a1 = *(psrc1);
                const Byte a2 = *(psrc2);

                const short tmp = a0 + a1 + a2;
                sum -= pre0 + pre1 + tmp;

                cnvVal(*pdst++, sum / 16.0);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // sobel filter 
    //--------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------
    // scharr filter 
    //--------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------
    // median filter 
    //--------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------
    // normalize filter 
    //--------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------
    //  bilateral filter 
    //--------------------------------------------------------------------------------

  
}

#endif
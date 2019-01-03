//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FILTER_H__
#define __SP_FILTER_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE, typename TYPE0, typename ELEM0 = ELEM>
    SP_CPUFUNC void filter(Mem<TYPE> &dst, const Mem<TYPE0> &src, const Mem<double> &kernel){
        SP_ASSERT(isValid(2, src) && isValid(2, kernel));
        
        dst.resize(2, src.dsize);
        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>*>(&dst) != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int halfX = kernel.dsize[0] / 2;
        const int halfY = kernel.dsize[1] / 2;

        const Rect rect = getRect2(dst.dsize);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){

                for (int c = 0; c < ch; c++){
                    double sum = 0.0, div = 0.0;

                    for (int ky = -halfY; ky <= halfY; ky++){
                        for (int kx = -halfX; kx <= halfX; kx++){
                            if (isInRect2(rect, u + kx, v + ky) == false) continue;
                            
                            const ELEM0 &val = acs2<TYPE0, ELEM0>(tmp, u + kx, v + ky, c);
                            const double s = acs2(kernel, kx + halfX, ky + halfY);

                            sum += s * val;
                            div += fabs(s);
                        }
                    }

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), (div > 0.0) ? sum / div : 0.0);
                }
            }
        }

    }

    template <typename TYPE, typename ELEM = TYPE, typename TYPE0, typename ELEM0 = ELEM>
    SP_CPUFUNC void filterX(Mem<TYPE> &dst, const Mem<TYPE0> &src, const Mem<double> &kernel){
        SP_ASSERT(isValid(2, src) && isValid(1, kernel));

        dst.resize(2, src.dsize);
        const Mem2<TYPE> &tmp = (reinterpret_cast<const Mem<TYPE0>*>(&dst) != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int halfX = kernel.dsize[0] / 2;

        const Rect rect = getRect2(dst.dsize);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){

                for (int c = 0; c < ch; c++){
                    double sum = 0.0, div = 0.0;

                    for (int kx = -halfX; kx <= halfX; kx++){
                        if (isInRect2(rect, u + kx, v) == false) continue;
                        
                        const ELEM0 &val = acs2<TYPE0, ELEM0>(tmp, u + kx, v, c);
                        const double s = acs1(kernel, kx + halfX);

                        sum += s * val;
                        div += fabs(s);
                    }

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), (div > 0.0) ? sum / div : 0.0);
                }
            }
        }
    }


    template <typename TYPE, typename ELEM = TYPE, typename TYPE0, typename ELEM0 = ELEM>
    SP_CPUFUNC void filterY(Mem<TYPE> &dst, const Mem<TYPE0> &src, const Mem<double> &kernel){
        SP_ASSERT(isValid(2, src) && isValid(1, kernel));

        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (reinterpret_cast<const Mem<TYPE0>*>(&dst) != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int halfY = kernel.dsize[0] / 2;

        const Rect rect = getRect2(dst.dsize);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){

                for (int c = 0; c < ch; c++){
                    double sum = 0.0, div = 0.0;

                    for (int ky = -halfY; ky <= halfY; ky++){
                        if (isInRect2(rect, u, v + ky) == false) continue;
                        
                        const ELEM0 &val = acs2<TYPE0, ELEM0>(tmp, u, v + ky, c);
                        const double s = acs1(kernel, ky + halfY);

                        sum += s * val;
                        div += fabs(s);
                    }

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), (div > 0.0) ? sum / div : 0.0);
                }
            }
        }
    }

    //--------------------------------------------------------------------------------
    // gaussian filter 
    //--------------------------------------------------------------------------------

    // filter window size <-> gaussian sigma
    // 
    // half = (window size) / 2
    // sigma = 0.3 * (half - 1) + 0.8
    // half = round((sigma - 0.8) / 0.3 + 1)

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void gaussianFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const double sigma = 0.8){
        SP_ASSERT(isValid(2, src));

        const int half = maxVal(1, round((sigma - 0.8) / 0.3 + 1));

        Mem1<double> kernel(2 * half + 1);
        for (int k = -half; k <= half; k++){
            const double r = k * k;
            kernel(k + half) = exp(-r / (2.0 * square(sigma)));
        }

        Mem2<TYPE> tmp;
        filterX<TYPE, ELEM>(tmp, src, kernel);
        filterY<TYPE, ELEM>(dst, tmp, kernel);
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void gaussianFilter3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>*>(&dst) != &src) ? src : clone(src);

        const int dsize0 = src.dsize[0];
        const int dsize1 = src.dsize[1];
        const int dsize[2] = { dsize0, dsize1 };

        dst.resize(2, dsize);

        const TYPE0 *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < dsize1; v++) {
            const int v0 = v + ((v == 0) ? 0 : -1);
            const int v1 = v + 0;
            const int v2 = v + ((v == dsize1 - 1) ? 0 : +1);

            const TYPE0 *psrc0 = &psrc[v0 * dsize0];
            const TYPE0 *psrc1 = &psrc[v1 * dsize0];
            const TYPE0 *psrc2 = &psrc[v2 * dsize0];

            TYPE *pd = &pdst[v * dsize0];

            for (int u = 0; u < dsize0; u++) {
                const int u0 = u + ((u == 0) ? 0 : -1);
                const int u1 = u + 0;
                const int u2 = u + ((u == dsize0 - 1) ? 0 : +1);

                const TYPE0 a00 = psrc0[u0];
                const TYPE0 a01 = psrc0[u1];
                const TYPE0 a02 = psrc0[u2];

                const TYPE0 a10 = psrc1[u0];
                const TYPE0 a11 = psrc1[u1];
                const TYPE0 a12 = psrc1[u2];

                const TYPE0 a20 = psrc2[u0];
                const TYPE0 a21 = psrc2[u1];
                const TYPE0 a22 = psrc2[u2];

                const double d = (a00 + 2.0 * a01 + a02) + 2.0 * (a10 + 2.0 * a11 + a12) + (a20 + 2.0 * a21 + a22);
                cnvVal(*pd++, d / 16.0);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // box filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void boxFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const int winSize) {
        SP_ASSERT(isValid(2, src));

        Mem1<double> kernel(winSize);
        for (int k = 0; k < winSize; k++) {
            kernel(k) = 1.0;
        }

        Mem2<TYPE> tmp;
        filterX<TYPE, ELEM>(tmp, src, kernel);
        filterY<TYPE, ELEM>(dst, tmp, kernel);
    }
    
    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void boxFilter3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>*>(&dst) != &src) ? src : clone(src);

        const int dsize0 = src.dsize[0];
        const int dsize1 = src.dsize[1];
        const int dsize[2] = { dsize0, dsize1 };

        dst.resize(2, dsize);

        const TYPE0 *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < dsize1; v++) {
            const int v0 = v + ((v == 0) ? 0 : -1);
            const int v1 = v + 0;
            const int v2 = v + ((v == dsize1 - 1) ? 0 : +1);

            const TYPE0 *psrc0 = &psrc[v0 * dsize0];
            const TYPE0 *psrc1 = &psrc[v1 * dsize0];
            const TYPE0 *psrc2 = &psrc[v2 * dsize0];

            TYPE *pd = &pdst[v * dsize0];

            for (int u = 0; u < dsize0; u++) {
                const int u0 = u + ((u == 0) ? 0 : -1);
                const int u1 = u + 0;
                const int u2 = u + ((u == dsize0 - 1) ? 0 : +1);

                const TYPE0 a00 = psrc0[u0];
                const TYPE0 a01 = psrc0[u1];
                const TYPE0 a02 = psrc0[u2];

                const TYPE0 a10 = psrc1[u0];
                const TYPE0 a11 = psrc1[u1];
                const TYPE0 a12 = psrc1[u2];

                const TYPE0 a20 = psrc2[u0];
                const TYPE0 a21 = psrc2[u1];
                const TYPE0 a22 = psrc2[u2];

                const double d = (a00 + a01 + a02 + a10 + a11 + a12 + a20 + a21 + a22) / 9.0;
                cnvVal(*pd++, d);
            }
        }
    }

 

    //--------------------------------------------------------------------------------
    // max/min filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC void maxFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const int winSize) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int offset = winSize / 2;

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                TYPE maxv = acs2(tmp, u, v);

                for (int ky = 0; ky < winSize; ky++){
                    for (int kx = 0; kx < winSize; kx++) {

                        const TYPE &val = acs2(tmp, u + kx - offset, v + ky - offset);
                        maxv = maxVal(maxv, val);
                    }
                }

                cnvVal(acs2(dst, u, v), maxv);
            }
        }
    }

    template <typename TYPE>
    SP_CPUFUNC void minFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const int winSize) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int offset = winSize / 2;

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                TYPE minv = acs2(tmp, u, v);

                for (int ky = 0; ky < winSize; ky++) {
                    for (int kx = 0; kx < winSize; kx++) {

                        const TYPE &val = acs2(tmp, u + kx - offset, v + ky - offset);
                        minv = minVal(minv, val);
                    }
                }

                cnvVal(acs2(dst, u, v), minv);
            }
        }
    }


    //--------------------------------------------------------------------------------
    // laplacian filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void laplacianFilter(Mem<TYPE> &dst, const Mem<TYPE0> &src, const double sigma = 0.8){
        SP_ASSERT(isValid(2, src));

        const int half = maxVal(1, round((sigma - 0.8) / 0.3 + 1));

        Mem2<double> kernel(2 * half + 1, 2 * half + 1);
        for (int y = -half; y <= half; y++){
            for (int x = -half; x <= half; x++){
                const double r = x * x + y * y;
                kernel(x + half, y + half) = (r - 2 * square(sigma)) * exp(-r / (2 * square(sigma)));
            }
        }

        filter(dst, src, kernel);
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void laplacianFilter3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>*>(&dst) != &src) ? src : clone(src);

        const int dsize0 = src.dsize[0];
        const int dsize1 = src.dsize[1];
        const int dsize[2] = { dsize0, dsize1 };

        dst.resize(2, dsize);

        const TYPE0 *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < dsize1; v++) {
            const int v0 = v + ((v == 0) ? 0 : -1);
            const int v1 = v + 0;
            const int v2 = v + ((v == dsize1 - 1) ? 0 : +1);

            const TYPE0 *psrc0 = &psrc[v0 * dsize0];
            const TYPE0 *psrc1 = &psrc[v1 * dsize0];
            const TYPE0 *psrc2 = &psrc[v2 * dsize0];

            TYPE *pd = &pdst[v * dsize0];

            for (int u = 0; u < dsize0; u++) {
                const int u0 = u + ((u == 0) ? 0 : -1);
                const int u1 = u + 0;
                const int u2 = u + ((u == dsize0 - 1) ? 0 : +1);

                const TYPE0 a00 = psrc0[u0];
                const TYPE0 a01 = psrc0[u1];
                const TYPE0 a02 = psrc0[u2];

                const TYPE0 a10 = psrc1[u0];
                const TYPE0 a11 = psrc1[u1];
                const TYPE0 a12 = psrc1[u2];
                
                const TYPE0 a20 = psrc2[u0];
                const TYPE0 a21 = psrc2[u1];
                const TYPE0 a22 = psrc2[u2];

                const double d = (8 * a11 - (a00 + a01 + a02 + a10 + a12 + a20 + a21 + a22)) / 16.0;
                cnvVal(*pd++, d);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // sobel filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void sobelFilter3x3(Mem<TYPE> &dX, Mem<TYPE> &dY, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        const int dsize0 = src.dsize[0];
        const int dsize1 = src.dsize[1];
        const int dsize[2] = { dsize0, dsize1 };

        dX.resize(2, dsize);
        dY.resize(2, dsize);

        const TYPE0 *psrc = src.ptr;
        TYPE *pdx = dX.ptr;
        TYPE *pdy = dY.ptr;

        for (int v = 0; v < dsize1; v++) {
            const int v0 = v + ((v == 0) ? 0 : -1);
            const int v1 = v + 0;
            const int v2 = v + ((v == dsize1 - 1) ? 0 : +1);

            const TYPE0 *psrc0 = &psrc[v0 * dsize0];
            const TYPE0 *psrc1 = &psrc[v1 * dsize0];
            const TYPE0 *psrc2 = &psrc[v2 * dsize0];

            for (int u = 0; u < dsize0; u++) {
                const int u0 = u + ((u == 0) ? 0 : -1);
                const int u1 = u + 0;
                const int u2 = u + ((u == dsize0 - 1) ? 0 : +1);

                const TYPE0 a00 = psrc0[u0];
                const TYPE0 a01 = psrc0[u1];
                const TYPE0 a02 = psrc0[u2];

                const TYPE0 a10 = psrc1[u0];
                const TYPE0 a11 = psrc1[u1];
                const TYPE0 a12 = psrc1[u2];

                const TYPE0 a20 = psrc2[u0];
                const TYPE0 a21 = psrc2[u1];
                const TYPE0 a22 = psrc2[u2];

                const double dx = ((a02 + 2 * a12 + a22) - (a00 + 2 * a10 + a20)) / 8.0;
                const double dy = ((a20 + 2 * a21 + a22) - (a00 + 2 * a01 + a02)) / 8.0;

                cnvVal(*pdx++, dx);
                cnvVal(*pdy++, dy);
            }
        }
    }
 
    //--------------------------------------------------------------------------------
    // scharr filter 
    //--------------------------------------------------------------------------------
    
    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void scharrFilter3x3(Mem<TYPE> &dX, Mem<TYPE> &dY, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        const int dsize0 = src.dsize[0];
        const int dsize1 = src.dsize[1];
        const int dsize[2] = { dsize0, dsize1 };

        dX.resize(2, dsize);
        dY.resize(2, dsize);

        const TYPE0 *psrc = src.ptr;
        TYPE *pdx = dX.ptr;
        TYPE *pdy = dY.ptr;

        for (int v = 0; v < dsize1; v++) {
            const int v0 = v + ((v == 0) ? 0 : -1);
            const int v1 = v + 0;
            const int v2 = v + ((v == dsize1 - 1) ? 0 : +1);

            const TYPE0 *psrc0 = &psrc[v0 * dsize0];
            const TYPE0 *psrc1 = &psrc[v1 * dsize0];
            const TYPE0 *psrc2 = &psrc[v2 * dsize0];

            for (int u = 0; u < dsize0; u++) {
                const int u0 = u + ((u == 0) ? 0 : -1);
                const int u1 = u + 0;
                const int u2 = u + ((u == dsize0 - 1) ? 0 : +1);

                const TYPE0 a00 = psrc0[u0];
                const TYPE0 a01 = psrc0[u1];
                const TYPE0 a02 = psrc0[u2];

                const TYPE0 a10 = psrc1[u0];
                const TYPE0 a11 = psrc1[u1];
                const TYPE0 a12 = psrc1[u2];

                const TYPE0 a20 = psrc2[u0];
                const TYPE0 a21 = psrc2[u1];
                const TYPE0 a22 = psrc2[u2];

                const double dx = ((3 * a02 + 10 * a12 + 3 * a22) - (3 * a00 + 10 * a10 + 3 * a20)) / 32.0;
                const double dy = ((3 * a20 + 10 * a21 + 3 * a22) - (3 * a00 + 10 * a01 + 3 * a02)) / 32.0;

                cnvVal(*pdx++, dx);
                cnvVal(*pdy++, dy);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // median filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void medianFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const int winSize) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int offset = winSize / 2;
        const int ch = sizeof(TYPE) / sizeof(ELEM);

        Mem1<ELEM> list;
        list.resize(winSize * winSize);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                for (int c = 0; c < ch; c++) {
                    for (int ky = 0; ky < winSize; ky++) {
                        for (int kx = 0; kx < winSize; kx++) {
                            ELEM val = acs2<TYPE, ELEM>(src, u + kx - offset, v + ky - offset, c);
                            list[ky * winSize + kx] = val;
                        }
                    }
                    acs2<TYPE, ELEM>(dst, u, v, c) = medianVal(list);
                }
            }
        }
    }


    //--------------------------------------------------------------------------------
    // normalize filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void normalizeFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const int winSize, const int maxv = SP_BYTEMAX) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        Mem2<TYPE> tmp;
        {
            Mem1<double> kernel(winSize);
            for (int k = 0; k < winSize; k++) {
                kernel(k) = 1.0;
            }

            filterX<TYPE, ELEM>(tmp, src, kernel);
            filterY<TYPE, ELEM>(tmp, tmp, kernel);
        }

        const int ch = sizeof(TYPE) / sizeof(ELEM);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                for (int c = 0; c < ch; c++) {
                    acs2<TYPE, ELEM>(dst, u, v, c) = (acs2<TYPE, ELEM>(src, u, v, c) - acs2<TYPE, ELEM>(tmp, u, v, c) + maxv) / 2 ;
                }
            }
        }
    }


    //--------------------------------------------------------------------------------
    //  bilateral filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void bilateralFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const double sigma_s, const double sigma_c){
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int half = maxVal(1, round((sigma_s - 0.8) / 0.3 + 1));

        Mem2<double> kernel(2 * half + 1, 2 * half + 1);
        for (int y = -half; y <= +half; y++) {
            for (int x = -half; x <= +half; x++) {
                const double r2 = square(x) + square(y);
                kernel(x + half, y + half) = exp(-r2 / (2.0 * square(sigma_s)));
            }
        }

        const double expscale = 10.0;
        Mem1<double> exptable(100);
        for (int i = 0; i < exptable.size(); i++){
            const double r = square(i / expscale);
            const double v = exp(-r / 2.0);
            exptable[i] = v;
        }

        const Rect rect = getRect2(dst.dsize);

        const int ch = sizeof(TYPE) / sizeof(ELEM);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){

                for (int c = 0; c < ch; c++) {
                    const ELEM base = acs2<TYPE, ELEM>(tmp, u, v, c);

                    double sum = 0.0, div = 0.0;
                    for (int ky = -half; ky < +half; ky++){
                        for (int kx = -half; kx < +half; kx++){
                            if (isInRect2(rect, u + kx, v + ky) == false) continue;

                            const ELEM &val = acs2<TYPE, ELEM>(tmp, u + kx, v + ky, c);

                            const double a = kernel(kx + half, ky + half);
                            const double b = exptable(round(fabs(val - base) * expscale / sigma_c));

                            sum += a * b * val;
                            div += a * b;
                        }
                    }

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), sum / div);
                }
            }
        }
    }

}

#endif
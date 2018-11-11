//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FILTER_H__
#define __SP_FILTER_H__

#include "spcore/spcore.h"

namespace sp{

    // filter size <-> gaussian sigma
    // 
    // sigma = 0.3 * (size / 2 - 1) + 0.8
    // size = 2 * round((sigma - 0.8) / 0.3) + 1

    //--------------------------------------------------------------------------------
    // filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void filter(Mem<TYPE> &dst, const Mem<TYPE> &src, const Mem<double> &kernel){
        SP_ASSERT(isValid(2, src) && isValid(2, kernel));
        
        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int sizeX = kernel.dsize[0] / 2;
        const int sizeY = kernel.dsize[1] / 2;

        const Rect rect = getRect2(dst.dsize);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){

                for (int c = 0; c < ch; c++){
                    double sum = 0.0, div = 0.0;

                    for (int ky = -sizeY; ky <= sizeY; ky++){
                        for (int kx = -sizeX; kx <= sizeX; kx++){
                            if (isInRect2(rect, u + kx, v + ky) == false) continue;
                            
                            const ELEM &val = acs2<TYPE, ELEM>(tmp, u + kx, v + ky, c);
                            const double s = acs2(kernel, kx + sizeX, ky + sizeY);

                            sum += s * val;
                            div += fabs(s);
                        }
                    }

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), (div > 0.0) ? sum / div : 0.0);
                }
            }
        }

    }

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void filterX(Mem<TYPE> &dst, const Mem<TYPE> &src, const Mem<double> &kernel){
        SP_ASSERT(isValid(2, src) && isValid(1, kernel));

        dst.resize(2, src.dsize);
        const Mem2<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int sizeX = kernel.dsize[0] / 2;

        const Rect rect = getRect2(dst.dsize);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){

                for (int c = 0; c < ch; c++){
                    double sum = 0.0, div = 0.0;

                    for (int kx = -sizeX; kx <= sizeX; kx++){
                        if (isInRect2(rect, u + kx, v) == false) continue;
                        
                        const ELEM &val = acs2<TYPE, ELEM>(tmp, u + kx, v, c);
                        const double s = acs1(kernel, kx + sizeX);

                        sum += s * val;
                        div += fabs(s);
                    }

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), (div > 0.0) ? sum / div : 0.0);
                }
            }
        }
    }


    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void filterY(Mem<TYPE> &dst, const Mem<TYPE> &src, const Mem<double> &kernel){
        SP_ASSERT(isValid(2, src) && isValid(1, kernel));

        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int sizeY = kernel.dsize[0] / 2;

        const Rect rect = getRect2(dst.dsize);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){

                for (int c = 0; c < ch; c++){
                    double sum = 0.0, div = 0.0;

                    for (int ky = -sizeY; ky <= sizeY; ky++){
                        if (isInRect2(rect, u, v + ky) == false) continue;
                        
                        const ELEM &val = acs2<TYPE, ELEM>(tmp, u, v + ky, c);
                        const double s = acs1(kernel, ky + sizeY);

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

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void gaussianFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const double sigma = 0.8){
        SP_ASSERT(isValid(2, src));

        const int size = maxVal(1, round((sigma - 0.8) / 0.3));

        Mem1<double> kernel(2 * size + 1);
        for (int k = -size; k <= size; k++){
            const double r = k * k;
            kernel(k + size) = exp(-r / (2.0 * square(sigma)));
        }

        Mem2<TYPE> tmp;
        filterX<TYPE, ELEM>(tmp, src, kernel);
        filterY<TYPE, ELEM>(dst, tmp, kernel);
    }

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void gaussianFilter3x3(Mem<TYPE> &dst, const Mem<TYPE> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                for (int c = 0; c < ch; c++) {
                    double sum = 0.0;
                    sum += acs2<TYPE, ELEM>(tmp, u - 1, v - 1, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 0, v - 1, c) * 2.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 1, v - 1, c) * 1.0;

                    sum += acs2<TYPE, ELEM>(tmp, u - 1, v + 0, c) * 2.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 0, v + 0, c) * 4.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 1, v + 0, c) * 2.0;

                    sum += acs2<TYPE, ELEM>(tmp, u - 1, v + 1, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 0, v + 1, c) * 2.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 1, v + 1, c) * 1.0;

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), sum / 16.0);
                }
            }
        }
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void gaussianFilter3x3Fast(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        const int w = src.dsize[0];
        const int h = src.dsize[1];

        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

        const TYPE0 *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < h; v++) {
            const int ys = (v == 0) ? 0 : -1;
            const int yc = 0;
            const int ye = (v == h - 1) ? 0 : +1;

            const int vs = v + ys;
            const int vc = v + yc;
            const int ve = v + ye;

            const TYPE0 *psrc0 = &psrc[vs * w];
            const TYPE0 *psrc1 = &psrc[vc * w];
            const TYPE0 *psrc2 = &psrc[ve * w];

            TYPE *pd = &dst[vc * w];

            double pre0 = 0.0;
            double pre1 = 0.0;
            {
                const TYPE0 a0 = *psrc0++;
                const TYPE0 a1 = *psrc1++;
                const TYPE0 a2 = *psrc2++;

                pre0 = a0 + 2 * a1 + a2;
                pre1 = a0 + 2 * a1 + a2;
            }

            for (int u = 0; u < w - 1; u++) {

                const TYPE0 a0 = *psrc0++;
                const TYPE0 a1 = *psrc1++;
                const TYPE0 a2 = *psrc2++;

                const double p = a0 + 2 * a1 + a2;
                double d = pre0 + 2 * pre1 + p;
                d /= 16.0;

                pre0 = pre1;
                pre1 = p;

                cnvVal(*pd++, d);
            }
            {
                double d = pre0 + 2 * pre1 + pre1;
                d /= 16.0;

                cnvVal(*pd++, d);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // box filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void boxFilter3x3(Mem<TYPE> &dst, const Mem<TYPE> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem2<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                for (int c = 0; c < ch; c++) {
                    double sum = 0.0;
                    sum += acs2<TYPE, ELEM>(tmp, u - 1, v - 1, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 0, v - 1, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 1, v - 1, c) * 1.0;

                    sum += acs2<TYPE, ELEM>(tmp, u - 1, v + 0, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 0, v + 0, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 1, v + 0, c) * 1.0;

                    sum += acs2<TYPE, ELEM>(tmp, u - 1, v + 1, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 0, v + 1, c) * 1.0;
                    sum += acs2<TYPE, ELEM>(tmp, u + 1, v + 1, c) * 1.0;

                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), sum / 9.0);
                }
            }
        }
    }

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void boxFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const int winSize){
        SP_ASSERT(isValid(2, src));

        Mem1<double> kernel(winSize);
        for (int k = 0; k < winSize; k++){
            kernel(k) = 1.0;
        }

        Mem2<TYPE> tmp;
        filterX<TYPE, ELEM>(tmp, src, kernel);
        filterY<TYPE, ELEM>(dst, tmp, kernel);
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

        const int size = maxVal(1, round((sigma - 0.8) / 0.3));

        Mem2<double> kernel(2 * size + 1, 2 * size + 1);
        for (int y = -size; y <= size; y++){
            for (int x = -size; x <= size; x++){
                const double r = x * x + y * y;
                kernel(x + size, y + size) = (r - 2 * square(sigma)) * exp(-r / (2 * square(sigma)));
            }
        }

        filter(dst, src, kernel);
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void laplacianFilter3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                double sum = 0.0;
                sum += acs2(tmp, u - 1, v - 1) * (-1.0);
                sum += acs2(tmp, u + 0, v - 1) * (-1.0);
                sum += acs2(tmp, u + 1, v - 1) * (-1.0);

                sum += acs2(tmp, u - 1, v + 0) * (-1.0);
                sum += acs2(tmp, u + 0, v + 0) * (+8.0);
                sum += acs2(tmp, u + 1, v + 0) * (-1.0);

                sum += acs2(tmp, u - 1, v + 1) * (-1.0);
                sum += acs2(tmp, u + 0, v + 1) * (-1.0);
                sum += acs2(tmp, u + 1, v + 1) * (-1.0);

                cnvVal(acs2(dst, u, v), sum / 16.0);
            }
        }
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void laplacianFilter5x5(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                double sum = 0.0;
                sum += acs2(tmp, u - 2, v - 2) * (-1.0);
                sum += acs2(tmp, u - 1, v - 2) * (-3.0);
                sum += acs2(tmp, u + 0, v - 2) * (-4.0);
                sum += acs2(tmp, u + 1, v - 2) * (-3.0);
                sum += acs2(tmp, u + 2, v - 2) * (-1.0);

                sum += acs2(tmp, u - 2, v - 1) * (-3.0);
                sum += acs2(tmp, u - 1, v - 1) * (+0.0);
                sum += acs2(tmp, u + 0, v - 1) * (+6.0);
                sum += acs2(tmp, u + 1, v - 1) * (+0.0);
                sum += acs2(tmp, u + 2, v - 1) * (-3.0);

                sum += acs2(tmp, u - 2, v + 0) * (-4.0);
                sum += acs2(tmp, u - 1, v + 0) * (+6.0);
                sum += acs2(tmp, u + 0, v + 0) * (+20.0);
                sum += acs2(tmp, u + 1, v + 0) * (+6.0);
                sum += acs2(tmp, u + 2, v + 0) * (-4.0);

                sum += acs2(tmp, u - 2, v + 1) * (-3.0);
                sum += acs2(tmp, u - 1, v + 1) * (+0.0);
                sum += acs2(tmp, u + 0, v + 1) * (+6.0);
                sum += acs2(tmp, u + 1, v + 1) * (+0.0);
                sum += acs2(tmp, u + 2, v + 1) * (-3.0);

                sum += acs2(tmp, u - 2, v + 2) * (-1.0);
                sum += acs2(tmp, u - 1, v + 2) * (-3.0);
                sum += acs2(tmp, u + 0, v + 2) * (-4.0);
                sum += acs2(tmp, u + 1, v + 2) * (-3.0);
                sum += acs2(tmp, u + 2, v + 2) * (-1.0);
                
                cnvVal(acs2(dst, u, v), sum / 88.0);
            }
        }
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void laplacianFilter3x3Fast(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        const int w = src.dsize[0];
        const int h = src.dsize[1];

        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

        const TYPE0 *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < h; v++) {
            const int y0 = (v == 0) ? 0 : -1;
            const int y1 = 0;
            const int y2 = (v == h - 1) ? 0 : +1;

            const int v0 = v + y0;
            const int v1 = v + y1;
            const int v2 = v + y2;

            const TYPE0 *psrc0 = &psrc[v0 * w];
            const TYPE0 *psrc1 = &psrc[v1 * w];
            const TYPE0 *psrc2 = &psrc[v2 * w];

            TYPE *pd = &dst[v1 * w];

            double pre0 = 0.0;
            double pre1 = 0.0;
            {
                const TYPE0 a0 = *psrc0;
                const TYPE0 a1 = *psrc1;
                const TYPE0 a2 = *psrc2;

                pre0 = a0 + a1 + a2;
                pre1 = a0 + a1 + a2;
            }

            for (int u = 0; u < w - 1; u++) {

                double d = 9 * *psrc1;

                const TYPE0 a0 = *(++psrc0);
                const TYPE0 a1 = *(++psrc1);
                const TYPE0 a2 = *(++psrc2);

                const double p = a0 + a1 + a2;
                d -= pre0 + pre1 + p;
                d /= 16.0;

                pre0 = pre1;
                pre1 = p;

                cnvVal(*pd++, d);
            }
            {
                const int u = w - 1;
                double d = 9 * *psrc1;

                const TYPE0 a0 = *(psrc0);
                const TYPE0 a1 = *(psrc1);
                const TYPE0 a2 = *(psrc2);

                const double p = a0 + a1 + a2;
                d -= pre0 + pre1 + p;
                d /= 16.0;

                cnvVal(*pd++, d);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // sobel filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void sobelFilterX3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                
                double sum = 0.0;
                sum += acs2(tmp, u - 1, v - 1) * (-1.0);
                sum += acs2(tmp, u - 1, v + 0) * (-2.0);
                sum += acs2(tmp, u - 1, v + 1) * (-1.0);

                sum += acs2(tmp, u + 1, v - 1) * (+1.0);
                sum += acs2(tmp, u + 1, v + 0) * (+2.0);
                sum += acs2(tmp, u + 1, v + 1) * (+1.0);

                cnvVal(acs2<TYPE>(dst, u, v), sum / 8.0);
            }
        }
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void sobelFilterY3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                
                double sum = 0.0;
                sum += acs2(tmp, u - 1, v - 1) * (-1.0);
                sum += acs2(tmp, u + 0, v - 1) * (-2.0);
                sum += acs2(tmp, u + 1, v - 1) * (-1.0);

                sum += acs2(tmp, u - 1, v + 1) * (+1.0);
                sum += acs2(tmp, u + 0, v + 1) * (+2.0);
                sum += acs2(tmp, u + 1, v + 1) * (+1.0);

                cnvVal(acs2<TYPE>(dst, u, v), sum / 8.0);
            }
        }
    }


    //--------------------------------------------------------------------------------
    // scharr filter 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void scharrFilterX3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                double sum = 0.0;
                sum += acs2(tmp, u - 1, v - 1) * (-3.0);
                sum += acs2(tmp, u - 1, v + 0) * (-10.0);
                sum += acs2(tmp, u - 1, v + 1) * (-3.0);

                sum += acs2(tmp, u + 1, v - 1) * (+3.0);
                sum += acs2(tmp, u + 1, v + 0) * (+10.0);
                sum += acs2(tmp, u + 1, v + 1) * (+3.0);

                cnvVal(acs2<TYPE>(dst, u, v), sum / 32.0);
            }
        }
    }

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void scharrFilterY3x3(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

#if SP_USE_OMP
#pragma omp parallel for
#endif
        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {

                double sum = 0.0;
                sum += acs2(tmp, u - 1, v - 1) * (-3.0);
                sum += acs2(tmp, u + 0, v - 1) * (-10.0);
                sum += acs2(tmp, u + 1, v - 1) * (-3.0);

                sum += acs2(tmp, u - 1, v + 1) * (+3.0);
                sum += acs2(tmp, u + 0, v + 1) * (+10.0);
                sum += acs2(tmp, u + 1, v + 1) * (+3.0);

                cnvVal(acs2<TYPE>(dst, u, v), sum / 32.0);
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

        const int size = maxVal(1, round((sigma_s - 0.8) / 0.3));

        const int winSize = 2 * size + 1;
        const int offset = winSize / 2;

        Mem2<double> kernel(winSize, winSize);
        for (int y = 0; y < winSize; y++){
            for (int x = 0; x < winSize; x++){
                const double r2 = square(x - offset) + square(y - offset);
                kernel(x, y) = exp(-r2 / (2.0 * square(sigma_s)));
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
                    for (int ky = 0; ky < winSize; ky++){
                        for (int kx = 0; kx < winSize; kx++){
                            if (isInRect2(rect, u + kx, v + ky) == false) continue;

                            const ELEM &val = acs2<TYPE, ELEM>(tmp, u + kx - offset, v + ky - offset, c);

                            const double a = kernel(kx, ky);
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
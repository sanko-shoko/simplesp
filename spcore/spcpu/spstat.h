//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STAT_H__
#define __SP_STAT_H__

#include "spcore/spcom.h"
#include "spcore/spcpu/spmop.h"

#include <stdlib.h>

namespace sp{

    //--------------------------------------------------------------------------------
    // sort
    //--------------------------------------------------------------------------------

    SP_GENFUNC void qsort(void *base, const int nsize, const int esize, int compare(const void *a, const void *b)) {
        ::qsort(base, nsize, esize, compare);
    }

    template<typename TYPE>
    SP_CPUFUNC int compare_min(const void *a, const void *b){
        return (*static_cast<const TYPE*>(a) > *static_cast<const TYPE*>(b)) ? +1 : -1;
    }

    template<typename TYPE>
    SP_CPUFUNC int compare_max(const void *a, const void *b){
        return (*static_cast<const TYPE*>(a) > *static_cast<const TYPE*>(b)) ? -1 : +1;
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(TYPE *mem, const int size, const bool minOrder = true){
        qsort(mem, size, sizeof(TYPE), (minOrder) ? compare_min<TYPE> : compare_max<TYPE>);
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(TYPE *mem, const int size, int compare(const void *a, const void *b)) {
        qsort(mem, size, sizeof(TYPE), compare);
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(Mem<TYPE> &mem, const bool minOrder = true){
        sort(mem.ptr, mem.size(), minOrder);
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(Mem<TYPE> &mem, int compare(const void *a, const void *b)) {
        sort(mem.ptr, mem.size(), compare);
    }


    //--------------------------------------------------------------------------------
    // max / min
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC TYPE max(const Mem<TYPE> &mem){
        SP_ASSERT(mem.size() != 0);

        TYPE maxv = mem[0];
        for (int i = 1; i < mem.size(); i++){
            maxv = max(maxv, mem[i]);
        }
        return maxv;
    }

    template<typename TYPE>
    SP_CPUFUNC TYPE min(const Mem<TYPE> &mem){
        SP_ASSERT(mem.size() != 0);

        TYPE minv = mem[0];
        for (int i = 0; i < mem.size(); i++){
            minv = min(minv, mem[i]);
        }
        return minv;
    }


    template<typename TYPE>
    SP_CPUFUNC int maxArg(const Mem<TYPE> &mem){
        SP_REAL maxv = -SP_INFINITY;
        int ret = -1;
        for (int i = 0; i < mem.size(); i++){
            if (mem[i] > maxv){
                maxv = mem[i];
                ret = i;
            }
        }
        return ret;
    }

    template<typename TYPE>
    SP_CPUFUNC int minArg(const Mem<TYPE> &mem){
        SP_REAL minv = +SP_INFINITY;
        int ret = -1;
        for (int i = 0; i < mem.size(); i++){
            if (mem[i] < minv){
                minv = mem[i];
                ret = i;
            }
        }
        return ret;
    }

    //--------------------------------------------------------------------------------
    // count up
    //--------------------------------------------------------------------------------
    
    template<typename TYPE>
    SP_CPUFUNC int cntVal(const Mem<TYPE> &mem, const TYPE val) {
        int cnt = 0;
        for (int i = 0; i < mem.size(); i++) {
            if (mem[i] == val) cnt++;
        }
        return cnt;
    }


    //--------------------------------------------------------------------------------
    // base
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC Mem<TYPE> sqVal(const Mem<TYPE> &mem){
        Mem<TYPE> dst(mem.dim, mem.dsize);

        for (int i = 0; i < mem.size(); i++){
            dst[i] = sq(mem[i]);
        }
        return dst;
    }

    template<typename TYPE>
    SP_CPUFUNC Mem<TYPE> sqrtVal(const Mem<TYPE> &mem){
        Mem<TYPE> dst(mem.dim, mem.dsize);

        for (int i = 0; i < mem.size(); i++){
            dst[i] = sqrt(mem[i]);
        }
        return dst;
    }


    //--------------------------------------------------------------------------------
    // value
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC SP_REAL sumVal(const Mem<TYPE> &mem){
        SP_REAL sum = 0.0;

        for (int i = 0; i < mem.size(); i++){
            sum += mem[i];
        }
        return sum;
    }

    template<typename TYPE>
    SP_CPUFUNC SP_REAL meanVal(const Mem<TYPE> &mem){
        return sumVal(mem) / mem.size();
    }


    template<typename TYPE>
    SP_CPUFUNC SP_REAL sumSq(const Mem<TYPE> &mem){
        SP_REAL sum = 0.0;

        for (int i = 0; i < mem.size(); i++){
            sum += sq(mem[i]);
        }
        return sum;
    }

    template<typename TYPE>
    SP_CPUFUNC SP_REAL meanSq(const Mem<TYPE> &mem){
        return sumSq(mem) / mem.size();
    }


    template<typename TYPE>
    SP_CPUFUNC SP_REAL sumSqrt(const Mem<TYPE> &mem){
        SP_REAL sum = 0.0;

        for (int i = 0; i < mem.size(); i++){
            sum += sqrt(mem[i]);
        }
        return sum;
    }

    template<typename TYPE>
    SP_CPUFUNC SP_REAL meanSqrt(const Mem<TYPE> &mem){
        return sumSqrt(mem) / mem.size();
    }


    template<typename TYPE>
    SP_CPUFUNC SP_REAL sumAbs(const Mem<TYPE> &mem){
        SP_REAL sum = 0.0;

        for (int i = 0; i < mem.size(); i++){
            sum += fabs(static_cast<SP_REAL>(mem[i]));
        }
        return sum;
    }

    template<typename TYPE>
    SP_CPUFUNC SP_REAL meanAbs(const Mem<TYPE> &mem){
        return sumAbs(mem) / mem.size();
    }


    template<typename TYPE>
    SP_CPUFUNC TYPE medianVal(const Mem<TYPE> &mem){
        TYPE val;

        if (mem.size() > 0) {
            Mem<TYPE> tmp = mem;
            sort(tmp);
            val = tmp[tmp.size() / 2];
        }
        else {
            val = cast<TYPE>(0);
        }

        return val;
    }


    //--------------------------------------------------------------------------------
    // matrix
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat sumVal(const Mat &mat, const int axis){
        SP_ASSERT(axis == 0 || axis == 1);

        Mat sum((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
        sum.zero();

        const SP_REAL *pMat = mat.ptr;

        for (int r = 0; r < mat.rows(); r++) {
            for (int c = 0; c < mat.cols(); c++) {
                sum[(axis == 0) ? c : r] += *pMat++;
            }
        }
        return sum;
    }

    SP_CPUFUNC Mat meanVal(const Mat &mat, const int axis){
        return sumVal(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
    }


    SP_CPUFUNC Mat sumSq(const Mat &mat, const int axis){
        SP_ASSERT(axis == 0 || axis == 1);

        Mat sum((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
        sum.zero();

        const SP_REAL *pMat = mat.ptr;

        for (int r = 0; r < mat.rows(); r++){
            for (int c = 0; c < mat.cols(); c++){
                sum[(axis == 0) ? c : r] += sq(*pMat++);
            }
        }
        return sum;
    }

    SP_CPUFUNC Mat meanSq(const Mat &mat, const int axis){
        return sumSq(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
    }


    SP_CPUFUNC Mat sumAbs(const Mat &mat, const int axis){
        SP_ASSERT(axis == 0 || axis == 1);

        Mat sum((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
        sum.zero();

        const SP_REAL *pMat = mat.ptr;

        for (int r = 0; r < mat.rows(); r++){
            for (int c = 0; c < mat.cols(); c++){
                sum[(axis == 0) ? c : r] += fabs(*pMat++);
            }
        }
        return sum;
    }

    SP_CPUFUNC Mat meanAbs(const Mat &mat, const int axis){
        return sumAbs(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
    }



    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

    template<typename VEC>
    SP_CPUFUNC VEC sumVec(const Mem<VEC> &vecs){
        VEC sum;
        memset(&sum, 0, sizeof(VEC));

        for (int i = 0; i < vecs.size(); i++){
            sum += vecs[i];
        }
        return sum;
    }

    template<typename VEC>
    SP_CPUFUNC VEC meanVec(const Mem<VEC> &vecs){
        return sumVec(vecs) / vecs.size();
    }

    template<typename VEC>
    SP_CPUFUNC Mem<SP_REAL> normVec(const Mem<VEC> &vecs){
        Mem<SP_REAL> dst(vecs.dim, vecs.dsize);

        for (int i = 0; i < dst.size(); i++){
            dst[i] = normVec(vecs[i]);
        }
        return dst;
    }


    //--------------------------------------------------------------------------------
    // eval
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC SP_REAL evalErr(const Mem1<SP_REAL> &errs, const double thresh = 5.0) {
        double eval = 0.0;
        for (int i = 0; i < errs.size(); i++) {
            if (errs[i] < thresh) eval += 1.0;
        }
        return eval / errs.size();
    }

    SP_CPUFUNC SP_REAL evalErr(const SP_REAL err, const double thresh = 5.0) {
        return (err < thresh) ? 1.0 : 0.0;
    }

    
    template<typename TYPE>
    SP_CPUFUNC Mem1<TYPE> denoise(const Mem<TYPE> &src, const Mem<SP_REAL> &errs, const double thresh = 5.0) {
        Mem1<TYPE> dst;
        dst.reserve(src.size());
        for (int i = 0; i < src.size(); i++) {
            if (errs[i] > thresh) continue;
            dst.push(src[i]);
        }
        return dst;
    }


    //--------------------------------------------------------------------------------
    // histogram
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void histogram(Mem<int> &hist, const Mem<TYPE> &src, const int bins) {

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int dsize[2] = { bins, ch };

        hist.resize((ch == 1) ? 1 : 2, dsize);
        hist.zero();

        for (int i = 0; i < src.size(); i++) {
            for (int c = 0; c < ch; c++) {
                const int val = acs1<TYPE, ELEM>(src, i, c);
                acs2(hist, val, c)++;
            }
        }
    }

}

#endif
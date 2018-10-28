//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SOLVE_H__
#define __SP_SOLVE_H__

#include "spcore/spcpu/spmop.h"
#include "spcore/spcpu/spstat.h"


namespace sp{

    //--------------------------------------------------------------------------------
    // solve util
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM = double>
    SP_CPUFUNC bool normalize(Mat &T, Mem<TYPE> &dst, const Mem<TYPE> &mem) {

        const int dim = sizeof(TYPE) / sizeof(ELEM);

        Mat data(mem.size(), dim, mem.ptr);

        const Mat mean = meanVal(data, 0);
        data -= mean;

        double scale = meanSqrt(sumSq(data, 1));
        if (scale < SP_SMALL) return false;
        data /= scale;

        T = eyeMat(dim + 1, dim + 1);
        for (int c = 0; c < dim; c++) {
            T(c, c) /= scale;
            T(c, dim) -= mean(0, c) / scale;
        }

        dst.resize(mem.dim, mem.dsize);
        ELEM *ptr = reinterpret_cast<ELEM*>(dst.ptr);
        for (int i = 0; i < dim * dst.size(); i++) {
            cnvVal(ptr[i], data[i]);
        }
        return true;
    }

    SP_CPUFUNC Mat calcAtWeight(const Mat &A, const Mem<double> errs = Mem<double>(), const double minErr = 0.1){
        Mat AtW = trnMat(A);

        if (errs.size() > 0){
            const int num = AtW.cols() / errs.size();

            const double sigma = 1.4826 * medianVal(errs);
            const double thresh = 3.0 * maxVal(sigma, minErr);

            for (int r = 0; r < AtW.rows(); r++){
                for (int c = 0; c < AtW.cols(); c++){
                    AtW(r, c) *= funcTukey(errs[c / num], thresh);
                }
            }
        }
        return AtW;
    }

    // solve equation (A * X = B)
    SP_CPUFUNC bool solveEq(Mat &result, const Mat &A, const Mat &B, const Mem<double> errs = Mem<double>(), const double minErr = 0.1){
        
        const Mat AtW = calcAtWeight(A, errs, minErr);

        result = invMat(AtW * A) * AtW * B;
        return (result.size() > 0) ? true : false;
    }

    // solve equation (A * X = 0)
    SP_CPUFUNC bool solveEqZero(Mat &result, const Mat &A, const Mem<double> errs = Mem<double>(), const double minErr = 0.1){
        
        const Mat AtW = calcAtWeight(A, errs, minErr);

        Mat eigVec, eigVal;
        if (eigMat(eigVec, eigVal, AtW * A, true) == false) return false;

        result.resize(eigVec.rows(), 1);
        for (int i = 0; i < eigVec.rows(); i++) {
            result[i] = eigVec(i, 0);
        }
        return true;
    }

    //--------------------------------------------------------------------------------
    // ransac util
    //--------------------------------------------------------------------------------

    // ransac sampling max
#define SP_RANSAC_ITMAX 1000
#define SP_RANSAC_MINRATE 2
#define SP_RANSAC_MINEVAL 0.2

    // ransac adaptive stop
    SP_CPUFUNC int adaptiveStop(const double rate, const int num, const double n = 0.99){
        const double e = maxVal(rate, 0.1);
        const int k = round(1.0 + log(1.0 - n) / log(1.0 - pow(e, num)));
        return minVal(k, SP_RANSAC_ITMAX);
    }




}
#endif
//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SOLVE_H__
#define __SP_SOLVE_H__

#include "spcore/spcpu/spmop.h"
#include "spcore/spcpu/spstat.h"


namespace sp{

    //--------------------------------------------------------------------------------
    // solveer
    //--------------------------------------------------------------------------------

    namespace solver {

        template<typename TYPE, typename ELEM = SP_REAL>
        SP_CPUFUNC bool normalize(Mem<TYPE> &dst, Mat &T, const Mem<TYPE> &mem) {

            const int dim = sizeof(TYPE) / sizeof(ELEM);

            Mat data(mem.size(), dim, mem.ptr);

            const Mat mean = meanVal(data, 0);
            data -= mean;

            SP_REAL scale = meanSqrt(sumSq(data, 1));
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
                ptr[i] = cast<ELEM>(data[i]);
            }
            return true;
        }


        SP_CPUFUNC Mat calcW(const Mem<SP_REAL> errs, const int step = 1, const double minErr = 0.1) {
            const int nsize = errs.size();

            Mat W(nsize * step, 1);

            const double sigma = 1.4826 * medianVal(errs);
            const double thresh = max(3.0 * sigma, minErr);

            SP_REAL *pw = W.ptr;
            const SP_REAL *pe = errs.ptr;

            for (int i = 0; i < nsize; i++) {
                const double w = funcTukey(*pe++, thresh);
                for (int s = 0; s < step; s++) {
                    *pw++ = w;
                }
            }

            return W;
        }

        SP_CPUFUNC Mat calcAtA(const Mat &A, const Mat W = Mat()) {
            Mat M(A.cols(), A.cols());
            M.zero();

            const int rsize = M.rows();
            const int csize = M.cols();

            const int nsize = A.rows();

            for (int i = 0; i < nsize; i++) {
                SP_REAL *pm = M.ptr;
                const SP_REAL *pa_ = &A(i, 0);
                const SP_REAL w = (W.ptr != NULL) ? W[i] : 1.0;

                for (int r = 0; r < rsize; r++) {
                    const SP_REAL *pa = pa_;
                    pm += r;
                    pa += r;

                    const SP_REAL a = *pa;
                    for (int c = r; c < csize; c++) {
                        (*pm++) += a * (*pa++) * w;
                    }
                }
            }

            // fill
            {
                for (int r = 0; r < rsize; r++) {
                    for (int c = r + 1; c < csize; c++) {
                        M(c, r) = M(r, c);
                    }
                }
            }

            return M;
        }

        SP_CPUFUNC Mat calcAtB(const Mat &A, const Mat &B, const Mat W = Mat()) {
            Mat M(A.cols(), 1);
            M.zero();

            const int rsize = M.rows();
            const int csize = M.cols();

            const int nsize = A.rows();

            for (int i = 0; i < nsize; i++) {
                SP_REAL *pm = M.ptr;
                const SP_REAL *pa = &A(i, 0);
                const SP_REAL b = B(i, 0);
                const SP_REAL w = (W.ptr != NULL) ? W[i] : 1.0;

                for (int r = 0; r < rsize; r++) {
                    *pm++ += (*pa++) * b * w;
                }
            }

            return M;
        }

        // solve equation (A * X = B)
        SP_CPUFUNC bool solveAX_B(Mat &result, const Mat &A, const Mat &B, const Mat W = Mat()) {
            const Mat AtA = calcAtA(A, W);
            const Mat AtB = calcAtB(A, B, W);

            result = invMat(AtA) * AtB;

            return (result.size() > 0) ? true : false;
        }

        // solve equation (A * X = 0)
        SP_CPUFUNC bool solveAX_Z(Mat &result, const Mat &A, const Mat W = Mat()) {

            const Mat AtA = calcAtA(A, W);

            Mat eigVec, eigVal;
            if (eigMat(eigVec, eigVal, AtA, true) == false) return false;

            result.resize(eigVec.rows(), 1);
            for (int i = 0; i < eigVec.rows(); i++) {
                result[i] = eigVec(i, 0);
            }
            return true;
        }
    }


    //--------------------------------------------------------------------------------
    // ransac util
    //--------------------------------------------------------------------------------

    // ransac sampling max
#define SP_RANSAC_ITMAX 1000
#define SP_RANSAC_MINEVAL 0.2

    // ransac adaptive stop
    SP_CPUFUNC int ransacAdaptiveStop(const double rate, const int unit, const double n = 0.99){
        const SP_REAL e = max(rate, 0.1);
        const int k = round(1.0 + log(1.0 - n) / log(1.0 - pow(e, unit)));
        return min(k, SP_RANSAC_ITMAX);
    }

    SP_CPUFUNC SP_REAL ransacEval(const Mem<SP_REAL> &errs, const int unit, const double thresh) {
        int cnt = 0;
        for (int i = 0; i < errs.size(); i++) {
            if (errs[i] < thresh) cnt++;
        }
        const double eval = static_cast<double>(cnt - unit) / (errs.size() - unit);
     
        return SP_CAST_REAL(eval);
    }

    template<typename TYPE>
    class RandomSample {

        // data ptr
        const Mem<TYPE> *ptr;

        Mem1<TYPE> buf;

        // sampling unit num
        int unit;

        // prev shuffle seed
        int prev;

    public:
        RandomSample(const Mem<TYPE> &src, const int unit) {
            this->ptr = &src;
            this->unit = unit;
            this->prev = -1;
        }

        Mem1<TYPE> gen(const int i) {

            const int s = i / (ptr->size() - unit);
            const int p = i % (ptr->size() - unit);

            if (s != prev) {
                prev = s;
                buf = shuffle(*ptr, s);
            }

            Mem1<TYPE> next(unit, &buf[p]);
            return next;
        }
    };
}
#endif
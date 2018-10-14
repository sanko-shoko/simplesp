//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MATH_H__
#define __SP_MATH_H__

#include "spcore/spcom.h"
#include "spcore/spwrap.h"
#include "spcore/spgen/spbase.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    // x * x
    SP_GENFUNC double square(const double x) {
        return x * x;
    }

    // x * x * x
    SP_GENFUNC double cubic(const double x) {
        return x * x * x;
    }

    // cubic root
    SP_GENFUNC double cbrt(const double x) {
        const double z = pow(fabs(x), 1.0 / 3.0);
        return (x >= 0.0) ? z : -z;
    }

    // sqrt(a * a + b * b) without destructive underflow or overflow
    SP_GENFUNC double pythag(const double a, const double b) {
        const double x = fabs(a);
        const double y = fabs(b);
        if (x > y) {
            return x * sqrt(1.0 + (y / x) * (y / x));
        }
        else {
            return (y == 0.0) ? 0.0 : y * sqrt(1.0 + (x / y) * (x / y));
        }
    }

    // combination
    SP_GENFUNC int nCk(const int n, const int k) {
        int ret = 1;
        for (int i = 1; i <= k; i++) {
            ret = ret * (n - i + 1) / i;
        }
        return ret;
    }
    

    //--------------------------------------------------------------------------------
    // complex
    //--------------------------------------------------------------------------------

    // get complex
    SP_GENFUNC Cmp getCmp(const double re, const double im) {
        Cmp dst;
        dst.re = re, dst.im = im;
        return dst;
    }

    // addition
    SP_GENFUNC Cmp addCmp(const Cmp &cmp0, const Cmp &cmp1) {
        return getCmp(cmp0.re + cmp1.re, cmp0.im + cmp1.im);
    }

    // addition
    SP_GENFUNC Cmp addCmp(const Cmp &cmp, const double val) {
        return getCmp(cmp.re + val, cmp.im);
    }

    // addition
    SP_GENFUNC Cmp addCmp(const double val, const Cmp &cmp) {
        return getCmp(val + cmp.re, cmp.im);
    }

    // subtraction
    SP_GENFUNC Cmp subCmp(const Cmp &cmp0, const Cmp &cmp1) {
        return getCmp(cmp0.re - cmp1.re, cmp0.im - cmp1.im);
    }

    // subtraction
    SP_GENFUNC Cmp subCmp(const Cmp &cmp, const double val) {
        return getCmp(cmp.re - val, cmp.im);
    }

    // subtraction
    SP_GENFUNC Cmp subCmp(const double val, const Cmp &cmp) {
        return getCmp(val - cmp.re, cmp.im);
    }

    // multiple
    SP_GENFUNC Cmp mulCmp(const Cmp &cmp0, const Cmp &cmp1) {
        return getCmp(cmp0.re * cmp1.re - cmp0.im * cmp1.im, cmp0.re * cmp1.im + cmp0.im * cmp1.re);
    }

    // multiple
    SP_GENFUNC Cmp mulCmp(const Cmp &cmp, const double val) {
        return getCmp(cmp.re * val, cmp.im * val);
    }

    // multiple
    SP_GENFUNC Cmp mulCmp(const double val, const Cmp &cmp) {
        return getCmp(cmp.re * val, cmp.im * val);
    }

    // division
    SP_GENFUNC Cmp divCmp(const Cmp &cmp0, const Cmp &cmp1) {
        const Cmp tmp = mulCmp(cmp0, getCmp(cmp1.re, -cmp1.im));
        const double div = cmp1.re * cmp1.re + cmp1.im * cmp1.im;
        return getCmp(tmp.re / div, tmp.im / div);
    }

    // division
    SP_GENFUNC Cmp divCmp(const Cmp &cmp, const double val) {
        return getCmp(cmp.re / val, cmp.im / val);
    }

    // division
    SP_GENFUNC Cmp divCmp(const double val, const Cmp &cmp) {
        return divCmp(getCmp(val, 0.0), cmp);
    }

    SP_GENFUNC double fabs(const Cmp cmp) {
        return pythag(cmp.re, cmp.im);
    }


    //--------------------------------------------------------------------------------
    // complex operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Cmp operator + (const Cmp &cmp0, const Cmp &cmp1) {
        return addCmp(cmp0, cmp1);
    }
    SP_GENFUNC Cmp operator + (const Cmp &cmp, const double val) {
        return addCmp(cmp, val);
    }
    SP_GENFUNC Cmp operator + (const double val, const Cmp &cmp) {
        return addCmp(val, cmp);
    }
    SP_GENFUNC Cmp operator + (const Cmp &cmp) {
        return cmp;
    }

    SP_GENFUNC Cmp operator - (const Cmp &cmp0, const Cmp &cmp1) {
        return subCmp(cmp0, cmp1);
    }
    SP_GENFUNC Cmp operator - (const Cmp &cmp, const double val) {
        return subCmp(cmp, val);
    }
    SP_GENFUNC Cmp operator - (const double val, const Cmp &cmp) {
        return subCmp(val, cmp);
    }
    SP_GENFUNC Cmp operator - (const Cmp &cmp) {
        return mulCmp(cmp, -1.0);
    }

    SP_GENFUNC Cmp operator * (const Cmp &cmp0, const Cmp &cmp1) {
        return mulCmp(cmp0, cmp1);
    }
    SP_GENFUNC Cmp operator * (const Cmp &cmp, const double val) {
        return mulCmp(cmp, val);
    }
    SP_GENFUNC Cmp operator * (const double val, const Cmp &cmp) {
        return mulCmp(val, cmp);
    }

    SP_GENFUNC Cmp operator / (const Cmp &cmp0, const Cmp &cmp1) {
        return divCmp(cmp0, cmp1);
    }
    SP_GENFUNC Cmp operator / (const Cmp &cmp, const double val) {
        return divCmp(cmp, val);
    }
    SP_GENFUNC Cmp operator / (const double val, const Cmp &cmp) {
        return divCmp(val, cmp);
    }

    SP_GENFUNC void operator += (Cmp &cmp0, const Cmp &cmp1) {
        cmp0 = addCmp(cmp0, cmp1);
    }
    SP_GENFUNC void operator += (Cmp &cmp, const double val) {
        cmp = addCmp(cmp, val);
    }

    SP_GENFUNC void operator -= (Cmp &cmp0, const Cmp &cmp1) {
        cmp0 = subCmp(cmp0, cmp1);
    }
    SP_GENFUNC void operator -= (Cmp &cmp, const double val) {
        cmp = subCmp(cmp, val);
    }

    SP_GENFUNC void operator *= (Cmp &cmp0, const Cmp &cmp1) {
        cmp0 = mulCmp(cmp0, cmp1);
    }
    SP_GENFUNC void operator *= (Cmp &cmp, const double val) {
        cmp = mulCmp(cmp, val);
    }

    SP_GENFUNC void operator /= (Cmp &cmp0, const Cmp &cmp1) {
        cmp0 = divCmp(cmp0, cmp1);
    }
    SP_GENFUNC void operator /= (Cmp &cmp, const double val) {
        cmp = divCmp(cmp, val);
    }


    //--------------------------------------------------------------------------------
    // function
    //--------------------------------------------------------------------------------

    // gauss function
    SP_GENFUNC double funcGauss(const double x, const double mean, const double sigma){
        double ret = 0.0;

        if (fabs(sigma) > 0){
            ret = exp(-(x - mean) * (x - mean) / (2 * sigma * sigma)) / (sqrt(2 * SP_PI) * sigma);
        }
        return ret;
    }

    // tukey function
    SP_GENFUNC double funcTukey(const double x, const double t){
        double ret = 0.0;

        if (fabs(x) < t){
            const double v = 1.0 - (x * x) / (t * t);
            ret = v * v;
        }
        return ret;
    }

    // f = cs[0] * x^(n-1) + cs[1] * x^(n-2) + ...
    SP_GENFUNC double funcX(const double x, const int n, const double *cs) {
        // Horner method
        double f = 0.0;
        for (int i = 0; i < n; i++) {
            f = f * x + cs[i];
        }
        return f;
    }

    // f = sc[0] * x^(n-1) + cs[1] * x^(n-2) + ...
    SP_GENFUNC Cmp funcX(const Cmp &x, const int n, const double *cs) {
        // Horner method
        Cmp f = getCmp(0.0, 0.0);
        for (int i = 0; i < n; i++) {
            f = f * x + cs[i];
        }
        return f;
    }

    // f' = cs[0] * (n-1) * x^(n-2) + cs[1] * (n-2) * x^(n-3) + ...
    SP_GENFUNC double dfuncX(const double x, const int n, const double *cs) {
        // Horner method
        double df = 0.0;
        for (int i = 0; i < n - 1; i++) {
            df = df * x + (n - 1 - i) * cs[i];
        }
        return df;
    }

    // f' = cs[0] * (n-1) * x^(n-2) + cs[1] * (n-2) * x^(n-3) + ...
    SP_GENFUNC Cmp dfuncX(const Cmp &x, const int n, const double *cs) {
        // Horner method
        Cmp df = getCmp(0.0, 0.0);
        for (int i = 0; i < n - 1; i++) {
            df = df * x + (n - 1 - i) * cs[i];
        }
        return df;
    }


    //--------------------------------------------------------------------------------
    // mem
    //--------------------------------------------------------------------------------
    
    // set
    template <typename TYPE, typename TYPE0>
    SP_GENFUNC void setMem(TYPE *dst, const int size, const TYPE0 *mem0){
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i];
        }
    }

    // convert
    template <typename TYPE, typename TYPE0>
    SP_GENFUNC void cnvMem(TYPE *dst, const int size, const TYPE0 *mem0, const double scale = 1.0, const double base = 0.0) {
        for (int i = 0; i < size; i++) {
            cnvVal(dst[i], (mem0[i] - base) * scale);
        }
    }

    // addition
    template <typename TYPE, typename TYPE0, typename TYPE1>
    SP_GENFUNC void addMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i] + mem1[i];
        }
    }

    // subtraction
    template <typename TYPE, typename TYPE0, typename TYPE1>
    SP_GENFUNC void subMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i] - mem1[i];
        }
    }

    // multiple
    template <typename TYPE, typename TYPE0, typename TYPE1>
    SP_GENFUNC void mulMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i] * mem1[i];
        }
    }

    // division
    template <typename TYPE, typename TYPE0, typename TYPE1>
    SP_GENFUNC void divMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
        for (int i = 0; i < size; i++){
            if (mem1[i] == 0.0) continue;
            dst[i] = mem0[i] / mem1[i];
        }
    }


    // set
    template <typename TYPE, typename ELEM>
    SP_GENFUNC void setElm(TYPE *dst, const int size, const ELEM &elm){
        for (int i = 0; i < size; i++){
            dst[i] = elm;
        }
    }

    // addition
    template <typename TYPE, typename TYPE0, typename ELEM>
    SP_GENFUNC void addElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i] + elm;
        }
    }

    // subtraction
    template <typename TYPE, typename TYPE0, typename ELEM>
    SP_GENFUNC void subElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i] - elm;
        }
    }

    // multiple
    template <typename TYPE, typename TYPE0, typename ELEM>
    SP_GENFUNC void mulElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i] * elm;
        }
    }

    // division
    template <typename TYPE, typename TYPE0, typename ELEM>
    SP_GENFUNC void divElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
        if (elm == 0.0) return;
        for (int i = 0; i < size; i++){
            dst[i] = mem0[i] / elm;
        }
    }


    //--------------------------------------------------------------------------------
    // matrix
    //--------------------------------------------------------------------------------

    // identity matrix
    SP_GENFUNC void eyeMat(double *dst, const int rows, const int cols){
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++){
                dst[r * cols + c] = (r == c) ? 1.0 : 0.0;
            }
        }
    }

    // zero matrix
    SP_GENFUNC void zeroMat(double *dst, const int rows, const int cols){
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++){
                dst[r * cols + c] = 0.0;
            }
        }
    }

    // extension matrix
    SP_GENFUNC void extMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0){
        eyeMat(dst, rows, cols);

        for (int r = 0; r < minVal(rows, rows0); r++){
            for (int c = 0; c < minVal(cols, cols0); c++){
                dst[r * cols + c] = mat0[r * cols0 + c];
            }
        }
    }

    // multiple
    SP_GENFUNC void mulMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0, const double *mat1, const int rows1, const int cols1){

        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++){
                double &d = dst[r * cols + c];
                d = 0;
                for (int i = 0; i < cols0; i++){
                    d += mat0[r * cols0 + i] * mat1[i * cols1 + c];
                }
            }
        }
    }


    // transpose
    SP_GENFUNC void trnMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0){
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++){
                dst[r * cols + c] = mat0[c * cols0 + r];
            }
        }
    }

    // covariance
    SP_GENFUNC void covMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0){
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++){
                double &d = dst[r * cols + c];
                d = 0;
                for (int i = 0; i < rows0; i++){
                    d += mat0[i * cols0 + r] * mat0[i * cols0 + c];
                }
            }
        }
    }

    // skew
    SP_GENFUNC void skewMat(double *dst, const int rows, const int cols, const Vec3 &vec){
        dst[0 * 3 + 0] = 0.0;
        dst[0 * 3 + 1] = -vec.z;
        dst[0 * 3 + 2] = +vec.y;

        dst[1 * 3 + 0] = +vec.z;
        dst[1 * 3 + 1] = 0.0;
        dst[1 * 3 + 2] = -vec.x;

        dst[2 * 3 + 0] = -vec.y;
        dst[2 * 3 + 1] = +vec.x;
        dst[2 * 3 + 2] = 0.0;
    }

    // norm
    SP_GENFUNC double normMat(const double *mat, const int rows, const int cols, const double *base = NULL){
        double norm = 0.0;
        for (int i = 0; i < rows * cols; i++){
            norm += (base == NULL) ? square(mat[i]) : square(mat[i] - base[i]);
        }
        return sqrt(norm);
    }


    //--------------------------------------------------------------------------------
    // matrix determinant
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC double detMat22(const double *mat){
        return mat[0 * 2 + 0] * mat[1 * 2 + 1] - mat[0 * 2 + 1] * mat[1 * 2 + 0];
    }

    SP_GENFUNC double detMat33(const double *mat){
        const double v0 = mat[0 * 3 + 0] * (mat[1 * 3 + 1] * mat[2 * 3 + 2] - mat[2 * 3 + 1] * mat[1 * 3 + 2]);
        const double v1 = mat[0 * 3 + 1] * (mat[1 * 3 + 0] * mat[2 * 3 + 2] - mat[2 * 3 + 0] * mat[1 * 3 + 2]);
        const double v2 = mat[0 * 3 + 2] * (mat[1 * 3 + 0] * mat[2 * 3 + 1] - mat[2 * 3 + 0] * mat[1 * 3 + 1]);

        return v0 - v1 + v2;
    }

    SP_GENFUNC double detMat(const double *mat, const int rows, const int cols, double *buf){

        if (rows != cols) return 0.0;
        const int size = rows;

        for (int r = 0; r < size; r++){
            for (int c = 0; c < size; c++){
                buf[r * size + c] = mat[r * size + c];
            }
        }

        double dst = 1.0;
        for (int i = 0; i < size; i++){
            if (i == size - 1){
                dst *= buf[i * size + i];
                continue;
            }

            // partial pivoting
            {
                int pivot = i;
                double maxVal = 0.0;
                for (int r = i; r < size; r++){
                    const double val = fabs(buf[r * size + i]);
                    if (val > maxVal){
                        maxVal = val;
                        pivot = r;
                    }
                }

                if (pivot > i){
                    for (int c = 0; c < size; c++){
                        swap(buf[i * size + c], buf[pivot * size + c]);
                    }
                    dst *= -1.0;
                }
            }

            // div
            {
                const double div = buf[i * size + i];
                if (fabs(div) < SP_SMALL) return 0.0;

                dst *= div;
                for (int c = i; c < size; c++){
                    buf[i * size + c] /= div;
                }

                for (int r = i + 1; r < size; r++){
                    const double scl = buf[r * size + i];
                    for (int c = i + 1; c < size; c++){
                        buf[r * size + c] -= buf[i * size + c] * scl;
                    }
                }
            }
        }
        return dst;
    }

    //--------------------------------------------------------------------------------
    // matrix inverse
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC bool invMat22(double *dst, const double *mat){
        const double det = detMat22(mat);
        if (fabs(det) < SP_SMALL) return false;

        dst[0 * 2 + 0] = +mat[1 * 2 + 1];
        dst[0 * 2 + 1] = -mat[0 * 2 + 1];
        dst[1 * 2 + 0] = -mat[1 * 2 + 0];
        dst[1 * 2 + 1] = +mat[0 * 2 + 0];

        mulElm(dst, 2 * 2, dst, 1.0 / det);
        return true;
    }

    SP_GENFUNC bool invMat33(double *dst, const double *mat){
        const double det = detMat33(mat);
        if (fabs(det) < SP_SMALL) return false;

        dst[0 * 3 + 0] = +(mat[1 * 3 + 1] * mat[2 * 3 + 2] - mat[1 * 3 + 2] * mat[2 * 3 + 1]);
        dst[0 * 3 + 1] = -(mat[0 * 3 + 1] * mat[2 * 3 + 2] - mat[0 * 3 + 2] * mat[2 * 3 + 1]);
        dst[0 * 3 + 2] = +(mat[0 * 3 + 1] * mat[1 * 3 + 2] - mat[0 * 3 + 2] * mat[1 * 3 + 1]);

        dst[1 * 3 + 0] = -(mat[1 * 3 + 0] * mat[2 * 3 + 2] - mat[1 * 3 + 2] * mat[2 * 3 + 0]);
        dst[1 * 3 + 1] = +(mat[0 * 3 + 0] * mat[2 * 3 + 2] - mat[0 * 3 + 2] * mat[2 * 3 + 0]);
        dst[1 * 3 + 2] = -(mat[0 * 3 + 0] * mat[1 * 3 + 2] - mat[0 * 3 + 2] * mat[1 * 3 + 0]);

        dst[2 * 3 + 0] = +(mat[1 * 3 + 0] * mat[2 * 3 + 1] - mat[1 * 3 + 1] * mat[2 * 3 + 0]);
        dst[2 * 3 + 1] = -(mat[0 * 3 + 0] * mat[2 * 3 + 1] - mat[0 * 3 + 1] * mat[2 * 3 + 0]);
        dst[2 * 3 + 2] = +(mat[0 * 3 + 0] * mat[1 * 3 + 1] - mat[0 * 3 + 1] * mat[1 * 3 + 0]);

        mulElm(dst, 3 * 3, dst, 1.0 / det);
        return true;
    }

    SP_GENFUNC bool invMat(double *dst, const double *mat, const int rows, const int cols, double *buf){

        if (rows != cols) return false;
        const int size = rows;

        for (int r = 0; r < size; r++){
            for (int c = 0; c < size; c++){
                dst[r * size + c] = (r == c) ? 1.0 : 0.0;
                buf[r * size + c] = mat[r * size + c];
            }
        }

        // Gauss-Jordan
        for (int i = 0; i < size; i++){
            // partial pivoting
            {
                int pivot = i;
                double maxVal = 0.0;
                for (int r = i; r < size; r++){
                    const double val = fabs(buf[r * size + i]);
                    if (val > maxVal){
                        maxVal = val;
                        pivot = r;
                    }
                }

                if (pivot > i){
                    for (int c = 0; c < size; c++){
                        swap(dst[i * size + c], dst[pivot * size + c]);
                        swap(buf[i * size + c], buf[pivot * size + c]);
                    }
                }
            }

            // div
            {
                const double div = buf[i * size + i];
                if (fabs(div) < SP_SMALL) return false;

                for (int c = 0; c < size; c++){
                    dst[i * size + c] /= div;
                    buf[i * size + c] /= div;
                }

                for (int r = 0; r < size; r++){
                    if (r == i) continue;

                    const double scl = buf[r * size + i];
                    for (int c = 0; c < size; c++){
                        dst[r * size + c] -= dst[i * size + c] * scl;
                        buf[r * size + c] -= buf[i * size + c] * scl;
                    }
                }
            }
        }
        return true;
    }


    //--------------------------------------------------------------------------------
    // matrix eigen
    //--------------------------------------------------------------------------------

    SP_GENFUNC bool eigMat(double *eigVec, double *eigVal, const double *mat, const int rows, const int cols, const bool minOrder = true){
    
        if (rows < 2 || cols < 2 || rows != cols) return false;

        const int size = rows;

        for (int r = 0; r < size; r++){
            for (int c = 0; c < size; c++){
                eigVec[r * size + c] = (r == c) ? 1.0 : 0.0;
                eigVal[r * size + c] = mat[r * size + c];
            }
        }

        const int maxit = size * size;

        // jacobi algorithm
        for (int it = 0; it < maxit; it++){
            int p = 0, q = 0;
            double maxv = 0.0;

            for (int r = 0; r < size; r++){
                for (int c = r + 1; c < size; c++){

                    const double val = fabs(eigVal[r * size + c]);
                    if (val > maxv){
                        maxv = val;
                        p = r;
                        q = c;
                    }
                }
            }
            if (maxv < SP_SMALL) break;

            const double app = eigVal[p * size + p];
            const double apq = eigVal[p * size + q];
            const double aqq = eigVal[q * size + q];

            double sinv, cosv;
            {
                const double a = (app - aqq) / 2.0;
                const double b = -apq;

                // g = cos(2A) = |a| / sqrt(a * a + b * b)
                const double g = fabs(a) / pythag(a, b);

                sinv = sqrt((1.0 - g) / 2.0) * sign(a * b);
                cosv = sqrt((1.0 + g) / 2.0);
            }

            for (int i = 0; i < size; i++){
                if (i == p || i == q) continue;
                const double tmpa = cosv * eigVal[p * size + i] - sinv * eigVal[q * size + i];
                const double tmpb = sinv * eigVal[p * size + i] + cosv * eigVal[q * size + i];

                eigVal[i * size + p] = eigVal[p * size + i] = tmpa;
                eigVal[i * size + q] = eigVal[q * size + i] = tmpb;
            }
            {
                eigVal[p * size + p] = cosv * cosv * app + sinv * sinv * aqq - 2 * sinv * cosv * apq;
                eigVal[p * size + q] = sinv * cosv * (app - aqq) + (cosv * cosv - sinv * sinv) * apq;
                eigVal[q * size + p] = sinv * cosv * (app - aqq) + (cosv * cosv - sinv * sinv) * apq;
                eigVal[q * size + q] = sinv * sinv * app + cosv * cosv * aqq + 2 * sinv * cosv * apq;
            }

            for (int i = 0; i < size; i++){
                const double tmpa = cosv * eigVec[i * size + p] - sinv * eigVec[i * size + q];
                const double tmpb = sinv * eigVec[i * size + p] + cosv * eigVec[i * size + q];

                eigVec[i * size + p] = tmpa;
                eigVec[i * size + q] = tmpb;
            }
        }

        for (int r = 0; r < size; r++){
            for (int c = 0; c < size; c++){
                if (c == r) continue;

                eigVal[r * size + c] = 0.0;
            }
        }

        // sort
        for (int c = 0; c < size - 1; c++){
            int maxid = c;
            int minid = c;

            double maxv = 0.0;
            double minv = SP_INFINITY;

            for (int i = c; i < size; i++){
                const double val = ::fabs(eigVal[i * size + i]);
                if (val > maxv){
                    maxv = val;
                    maxid = i;
                }
                if (val < minv){
                    minv = val;
                    minid = i;
                }
            }

            const int select = (minOrder == true) ? minid : maxid;

            if (select != c){
                swap(eigVal[c * size + c], eigVal[select * size + select]);
                for (int r = 0; r < size; r++){
                    swap(eigVec[r * size + c], eigVec[r * size + select]);
                }
            }
        }

        return true;
    }


    //--------------------------------------------------------------------------------
    // matrix svd (simgular value decomposition)
    //--------------------------------------------------------------------------------

    SP_GENFUNC bool svdMat(double *U, double *S, double *V, const double *mat, const int rows, const int cols, const bool minOrder = true){
        if (rows < 2 || cols < 2 || rows < cols) return false;

        for (int i = 0; i < rows * cols; i++){
            U[i] = mat[i];
        }
        for (int i = 0; i < cols * cols; i++){
            V[i] = 0.0;
            S[i] = 0.0;
        }
        double *Q = &S[0];
        double *R = &S[cols];

        // householder reduction to bidiagonal form
        for (int i = 0; i < cols; i++) {

            {
                double scale = 0.0;
                for (int r = i; r < rows; r++){
                    scale += fabs(U[r * cols + i]);
                }

                double val = 0.0;
                if (scale > 0.0) {
                    double s = 0.0;
                    for (int r = i; r < rows; r++) {
                        U[r * cols + i] /= scale;
                        s += U[r * cols + i] * U[r * cols + i];
                    }

                    double f = U[i * cols + i];
                    double g = -sign(f) * sqrt(s);
                    U[i * cols + i] = f - g;

                    double h = f * g - s;

                    for (int j = i + 1; j < cols; j++) {
                        s = 0.0;
                        for (int r = i; r < rows; r++){
                            s += U[r * cols + i] * U[r * cols + j];
                        }
                        f = s / h;
                        for (int r = i; r < rows; r++){
                            U[r * cols + j] += f * U[r * cols + i];
                        }
                    }
                    for (int r = i; r < rows; r++){
                        U[r * cols + i] *= scale;
                    }
                    val = scale * g;
                }
                Q[i] = val;
            }

            if (i < cols - 1) {
                double scale = 0.0;
                for (int c = i + 1; c < cols; c++){
                    scale += fabs(U[i * cols + c]);
                }

                double val = 0.0;
                if (scale > 0.0){
                    double s = 0.0;

                    for (int c = i + 1; c < cols; c++) {
                        U[i * cols + c] /= scale;
                        s += U[i * cols + c] * U[i * cols + c];
                    }
                    double f = U[i * cols + i + 1];
                    double g = -sign(f) * sqrt(s);
                    double h = f * g - s;
                    U[i * cols + i + 1] = f - g;

                    for (int c = i + 1; c < cols; c++){
                        R[c] = U[i * cols + c] / h;
                    }

                    for (int j = i + 1; j < rows; j++) {
                        s = 0.0;
                        for (int c = i + 1; c < cols; c++){
                            s += U[j * cols + c] * U[i * cols + c];
                        }
                        for (int c = i + 1; c < cols; c++){
                            U[j * cols + c] += s * R[c];
                        }
                    }
                    for (int c = i + 1; c < cols; c++){
                        U[i * cols + c] *= scale;
                    }
                    val = scale * g;
                }
                R[i + 1] = val;
            }
        }

        double unorm = 0.0;
        for (int i = 0; i < cols; i++) {
            unorm = maxVal(unorm, fabs(Q[i]) + fabs(R[i]));

        }

        // accumulation of right-hand transformations
        for (int i = cols - 1; i >= 0; i--) {
            const double g = R[i + 1];
            if (i < cols - 1){
                if (g){

                    // double division to avoid possible underflow
                    for (int j = i + 1; j < cols; j++){
                        V[j * cols + i] = (U[i * cols + j] / U[i * cols + (i + 1)]) / g;
                    }
                    for (int j = i + 1; j < cols; j++) {
                        double s = 0.0;
                        for (int k = i + 1; k < cols; k++){
                            s += U[i * cols + k] * V[k * cols + j];
                        }
                        for (int k = i + 1; k < cols; k++){
                            V[k * cols + j] += s * V[k * cols + i];
                        }
                    }
                }
                for (int j = i + 1; j < cols; j++){
                    V[i * cols + j] = V[j * cols + i] = 0.0;
                }
            }
            V[i * cols + i] = 1.0;
        }

        // accumulation of left-hand transformations
        for (int i = minVal(rows, cols) - 1; i >= 0; i--){ 
            
            for (int j = i + 1; j < cols; j++){
                U[i * cols + j] = 0.0;
            }

            double g = Q[i];
            if (g) {
                g = 1.0 / g;
                for (int j = i + 1; j < cols; j++) {
                    double s = 0.0;
                    for (int k = i + 1; k < rows; k++){
                        s += U[k * cols + i] * U[k * cols + j];
                    }
                    double f = (s / U[i * cols + i]) * g;

                    for (int k = i; k < rows; k++){
                        U[k * cols + j] += f * U[k * cols + i];
                    }
                }
                for (int j = i; j < rows; j++){
                    U[j * cols + i] *= g;
                }
            }
            else{
                for (int j = i; j < rows; j++){
                    U[j * cols + i] = 0.0;
                }
            }
            U[i * cols + i]++;
        }

        // diagonalization of the bidiagonal form
        const int maxit = 30;

        for (int k = cols - 1; k >= 0; k--){

            for (int it = 0; it < maxit; it++) {
                int flag = 1;

                int l = 0;
                int n = 0;

                // test for splitting
                for (l = k; l >= 0; l--) { 

                    // Note that buf[0] is always zero
                    n = l - 1; 
                    if ((fabs(R[l]) + unorm) == unorm) {
                        flag = 0;
                        break;
                    }
                    if ((fabs(Q[n]) + unorm) == unorm){
                        break;
                    }
                }
                if (flag) {
                    // cancellation of R[l], if l > 0
                    double c = 0.0;
                    double s = 1.0;
                    for (int i = l; i <= k; i++) {
                        double f = s * R[i];
                        R[i] = c * R[i];

                        if ((double)(fabs(f) + unorm) == unorm){
                            break;
                        }
                        double g = Q[i];
                        double h = pythag(f, g);
                        Q[i] = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = -f * h;
                        for (int j = 0; j < rows; j++) {
                            const double y = U[j * cols + n];
                            const double z = U[j * cols + i];
                            U[j * cols + n] = y * c + z * s;
                            U[j * cols + i] = z * c - y * s;
                        }
                    }
                }
                double z = Q[k];

                // convergence
                if (l == k) {
                    // singular value is made nonnegative
                    if (z < 0.0){
                        Q[k] = -z;
                        for (int j = 0; j < cols; j++){
                            V[j * cols + k] = -V[j * cols + k];
                        }
                    }
                    break;
                }

                if (it == maxit - 1){
                    SP_PRINTD("no convergence in SVD iterations");
                }

                // shift from bottom 2-by-2 minor
                n = k - 1;
                double x = Q[l];
                double y = Q[n];
                double g = R[n];
                double h = R[k];
                double f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
                g = pythag(f, 1.0);
                f = ((x - z)*(x + z) + h*((y / (f + sign(f) * g)) - h)) / x;
                
                double c = 1.0;
                double s = 1.0;

                // next QR transformation
                for (int j = l; j <= n; j++) {
                    int i = j + 1;
                    g = R[i];
                    y = Q[i];
                    h = s*g;
                    g = c*g;
                    z = pythag(f, h);
                    R[j] = z;
                    c = f / z;
                    s = h / z;
                    f = x*c + g*s;
                    g = g*c - x*s;
                    h = y*s;
                    y *= c;
                    for (int jj = 0; jj < cols; jj++) {
                        x = V[jj * cols + j];
                        z = V[jj * cols + i];
                        V[jj * cols + j] = x*c + z*s;
                        V[jj * cols + i] = z*c - x*s;
                    }
                    z = pythag(f, h);
                    Q[j] = z; /* Rotation can be arbitrary if z = 0. */
                    if (z) {
                        z = 1.0 / z;
                        c = f*z;
                        s = h*z;
                    }
                    f = c*g + s*y;
                    x = c*y - s*g;
                    for (int jj = 0; jj < rows; jj++) {
                        y = U[jj * cols + j];
                        z = U[jj * cols + i];
                        U[jj * cols + j] = y*c + z*s;
                        U[jj * cols + i] = z*c - y*s;
                    }
                }
                R[l] = 0.0;
                R[k] = f;
                Q[k] = x;
            }
        }

        for (int i = 0; i < cols; i++){
            R[i] = 0.0;
        }
        for (int i = 0; i < cols; i++){
            swap(S[i * cols + i], Q[i]);
        }

        // sort
        for (int c = 0; c < cols - 1; c++){
            int maxid = c;
            int minid = c;

            double maxv = 0.0;
            double minv = SP_INFINITY;

            for (int i = c; i < cols; i++){
                const double val = S[i * cols + i];
                if (val > maxv){
                    maxv = val;
                    maxid = i;
                }
                if (val < minv){
                    minv = val;
                    minid = i;
                }
            }

            const int select = (minOrder == true) ? minid : maxid;

            if (select != c){
                swap(S[c * cols + c], S[select * cols + select]);
                for (int r = 0; r < rows; r++){
                    swap(U[r * cols + c], U[r * cols + select]);
                }
                for (int r = 0; r < cols; r++){
                    swap(V[r * cols + c], V[r * cols + select]);
                }
            }
        }
        return true;
    }


    //--------------------------------------------------------------------------------
    // equation
    //--------------------------------------------------------------------------------

    // a * x^2 + b * x + c = 0
    SP_GENFUNC int eq2(Cmp xs[2], const double a, const double b, const double c) {
        if (fabs(a) < SP_SMALL) {
            if (fabs(b) < SP_SMALL) {
                return 0;
            }
            else {
                xs[0] = getCmp(-c / b, 0.0);
                return 1;
            }
        }

        const double D = b * b - 4.0 * a * c;

        int ret = 0;
        if (fabs(D) < SP_SMALL) {

            xs[0] = getCmp(-b / (2.0 * a), 0.0);

            ret = 1;
        }
        else if (D > 0.0) {

            xs[0] = getCmp((-b + sqrt(D)) / (2.0 * a), 0.0);
            xs[1] = getCmp((-b - sqrt(D)) / (2.0 * a), 0.0);

            ret = 2;
        }
        else if (D < 0.0) {

            xs[0] = getCmp(-b / (2.0 * a), +sqrt(-D) / (2.0 * a));
            xs[1] = getCmp(-b / (2.0 * a), -sqrt(-D) / (2.0 * a));

            ret = 2;
        }
        return ret;
    }

    // a * x^3 + b * x^2 + c * x + d = 0
    SP_GENFUNC int eq3(Cmp xs[3], const double a, const double b, const double c, const double d) {
        if (fabs(a) < SP_SMALL) {
            return eq2(xs, b, c, d);
        }

        const double nb = b / a;
        const double nc = c / a;
        const double nd = d / a;

        const double p = nc - nb * nb / 3.0;
        const double q = nd - nb * nc / 3.0 + 2.0 * nb * nb * nb / 27.0;

        const double D = -square(q / 2.0) - cubic(p / 3.0);

        const double A = -nb / 3.0;
        const double B = q / 2.0;

        int ret = 0;
        if (fabs(D) < SP_SMALL) {
            if (fabs(q) < SP_SMALL) {

                xs[0] = getCmp(A, 0.0);

                ret = 1;
            }
            else {

                xs[0] = getCmp(A - 2 * cbrt(B), 0.0);
                xs[1] = getCmp(A + cbrt(B), 0.0);

                ret = 2;
            }
        }
        else if (D > 0.0) {

            const double theta = atan2(sqrt(D), -B);
            const double R = pow(B * B + D, 1.0 / 6.0) * cos(theta / 3.0);
            const double Q = pow(B * B + D, 1.0 / 6.0) * sin(theta / 3.0);

            xs[0] = getCmp(A + 2 * R, 0.0);
            xs[1] = getCmp(A - R - Q * sqrt(3.0), 0.0);
            xs[2] = getCmp(A - R + Q * sqrt(3.0), 0.0);

            ret = 3;
        }
        else if (D < 0.0) {

            const double S = cbrt(-B + sqrt(-D));
            const double T = cbrt(-B - sqrt(-D));

            xs[0] = getCmp(A + (S + T), 0.0);
            xs[1] = getCmp(A - (S + T) / 2.0, +(S - T) * sqrt(3.0) / 2.0);
            xs[2] = getCmp(A - (S + T) / 2.0, -(S - T) * sqrt(3.0) / 2.0);

            ret = 3;
        }
        return ret;
    }

    // a * x^4 + b * x^3 + c * x^2 + d * x + e = 0
    SP_GENFUNC int eq4(Cmp xs[4], const double a, const double b, const double c, const double d, const double e) {
        if (fabs(a) < SP_SMALL) {
            return eq3(xs, b, c, d, e);
        }

        const double nb = b / a;
        const double nc = c / a;
        const double nd = d / a;
        const double ne = e / a;

        const double b2 = nb / 4;

        const double p = nc - 6 * b2 * b2;
        const double q = nd - 2 * nc * b2 + 8 * pow(b2, 3);

        const double r = ne - nd * b2 + nc * b2 * b2 - 3 * pow(b2, 4);
        const double x = 2 * p;
        const double y = p * p - 4 * r;
        const double z = -q * q;

        Cmp e3[3];
        const int nn = eq3(e3, 1.0, x, y, z);
        const double u = e3[0].re;

        const double R = -sqrt(u) / 2;
        const double S = (p + u) / 2;
        const double T = -q / (4 * R);
        const double Dp = R * R - S + T;
        const double Dm = R * R - S - T;

        int ret = 0;
        if (fabs(Dp) < SP_SMALL) {
            xs[ret + 0] = getCmp(-b2 + R, 0.0);

            ret++;
        }
        else if (Dp > 0.0){
            
            xs[ret + 0] = getCmp(-b2 + R + sqrt(Dp), 0.0);
            xs[ret + 1] = getCmp(-b2 + R - sqrt(Dp), 0.0);

            ret += 2;
        }
        else if (Dp < 0.0){
            
            xs[ret + 0] = getCmp(-b2 + R, +sqrt(-Dp));
            xs[ret + 1] = getCmp(-b2 + R, -sqrt(-Dp));

            ret += 2;
        }

        if (fabs(Dm) < SP_SMALL) {
            xs[ret + 0] = getCmp(-b2 - R, 0.0);

            ret++;
        }
        else if (Dm > 0.0) {

            xs[ret + 0] = getCmp(-b2 - R + sqrt(Dm), 0.0);
            xs[ret + 1] = getCmp(-b2 - R - sqrt(Dm), 0.0);

            ret += 2;
        }
        else if (Dm < 0.0) {

            xs[ret + 0] = getCmp(-b2 - R, +sqrt(-Dm));
            xs[ret + 1] = getCmp(-b2 - R, -sqrt(-Dm));

            ret += 2;
        }

        return ret;
    }

    // newton method
    SP_GENFUNC bool newton(double &x, const int csize, const double *cs, const int maxit = 50, const double eps = 1e-6) {

        for (int it = 0; it < maxit; it++) {
            const double f = funcX(x, csize, cs);
            const double df = dfuncX(x, csize, cs);

            const double dx = f / (df + 0.001);
            x = x - dx;
            
            const double err = fabs(funcX(x, csize, cs));
            if (err < eps) {
                return true;
            }
        }

        return false;
    }

    // newton method (Durand-Kerner method)
    SP_GENFUNC bool newton(Cmp *xs, const int csize, const double *cs, const int maxit = 50, const double eps = 1e-6) {
       
        const int n = csize - 1;
        for (int it = 0; it < maxit; it++) {

            for (int i = 0; i < n; i++) {
                const Cmp f = funcX(xs[i], csize, cs);
                
                Cmp df = getCmp(cs[0], 0.0);
                for (int j = 0; j < n; j++) {
                    if (j != i) {
                        df *= xs[i] - xs[j];
                    }
                }

                xs[i] = xs[i] - f / df;
            }

            double maxe = 0.0;
            for (int i = 0; i < n; i++) {
                const double err = fabs(funcX(xs[i], csize, cs));
                if (err > maxe) {
                    maxe = err;
                }
            }

            if (maxe < eps) {
                return true;
            }
        }

        return false;
    }

    SP_GENFUNC bool aberth(Cmp *xs, const int csize, const double *cs, const int maxit = 50, const double eps = 1e-6) {

        const int n = csize - 1;

        const Cmp zc = -getCmp(cs[1], 0.0) / (cs[0] * n);

        Cmp a[100];
        for (int i = 0; i < csize; i++) {
            a[i] = getCmp(cs[i], 0.0);
        }

        // horner coeff
        for (int i = 0; i < csize; i++) {
            for (int j = 1; j < csize - i; j++) {
                a[j] += zc * a[j - 1];
            }
        }

        double b[100];
        for (int i = 0; i < csize; ++i) {
            b[i] = (i == 0) ? +fabs(a[i]) : -fabs(a[i]);
        }

        double r = 100.0;
        if(newton(r, csize, b, maxit, eps) == false) return false;

        for (int i = 0; i < n; i++) {
            const double theta = (2 * SP_PI / n) * i + SP_PI / (2.0 * n);
            xs[i] = zc + r * getCmp(cos(theta), sin(theta));
        }

        return true;
    }


    // f(x) = 0, f(x) = cs[0] * x^(n-1) + cs[1] * x^(n-2) + ...
    SP_GENFUNC int eqn(Cmp xs[], const int csize, const double *cs, const int maxit = 50, const double eps = 1e-6) {
        if (csize < 2) return 0;

        if (fabs(cs[0]) < SP_SMALL) {
            return eqn(xs, csize - 1, cs + 1, maxit, eps);
        }
        else {
            if (aberth(xs, csize, cs, maxit, eps) == false) return 0;

            if (newton(xs, csize, cs, maxit, eps) == false) return 0;
            return csize - 1;
        }
    }
}

#endif
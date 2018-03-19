//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MOP_H__
#define __SP_MOP_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spmath.h"
#include "spcore/spgen/sppose.h"

#include "spcore/spcpu/spmem.h"

namespace sp{


    //--------------------------------------------------------------------------------
    // mem
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC void setMem(Mem<TYPE> &dst, const Mem<TYPE> &mem0){

        dst.resize(mem0.dim, mem0.dsize);
        setMem(dst.ptr, dst.size(), mem0.ptr);
    }

    // convert mem type [dst = (src - base) * scale)
    template<typename TYPE, typename TYPE0>
    SP_CPUFUNC void cnvMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const double scale = 1.0, const double base = 0.0){
        dst.resize(mem0.dim, mem0.dsize);
        for (int i = 0; i < mem0.size(); i++){
            cnvVal(dst[i], (mem0[i] - base) * scale);
        }
    }

    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void addMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1){
        if (cmpSize(mem0, mem1) == false) return;

        dst.resize(mem0.dim, mem0.dsize);
        addMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
    }

    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void subMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1){
        if (cmpSize(mem0, mem1) == false) return;

        dst.resize(mem0.dim, mem0.dsize);
        subMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
    }

    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void mulMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1){
        if (cmpSize(mem0, mem1) == false) return;

        dst.resize(mem0.dim, mem0.dsize);
        mulMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
    }

    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void divMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1){
        if (cmpSize(mem0, mem1) == false) return;

        dst.resize(mem0.dim, mem0.dsize);
        divMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
    }


    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void setElm(Mem<TYPE> &dst, const ELEM &elm){
        if (isValid(dst) == false) return;

        setElm(dst.ptr, dst.size(), elm);
    }

    template<typename TYPE, typename TYPE0, typename ELEM>
    SP_CPUFUNC void addElm(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const ELEM &elm){

        dst.resize(mem0.dim, mem0.dsize);
        addElm(dst.ptr, dst.size(), mem0.ptr, elm);
    }

    template<typename TYPE, typename TYPE0, typename ELEM>
    SP_CPUFUNC void subElm(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const ELEM &elm){

        dst.resize(mem0.dim, mem0.dsize);
        subElm(dst.ptr, dst.size(), mem0.ptr, elm);
    }

    template<typename TYPE, typename TYPE0, typename ELEM>
    SP_CPUFUNC void mulElm(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const ELEM &elm){

        dst.resize(mem0.dim, mem0.dsize);
        mulElm(dst.ptr, dst.size(), mem0.ptr, elm);
    }

    template<typename TYPE, typename TYPE0, typename ELEM>
    SP_CPUFUNC void divElm(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const ELEM &elm){

        dst.resize(mem0.dim, mem0.dsize);
        divElm(dst.ptr, dst.size(), mem0.ptr, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void addElm(Mem<TYPE> &dst, const ELEM &elm){
        addElm(dst, dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void subElm(Mem<TYPE> &dst, const ELEM &elm){
        subElm(dst, dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void mulElm(Mem<TYPE> &dst, const ELEM &elm){
        mulElm(dst, dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void divElm(Mem<TYPE> &dst, const ELEM &elm){
        divElm(dst, dst, elm);
    }


    //--------------------------------------------------------------------------------
    // mem util
    //--------------------------------------------------------------------------------

    // range [start, start + step, ...]
    template<typename TYPE = int>
    SP_CPUFUNC Mem1<TYPE> range(const int start, const int stop, const int step = 1) {
        Mem1<TYPE> index(stop - start);

        for (int i = 0; i < index.size(); i++) {
            cnvVal(index[i], start + i * step);
        }
        return index;
    }

    template<typename TYPE>
    SP_CPUFUNC Mem<TYPE> shuffle(const Mem<TYPE> &src, const int seed = 0) {

        Mem<int> index = range(0, src.size());

        srand(seed);
        for (int i = 0; i < index.size(); i++) {
            const int p = rand() % index.size();
            swap(index[i], index[p]);
        }

        Mem<TYPE> ret(src.dim, src.dsize);
        for (int i = 0; i < ret.size(); i++) {
            ret[i] = src[index[i]];
        }
        return ret;
    }


    //--------------------------------------------------------------------------------
    // mem1 operator
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem1<TYPE> operator + (const Mem1<TYPE> &mem0, const ELEM &elm){
        Mem1<TYPE> dst;
        addElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem1<TYPE> operator - (const Mem1<TYPE> &mem0, const ELEM &elm){
        Mem1<TYPE> dst;
        subElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem1<TYPE> operator * (const Mem1<TYPE> &mem0, const ELEM &elm){
        Mem1<TYPE> dst;
        mulElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem1<TYPE> operator / (const Mem1<TYPE> &mem0, const ELEM &elm){
        Mem1<TYPE> dst;
        divElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator += (Mem1<TYPE> &dst, const ELEM &elm){
        addElm(dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator -= (Mem1<TYPE> &dst, const ELEM &elm){
        subElm(dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator *= (Mem1<TYPE> &dst, const ELEM &elm){
        mulElm(dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator /= (Mem1<TYPE> &dst, const ELEM &elm){
        divElm(dst, elm);
    }

    //--------------------------------------------------------------------------------
    // mem2 operator
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem2<TYPE> operator + (const Mem2<TYPE> &mem0, const ELEM &elm) {
        Mem2<TYPE> dst;
        addElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem2<TYPE> operator - (const Mem2<TYPE> &mem0, const ELEM &elm) {
        Mem2<TYPE> dst;
        subElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem2<TYPE> operator * (const Mem2<TYPE> &mem0, const ELEM &elm) {
        Mem2<TYPE> dst;
        mulElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC Mem2<TYPE> operator / (const Mem2<TYPE> &mem0, const ELEM &elm) {
        Mem2<TYPE> dst;
        divElm(dst, mem0, elm);
        return dst;
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator += (Mem2<TYPE> &dst, const ELEM &elm) {
        addElm(dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator -= (Mem2<TYPE> &dst, const ELEM &elm) {
        subElm(dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator *= (Mem2<TYPE> &dst, const ELEM &elm) {
        mulElm(dst, elm);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void operator /= (Mem2<TYPE> &dst, const ELEM &elm) {
        divElm(dst, elm);
    }


    //--------------------------------------------------------------------------------
    // get matrix
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat getMat(const Vec2 &vec) {
        Mat dst(2, 1);
        dst[0] = vec.x;
        dst[1] = vec.y;
        return dst;
    }

    SP_CPUFUNC Mat getMat(const Vec3 &vec) {
        Mat dst(3, 1);
        dst[0] = vec.x;
        dst[1] = vec.y;
        dst[2] = vec.z;
        return dst;
    }

    SP_CPUFUNC Mat getMat(const Rot &rot) {
        Mat dst(3, 3);

        getMat(dst.ptr, dst.rows(), dst.cols(), rot);
        return dst;
    }

    SP_CPUFUNC Rot getRot(const Mat &mat) {
        return getRot(mat.ptr, mat.rows(), mat.cols());
    }

    SP_CPUFUNC Mat getMat(const Pose &pose) {
        Mat dst(3, 4);

        getMat(dst.ptr, dst.rows(), dst.cols(), pose);
        return dst;
    }

    SP_CPUFUNC Mat getMat(const CamParam &cam) {
        Mat dst(3, 3);

        getMat(dst.ptr, dst.rows(), dst.cols(), cam);
        return dst;
    }

    SP_CPUFUNC Mat getMatAngleX(const double angle) {
        Mat dst(3, 3);

        getMatAngleX(dst.ptr, dst.rows(), dst.cols(), angle);
        return dst;
    }

    SP_CPUFUNC Mat getMatAngleY(const double angle) {
        Mat dst(3, 3);

        getMatAngleY(dst.ptr, dst.rows(), dst.cols(), angle);
        return dst;
    }

    SP_CPUFUNC Mat getMatAngleZ(const double angle) {
        Mat dst(3, 3);

        getMatAngleZ(dst.ptr, dst.rows(), dst.cols(), angle);
        return dst;
    }

    SP_GENFUNC Mat getMatEuler(const Vec3 &euler) {
        return getMat(getRotEuler(euler));
    }

    SP_CPUFUNC Vec3 getEuler(const Mat &mat) {
        return getEuler(mat.ptr, mat.rows(), mat.cols());
    }

    //--------------------------------------------------------------------------------
    // matrix util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat eyeMat(const int rows, const int cols){
        Mat dst(rows, cols);
    
        eyeMat(dst.ptr, dst.rows(), dst.cols());
        return dst;
    }

    SP_CPUFUNC Mat zeroMat(const int rows, const int cols){
        Mat dst(rows, cols);

        zeroMat(dst.ptr, dst.rows(), dst.cols());
        return dst;
    }

    SP_CPUFUNC Mat extMat(const int rows, const int cols, const Mat &mat){
        Mat dst(rows, cols);

        extMat(dst.ptr, dst.rows(), dst.cols(), mat.ptr, mat.rows(), mat.cols());
        return dst;
    }

    SP_CPUFUNC Mat trnMat(const Mat &mat){
        Mat dst(mat.cols(), mat.rows());

        trnMat(dst.ptr, dst.rows(), dst.cols(), mat.ptr, mat.rows(), mat.cols());
        return dst;
    }

    SP_CPUFUNC Mat covMat(const Mat &mat){
        Mat dst(mat.cols(), mat.cols());

        covMat(dst.ptr, dst.rows(), dst.cols(), mat.ptr, mat.rows(), mat.cols());
        return dst;
    }

    SP_CPUFUNC Mat skewMat(const Vec3 &vec){
        Mat dst(3, 3);
        skewMat(dst.ptr, 3, 3, vec);
        return dst;
    }

    SP_CPUFUNC double normMat(const Mat &mat){
        return normMat(mat.ptr, mat.rows(), mat.cols());
    }

    SP_CPUFUNC double detMat(const Mat &mat){
        Mat buf(mat.dsize);
        return detMat(mat.ptr, mat.rows(), mat.cols(), buf.ptr);
    }

    SP_CPUFUNC Mat invMat(const Mat &mat) {
        Mat dst(mat.dsize);
        Mat buf(mat.dsize);
        if (invMat(dst.ptr, mat.ptr, mat.rows(), mat.cols(), buf.ptr) == false) {
            dst.clear();
        }

        return dst;
    }

    SP_CPUFUNC bool eigMat(Mat &eigVec, Mat &eigVal, const Mat &mat, const bool minOrder = true){
        if (mat.rows() > 0 && mat.rows() != mat.cols()) return false;

        eigVec.resize(mat.dsize);
        eigVal.resize(mat.dsize);
        Mat V(mat.dsize);
        return svdMat(eigVec.ptr, eigVal.ptr, V.ptr, mat.ptr, mat.rows(), mat.cols(), minOrder);

        //return eigMat(eigVec.ptr, eigVal.ptr, mat.ptr, mat.rows(), mat.cols(), minOrder);
    }

    SP_CPUFUNC bool svdMat(Mat &U, Mat &S, Mat &V, const Mat &mat, const bool minOrder = true){
        Mat m;
        if (mat.rows() < mat.cols()){
            m = zeroMat(mat.cols(), mat.cols());
            for (int i = 0; i < mat.size(); i++){
                m[i] = mat[i];
            }
        }
        else{
            m = mat;
        }
        U.resize(m.rows(), m.cols());
        S.resize(m.cols(), m.cols());
        V.resize(m.cols(), m.cols());

        return svdMat(U.ptr, S.ptr, V.ptr, m.ptr, m.rows(), m.cols(), minOrder);
    }


    //--------------------------------------------------------------------------------
    // calc matrix
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC void mulMat(Mat &dst, const Mat &mat0, const Mat &mat1){
        if (mat0.rows() > 0 && mat1.cols() > 0 && mat0.cols() == mat1.rows()){
            dst.resize(mat0.rows(), mat1.cols());
            mulMat(dst.ptr, dst.dsize[1], dst.dsize[0], mat0.ptr, mat0.rows(), mat0.cols(), mat1.ptr, mat1.rows(), mat1.cols());
        }
    }

    SP_CPUFUNC void addMat(Mat &dst, const Mat &mat0, const Mat &mat1){
        if (mat0.rows() == mat1.rows() && mat0.cols() == mat1.cols()){
            dst.resize(mat0.dsize);
            addMem(dst.ptr, dst.size(), mat0.ptr, mat1.ptr);
            return;
        }
        if (mat0.rows() > 0 && mat1.rows() == 1 && mat0.cols() == mat1.cols()){
            dst.resize(mat0.dsize);
            for (int r = 0; r < dst.rows(); r++){
                addMem(&dst(r, 0), dst.cols(), &mat0(r, 0), &mat1(0, 0));
            }
            return;
        }
        if (mat0.rows() == mat1.rows() && mat0.cols() > 0 && mat1.cols() == 1){
            dst.resize(mat0.dsize);
            for (int r = 0; r < dst.rows(); r++){
                addElm(&dst(r, 0), dst.cols(), &mat0(r, 0), mat1(r, 0));
            }
            return;
        }
    }

    SP_CPUFUNC void subMat(Mat &dst, const Mat &mat0, const Mat &mat1){
        if (mat0.rows() == mat1.rows() && mat0.cols() == mat1.cols()){
            dst.resize(mat0.dsize);
            subMem(dst.ptr, dst.size(), mat0.ptr, mat1.ptr);
            return;
        }
        if (mat0.rows() > 0 && mat1.rows() == 1 && mat0.cols() == mat1.cols()){
            dst.resize(mat0.dsize);
            for (int r = 0; r < dst.rows(); r++){
                subMem(&dst(r, 0), dst.cols(), &mat0(r, 0), &mat1(0, 0));
            }
            return;
        }
        if (mat0.rows() == mat1.rows() && mat0.cols() > 0 && mat1.cols() == 1){
            dst.resize(mat0.dsize);
            for (int r = 0; r < dst.rows(); r++){
                subElm(&dst(r, 0), dst.cols(), &mat0(r, 0), mat1(r, 0));
            }
            return;
        }
    }

    //--------------------------------------------------------------------------------
    // matrix operator
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC Mat operator * (const Mat &mat0, const Mat &mat1){
        Mat dst;
        mulMat(dst, mat0, mat1);
        return dst;
    }

    SP_CPUFUNC Mat operator + (const Mat &mat0, const Mat &mat1){
        Mat dst;
        addMat(dst, mat0, mat1);
        return dst;
    }

    SP_CPUFUNC Mat operator - (const Mat &mat0, const Mat &mat1){
        Mat dst;
        subMat(dst, mat0, mat1);
        return dst;
    }

    SP_CPUFUNC void operator += (Mat &dst, const Mat &mat0){
        addMat(dst, dst, mat0);
    }

    SP_CPUFUNC void operator -= (Mat &dst, const Mat &mat0){
        subMat(dst, dst, mat0);
    }

    SP_CPUFUNC Mat operator + (const Mat &mat0, const double val){
        Mat dst;
        addElm(dst, mat0, val);
        return dst;
    }

    SP_CPUFUNC Mat operator - (const Mat &mat0, const double val){
        Mat dst;
        subElm(dst, mat0, val);
        return dst;
    }

    SP_CPUFUNC Mat operator * (const Mat &mat0, const double val){
        Mat dst;
        mulElm(dst, mat0, val);
        return dst;
    }

    SP_CPUFUNC Mat operator / (const Mat &mat0, const double val){
        Mat dst;
        divElm(dst, mat0, val);
        return dst;
    }

    SP_CPUFUNC void operator += (Mat &dst, const double val){
        addElm(dst, dst, val);
    }

    SP_CPUFUNC void operator -= (Mat &dst, const double val){
        subElm(dst, dst, val);
    }

    SP_CPUFUNC void operator *= (Mat &dst, const double val){
        mulElm(dst, dst, val);
    }

    SP_CPUFUNC void operator /= (Mat &dst, const double val){
        divElm(dst, dst, val);
    }


    //--------------------------------------------------------------------------------
    // vector operator
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Vec2 operator * (const Mat &mat, const Vec2 vec){
        return mulMat(mat.ptr, mat.rows(), mat.cols(), vec);
    }

    SP_CPUFUNC Vec3 operator * (const Mat &mat, const Vec3 vec){
        return mulMat(mat.ptr, mat.rows(), mat.cols(), vec);
    }

    SP_CPUFUNC VecPN2 operator * (const Mat &mat, const VecPN2 vec){
        return mulMat(mat.ptr, mat.rows(), mat.cols(), vec);
    }

    SP_CPUFUNC VecPN3 operator * (const Mat &mat, const VecPN3 vec){
        return mulMat(mat.ptr, mat.rows(), mat.cols(), vec);
    }

    SP_CPUFUNC Mesh3 operator * (const Mat &mat, const Mesh3 mesh){
        return mulMat(mat.ptr, mat.rows(), mat.cols(), mesh);
    }


    //--------------------------------------------------------------------------------
    // vector util
    //--------------------------------------------------------------------------------
    
    template<typename TYPE>
    SP_CPUFUNC Vec2 getVec(const MemA<TYPE, 2> &vec) {
        return getVec(vec[0], vec[1]);
    }

    template<typename TYPE>
    SP_CPUFUNC Vec3 getVec(const MemA<TYPE, 3> &vec) {
        return getVec(vec[0], vec[1], vec[2]);
    }

    SP_CPUFUNC Mem<Vec3> extVec(const Mem<Vec2> &vec, const double z) {
        Mem<Vec3> dst(vec.dim, vec.dsize);
        for (int i = 0; i < dst.size(); i++) {
            dst[i] = extVec(vec[i], z);
        }
        return dst;
    }

    SP_CPUFUNC Mem2<Vec2> grid(const int dsize0, const int dsize1) {
        Mem2<Vec2> map(dsize0, dsize1);

        for (int y = 0; y < dsize1; y++) {
            for (int x = 0; x < dsize0; x++) {
                map(x, y) = getVec(x, y);
            }
        }

        return map;
    }

}

#endif
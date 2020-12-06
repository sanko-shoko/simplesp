//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MOP_H__
#define __SP_MOP_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spmath.h"
#include "spcore/spgen/sptype.h"

#include "spcore/spcpu/spmem.h"

#include <stdlib.h>

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
        cnvMem(dst.ptr, dst.size(), mem0.ptr, scale, base);
    }

    template<typename TYPE, typename ELEM>
    SP_CPUFUNC void setElm(Mem<TYPE> &dst, const ELEM &elm) {
        setElm(dst.ptr, dst.size(), elm);
    }

    //--------------------------------------------------------------------------------
    // mem operator
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void addMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1){
        SP_ASSERT(mem0.dim == mem1.dim && cmp(mem0.dsize, mem1.dsize, mem0.dim));
        dst.resize(mem0.dim, mem0.dsize);
        addMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
    }
    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void subMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1) {
        SP_ASSERT(mem0.dim == mem1.dim && cmp(mem0.dsize, mem1.dsize, mem0.dim));
        dst.resize(mem0.dim, mem0.dsize);
        subMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
    }
    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void mulMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1) {
        SP_ASSERT(mem0.dim == mem1.dim && cmp(mem0.dsize, mem1.dsize, mem0.dim));
        dst.resize(mem0.dim, mem0.dsize);
        mulMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
    }
    template<typename TYPE, typename TYPE0, typename TYPE1>
    SP_CPUFUNC void divMem(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const Mem<TYPE1> &mem1) {
        SP_ASSERT(mem0.dim == mem1.dim && cmp(mem0.dsize, mem1.dsize, mem0.dim));
        dst.resize(mem0.dim, mem0.dsize);
        divMem(dst.ptr, dst.size(), mem0.ptr, mem1.ptr);
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
    SP_CPUFUNC void mulElm(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const ELEM &elm) {
        dst.resize(mem0.dim, mem0.dsize);
        mulElm(dst.ptr, dst.size(), mem0.ptr, elm);
    }
    template<typename TYPE, typename TYPE0, typename ELEM>
    SP_CPUFUNC void divElm(Mem<TYPE> &dst, const Mem<TYPE0> &mem0, const ELEM &elm) {
        dst.resize(mem0.dim, mem0.dsize);
        divElm(dst.ptr, dst.size(), mem0.ptr, elm);
    }

    //--------------------------------------------------------------------------------
    // mem1 operator
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem1<TYPE> operator + (const Mem1<TYPE> &mem0, const ELEM &elm) { Mem1<TYPE> dst; addElm(dst, mem0, elm); return dst; }
    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem1<TYPE> operator - (const Mem1<TYPE> &mem0, const ELEM &elm) { Mem1<TYPE> dst; subElm(dst, mem0, elm); return dst; }
    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem1<TYPE> operator * (const Mem1<TYPE> &mem0, const ELEM &elm) { Mem1<TYPE> dst; mulElm(dst, mem0, elm); return dst; }
    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem1<TYPE> operator / (const Mem1<TYPE> &mem0, const ELEM &elm) { Mem1<TYPE> dst; divElm(dst, mem0, elm); return dst; }

    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator += (Mem1<TYPE> &dst, const ELEM &elm) { addElm(dst, dst, elm); }
    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator -= (Mem1<TYPE> &dst, const ELEM &elm) { subElm(dst, dst, elm); }
    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator *= (Mem1<TYPE> &dst, const ELEM &elm) { mulElm(dst, dst, elm); }
    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator /= (Mem1<TYPE> &dst, const ELEM &elm) { divElm(dst, dst, elm); }

    //--------------------------------------------------------------------------------
    // mem2 operator
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem2<TYPE> operator + (const Mem2<TYPE> &mem0, const ELEM &elm) { Mem2<TYPE> dst; addElm(dst, mem0, elm); return dst; }
    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem2<TYPE> operator - (const Mem2<TYPE> &mem0, const ELEM &elm) { Mem2<TYPE> dst; subElm(dst, mem0, elm); return dst; }
    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem2<TYPE> operator * (const Mem2<TYPE> &mem0, const ELEM &elm) { Mem2<TYPE> dst; mulElm(dst, mem0, elm); return dst; }
    template<typename TYPE, typename ELEM> SP_CPUFUNC Mem2<TYPE> operator / (const Mem2<TYPE> &mem0, const ELEM &elm) { Mem2<TYPE> dst; divElm(dst, mem0, elm); return dst; }

    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator += (Mem2<TYPE> &dst, const ELEM &elm) { addElm(dst, dst, elm); }
    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator -= (Mem2<TYPE> &dst, const ELEM &elm) { subElm(dst, dst, elm); }
    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator *= (Mem2<TYPE> &dst, const ELEM &elm) { mulElm(dst, dst, elm); }
    template<typename TYPE, typename ELEM> SP_CPUFUNC void operator /= (Mem2<TYPE> &dst, const ELEM &elm) { divElm(dst, dst, elm); }

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

    SP_CPUFUNC Mat getMat(const Pose &pose, const int rows = 3, const int cols = 4) {
        Mat dst(rows, cols);

        getMat(dst.ptr, dst.rows(), dst.cols(), pose);
        return dst;
    }

    SP_CPUFUNC Mat getMat(const CamParam &cam) {
        Mat dst(3, 3);

        getMat(dst.ptr, dst.rows(), dst.cols(), cam);
        return dst;
    }

    SP_CPUFUNC Mat getMatAngleX(const SP_REAL angle) {
        Mat dst(3, 3);

        getMatAngleX(dst.ptr, dst.rows(), dst.cols(), angle);
        return dst;
    }

    SP_CPUFUNC Mat getMatAngleY(const SP_REAL angle) {
        Mat dst(3, 3);

        getMatAngleY(dst.ptr, dst.rows(), dst.cols(), angle);
        return dst;
    }

    SP_CPUFUNC Mat getMatAngleZ(const SP_REAL angle) {
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

    SP_GENFUNC Mat getMatRodrigues(const Vec3 &vec) {
        Mat dst(3, 3);

        getMatRodrigues(dst.ptr, dst.rows(), dst.cols(), vec);
        return dst;
    }

    SP_GENFUNC Mat getMatRodrigues(const Vec3 &vec, const double angle) {
        Mat dst(3, 3);

        getMatRodrigues(dst.ptr, dst.rows(), dst.cols(), vec, angle);
        return dst;
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

    SP_CPUFUNC SP_REAL normMat(const Mat &mat){
        return normMat(mat.ptr, mat.rows(), mat.cols());
    }

    SP_CPUFUNC SP_REAL detMat(const Mat &mat){
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
        if (svdMat(eigVec.ptr, eigVal.ptr, V.ptr, mat.ptr, mat.rows(), mat.cols(), minOrder) == false) return false;

        //if (eigMat(eigVec.ptr, eigVal.ptr, mat.ptr, mat.rows(), mat.cols(), minOrder) == false) return false;

        return true;
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

        if (svdMat(U.ptr, S.ptr, V.ptr, m.ptr, m.rows(), m.cols(), minOrder) == false) return false;

        return true;
    }

    SP_GENFUNC int eqn(Mem1<Cmp> &xs, const Mem1<SP_REAL> cs, const int maxit = 20, const SP_REAL eps = 1.0e-10) {
        Mem1<Cmp> tmp(cs.size() - 1);
        const int n = eqn(tmp.ptr, cs.size(), cs.ptr, maxit, eps);

        xs.clear();
        for (int i = 0; i < n; i++) {
            xs.push(tmp[i]);
        }
        return n;
    }

    //--------------------------------------------------------------------------------
    // calc matrix
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC void mulMat(Mat &dst, const Mat &mat0, const Mat &mat1){
        if (mat0.cols() == mat1.rows() && mat0.rows() > 0 && mat1.cols() > 0){
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
        if (mat0.cols() == mat1.cols() && mat0.rows() > 0 && mat1.rows() == 1){
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
        if (mat0.cols() == mat1.cols() && mat0.rows() > 0 && mat1.rows() == 1){
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
    // vector util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mem1<Vec3> getVec3(const Mem1<Vec2> &vec, const double z) {
        Mem<Vec3> dst(vec.dim, vec.dsize);
        for (int i = 0; i < dst.size(); i++) {
            dst[i] = getVec3(vec[i].x, vec[i].y, z);
        }
        return dst;
    }

    SP_CPUFUNC Mem2<Vec2> grid(const int dsize0, const int dsize1) {
        Mem2<Vec2> map(dsize0, dsize1);

        for (int y = 0; y < dsize1; y++) {
            for (int x = 0; x < dsize0; x++) {
                map(x, y) = getVec2(x, y);
            }
        }

        return map;
    }


    //--------------------------------------------------------------------------------
    // rect
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rect2 getRect2(const Mem1<Vec2> &vecs) {
        Rect2 rect = getRect2(vecs[0]);
        for (int i = 1; i < vecs.size(); i++) {
            rect = orRect(rect, getRect2(vecs[i]));
        }
        return rect;
    }

    SP_GENFUNC Rect3 getRect3(const Mem1<Vec3> &vecs) {
        Rect3 rect = getRect3(vecs[0]);
        for (int i = 1; i < vecs.size(); i++) {
            rect = orRect(rect, getRect3(vecs[i]));
        }
        return rect;
    }


    //--------------------------------------------------------------------------------
    // transform
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rot getRot(const Mat &mat) {
        return getRot(mat.ptr, mat.rows(), mat.cols());
    }

    SP_GENFUNC Pose getPose(const Mat &mat) {
        return getPose(mat.ptr, mat.rows(), mat.cols());
    }

    //--------------------------------------------------------------------------------
    // camera
    //--------------------------------------------------------------------------------

    SP_GENFUNC Mem1<Vec2> mulCam(const CamParam cam, const Mem1<Vec2> &npxs) {
        Mem1<Vec2> pixs(npxs.size());
        for (int i = 0; i < pixs.size(); i++) {
            pixs[i] = mulCam(cam, npxs[i]);
        }
        return pixs;
    }

    SP_GENFUNC Mem1<Vec2> invCam(const CamParam cam, const Mem1<Vec2> &pixs) {
        Mem1<Vec2> npxs(pixs.size());
        for (int i = 0; i < npxs.size(); i++) {
            npxs[i] = invCam(cam, pixs[i]);
        }
        return npxs;
    }

    SP_GENFUNC Mem1<Vec2> pixDist(const CamParam cam, const Mem1<Vec2> &pixs) {
        Mem1<Vec2> dpixs(pixs.size());
        for (int i = 0; i < dpixs.size(); i++) {
            dpixs[i] = pixDist(cam, pixs[i]);
        }
        return dpixs;
    }

    SP_GENFUNC Mem1<Vec2> pixUndist(const CamParam cam, const Mem1<Vec2> &pixs) {
        Mem1<Vec2> udpixs(pixs.size());
        for (int i = 0; i < udpixs.size(); i++) {
            udpixs[i] = pixUndist(cam, pixs[i]);
        }
        return udpixs;
    }

    SP_GENFUNC Mem1<Vec2> npxDist(const CamParam cam, const Mem1<Vec2> &npxs) {
        Mem1<Vec2> dnpxs(npxs.size());
        for (int i = 0; i < dnpxs.size(); i++) {
            dnpxs[i] = pixDist(cam, npxs[i]);
        }
        return dnpxs;
    }

    SP_GENFUNC Mem1<Vec2> npxUndist(const CamParam cam, const Mem1<Vec2> &npxs) {
        Mem1<Vec2> udnpxs(npxs.size());
        for (int i = 0; i < udnpxs.size(); i++) {
            udnpxs[i] = pixUndist(cam, npxs[i]);
        }
        return udnpxs;
    }

    SP_GENFUNC Mem1<Vec2> mulCamD(const CamParam cam, const Mem1<Vec2> &npxs) {
        Mem1<Vec2> pixs(npxs.size());
        for (int i = 0; i < pixs.size(); i++) {
            pixs[i] = mulCamD(cam, npxs[i]);
        }
        return pixs;
    }

    SP_GENFUNC Mem1<Vec2> invCamD(const CamParam cam, const Mem1<Vec2> &pixs) {
        Mem1<Vec2> npxs(pixs.size());
        for (int i = 0; i < npxs.size(); i++) {
            npxs[i] = invCamD(cam, pixs[i]);
        }
        return npxs;
    }


    //--------------------------------------------------------------------------------
    // matrix operator
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat operator + (const Mat &mat0, const Mat &mat1) { Mat dst; addMat(dst, mat0, mat1); return dst; }
    SP_CPUFUNC Mat operator - (const Mat &mat0, const Mat &mat1) { Mat dst; subMat(dst, mat0, mat1); return dst; }
    SP_CPUFUNC Mat operator * (const Mat &mat0, const Mat &mat1) { Mat dst; mulMat(dst, mat0, mat1); return dst; }

    SP_CPUFUNC void operator += (Mat &dst, const Mat &mat0) { addMat(dst, dst, mat0); }
    SP_CPUFUNC void operator -= (Mat &dst, const Mat &mat0) { subMat(dst, dst, mat0); }

    SP_CPUFUNC Mat operator + (const Mat &mat0, const double val) { Mat dst; addElm(dst, mat0, val); return dst; }
    SP_CPUFUNC Mat operator - (const Mat &mat0, const double val) { Mat dst; subElm(dst, mat0, val); return dst; }
    SP_CPUFUNC Mat operator * (const Mat &mat0, const double val) { Mat dst; mulElm(dst, mat0, val); return dst; }
    SP_CPUFUNC Mat operator / (const Mat &mat0, const double val) { Mat dst; divElm(dst, mat0, val); return dst; }

    SP_CPUFUNC void operator += (Mat &dst, const double val) { addElm(dst, dst, val); }
    SP_CPUFUNC void operator -= (Mat &dst, const double val) { subElm(dst, dst, val); }
    SP_CPUFUNC void operator *= (Mat &dst, const double val) { mulElm(dst, dst, val); }
    SP_CPUFUNC void operator /= (Mat &dst, const double val) { divElm(dst, dst, val); }

    //--------------------------------------------------------------------------------
    // matrix - pose operator
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat operator * (const Mat &mat, const Pose &pose) {
        SP_ASSERT(mat.cols() == 4 && (mat.rows() == 3 || mat.rows() == 4));

        Mat dst = mat * getMat(pose, mat.rows(), mat.cols());
        return dst;
    }

    SP_CPUFUNC Mat operator * (const Pose &pose, const Mat &mat) {
        SP_ASSERT(mat.cols() == 4 && (mat.rows() == 3 || mat.rows() == 4));

        Mat dst = getMat(pose, mat.rows(), mat.cols()) * mat;
        return dst;
    }

    //--------------------------------------------------------------------------------
    // matrix - vector operator
    //--------------------------------------------------------------------------------

    template<typename VEC>
    SP_CPUFUNC Mem1<VEC> _matvec(const Mat &mat, const Mem1<VEC> &vecs) {
        Mem1<VEC> dst(vecs.size());
        for (int i = 0; i < dst.size(); i++) {
            dst[i] = mat * vecs[i];
        }
        return dst;
    }

    SP_CPUFUNC Vec2 operator * (const Mat &mat, const Vec2 vec) { return mulMat(mat.ptr, mat.rows(), mat.cols(), vec); }
    SP_CPUFUNC Vec3 operator * (const Mat &mat, const Vec3 vec) { return mulMat(mat.ptr, mat.rows(), mat.cols(), vec); }
    SP_CPUFUNC VecPD2 operator * (const Mat &mat, const VecPD2 vec) { return mulMat(mat.ptr, mat.rows(), mat.cols(), vec); }
    SP_CPUFUNC VecPD3 operator * (const Mat &mat, const VecPD3 vec) { return mulMat(mat.ptr, mat.rows(), mat.cols(), vec); }
    SP_CPUFUNC Line3 operator * (const Mat &mat, const Line3 line) { return mulMat(mat.ptr, mat.rows(), mat.cols(), line); }
    SP_CPUFUNC Mesh3 operator * (const Mat &mat, const Mesh3 mesh) { return mulMat(mat.ptr, mat.rows(), mat.cols(), mesh); }

    SP_CPUFUNC Mem1<Vec2> operator * (const Mat &mat, const Mem1<Vec2> &vecs) { return _matvec(mat, vecs); }
    SP_CPUFUNC Mem1<Vec3> operator * (const Mat &mat, const Mem1<Vec3> &vecs) { return _matvec(mat, vecs); }
    SP_CPUFUNC Mem1<VecPD2> operator * (const Mat &mat, const Mem1<VecPD2> &vecs) { return _matvec(mat, vecs); }
    SP_CPUFUNC Mem1<VecPD3> operator * (const Mat &mat, const Mem1<VecPD3> &vecs) { return _matvec(mat, vecs); }
    SP_CPUFUNC Mem1<Line3> operator * (const Mat &mat, const Mem1<Line3> &lines) { return _matvec(mat, lines); }
    SP_CPUFUNC Mem1<Mesh3> operator * (const Mat &mat, const Mem1<Mesh3> &meshes) { return _matvec(mat, meshes); }

    SP_CPUFUNC Mem1<Vec2> operator * (const Pose &pose, const Mem1<Vec2> &vecs) { return getMat(pose) * vecs; }
    SP_CPUFUNC Mem1<Vec3> operator * (const Pose &pose, const Mem1<Vec3> &vecs) { return getMat(pose) * vecs; }
    SP_CPUFUNC Mem1<VecPD2> operator * (const Pose &pose, const Mem1<VecPD2> &vecs) { return getMat(pose) * vecs; }
    SP_CPUFUNC Mem1<VecPD3> operator * (const Pose &pose, const Mem1<VecPD3> &vecs) { return getMat(pose) * vecs; }
    SP_CPUFUNC Mem1<Line3> operator * (const Pose &pose, const Mem1<Line3> &lines) { return getMat(pose) * lines; }
    SP_CPUFUNC Mem1<Mesh3> operator * (const Pose &pose, const Mem1<Mesh3> &meshes) { return getMat(pose) * meshes; }


    //--------------------------------------------------------------------------------
    // shuffle
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mem1<int> shuffle(const int size, const int seed = 0) {

        Mem1<int> index(size);
        for (int i = 0; i < index.size(); i++) {
            index[i] = i;
        }

        srand(seed);
        for (int i = 0; i < index.size(); i++) {
            const int p = rand() % index.size();
            swap(index[i], index[p]);
        }
        return index;
    }

    template<typename TYPE>
    SP_CPUFUNC Mem1<TYPE> shuffle(const Mem<TYPE> &src, const Mem1<int> &index) {
        Mem1<TYPE> ret(src.size());
        for (int i = 0; i < ret.size(); i++) {
            ret[i] = src[index[i]];
        }
        return ret;
    }

    template<typename TYPE>
    SP_CPUFUNC Mem1<TYPE> shuffle(const Mem<TYPE> &src, const int seed = 0) {
        const Mem1<int> index = shuffle(src.size(), seed);
        return shuffle(src, index);
    }

    //--------------------------------------------------------------------------------
    // sort
    //--------------------------------------------------------------------------------

    SP_GENFUNC void qsort(void *base, const int nsize, const int esize, int compare(const void *a, const void *b)) {
        ::qsort(base, nsize, esize, compare);
    }

    template<typename TYPE>
    SP_CPUFUNC int compare_min(const void *a, const void *b) {
        return (*static_cast<const TYPE*>(a) > *static_cast<const TYPE*>(b)) ? +1 : -1;
    }

    template<typename TYPE>
    SP_CPUFUNC int compare_max(const void *a, const void *b) {
        return (*static_cast<const TYPE*>(a) > *static_cast<const TYPE*>(b)) ? -1 : +1;
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(TYPE *mem, const int size, const bool minorder = true) {
        qsort(mem, size, sizeof(TYPE), (minorder) ? compare_min<TYPE> : compare_max<TYPE>);
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(TYPE *mem, const int size, int compare(const void *a, const void *b)) {
        qsort(mem, size, sizeof(TYPE), compare);
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(Mem<TYPE> &mem, const bool minorder = true) {
        sort(mem.ptr, mem.size(), minorder);
    }

    template<typename TYPE>
    SP_CPUFUNC void sort(Mem<TYPE> &mem, int compare(const void *a, const void *b)) {
        sort(mem.ptr, mem.size(), compare);
    }

    //--------------------------------------------------------------------------------
    // max / min
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC TYPE max(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);

        TYPE maxv = mem[0];
        for (int i = 1; i < mem.size(); i++) {
            maxv = max(maxv, mem[i]);
        }
        return maxv;
    }

    template<typename TYPE>
    SP_CPUFUNC TYPE min(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);

        TYPE minv = mem[0];
        for (int i = 0; i < mem.size(); i++) {
            minv = min(minv, mem[i]);
        }
        return minv;
    }

    template<typename TYPE>
    SP_CPUFUNC int maxarg(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);

        int ret = 0;
        double maxv = mem[0];
        for (int i = 1; i < mem.size(); i++) {
            if (mem[i] > maxv) {
                maxv = mem[i];
                ret = i;
            }
        }
        return ret;
    }

    template<typename TYPE>
    SP_CPUFUNC int minarg(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);

        int ret = 0;
        double minv = mem[0];
        for (int i = 1; i < mem.size(); i++) {
            if (mem[i] < minv) {
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
    SP_CPUFUNC int count(const Mem<TYPE> &mem, const TYPE &val) {
        int cnt = 0;
        for (int i = 0; i < mem.size(); i++) {
            if (mem[i] == val) cnt++;
        }
        return cnt;
    }

    //--------------------------------------------------------------------------------
    // filter
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC Mem1<TYPE> filter(const Mem<TYPE> &src, const Mem<bool> &mask, const bool flag = true) {
        Mem1<TYPE> ret;
        ret.reserve(src.size());
        for (int i = 0; i < src.size(); i++) {
            if (mask[i] == flag) {
                ret.push(src[i]);
            }
        }
        return ret;
    }

    template<typename TYPE>
    SP_CPUFUNC Mem1<TYPE> filter(const Mem<TYPE> &src, const Mem<SP_REAL> &errs, const double thresh = 5.0) {
        Mem1<TYPE> ret;
        ret.reserve(src.size());
        for (int i = 0; i < src.size(); i++) {
            if (errs[i] < thresh) {
                ret.push(src[i]);
            }
        }
        return ret;
    }

    //--------------------------------------------------------------------------------
    // sum & mean
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC SP_REAL _sumval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        SP_REAL ret = 0.0;
        for (int i = 0; i < mem.size(); i++) {
            ret += mem[i];
        }
        return ret;
    }
    template<typename TYPE>
    SP_CPUFUNC SP_REAL _meanval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        return _sumval(mem) / mem.size();
    }

    SP_CPUFUNC SP_REAL sum(const Mem<s08> &mem) { return _sumval(mem); }
    SP_CPUFUNC SP_REAL sum(const Mem<u08> &mem) { return _sumval(mem); }
    SP_CPUFUNC SP_REAL sum(const Mem<s16> &mem) { return _sumval(mem); }
    SP_CPUFUNC SP_REAL sum(const Mem<u16> &mem) { return _sumval(mem); }
    SP_CPUFUNC SP_REAL sum(const Mem<s32> &mem) { return _sumval(mem); }
    SP_CPUFUNC SP_REAL sum(const Mem<u32> &mem) { return _sumval(mem); }
    SP_CPUFUNC SP_REAL sum(const Mem<f32> &mem) { return _sumval(mem); }
    SP_CPUFUNC SP_REAL sum(const Mem<f64> &mem) { return _sumval(mem); }

    SP_CPUFUNC SP_REAL mean(const Mem<s08> &mem) { return _meanval(mem); }
    SP_CPUFUNC SP_REAL mean(const Mem<u08> &mem) { return _meanval(mem); }
    SP_CPUFUNC SP_REAL mean(const Mem<s16> &mem) { return _meanval(mem); }
    SP_CPUFUNC SP_REAL mean(const Mem<u16> &mem) { return _meanval(mem); }
    SP_CPUFUNC SP_REAL mean(const Mem<s32> &mem) { return _meanval(mem); }
    SP_CPUFUNC SP_REAL mean(const Mem<u32> &mem) { return _meanval(mem); }
    SP_CPUFUNC SP_REAL mean(const Mem<f32> &mem) { return _meanval(mem); }
    SP_CPUFUNC SP_REAL mean(const Mem<f64> &mem) { return _meanval(mem); }

    //--------------------------------------------------------------------------------
    // sq sum & mean
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC SP_REAL _sqsumval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        SP_REAL ret = 0.0;
        for (int i = 0; i < mem.size(); i++) {
            ret += sq(mem[i]);
        }
        return ret;
    }
    template<typename TYPE>
    SP_CPUFUNC SP_REAL _sqmeanval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        return _sqsumval(mem) / mem.size();
    }

    SP_CPUFUNC SP_REAL sqsum(const Mem<s08> &mem) { return _sqsumval(mem); }
    SP_CPUFUNC SP_REAL sqsum(const Mem<u08> &mem) { return _sqsumval(mem); }
    SP_CPUFUNC SP_REAL sqsum(const Mem<s16> &mem) { return _sqsumval(mem); }
    SP_CPUFUNC SP_REAL sqsum(const Mem<u16> &mem) { return _sqsumval(mem); }
    SP_CPUFUNC SP_REAL sqsum(const Mem<s32> &mem) { return _sqsumval(mem); }
    SP_CPUFUNC SP_REAL sqsum(const Mem<u32> &mem) { return _sqsumval(mem); }
    SP_CPUFUNC SP_REAL sqsum(const Mem<f32> &mem) { return _sqsumval(mem); }
    SP_CPUFUNC SP_REAL sqsum(const Mem<f64> &mem) { return _sqsumval(mem); }

    SP_CPUFUNC SP_REAL sqmean(const Mem<s08> &mem) { return _sqmeanval(mem); }
    SP_CPUFUNC SP_REAL sqmean(const Mem<u08> &mem) { return _sqmeanval(mem); }
    SP_CPUFUNC SP_REAL sqmean(const Mem<s16> &mem) { return _sqmeanval(mem); }
    SP_CPUFUNC SP_REAL sqmean(const Mem<u16> &mem) { return _sqmeanval(mem); }
    SP_CPUFUNC SP_REAL sqmean(const Mem<s32> &mem) { return _sqmeanval(mem); }
    SP_CPUFUNC SP_REAL sqmean(const Mem<u32> &mem) { return _sqmeanval(mem); }
    SP_CPUFUNC SP_REAL sqmean(const Mem<f32> &mem) { return _sqmeanval(mem); }
    SP_CPUFUNC SP_REAL sqmean(const Mem<f64> &mem) { return _sqmeanval(mem); }

    //--------------------------------------------------------------------------------
    // sqrt sum & mean
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC SP_REAL _sqrtsumval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        SP_REAL ret = 0.0;
        for (int i = 0; i < mem.size(); i++) {
            ret += sqrt(mem[i]);
        }
        return ret;
    }
    template<typename TYPE>
    SP_CPUFUNC SP_REAL _sqrtmeanval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        return _sqrtsumval(mem) / mem.size();
    }

    SP_CPUFUNC SP_REAL sqrtsum(const Mem<s08> &mem) { return _sqrtsumval(mem); }
    SP_CPUFUNC SP_REAL sqrtsum(const Mem<u08> &mem) { return _sqrtsumval(mem); }
    SP_CPUFUNC SP_REAL sqrtsum(const Mem<s16> &mem) { return _sqrtsumval(mem); }
    SP_CPUFUNC SP_REAL sqrtsum(const Mem<u16> &mem) { return _sqrtsumval(mem); }
    SP_CPUFUNC SP_REAL sqrtsum(const Mem<s32> &mem) { return _sqrtsumval(mem); }
    SP_CPUFUNC SP_REAL sqrtsum(const Mem<u32> &mem) { return _sqrtsumval(mem); }
    SP_CPUFUNC SP_REAL sqrtsum(const Mem<f32> &mem) { return _sqrtsumval(mem); }
    SP_CPUFUNC SP_REAL sqrtsum(const Mem<f64> &mem) { return _sqrtsumval(mem); }

    SP_CPUFUNC SP_REAL sqrtmean(const Mem<s08> &mem) { return _sqrtmeanval(mem); }
    SP_CPUFUNC SP_REAL sqrtmean(const Mem<u08> &mem) { return _sqrtmeanval(mem); }
    SP_CPUFUNC SP_REAL sqrtmean(const Mem<s16> &mem) { return _sqrtmeanval(mem); }
    SP_CPUFUNC SP_REAL sqrtmean(const Mem<u16> &mem) { return _sqrtmeanval(mem); }
    SP_CPUFUNC SP_REAL sqrtmean(const Mem<s32> &mem) { return _sqrtmeanval(mem); }
    SP_CPUFUNC SP_REAL sqrtmean(const Mem<u32> &mem) { return _sqrtmeanval(mem); }
    SP_CPUFUNC SP_REAL sqrtmean(const Mem<f32> &mem) { return _sqrtmeanval(mem); }
    SP_CPUFUNC SP_REAL sqrtmean(const Mem<f64> &mem) { return _sqrtmeanval(mem); }

    //--------------------------------------------------------------------------------
    // abs sum & mean
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC SP_REAL _abssumval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        SP_REAL ret = 0.0;
        for (int i = 0; i < mem.size(); i++) {
            ret += sqrt(mem[i]);
        }
        return ret;
    }
    template<typename TYPE>
    SP_CPUFUNC SP_REAL _absmeanval(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        return _abssumval(mem) / mem.size();
    }

    SP_CPUFUNC SP_REAL abssum(const Mem<s08> &mem) { return _abssumval(mem); }
    SP_CPUFUNC SP_REAL abssum(const Mem<u08> &mem) { return _abssumval(mem); }
    SP_CPUFUNC SP_REAL abssum(const Mem<s16> &mem) { return _abssumval(mem); }
    SP_CPUFUNC SP_REAL abssum(const Mem<u16> &mem) { return _abssumval(mem); }
    SP_CPUFUNC SP_REAL abssum(const Mem<s32> &mem) { return _abssumval(mem); }
    SP_CPUFUNC SP_REAL abssum(const Mem<u32> &mem) { return _abssumval(mem); }
    SP_CPUFUNC SP_REAL abssum(const Mem<f32> &mem) { return _abssumval(mem); }
    SP_CPUFUNC SP_REAL abssum(const Mem<f64> &mem) { return _abssumval(mem); }

    SP_CPUFUNC SP_REAL absmean(const Mem<s08> &mem) { return _absmeanval(mem); }
    SP_CPUFUNC SP_REAL absmean(const Mem<u08> &mem) { return _absmeanval(mem); }
    SP_CPUFUNC SP_REAL absmean(const Mem<s16> &mem) { return _absmeanval(mem); }
    SP_CPUFUNC SP_REAL absmean(const Mem<u16> &mem) { return _absmeanval(mem); }
    SP_CPUFUNC SP_REAL absmean(const Mem<s32> &mem) { return _absmeanval(mem); }
    SP_CPUFUNC SP_REAL absmean(const Mem<u32> &mem) { return _absmeanval(mem); }
    SP_CPUFUNC SP_REAL absmean(const Mem<f32> &mem) { return _absmeanval(mem); }
    SP_CPUFUNC SP_REAL absmean(const Mem<f64> &mem) { return _absmeanval(mem); }

    //--------------------------------------------------------------------------------
    // median
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC TYPE median(const Mem<TYPE> &mem) {
        SP_ASSERT(mem.size() != 0);
        Mem<TYPE> tmp = mem;
        sort(tmp);
        return tmp[tmp.size() / 2];
    }

    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

    template<typename VEC>
    SP_CPUFUNC VEC _sumvec(const Mem<VEC> &vecs) {
        VEC ret;
        memset(&ret, 0, sizeof(VEC));
        for (int i = 0; i < vecs.size(); i++) {
            ret += vecs[i];
        }
        return ret;
    }
    template<typename VEC>
    SP_CPUFUNC VEC _meanvec(const Mem<VEC> &vecs) {
        return _sumvec(vecs) / vecs.size();
    }
    SP_CPUFUNC Vec2 sum(const Mem<Vec2> &mem) { return _sumvec(mem); }
    SP_CPUFUNC Vec3 sum(const Mem<Vec3> &mem) { return _sumvec(mem); }

    SP_CPUFUNC Vec2 mean(const Mem<Vec2> &mem) { return _meanvec(mem); }
    SP_CPUFUNC Vec3 mean(const Mem<Vec3> &mem) { return _meanvec(mem); }

    //--------------------------------------------------------------------------------
    // matrix
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat sum(const Mat &mat, const int axis) {
        SP_ASSERT(axis == 0 || axis == 1);

        Mat ret((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
        ret.zero();

        const SP_REAL *pMat = mat.ptr;

        for (int r = 0; r < mat.rows(); r++) {
            for (int c = 0; c < mat.cols(); c++) {
                ret[(axis == 0) ? c : r] += *pMat++;
            }
        }
        return ret;
    }

    SP_CPUFUNC Mat mean(const Mat &mat, const int axis) {
        return sum(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
    }

    SP_CPUFUNC Mat sqsum(const Mat &mat, const int axis) {
        SP_ASSERT(axis == 0 || axis == 1);

        Mat ret((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
        ret.zero();

        const SP_REAL *pMat = mat.ptr;

        for (int r = 0; r < mat.rows(); r++) {
            for (int c = 0; c < mat.cols(); c++) {
                ret[(axis == 0) ? c : r] += sq(*pMat++);
            }
        }
        return ret;
    }
    SP_CPUFUNC Mat sqmean(const Mat &mat, const int axis) {
        return sqsum(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
    }

    SP_CPUFUNC Mat abssum(const Mat &mat, const int axis) {
        SP_ASSERT(axis == 0 || axis == 1);

        Mat ret((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
        ret.zero();

        const SP_REAL *pMat = mat.ptr;

        for (int r = 0; r < mat.rows(); r++) {
            for (int c = 0; c < mat.cols(); c++) {
                ret[(axis == 0) ? c : r] += fabs(*pMat++);
            }
        }
        return ret;
    }

    SP_CPUFUNC Mat absmean(const Mat &mat, const int axis) {
        return abssum(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
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
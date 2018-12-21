//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_IMAGE_H__
#define __SP_IMAGE_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // rescale 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void rescale(Mem<TYPE> &dst, const Mem<TYPE> &src, const double dscale0, const double dscale1){
        SP_ASSERT(isValid(2, src));

        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const int dsize0 = round(tmp.dsize[0] * dscale0);
        const int dsize1 = round(tmp.dsize[1] * dscale1);

        const int dsize[2] = { dsize0, dsize1 };
        dst.resize(2, dsize);

        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){
                const double su = u / dscale0;
                const double sv = v / dscale1;

                for (int c = 0; c < ch; c++){
                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), acs2<TYPE, ELEM>(tmp, su, sv, c));
                }
            }
        }
    }
    
    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void rescale(Mem<TYPE> &dst, const Mem<TYPE> &src){
        SP_ASSERT(isValid(2, src));

        const double dscale0 = static_cast<double>(dst.dsize[0]) / src.dsize[0];
        const double dscale1 = static_cast<double>(dst.dsize[1]) / src.dsize[1];

        rescale<TYPE, ELEM>(dst, src, dscale0, dscale1);
    }

    SP_CPUFUNC void rescaleFast(Mem<Byte> &dst, const Mem<Byte> &src, const double dscale0, const double dscale1) {
        SP_ASSERT(isValid(2, src));

        const Mem<Byte> &tmp = (&dst != &src) ? src : clone(src);

        const int dsize0 = round(tmp.dsize[0] * dscale0);
        const int dsize1 = round(tmp.dsize[1] * dscale1);

        const int dsize[2] = { dsize0, dsize1 };
        dst.resize(2, dsize);

        //const Byte *pSrc = tmp.ptr;
        Byte *pDst = dst.ptr;

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                const double su = u / dscale0;
                const double sv = v / dscale1;

                cnvVal(*pDst++, acs2<Byte>(tmp, su, sv));
            }
        }
    }

    SP_CPUFUNC void rescaleFast(Mem<Byte> &dst, const Mem<Byte> &src) {
        SP_ASSERT(isValid(2, src));

        const double dscale0 = static_cast<double>(dst.dsize[0]) / src.dsize[0];
        const double dscale1 = static_cast<double>(dst.dsize[1]) / src.dsize[1];

        rescaleFast(dst, src, dscale0, dscale1);
    }


    //--------------------------------------------------------------------------------
    // pyramid down 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void pyrdown(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>*>(&dst) != &src) ? src : clone(src);

        const int sdsize0 = src.dsize[0];
        const int sdsize1 = src.dsize[1];

        const int ddsize0 = (sdsize0 + 1) / 2;
        const int ddsize1 = (sdsize1 + 1) / 2;
        const int ddsize[2] = { ddsize0, ddsize1 };

        dst.resize(2, ddsize);

        const TYPE0 *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < ddsize1; v++) {
            const int sv = 2 * v;

            const int sv0 = sv + ((sv == 0) ? 0 : -1);
            const int sv1 = sv + 0;
            const int sv2 = sv + ((sv == sdsize1 - 1) ? 0 : +1);

            const TYPE0 *psrc0 = &psrc[sv0 * sdsize0];
            const TYPE0 *psrc1 = &psrc[sv1 * sdsize0];
            const TYPE0 *psrc2 = &psrc[sv2 * sdsize0];

            TYPE *pd = &pdst[v * ddsize0];

            for (int u = 0; u < ddsize0; u++) {
                const int su = 2 * u;

                const int su0 = su + ((su == 0) ? 0 : -1);
                const int su1 = su + 0;
                const int su2 = su + ((su == sdsize0 - 1) ? 0 : +1);
                
                const TYPE0 a00 = psrc0[su0];
                const TYPE0 a01 = psrc0[su1];
                const TYPE0 a02 = psrc0[su2];

                const TYPE0 a10 = psrc1[su0];
                const TYPE0 a11 = psrc1[su1];
                const TYPE0 a12 = psrc1[su2];

                const TYPE0 a20 = psrc2[su0];
                const TYPE0 a21 = psrc2[su1];
                const TYPE0 a22 = psrc2[su2];

                const double d = ((a00 + 2.0 * a01 + a02) + 2.0 * (a10 + 2.0 * a11 + a12) + (a20 + 2.0 * a21 + a22)) / 16.0;
                cnvVal(*pd++, d);
            }
        }
    }


    //--------------------------------------------------------------------------------
    // crop 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void crop(Mem<TYPE> &dst, const Mem<TYPE> &src, const Rect &rect, const double angle = 0.0){
        SP_ASSERT(isValid(2, src));

        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);

        dst.resize(2, rect.dsize);

        if (angle == 0.0){
            for (int v = 0; v < rect.dsize[1]; v++){
                for (int u = 0; u < rect.dsize[0]; u++){
                    for (int c = 0; c < ch; c++){
                        cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), acs2<TYPE, ELEM>(tmp, u + rect.dbase[0], v + rect.dbase[1], c));
                    }
                }
            }
        }
        else{
            const double cv = cos(-angle);
            const double sv = sin(-angle);
            const Vec2 cent = getVec((rect.dbase[0] + rect.dsize[0] - 1) * 0.5, (rect.dbase[1] + rect.dsize[1] - 1) * 0.5);
            for (int v = 0; v < rect.dsize[1]; v++){
                for (int u = 0; u < rect.dsize[0]; u++){
                    const double x = u + rect.dbase[0] - cent.x;
                    const double y = v + rect.dbase[1] - cent.y;

                    const double rx = cv * x - sv * y;
                    const double ry = sv * x + cv * y;

                    for (int c = 0; c < ch; c++){
                        cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), acs2<TYPE, ELEM>(tmp, cent.x + rx, cent.y + ry, c));
                    }
                }
            }
        }
    }

    //--------------------------------------------------------------------------------
    // merge 
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC void merge(Mem<TYPE> &dst, const Mem<TYPE> &src0, const Mem<TYPE> &src1, const bool horizon = true){
        SP_ASSERT(isValid(2, src0));
        SP_ASSERT(isValid(2, src1));

        const Mem<TYPE> &tmp0 = (&dst != &src0) ? src0 : clone(src0);
        const Mem<TYPE> &tmp1 = (&dst != &src1) ? src1 : clone(src1);
        
        const int dsize0 = (horizon == true) ? src0.dsize[0] + src1.dsize[0] : maxVal(src0.dsize[0], src1.dsize[0]);
        const int dsize1 = (horizon == true) ? maxVal(src0.dsize[1], src1.dsize[1]) : src0.dsize[1] + src1.dsize[1];
        
        const int dsize[2] = { dsize0, dsize1 };
        dst.resize(2, dsize);
        dst.zero();

        for (int v = 0; v < tmp0.dsize[1]; v++){
            for (int u = 0; u < tmp0.dsize[0]; u++){
                acs2(dst, u, v) = acs2(tmp0, u, v);
            }
        }

        const int offsetX = (horizon == true) ? tmp0.dsize[0] : 0;
        const int offsetY = (horizon == true) ? 0 : tmp0.dsize[1];

        for (int v = 0; v < tmp1.dsize[1]; v++){
            for (int u = 0; u < tmp1.dsize[0]; u++){
                acs2(dst, u + offsetX, v + offsetY) = acs2(tmp1, u, v);
            }
        }
    }

    template <typename TYPE>
    SP_CPUFUNC void blend(Mem<TYPE> &dst, const Mem<TYPE> &src0, const Mem<TYPE> &src1, const double rate = 0.5) {
        SP_ASSERT(isValid(2, src0));
        SP_ASSERT(isValid(2, src1));
        SP_ASSERT(cmpSize(2, src0.dsize, src1.dsize));

        const Mem<TYPE> &tmp0 = (&dst != &src0) ? src0 : clone(src0);
        const Mem<TYPE> &tmp1 = (&dst != &src1) ? src1 : clone(src1);

        dst.resize(2, src0.dsize);
        dst.zero();

        for (int v = 0; v < src0.dsize[1]; v++) {
            for (int u = 0; u < src0.dsize[0]; u++) {
                acs2(dst, u, v) = blendCol(acs2(tmp0, u, v), acs2(tmp1, u, v), rate);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // invert
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void invert(Mem<TYPE> &dst, const Mem<TYPE> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                for (int c = 0; c < ch; c++) {
                    acs2<TYPE, ELEM>(dst, u, v, c) = SP_BYTEMAX - acs2<TYPE, ELEM>(tmp, u, v, c);
                }
            }
        }
    }

    //--------------------------------------------------------------------------------
    // flip 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void flipX(Mem<TYPE> &dst, const Mem<TYPE> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                for (int c = 0; c < ch; c++) {
                    acs2<TYPE, ELEM>(dst, u, v, c) = acs2<TYPE, ELEM>(tmp, (dst.dsize[0] - 1) - u, v, c);
                }
            }
        }
    }

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void flipY(Mem<TYPE> &dst, const Mem<TYPE> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                for (int c = 0; c < ch; c++) {
                    acs2<TYPE, ELEM>(dst, u, v, c) = acs2<TYPE, ELEM>(tmp, u, (dst.dsize[1] - 1) - v, c);
                }
            }
        }
    }


    //--------------------------------------------------------------------------------
    // remap
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void remap(Mem<TYPE> &dst, const Mem<TYPE> &src, const Mem<Vec2> &table, const bool useExt = false){
        SP_ASSERT(isValid(2, src));
        SP_ASSERT(isValid(2, table));
        SP_ASSERT(cmpSize(2, src.dsize, table.dsize));
        
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const Rect rect = getRect2(tmp.dsize);
        
        dst.resize(2, tmp.dsize);
        dst.zero();

        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){
                const Vec2 &vec = acs2(table, u, v);
                if (useExt == false && isInRect2(rect, u + vec.x, v + vec.y) == false) continue;
                
                for (int c = 0; c < ch; c++){
                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), acs2<TYPE, ELEM>(tmp, u + vec.x, v + vec.y, c));
                }
            }
        }

    }

    //--------------------------------------------------------------------------------
    // warp
    //--------------------------------------------------------------------------------

    template<typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void warp(Mem<TYPE> &dst, const Mem<TYPE> &src, const Mat &mat){
        SP_ASSERT(isValid(2, src));

        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        Mat _mat = mat;
        if (mat.rows() == 2 && (mat.cols() == 2 || mat.cols() == 3)) {
            _mat = extMat(3, 3, mat);
        }

        if (mat.rows() != 3 || mat.cols() != 3) return;
        const Mat imat = invMat(mat);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const Rect rect = getRect2(tmp.dsize);

        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){
                const Vec2 vec = imat * getVec(u, v);
                if (isInRect2(rect, vec.x, vec.y) == false) continue;

                for (int c = 0; c < ch; c++){
                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), acs2<TYPE, ELEM>(tmp, vec.x, vec.y, c));
                }
            }
        }
    }


    //--------------------------------------------------------------------------------
    // convert 
    //--------------------------------------------------------------------------------

    template<typename TYPE0, typename TYPE1>
    SP_CPUFUNC void cnvImg(Mem<TYPE0> &dst, const Mem<TYPE1> &src){
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        for (int i = 0; i < dst.size(); i++){
            cnvCol(dst[i], src[i]);
        }
    }

    SP_CPUFUNC void cnvImgToHSV(Mem<Vec3> &dst, const Mem<Col3> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        for (int i = 0; i < dst.size(); i++) {
            cnvColToHSV(dst[i], src[i]);
        }
    }
        
    template <typename TYPE0, typename TYPE1>
    SP_CPUFUNC void cnvDepthToImg(Mem<TYPE0> &dst, const Mem<TYPE1> &src, const double nearPlane = 100.0, const double farPlane = 10000.0){
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++){
            const double depth = extractDepth(src[i]);

            if (depth >= nearPlane && depth <= farPlane){
                cnvDepthToCol(dst[i], depth, nearPlane, farPlane);
            }
        }
    }

    template <typename TYPE>
    SP_CPUFUNC void cnvNormalToImg(Mem<TYPE> &dst, const Mem<VecPN3> &src, const double nearPlane = 100.0, const double farPlane = 10000.0){
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++){
            const double depth = extractDepth(src[i]);

            if (depth >= nearPlane && depth <= farPlane){
                cnvNormalToCol(dst[i], src[i].nrm);
            }
        }
    }

    template <typename TYPE>
    SP_CPUFUNC void cnvDispToImg(Mem<TYPE> &dst, const Mem<Disp> &src, const int maxDisp, const int minDisp) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++) {
            const Disp &disp = src[i];

            if (disp.eval > 0.0) {
                cnvDispToCol(dst[i], disp, maxDisp, minDisp);
            }
        }
    }

    SP_CPUFUNC void cnvLabelToImg(Mem<Col3> &dst, const Mem<int> &src){
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++){
            if (src[i] < 0) continue;

            srand(src[i]);
            cnvHSVToCol(dst[i], getVec((randValUnif() + 1.0) * SP_PI, 1.0, 1.0));
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void cnvPtrToImg(Mem<TYPE> &dst, const void *src, const int dsize0, const int dsize1, const int ch){

        switch (ch) {
        case 1:
        {
            Mem2<Byte> gry(dsize0, dsize1, src);
            cnvImg(dst, gry);
            break;
        }
        case 3:
        {
            Mem2<Col3> col(dsize0, dsize1, src);
            cnvImg(dst, col);
            break;
        }
        case 4:
        {
            Mem2<Col4> col(dsize0, dsize1, src);
            cnvImg(dst, col);
            break;
        }
        default:
            dst.clear();
            break;
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void cnvImgToPtr(void *dst, const Mem<TYPE> &src, const int ch){
        SP_ASSERT(isValid(2, src));

        switch (ch){
        case 1:
        {
            Mem2<Byte> gry;
            cnvImg(gry, src);
            memcpy(dst, gry.ptr, gry.size());
            break;
        }
        case 3:
        {
            Mem2<Col3> col;
            cnvImg(col, src);
            memcpy(dst, col.ptr, col.size() * 3);
            break;
        }
        default:
            dst = NULL;
            break;
        }
    }

}

#endif
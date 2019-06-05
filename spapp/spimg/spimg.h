//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
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
        SP_ASSERT(isValid(src, 2));

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
        SP_ASSERT(isValid(src, 2));

        const SP_REAL dscale0 = static_cast<SP_REAL>(dst.dsize[0]) / src.dsize[0];
        const SP_REAL dscale1 = static_cast<SP_REAL>(dst.dsize[1]) / src.dsize[1];

        rescale<TYPE, ELEM>(dst, src, dscale0, dscale1);
    }

    SP_CPUFUNC void rescaleFast(Mem<Byte> &dst, const Mem<Byte> &src, const double dscale0, const double dscale1) {
        SP_ASSERT(isValid(src, 2));

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
        SP_ASSERT(isValid(src, 2));

        const SP_REAL dscale0 = static_cast<SP_REAL>(dst.dsize[0]) / src.dsize[0];
        const SP_REAL dscale1 = static_cast<SP_REAL>(dst.dsize[1]) / src.dsize[1];

        rescaleFast(dst, src, dscale0, dscale1);
    }


    //--------------------------------------------------------------------------------
    // pyramid down 
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC void pyrdown(Mem<TYPE> &dst, const Mem<TYPE> &src) {
        SP_ASSERT(isValid(src, 2));

        const Mem<TYPE> &tmp = (reinterpret_cast<const Mem<TYPE>*>(&dst) != &src) ? src : clone(src);

        const int sdsize0 = src.dsize[0];
        const int sdsize1 = src.dsize[1];

        const int ddsize0 = (sdsize0 + 1) / 2;
        const int ddsize1 = (sdsize1 + 1) / 2;
        const int ddsize[2] = { ddsize0, ddsize1 };

        dst.resize(2, ddsize);

        const TYPE *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < ddsize1; v++) {
            const int sv = 2 * v;

            const int sv0 = sv + ((sv == 0) ? 0 : -1);
            const int sv1 = sv + 0;
            const int sv2 = sv + ((sv == sdsize1 - 1) ? 0 : +1);

            const TYPE *psrc0 = &psrc[sv0 * sdsize0];
            const TYPE *psrc1 = &psrc[sv1 * sdsize0];
            const TYPE *psrc2 = &psrc[sv2 * sdsize0];

            TYPE *pd = &pdst[v * ddsize0];

            for (int u = 0; u < ddsize0; u++) {
                const int su = 2 * u;

                const int su0 = su + ((su == 0) ? 0 : -1);
                const int su1 = su + 0;
                const int su2 = su + ((su == sdsize0 - 1) ? 0 : +1);
                
                const TYPE a00 = psrc0[su0];
                const TYPE a01 = psrc0[su1];
                const TYPE a02 = psrc0[su2];

                const TYPE a10 = psrc1[su0];
                const TYPE a11 = psrc1[su1];
                const TYPE a12 = psrc1[su2];

                const TYPE a20 = psrc2[su0];
                const TYPE a21 = psrc2[su1];
                const TYPE a22 = psrc2[su2];

                const SP_REAL d = ((a00 + 2.0 * a01 + a02) + 2.0 * (a10 + 2.0 * a11 + a12) + (a20 + 2.0 * a21 + a22)) / 16.0;
                cnvVal(*pd++, d);
            }
        }
    }


    //--------------------------------------------------------------------------------
    // crop 
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename ELEM = TYPE>
    SP_CPUFUNC void crop(Mem<TYPE> &dst, const Mem<TYPE> &src, const Rect2 &rect, const double angle = 0.0){
        SP_ASSERT(isValid(src, 2));

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
            const SP_REAL cv = cos(-angle);
            const SP_REAL sv = sin(-angle);
            const Vec2 cent = getVec2((rect.dbase[0] + rect.dsize[0] - 1) * 0.5, (rect.dbase[1] + rect.dsize[1] - 1) * 0.5);
            for (int v = 0; v < rect.dsize[1]; v++){
                for (int u = 0; u < rect.dsize[0]; u++){
                    const SP_REAL x = u + rect.dbase[0] - cent.x;
                    const SP_REAL y = v + rect.dbase[1] - cent.y;

                    const SP_REAL rx = cv * x - sv * y;
                    const SP_REAL ry = sv * x + cv * y;

                    for (int c = 0; c < ch; c++){
                        cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), acs2<TYPE, ELEM>(tmp, cent.x + rx, cent.y + ry, c));
                    }
                }
            }
        }
    }

    //--------------------------------------------------------------------------------
    // concat 
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC void concat(Mem<TYPE> &dst, const Mem<TYPE> &src0, const Mem<TYPE> &src1, const bool horizon = true){
        SP_ASSERT(isValid(src0, 2));
        SP_ASSERT(isValid(src1, 2));

        const Mem<TYPE> &tmp0 = (&dst != &src0) ? src0 : clone(src0);
        const Mem<TYPE> &tmp1 = (&dst != &src1) ? src1 : clone(src1);
        
        const int dsize0 = (horizon == true) ? src0.dsize[0] + src1.dsize[0] : maxval(src0.dsize[0], src1.dsize[0]);
        const int dsize1 = (horizon == true) ? maxval(src0.dsize[1], src1.dsize[1]) : src0.dsize[1] + src1.dsize[1];
        
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
        SP_ASSERT(isValid(src0, 2));
        SP_ASSERT(isValid(src1, 2));
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
        SP_ASSERT(isValid(src, 2));

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
        SP_ASSERT(isValid(src, 2));

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
        SP_ASSERT(isValid(src, 2));

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
        SP_ASSERT(isValid(src, 2));
        SP_ASSERT(isValid(table, 2));
        SP_ASSERT(cmpSize(2, src.dsize, table.dsize));
        
        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const Rect2 rect = getRect2(tmp.dsize);
        
        dst.resize(2, tmp.dsize);
        dst.zero();

        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){
                const Vec2 &vec = acs2(table, u, v);
                if (useExt == false && inRect(rect, u + vec.x, v + vec.y) == false) continue;
                
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
        SP_ASSERT(isValid(src, 2));

        const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

        Mat _mat = mat;
        if (mat.rows() == 2 && (mat.cols() == 2 || mat.cols() == 3)) {
            _mat = extMat(3, 3, mat);
        }

        if (mat.rows() != 3 || mat.cols() != 3) return;
        const Mat imat = invMat(mat);

        const int ch = sizeof(TYPE) / sizeof(ELEM);
        const Rect2 rect = getRect2(tmp.dsize);

        for (int v = 0; v < dst.dsize[1]; v++){
            for (int u = 0; u < dst.dsize[0]; u++){
                const Vec2 vec = imat * getVec2(u, v);
                if (inRect(rect, vec.x, vec.y) == false) continue;

                for (int c = 0; c < ch; c++){
                    cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), acs2<TYPE, ELEM>(tmp, vec.x, vec.y, c));
                }
            }
        }
    }


    //--------------------------------------------------------------------------------
    // convert 
    //--------------------------------------------------------------------------------

    template<typename DST, typename SRC>
    SP_CPUFUNC void cnvImg(Mem<DST> &dst, const Mem<SRC> &src){
        SP_ASSERT(isValid(src, 2));

        dst.resize(2, src.dsize);

        for (int i = 0; i < dst.size(); i++){
            cnvCol(dst[i], src[i]);
        }
    }
        
    template <typename TYPE0, typename TYPE1>
    SP_CPUFUNC void cnvDepthToImg(Mem<TYPE0> &dst, const Mem<TYPE1> &src, const double nearPlane = 100.0, const double farPlane = 10000.0){
        SP_ASSERT(isValid(src, 2));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++){
            const SP_REAL depth = extractZ(src[i]);

            if (depth >= nearPlane && depth <= farPlane){
                cnvDepthToCol(dst[i], depth, nearPlane, farPlane);
            }
        }
    }

    template <typename TYPE>
    SP_CPUFUNC void cnvNormalToImg(Mem<TYPE> &dst, const Mem<VecPN3> &src, const double nearPlane = 100.0, const double farPlane = 10000.0){
        SP_ASSERT(isValid(src, 2));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++){
            const SP_REAL depth = extractZ(src[i]);

            if (depth >= nearPlane && depth <= farPlane){
                cnvNormalToCol(dst[i], src[i].nrm);
            }
        }
    }

    template <typename TYPE>
    SP_CPUFUNC void cnvDispToImg(Mem<TYPE> &dst, const Mem<float> &src, const Mem<float> &eval, const int maxDisp, const int minDisp) {
        SP_ASSERT(isValid(src, 2));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++) {
            if (eval[i] > 0.0) {
                cnvDispToCol(dst[i], src[i], maxDisp, minDisp);
            }
        }
    }

    template <typename TYPE>
    SP_CPUFUNC void cnvDispToImg(Mem<TYPE> &dst, const Mem<float> &src, const int maxDisp, const int minDisp) {
        SP_ASSERT(isValid(src, 2));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++) {
            cnvDispToCol(dst[i], src[i], maxDisp, minDisp);
        }
    }

    SP_CPUFUNC void cnvLabelToImg(Mem<Col3> &dst, const Mem<int> &src){
        SP_ASSERT(isValid(src, 2));

        dst.resize(2, src.dsize);
        dst.zero();

        for (int i = 0; i < dst.size(); i++){
            if (src[i] < 0) continue;

            srand(src[i]);
            cnvHSVToCol(dst[i], getVec3((randu() + 1.0) * SP_PI, 1.0, 1.0));
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
        SP_ASSERT(isValid(src, 2));

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
        case 4:
        {
            Mem2<Col4> col;
            cnvImg(col, src);
            memcpy(dst, col.ptr, col.size() * 4);
            break;
        }
        default:
            dst = NULL;
            break;
        }
    }

}

#endif
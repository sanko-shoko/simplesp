//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_ACS_H__
#define __SP_ACS_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spbase.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // get rect
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rect getRect(const int dim, const int *dbase, const int *dsize){
        Rect rect;

        rect.dim = dim;
        for (int i = 0; i < SP_DIMMAX; i++){
            rect.dbase[i] = (i < dim && dbase != NULL) ? dbase[i] : 0;
            rect.dsize[i] = (i < dim && dsize != NULL) ? dsize[i] : 0;
        }
        return rect;
    }

    SP_GENFUNC Rect getRect2(const int dbase0, const int dbase1, const int dsize0, const int dsize1){
        const int dbase[] = { dbase0, dbase1 };
        const int dsize[] = { dsize0, dsize1 };
        return getRect(2, dbase, dsize);
    }

    SP_GENFUNC Rect getRect2(const int *dsize){
        return getRect(2, NULL, dsize);
    }

    SP_GENFUNC Rect getRect2(const Vec2 &vec) {
        return getRect2(round(vec.x), round(vec.y), 0, 0);
    }

    SP_GENFUNC Rect getRect3(const int dbase0, const int dbase1, const int dbase2, const int dsize0, const int dsize1, const int dsize2){
        const int dbase[] = { dbase0, dbase1, dbase2 };
        const int dsize[] = { dsize0, dsize1, dsize2 };
        return getRect(3, dbase, dsize);
    }

    SP_GENFUNC Rect getRect3(const int *dsize){
        return getRect(3, NULL, dsize);
    }

    SP_GENFUNC Rect getRect3(const Vec3 &vec) {
        return getRect3(round(vec.x), round(vec.y), round(vec.z), 0, 0, 0);
    }


    //--------------------------------------------------------------------------------
    // check in rect
    //--------------------------------------------------------------------------------

    SP_GENFUNC bool isInRect(const Rect &rect, const double *d){
        for (int i = 0; i < rect.dim; i++){
            if (d[i] < rect.dbase[i]) return false;
            if (d[i] > rect.dbase[i] + rect.dsize[i] - 1) return false;
        }
        return true;
    }

    SP_GENFUNC bool isInRect(const Rect &rect, const Rect &test){
        if (rect.dim != test.dim) return false;

        for (int i = 0; i < rect.dim; i++){
            if (test.dbase[i] < rect.dbase[i]) return false;
            if (test.dbase[i] + test.dsize[i] > rect.dbase[i] + rect.dsize[i]) return false;
        }
        return true;
    }

    SP_GENFUNC bool isInRect2(const Rect &rect, const double d0, const double d1){
        const double d[] = { d0, d1 };
        return isInRect(rect, d);
    }

    SP_GENFUNC bool isInRect2(const int *dsize, const double d0, const double d1){
        const double d[] = { d0, d1 };
        return isInRect(getRect2(dsize), d);
    }

    SP_GENFUNC bool isInRect3(const Rect &rect, const double d0, const double d1, const double d2){
        const double d[] = { d0, d1, d2 };
        return isInRect(rect, d);
    }

    SP_GENFUNC bool isInRect3(const int *dsize, const double d0, const double d1, const double d2){
        const double d[] = { d0, d1, d2 };
        return isInRect(getRect3(dsize), d);
    }

    //--------------------------------------------------------------------------------
    // rect util
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rect andRect(const Rect &rect0, const Rect &rect1){
        int dbase[SP_DIMMAX] = { 0 };
        int dsize[SP_DIMMAX] = { 0 };

        for (int i = 0; i < rect0.dim; i++){
            dbase[i] = maxVal(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = minVal(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
            if (dsize[i] < 0) {
                return getRect(rect0.dim, NULL, NULL);
            }
        }

        return getRect(rect0.dim, dbase, dsize);
    }

    SP_GENFUNC Rect orRect(const Rect &rect0, const Rect &rect1){
        int dbase[SP_DIMMAX] = { 0 };
        int dsize[SP_DIMMAX] = { 0 };

        for (int i = 0; i < rect0.dim; i++){
            dbase[i] = minVal(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = maxVal(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
            dsize[i] = maxVal(0, dsize[i]);
        }

        return getRect(rect0.dim, dbase, dsize);
    }

    SP_GENFUNC Rect adjustRect(const Rect &rect, const int val){
        int dbase[SP_DIMMAX] = { 0 };
        int dsize[SP_DIMMAX] = { 0 };

        for (int i = 0; i < rect.dim; i++){
            const int t = maxVal(val, -rect.dsize[i] / 2);
            dbase[i] = rect.dbase[i] - t;
            dsize[i] = rect.dsize[i] + 2 * t;
        }

        return getRect(rect.dim, dbase, dsize);
    }


    SP_GENFUNC Rect operator + (const Rect &rect, const int val){
        return adjustRect(rect, +val);
    }

    SP_GENFUNC Rect operator - (const Rect &rect, const int val){
        return adjustRect(rect, -val);
    }

    SP_GENFUNC void operator += (Rect &rect, const int val){
        rect = adjustRect(rect, +val);
    }

    SP_GENFUNC void operator -= (Rect &rect, const int val){
        rect = adjustRect(rect, -val);
    }


    //--------------------------------------------------------------------------------
    // check memory ptr
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_GENFUNC bool isValid(const int dim, const ExPtr<TYPE> &src){
        if (src.ptr == NULL || src.dim != dim) return false;

        for (int i = 0; i < src.dim; i++){
            if (src.dsize[i] == 0) return false;
        }
        return true;
    }

    template<typename TYPE>
    SP_GENFUNC bool isValid(const ExPtr<TYPE> &src){
        return src.dim > 0 ? isValid(src.dim, src) : false;
    }



    //--------------------------------------------------------------------------------
    // access ptr 1d (multi channel)
    //--------------------------------------------------------------------------------

    SP_GENFUNC const int acsid1(const int *dsize, const int d0){
        const int id0 = maxVal(0, minVal(dsize[0] - 1, d0));
        return id0;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC ELEM& acs1(ExPtr<TYPE> &src, const int d0, const int c = 0){
        const int id = acsid1(src.dsize, d0);
        return reinterpret_cast<ELEM*>(&src.ptr[id])[c];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC double acs1(ExPtr<TYPE> &src, const double d0, const int c = 0){
        const int id0 = static_cast<int>(d0);
        const double ad0 = d0 - id0;

        const double v0 = acs1<TYPE, ELEM>(src, id0 + 0, c) * (1 - ad0);
        const double v1 = acs1<TYPE, ELEM>(src, id0 + 1, c) * (0 + ad0);

        return v0 + v1;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC const ELEM& acs1(const ExPtr<TYPE> &src, const int d0, const int c = 0){
        return acs1<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, c);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC double acs1(const ExPtr<TYPE> &src, const double d0, const int c = 0){
        return acs1<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, c);
    }


    //--------------------------------------------------------------------------------
    // access ptr 2d (multi channel)
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC int acsid2(const int *dsize, const int d0, const int d1, const int c = 0){
        const int id0 = maxVal(0, minVal(dsize[0] - 1, d0));
        const int id1 = maxVal(0, minVal(dsize[1] - 1, d1));
        return id1 * dsize[0] + id0;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC ELEM& acs2(ExPtr<TYPE> &src, const int d0, const int d1, const int c = 0){
        const int id = acsid2(src.dsize, d0, d1);
        return reinterpret_cast<ELEM*>(&src.ptr[id])[c];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC double acs2(ExPtr<TYPE> &src, const double d0, const double d1, const int c = 0){
        const int id0 = static_cast<int>(d0);
        const int id1 = static_cast<int>(d1);
        const double ad0 = d0 - id0;
        const double ad1 = d1 - id1;

        const double v00 = acs2<TYPE, ELEM>(src, id0 + 0, id1 + 0, c) * (1 - ad0) * (1 - ad1);
        const double v10 = acs2<TYPE, ELEM>(src, id0 + 1, id1 + 0, c) * (0 + ad0) * (1 - ad1);
        const double v01 = acs2<TYPE, ELEM>(src, id0 + 0, id1 + 1, c) * (1 - ad0) * (0 + ad1);
        const double v11 = acs2<TYPE, ELEM>(src, id0 + 1, id1 + 1, c) * (0 + ad0) * (0 + ad1);

        return v00 + v10 + v01 + v11;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC const ELEM& acs2(const ExPtr<TYPE> &src, const int d0, const int d1, const int c = 0){
        return acs2<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, c);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC double acs2(const ExPtr<TYPE> &src, const double d0, const double d1, const int c = 0){
        return acs2<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, c);
    }


    //--------------------------------------------------------------------------------
    // access ptr 3d (multi channel)
    //--------------------------------------------------------------------------------

    SP_GENFUNC int acsid3(const int *dsize, const int d0, const int d1, const int d2, const int c = 0){
        const int id0 = maxVal(0, minVal(dsize[0] - 1, d0));
        const int id1 = maxVal(0, minVal(dsize[1] - 1, d1));
        const int id2 = maxVal(0, minVal(dsize[2] - 1, d2));
        return (id2 * dsize[1] + id1) * dsize[0] + id0;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC ELEM& acs3(ExPtr<TYPE> &src, const int d0, const int d1, const int d2, const int c = 0){
        const int id = acsid3(src.dsize, d0, d1, d2);
        return reinterpret_cast<ELEM*>(&src.ptr[id])[c];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC double acs3(ExPtr<TYPE> &src, const double d0, const double d1, const double d2, const int c = 0){
        const int id0 = static_cast<int>(d0);
        const int id1 = static_cast<int>(d1);
        const int id2 = static_cast<int>(d2);
        const double ad0 = d0 - id0;
        const double ad1 = d1 - id1;
        const double ad2 = d2 - id2;

        const double v000 = acs3<TYPE, ELEM>(src, id0 + 0, id1 + 0, id2 + 0, c) * (1 - ad0) * (1 - ad1) * (1 - ad2);
        const double v100 = acs3<TYPE, ELEM>(src, id0 + 1, id1 + 0, id2 + 0, c) * (0 + ad0) * (1 - ad1) * (1 - ad2);
        const double v010 = acs3<TYPE, ELEM>(src, id0 + 0, id1 + 1, id2 + 0, c) * (1 - ad0) * (0 + ad1) * (1 - ad2);
        const double v110 = acs3<TYPE, ELEM>(src, id0 + 1, id1 + 1, id2 + 0, c) * (0 + ad0) * (0 + ad1) * (1 - ad2);
        const double v001 = acs3<TYPE, ELEM>(src, id0 + 0, id1 + 0, id2 + 1, c) * (1 - ad0) * (1 - ad1) * (0 + ad2);
        const double v101 = acs3<TYPE, ELEM>(src, id0 + 1, id1 + 0, id2 + 1, c) * (0 + ad0) * (1 - ad1) * (0 + ad2);
        const double v011 = acs3<TYPE, ELEM>(src, id0 + 0, id1 + 1, id2 + 1, c) * (1 - ad0) * (0 + ad1) * (0 + ad2);
        const double v111 = acs3<TYPE, ELEM>(src, id0 + 1, id1 + 1, id2 + 1, c) * (0 + ad0) * (0 + ad1) * (0 + ad2);

        return v000 + v100 + v010 + v110 + v001 + v101 + v011 + v111;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC const ELEM& acs3(const ExPtr<TYPE> &src, const int d0, const int d1, const int d2, const int c = 0){
        return acs3<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, d2, c);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC double acs3(const ExPtr<TYPE> &src, const double d0, const double d1, const double d2, const int c = 0){
        return acs3<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, d2, c);
    }


    //--------------------------------------------------------------------------------
    // access ptr matrix
    //--------------------------------------------------------------------------------

    SP_GENFUNC double& acsm(ExPtr<double> &mat, const int r, const int c){
        return mat.ptr[r * mat.dsize[0] + c];
    }

    SP_GENFUNC const double& acsm(const ExPtr<double> &mat, const int r, const int c){
        return mat.ptr[r * mat.dsize[0] + c];
    }


    //--------------------------------------------------------------------------------
    // access ptr 2d color
    //--------------------------------------------------------------------------------

    SP_GENFUNC Col3& acsc(ExPtr<Col3> &src, const int d0, const int d1){
        const int id = acsid2(src.dsize, d0, d1);
        return src.ptr[id];
    }

    SP_GENFUNC Col3 acsc(ExPtr<Col3> &src, const double d0, const double d1){
        const int id0 = static_cast<int>(d0);
        const int id1 = static_cast<int>(d1);
        const double ad0 = d0 - id0;
        const double ad1 = d1 - id1;

        const Col3 v00 = acsc(src, id0 + 0, id1 + 0);
        const Col3 v10 = acsc(src, id0 + 1, id1 + 0);
        const Col3 v01 = acsc(src, id0 + 0, id1 + 1);
        const Col3 v11 = acsc(src, id0 + 1, id1 + 1);

        const double r00 = (1 - ad0) * (1 - ad1);
        const double r10 = (0 + ad0) * (1 - ad1);
        const double r01 = (1 - ad0) * (0 + ad1);
        const double r11 = (0 + ad0) * (0 + ad1);

        Col3 dst;
        dst.r = static_cast<Byte>(v00.r * r00 + v10.r * r10 + v01.r * r01 + v11.r * r11 + 0.5);
        dst.g = static_cast<Byte>(v00.g * r00 + v10.g * r10 + v01.g * r01 + v11.g * r11 + 0.5);
        dst.b = static_cast<Byte>(v00.b * r00 + v10.b * r10 + v01.b * r01 + v11.b * r11 + 0.5);

        return dst;
    }

    SP_GENFUNC const Col3& acsc(const ExPtr<Col3> &src, const int d0, const int d1){
        return acsc(*const_cast<ExPtr<Col3>*>(&src), d0, d1);
    }

    SP_GENFUNC Col3 acsc(const ExPtr<Col3> &src, const double d0, const double d1){
        return acsc(*const_cast<ExPtr<Col3>*>(&src), d0, d1);
    }


}

#endif
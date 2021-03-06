﻿//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BASE_H__
#define __SP_BASE_H__

#include "spcore/spcom.h"
#include <math.h>

namespace sp{

    //--------------------------------------------------------------------------------
    // wrapper
    //--------------------------------------------------------------------------------

    SP_GENFUNC int abs(const int x) { return ::abs(x); }

    SP_GENFUNC SP_REAL fabs(const double x) { return static_cast<SP_REAL>(::fabs(x)); }

    SP_GENFUNC SP_REAL pow(const double x, const double y) { return static_cast<SP_REAL>(::pow(x, y)); }

    SP_GENFUNC SP_REAL sin(const double x) { return static_cast<SP_REAL>(::sin(x)); }
    SP_GENFUNC SP_REAL cos(const double x) { return static_cast<SP_REAL>(::cos(x)); }
    SP_GENFUNC SP_REAL tan(const double x) { return static_cast<SP_REAL>(::tan(x)); }

    SP_GENFUNC SP_REAL asin(const double x) { const double t = (x > +1.0) ? +1.0 : (x < -1.0) ? -1.0 : x; return static_cast<SP_REAL>(::asin(t)); }
    SP_GENFUNC SP_REAL acos(const double x) { const double t = (x > +1.0) ? +1.0 : (x < -1.0) ? -1.0 : x; return static_cast<SP_REAL>(::acos(t)); }
    
    SP_GENFUNC SP_REAL atan (const double x) { return static_cast<SP_REAL>(::atan(x)); }
    SP_GENFUNC SP_REAL atan2(const double y, const double x) { return static_cast<SP_REAL>(::atan2(y, x)); }

    SP_GENFUNC SP_REAL sqrt(const double x) { return static_cast<SP_REAL>(::sqrt(x)); }

    SP_GENFUNC SP_REAL exp(const double x) { return static_cast<SP_REAL>(::exp(x)); }

    SP_GENFUNC SP_REAL log  (const double x) { return static_cast<SP_REAL>(::log(x)); }
    SP_GENFUNC SP_REAL log2 (const double x) { return static_cast<SP_REAL>(::log(x) / ::log(2.0)); }
    SP_GENFUNC SP_REAL log10(const double x) { return static_cast<SP_REAL>(::log(x) / ::log(10.0)); }


    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    // get round (ex. 1.5 -> 2)
    SP_GENFUNC int round(const double x) { return static_cast<int>((x > 0) ? (x + 0.5) : (x - 0.5)); }

    // get ceil (ex. 1.5 -> 2)
    SP_GENFUNC int ceil(const double x) { return static_cast<int>((x > 0) ? (x + 1.0) : (x - 1.0)); }

    // get floor (ex. 1.5 -> 1)
    SP_GENFUNC int floor(const double x){ return static_cast<int>(x); }

    // get sign (+1 or -1)
    template<typename TYPE> SP_GENFUNC int sign(const TYPE x){ return (x > 0) - (x < 0); }

    // swap
    template<typename TYPE> SP_GENFUNC void swap(TYPE &a, TYPE &b) { const TYPE tmp = a; a = b; b = tmp; }

    // get clone
    template<typename TYPE> SP_GENFUNC TYPE clone(const TYPE &src) { TYPE dst = src; return dst; }

    // get max value
    SP_GENFUNC const int max(const int a, const int b) { return (a > b) ? a : b; }

    // get min value
    SP_GENFUNC const int min(const int a, const int b) { return (a < b) ? a : b; }

    // get max value
    SP_GENFUNC const SP_REAL max(const double a, const double b) { return static_cast<SP_REAL>((a > b) ? a : b); }

    // get min value
    SP_GENFUNC const SP_REAL min(const double a, const double b) { return static_cast<SP_REAL>((a < b) ? a : b); }

    // get limit value
    SP_GENFUNC const int lim(const int v, const int minv, const int maxv) { return (v > maxv) ? maxv : ((v < minv) ? minv : v); }

    // get limit value
    SP_GENFUNC const SP_REAL lim(const double v, const double minv, const double maxv) { return static_cast<SP_REAL>((v > maxv) ? maxv : ((v < minv) ? minv : v)); }

     // x * x
    SP_GENFUNC SP_REAL sq(const double x) { return static_cast<SP_REAL>(x * x); }

    // x * x * x
    SP_GENFUNC SP_REAL cb(const double x) { return static_cast<SP_REAL>(x * x * x); }

    // cubic root
    SP_GENFUNC SP_REAL cbrt(const double x) { const double z = pow(fabs(x), 1.0 / 3.0); return static_cast<SP_REAL>((x >= 0.0) ? z : -z); }

    // sqrt(a * a + b * b) without destructive underflow or overflow
    SP_GENFUNC SP_REAL pythag(const double a, const double b) { 
        const double x = fabs(a);
        const double y = fabs(b);

        double ret = 0.0;
        if (x > y) {
            ret = x * sqrt(1.0 + (y / x) * (y / x));
        }
        else {
            ret = (y == 0.0) ? 0.0 : y * sqrt(1.0 + (x / y) * (x / y));
        }
        return static_cast<SP_REAL>(ret);
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
    // random
    //--------------------------------------------------------------------------------
    
    static unsigned int _randseed = 0;
    SP_GENFUNC unsigned int _snext(const unsigned int seed) {
        unsigned int s = seed + 1;
        s ^= (s << 13);
        s ^= (s >> 17);
        s ^= (s << 15);
        return s;
    }

    SP_GENFUNC void srand(const int seed) {
        _randseed = static_cast<unsigned int>(seed);
    }

    SP_GENFUNC int rand() {
        _randseed = _snext(_randseed);
        return static_cast<int>(_randseed >> 1);
    }

    SP_GENFUNC int rand(const int seed) {
        const unsigned int s = _snext(static_cast<unsigned int>(seed));
        return static_cast<int>(s >> 1);
    }

    // get random uniform (-1.0, 1.0)
    SP_GENFUNC SP_REAL randu(const int seed) {
        const unsigned int s = static_cast<unsigned int>(seed);

        const int maxv = 20000;
        const double a = static_cast<double>(rand(s) % (maxv + 1) + 1) / (maxv + 2);
        const double u = 2.0 * a - 1.0;
        return static_cast<SP_REAL>(u);
    }

    // get random uniform (-1.0, 1.0)
    SP_GENFUNC SP_REAL randu() {
        const unsigned s = _randseed;
        _randseed = _snext(s);
        return randu(s);
    }

    // get random gauss
    SP_GENFUNC SP_REAL randg(const int seed) {
        const unsigned s0 = static_cast<unsigned int>(seed);
        const unsigned s1 = _snext(s0);

        const int maxv = 20000;
        const double a = static_cast<double>(rand(s0) % (maxv + 1) + 1) / (maxv + 2);
        const double b = static_cast<double>(rand(s1) % (maxv + 1) + 1) / (maxv + 2);
        const double g = sqrt(-2.0 * log(a)) * sin(2.0 * SP_PI * b);
        return static_cast<SP_REAL>(g);
    }

    // get random gauss
    SP_GENFUNC SP_REAL randg() {
        const unsigned s = _randseed;
        _randseed = _snext(s);
        return randg(s);
    }


    //--------------------------------------------------------------------------------
    // byte order
    //--------------------------------------------------------------------------------

    // get byte order
    SP_GENFUNC ByteOrder getByteOrder() {
        const int v = 1;
        return (*(const char*)&v == 1) ? ByteOrder::LittleEndian : ByteOrder::BigEndian;
    }

    // reverse byte order
    template <typename TYPE> 
    SP_GENFUNC void revByteOrder(TYPE *ptr, const int size) {
        const int n = sizeof(TYPE);
        if (sizeof(TYPE) == 1) return;

        for (int i = 0; i < size; i++) {
            Byte *tmp = (Byte*)&ptr[i];

            for (int j = 0; j < n / 2; j++) {
                swap(tmp[j], tmp[n - j - 1]);
            }
        }
    }

    // get ordered value
    template<typename TYPE>
    SP_GENFUNC TYPE ordered(const TYPE &val, const ByteOrder order) {
        TYPE dst = val;
        if (getByteOrder() != order) revByteOrder(&dst, 1);
        return dst;
    }


    //--------------------------------------------------------------------------------
    // bit
    //--------------------------------------------------------------------------------

    static unsigned int SP_BITS_TABLE[256] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
    };

    SP_GENFUNC int cntBit(const Byte byte) {
        return SP_BITS_TABLE[byte];
    }

    SP_GENFUNC int cntBit(const Byte byte0, const Byte byte1) {
        return 8 - cntBit(byte0 ^ byte1);
    }

    SP_GENFUNC int cntBit(const Byte *bytes0, const Byte *bytes1, const int bsize) {
        int cnt = 0;
        for (int i = 0; i < bsize; i++) {
            cnt += cntBit(bytes0[i], bytes1[i]);
        }
        return cnt;
    }

    SP_GENFUNC void setBit(Byte *byte, const int p, const Byte b) {
        const Byte mask = 0x01 << p;
        *byte = (b != 0) ? *byte | mask : *byte & ~mask;
    }
  
    SP_GENFUNC Byte getBit(const Byte *byte, const int p) {
        const Byte mask = 0x01 << p;
        return (*byte & mask) ? 1 : 0;
    }

    template<typename TYPE>
    SP_GENFUNC void cnvBit(Byte *bytes, const int bsize, const TYPE *src, const int ssize, const TYPE thresh) {

        for (int i = 0; i < bsize; i++) {
            bytes[i] = 0;
        }

        for (int i = 0; i < ssize; i++) {
            if (src[i] >= thresh) {
                setBit(&bytes[i / 8], i % 8, 1);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // access vector
    //--------------------------------------------------------------------------------

    SP_GENFUNC SP_REAL& acsv(Vec2 &vec, const int i) {
        return ((SP_REAL*)&vec)[i];
    }
    SP_GENFUNC const SP_REAL& acsv(const Vec2 &vec, const int i) {
        return ((SP_REAL*)&vec)[i];
    }

    SP_GENFUNC SP_REAL& acsv(Vec3 &vec, const int i) {
        return ((SP_REAL*)&vec)[i];
    }
    SP_GENFUNC const SP_REAL& acsv(const Vec3 &vec, const int i) {
        return ((SP_REAL*)&vec)[i];
    }

    
    //--------------------------------------------------------------------------------
    // access ptr 1d (multi channel)
    //--------------------------------------------------------------------------------

    SP_GENFUNC const int acsid1(const int *dsize, const int d0) {
        const int id0 = max(0, min(dsize[0] - 1, d0));
        return id0;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC ELEM& acs1(ExPtr<TYPE> &src, const int d0, const int c = 0) {
        const int id = acsid1(src.dsize, d0);
        return reinterpret_cast<ELEM*>(&src.ptr[id])[c];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC SP_REAL acs1(ExPtr<TYPE> &src, const double d0, const int c = 0) {
        const int id0 = static_cast<int>(d0);
        const double ad0 = d0 - id0;

        const double v0 = acs1<TYPE, ELEM>(src, id0 + 0, c) * (1 - ad0);
        const double v1 = acs1<TYPE, ELEM>(src, id0 + 1, c) * (0 + ad0);

        return static_cast<SP_REAL>(v0 + v1);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC const ELEM& acs1(const ExPtr<TYPE> &src, const int d0, const int c = 0) {
        return acs1<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, c);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC SP_REAL acs1(const ExPtr<TYPE> &src, const double d0, const int c = 0) {
        return acs1<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, c);
    }


    SP_GENFUNC const int lacsid1(const int *dsize, const int d0) {
        int id0 = d0;
        while (id0 < 0) id0 += dsize[0];
        return id0 % dsize[0];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC ELEM& lacs1(ExPtr<TYPE> &src, const int d0, const int c = 0) {
        const int id = lacsid1(src.dsize, d0);
        return reinterpret_cast<ELEM*>(&src.ptr[id])[c];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC const ELEM& lacs1(const ExPtr<TYPE> &src, const int d0, const int c = 0) {
        return lacs1<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, c);
    }

    //--------------------------------------------------------------------------------
    // access ptr 2d (multi channel)
    //--------------------------------------------------------------------------------

    SP_GENFUNC int acsid2(const int *dsize, const int d0, const int d1, const int c = 0) {
        const int id0 = max(0, min(dsize[0] - 1, d0));
        const int id1 = max(0, min(dsize[1] - 1, d1));
        return id1 * dsize[0] + id0;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC ELEM& acs2(ExPtr<TYPE> &src, const int d0, const int d1, const int c = 0) {
        const int id = acsid2(src.dsize, d0, d1);
        return reinterpret_cast<ELEM*>(&src.ptr[id])[c];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC SP_REAL acs2(ExPtr<TYPE> &src, const double d0, const double d1, const int c = 0) {
        const int id0 = static_cast<int>(d0);
        const int id1 = static_cast<int>(d1);
        const double ad0 = d0 - id0;
        const double ad1 = d1 - id1;

        const double v00 = acs2<TYPE, ELEM>(src, id0 + 0, id1 + 0, c) * (1 - ad0) * (1 - ad1);
        const double v10 = acs2<TYPE, ELEM>(src, id0 + 1, id1 + 0, c) * (0 + ad0) * (1 - ad1);
        const double v01 = acs2<TYPE, ELEM>(src, id0 + 0, id1 + 1, c) * (1 - ad0) * (0 + ad1);
        const double v11 = acs2<TYPE, ELEM>(src, id0 + 1, id1 + 1, c) * (0 + ad0) * (0 + ad1);

        return static_cast<SP_REAL>(v00 + v10 + v01 + v11);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC const ELEM& acs2(const ExPtr<TYPE> &src, const int d0, const int d1, const int c = 0) {
        return acs2<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, c);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC SP_REAL acs2(const ExPtr<TYPE> &src, const double d0, const double d1, const int c = 0) {
        return acs2<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, c);
    }


    //--------------------------------------------------------------------------------
    // access ptr 3d (multi channel)
    //--------------------------------------------------------------------------------

    SP_GENFUNC int acsid3(const int *dsize, const int d0, const int d1, const int d2, const int c = 0) {
        const int id0 = max(0, min(dsize[0] - 1, d0));
        const int id1 = max(0, min(dsize[1] - 1, d1));
        const int id2 = max(0, min(dsize[2] - 1, d2));
        return (id2 * dsize[1] + id1) * dsize[0] + id0;
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC ELEM& acs3(ExPtr<TYPE> &src, const int d0, const int d1, const int d2, const int c = 0) {
        const int id = acsid3(src.dsize, d0, d1, d2);
        return reinterpret_cast<ELEM*>(&src.ptr[id])[c];
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC SP_REAL acs3(ExPtr<TYPE> &src, const SP_REAL d0, const double d1, const double d2, const int c = 0) {
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

        return static_cast<SP_REAL>(v000 + v100 + v010 + v110 + v001 + v101 + v011 + v111);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC const ELEM& acs3(const ExPtr<TYPE> &src, const int d0, const int d1, const int d2, const int c = 0) {
        return acs3<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, d2, c);
    }

    template<typename TYPE, typename ELEM = TYPE>
    SP_GENFUNC SP_REAL acs3(const ExPtr<TYPE> &src, const double d0, const double d1, const double d2, const int c = 0) {
        return acs3<TYPE, ELEM>(*const_cast<ExPtr<TYPE>*>(&src), d0, d1, d2, c);
    }


    //--------------------------------------------------------------------------------
    // access ptr 2d matrix
    //--------------------------------------------------------------------------------

    SP_GENFUNC SP_REAL& acsm(ExPtr<SP_REAL> &mat, const int r, const int c) {
        return mat.ptr[r * mat.dsize[0] + c];
    }

    SP_GENFUNC const SP_REAL& acsm(const ExPtr<SP_REAL> &mat, const int r, const int c) {
        return mat.ptr[r * mat.dsize[0] + c];
    }


    //--------------------------------------------------------------------------------
    // access ptr 2d color
    //--------------------------------------------------------------------------------

    SP_GENFUNC Col3& acsc(ExPtr<Col3> &src, const int d0, const int d1) {
        const int id = acsid2(src.dsize, d0, d1);
        return src.ptr[id];
    }

    SP_GENFUNC Col3 acsc(ExPtr<Col3> &src, const double d0, const double d1) {
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

    SP_GENFUNC const Col3& acsc(const ExPtr<Col3> &src, const int d0, const int d1) {
        return acsc(*const_cast<ExPtr<Col3>*>(&src), d0, d1);
    }

    SP_GENFUNC Col3 acsc(const ExPtr<Col3> &src, const double d0, const double d1) {
        return acsc(*const_cast<ExPtr<Col3>*>(&src), d0, d1);
    }

}

#endif
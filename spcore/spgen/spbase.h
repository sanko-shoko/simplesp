//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BASE_H__
#define __SP_BASE_H__

#include "spcore/spcom.h"
#include <math.h>

namespace sp{
#if SP_USE_WRAPPER

    //--------------------------------------------------------------------------------
    // wrapper
    //--------------------------------------------------------------------------------

    SP_GENFUNC int abs(const int x) {
        return ::abs(x);
    }

    SP_GENFUNC SP_REAL fabs(const double x) {
        return SP_RCAST(::fabs(x));
    }

    SP_GENFUNC SP_REAL pow(const double x, const double y) {
        return static_cast<SP_REAL>(::pow(x, y));
    }

    SP_GENFUNC SP_REAL sin(const double x) {
        return static_cast<SP_REAL>(::sin(x));
    }

    SP_GENFUNC SP_REAL asin(const double x) {
        const double t = (x > +1.0) ? +1.0 : (x < -1.0) ? -1.0 : x;
        return static_cast<SP_REAL>(::asin(t));
    }

    SP_GENFUNC SP_REAL cos(const double x) {
        return static_cast<SP_REAL>(::cos(x));
    }

    SP_GENFUNC SP_REAL acos(const double x) {
        const double t = (x > +1.0) ? +1.0 : (x < -1.0) ? -1.0 : x;
        return static_cast<SP_REAL>(::acos(t));
    }

    SP_GENFUNC SP_REAL tan(const double x) {
        return static_cast<SP_REAL>(::tan(x));
    }

    SP_GENFUNC SP_REAL atan(const double x) {
        return static_cast<SP_REAL>(::atan(x));
    }

    SP_GENFUNC SP_REAL atan2(const double y, const double x) {
        return static_cast<SP_REAL>(::atan2(y, x));
    }

    SP_GENFUNC SP_REAL sqrt(const double x) {
        return static_cast<SP_REAL>(::sqrt(x));
    }

    SP_GENFUNC SP_REAL exp(const double x) {
        return static_cast<SP_REAL>(::exp(x));
    }

    SP_GENFUNC SP_REAL log(const double x) {
        return static_cast<SP_REAL>(::log(x));
    }

    SP_GENFUNC SP_REAL log2(const double x) {
        return static_cast<SP_REAL>(::log(x) / ::log(2.0));
    }

    SP_GENFUNC SP_REAL log10(const double x) {
        return static_cast<SP_REAL>(::log(x) / ::log(10.0));
    }

#endif

    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    // get round (ex. 1.5 -> 2)
    SP_GENFUNC int round(const double val){
        return static_cast<int>((val > 0) ? (val + 0.5) : (val - 0.5));
    }

    // get ceil (ex. 1.5 -> 2)
    SP_GENFUNC int ceil(const double val) {
        return static_cast<int>((val > 0) ? (val + 1.0) : (val - 1.0));
    }

    // get floor (ex. 1.5 -> 1)
    SP_GENFUNC int floor(const double val){
        return static_cast<int>(val);
    }

    // get sign (+1 or -1)
    template<typename TYPE> SP_GENFUNC int sign(const TYPE val){
        return (val > 0) - (val < 0);
    }

    // swap
    template<typename TYPE> SP_GENFUNC void swap(TYPE &a, TYPE &b){
        const TYPE tmp = a;
        a = b;
        b = tmp;
    }

    // get clone
    template<typename TYPE> SP_GENFUNC TYPE clone(const TYPE &src){
        TYPE dst = src;
        return dst;
    }

    // get zero
    template<typename TYPE> SP_GENFUNC TYPE zero(){
        TYPE dst;
        for (int i = 0; i < sizeof(TYPE); i++){
            ((Byte*)&dst)[i] = 0;
        }
        return dst;
    }

    // get max value
    SP_GENFUNC const int maxval(const int a, const int b){
        return (a > b) ? a : b;
    }

    // get max value
    SP_GENFUNC const SP_REAL maxval(const double a, const double b){
        return SP_RCAST((a > b) ? a : b);
    }

    // get min value
    SP_GENFUNC const int minval(const int a, const int b){
        return (a < b) ? a : b;
    }

    // get min value
    SP_GENFUNC const SP_REAL minval(const double a, const double b){
        return SP_RCAST((a < b) ? a : b);
    }

    // get limit value
    SP_GENFUNC const int limval(const int v, const int maxv, const int minv) {
        return (v > maxv) ? maxv : ((v < minv) ? minv : v);
    }

    // get limit value
    SP_GENFUNC const SP_REAL limval(const double v, const double maxv, const double minv) {
        return SP_RCAST((v > maxv) ? maxv : ((v < minv) ? minv : v));
    }

     // x * x
    SP_GENFUNC SP_REAL square(const double x) {
        return x * x;
    }

    // x * x * x
    SP_GENFUNC SP_REAL cubic(const double x) {
        return x * x * x;
    }

    // cubic root
    SP_GENFUNC SP_REAL cbrt(const double x) {
        const double z = pow(fabs(x), 1.0 / 3.0);
        return SP_RCAST((x >= 0.0) ? z : -z);
    }

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
        return SP_RCAST(ret);
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
    
    static unsigned int SP_RANDSEED = 0;
    SP_GENFUNC void srand(const int seed) {
        SP_RANDSEED = static_cast<unsigned int>(seed);
    }

    SP_GENFUNC int rand() {
        unsigned int x = SP_RANDSEED + 1;
        x ^= (x << 13);
        x ^= (x >> 17);
        x ^= (x << 15);
        SP_RANDSEED = x;
        return static_cast<int>(x >> 1);
    }

    // get random uniform (-1.0, 1.0)
    SP_GENFUNC SP_REAL randu() {
        const int maxv = 2000;
        const double a = static_cast<double>(rand() % (maxv + 1) + 1) / (maxv + 2);
        const double u = 2.0 * a - 1.0;
        return SP_RCAST(u);
    }

    // get random gauss
    SP_GENFUNC SP_REAL randg() {
        const int maxv = 2000;
        const double a = static_cast<double>(rand() % (maxv + 1) + 1) / (maxv + 2);
        const double b = static_cast<double>(rand() % (maxv + 1) + 1) / (maxv + 2);
        const double g = sqrt(-2.0 * log(a)) * sin(2.0 * SP_PI * b);
        return SP_RCAST(g);
    }


    //--------------------------------------------------------------------------------
    // compare
    //--------------------------------------------------------------------------------

    // compare
    SP_GENFUNC bool cmpVal(const double a, const double b, const double t = 1.0e-6){
        return ((a - b) < +t && (a - b) > -t) ? true : false;
    }

    SP_GENFUNC bool cmpSize(const int dim, const int *dsize0, const int *dsize1){
        for (int i = 0; i < dim; i++){
            if (dsize0[i] != dsize1[i]) return false;
        }
        return true;
    }

    template<typename TYPE0, typename TYPE1>
    SP_GENFUNC bool cmpSize(const ExPtr<TYPE0> &mem0, const ExPtr<TYPE1> &mem1){
        if (mem0.dim != mem1.dim) return false;
        return cmpSize(mem0.dim, mem0.dsize, mem1.dsize);
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
    SP_GENFUNC TYPE getOrderedVal(const TYPE &val, const ByteOrder order) {
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
    // get rect
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rect2 getRect2(const int dbase0, const int dbase1, const int dsize0, const int dsize1) {
        Rect2 rect;
        rect.dbase[0] = dbase0;
        rect.dbase[1] = dbase1;
        rect.dsize[0] = dsize0;
        rect.dsize[1] = dsize1;
        return rect;
    }

    SP_GENFUNC Rect2 getRect2(const int *dbase, const int *dsize) {
        return getRect2(dbase[0], dbase[1], dsize[0], dsize[1]);
    }

    SP_GENFUNC Rect2 getRect2(const int *dsize) {
        return getRect2(0, 0, dsize[0], dsize[1]);
    }

    SP_GENFUNC Rect2 getRect2(const Vec2 &vec) {
        return getRect2(round(vec.x), round(vec.y), 1, 1);
    }

    SP_GENFUNC Rect3 getRect3(const int dbase0, const int dbase1, const int dbase2, const int dsize0, const int dsize1, const int dsize2) {
        Rect3 rect;
        rect.dbase[0] = dbase0;
        rect.dbase[1] = dbase1;
        rect.dbase[2] = dbase2;
        rect.dsize[0] = dsize0;
        rect.dsize[1] = dsize1;
        rect.dsize[2] = dsize2;
        return rect;
    }

    SP_GENFUNC Rect3 getRect3(const int *dbase, const int *dsize) {
        return getRect3(dbase[0], dbase[1], dbase[2], dsize[0], dsize[1], dsize[2]);
    }

    SP_GENFUNC Rect3 getRect3(const int *dsize) {
        return getRect3(0, 0, 0, dsize[0], dsize[1], dsize[2]);
    }

    SP_GENFUNC Rect3 getRect3(const Vec3 &vec) {
        return getRect3(round(vec.x), round(vec.y), round(vec.z), 1, 1, 1);
    }


    //--------------------------------------------------------------------------------
    // check in rect
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_GENFUNC bool inRect(const Rect2 &rect, const TYPE *d) {
        for (int i = 0; i < 2; i++) {
            if (d[i] < static_cast<TYPE>(rect.dbase[i])) return false;
            if (d[i] > static_cast<TYPE>(rect.dbase[i] + rect.dsize[i] - 1)) return false;
        }
        return true;
    }
    template<typename TYPE>
    SP_GENFUNC bool inRect(const Rect3 &rect, const TYPE *d) {
        for (int i = 0; i < 3; i++) {
            if (d[i] < static_cast<TYPE>(rect.dbase[i])) return false;
            if (d[i] > static_cast<TYPE>(rect.dbase[i] + rect.dsize[i] - 1)) return false;
        }
        return true;
    }
    
    SP_GENFUNC bool inRect(const Rect2 &rect, const Rect2 &test) {
        for (int i = 0; i < 2; i++) {
            if (test.dbase[i] < rect.dbase[i]) return false;
            if (test.dbase[i] + test.dsize[i] > rect.dbase[i] + rect.dsize[i]) return false;
        }
        return true;
    }
    SP_GENFUNC bool inRect(const Rect3 &rect, const Rect3 &test) {
        for (int i = 0; i < 3; i++) {
            if (test.dbase[i] < rect.dbase[i]) return false;
            if (test.dbase[i] + test.dsize[i] > rect.dbase[i] + rect.dsize[i]) return false;
        }
        return true;
    }

    SP_GENFUNC bool inRect(const Rect2 &rect, const double d0, const double d1) {
        const double d[] = { d0, d1 };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const double d0, const double d1) {
        const double d[] = { d0, d1 };
        return inRect(getRect2(dsize), d);
    }

    SP_GENFUNC bool inRect(const Rect2 &rect, const Vec2 &vec) {
        const double d[] = { vec.x, vec.y };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const Vec2 &vec) {
        const double d[] = { vec.x, vec.y };
        return inRect(getRect2(dsize), d);
    }

    SP_GENFUNC bool inRect(const Rect3 &rect, const double d0, const double d1, const double d2) {
        const double d[] = { d0, d1, d2 };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const double d0, const double d1, const double d2) {
        const double d[] = { d0, d1, d2 };
        return inRect(getRect3(dsize), d);
    }

    SP_GENFUNC bool inRect(const Rect3 &rect, const Vec3 &vec) {
        const double d[] = { vec.x, vec.y, vec.z };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const Vec3 &vec) {
        const double d[] = { vec.x, vec.y, vec.z };
        return inRect(getRect3(dsize), d);
    }

    //--------------------------------------------------------------------------------
    // rect util
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rect2 andRect(const Rect2 &rect0, const Rect2 &rect1) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            dbase[i] = maxval(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = minval(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 andRect(const Rect3 &rect0, const Rect3 &rect1) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            dbase[i] = maxval(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = minval(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
        }
        return getRect3(dbase, dsize);
    }

    SP_GENFUNC Rect2 orRect(const Rect2 &rect0, const Rect2 &rect1) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            dbase[i] = minval(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = maxval(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
            dsize[i] = maxval(0, dsize[i]);
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 orRect(const Rect3 &rect0, const Rect3 &rect1) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 3; i++) {
            dbase[i] = minval(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = maxval(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
            dsize[i] = maxval(0, dsize[i]);
        }
        return getRect3(dbase, dsize);
    }

    SP_GENFUNC Rect2 adjustRect(const Rect2 &rect, const int val) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            const int t = maxval(val, -rect.dsize[i] / 2);
            dbase[i] = rect.dbase[i] - t;
            dsize[i] = rect.dsize[i] + 2 * t;
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 adjustRect(const Rect3 &rect, const int val) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            const int t = maxval(val, -rect.dsize[i] / 2);
            dbase[i] = rect.dbase[i] - t;
            dsize[i] = rect.dsize[i] + 2 * t;
        }
        return getRect3(dbase, dsize);
    }

    SP_GENFUNC Rect2 shiftRect(const Rect2 &rect, const int *shift, int s = +1) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            dbase[i] = rect.dbase[i] + s * shift[i];
            dsize[i] = rect.dsize[i];
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 shiftRect(const Rect3 &rect, const int *shift, int s = +1) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            dbase[i] = rect.dbase[i] + s * shift[i];
            dsize[i] = rect.dsize[i];
        }
        return getRect3(dbase, dsize);
    }

    SP_GENFUNC bool cmpRect(const Rect2 &rect0, const Rect2 &rect1) {
        bool ret = true;
        for (int i = 0; i < 2; i++) {
            ret &= (rect0.dbase[i] == rect1.dbase[i]);
            ret &= (rect0.dsize[i] == rect1.dsize[i]);
        }
        return ret;
    }

    SP_GENFUNC bool cmpRect(const Rect3 &rect0, const Rect3 &rect1) {
        bool ret = true;
        for (int i = 0; i < 3; i++) {
            ret &= (rect0.dbase[i] == rect1.dbase[i]);
            ret &= (rect0.dsize[i] == rect1.dsize[i]);
        }
        return ret;
    }


    //--------------------------------------------------------------------------------
    // rect operator
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC Rect2 operator + (const Rect2 &rect, const int val) {
        return adjustRect(rect, +val);
    }
    SP_GENFUNC Rect3 operator + (const Rect3 &rect, const int val) {
        return adjustRect(rect, +val);
    }

    SP_GENFUNC Rect2 operator + (const Rect2 &rect, const int *shift) {
        return shiftRect(rect, shift, +1);
    }
    SP_GENFUNC Rect3 operator + (const Rect3 &rect, const int *shift) {
        return shiftRect(rect, shift, +1);
    }

    SP_GENFUNC Rect2 operator - (const Rect2 &rect, const int val) {
        return adjustRect(rect, -val);
    }
    SP_GENFUNC Rect3 operator - (const Rect3 &rect, const int val) {
        return adjustRect(rect, -val);
    }

    SP_GENFUNC Rect2 operator - (const Rect2 &rect, const int *shift) {
        return shiftRect(rect, shift, -1);
    }
    SP_GENFUNC Rect3 operator - (const Rect3 &rect, const int *shift) {
        return shiftRect(rect, shift, -1);
    }

    SP_GENFUNC void operator += (Rect2 &rect, const int val) {
        rect = adjustRect(rect, +val);
    }
    SP_GENFUNC void operator += (Rect3 &rect, const int val) {
        rect = adjustRect(rect, +val);
    }

    SP_GENFUNC void operator += (Rect2 &rect, const int *shift) {
        rect = shiftRect(rect, shift +1);
    }
    SP_GENFUNC void operator += (Rect3 &rect, const int *shift) {
        rect = shiftRect(rect, shift +1);
    }

    SP_GENFUNC void operator -= (Rect2 &rect, const int val) {
        rect = adjustRect(rect, -val);
    }
    SP_GENFUNC void operator -= (Rect3 &rect, const int val) {
        rect = adjustRect(rect, -val);
    }

    SP_GENFUNC void operator -= (Rect2 &rect, const int *shift) {
        rect = shiftRect(rect, shift, -1);
    }
    SP_GENFUNC void operator -= (Rect3 &rect, const int *shift) {
        rect = shiftRect(rect, shift, -1);
    }

    SP_GENFUNC bool operator == (const Rect2 &rect0, const Rect2 &rect1) {
        return cmpRect(rect0, rect1);
    }
    SP_GENFUNC bool operator == (const Rect3 &rect0, const Rect3 &rect1) {
        return cmpRect(rect0, rect1);
    }

    SP_GENFUNC bool operator != (const Rect2 &rect0, const Rect2 &rect1) {
        return !cmpRect(rect0, rect1);
    }
    SP_GENFUNC bool operator != (const Rect3 &rect0, const Rect3 &rect1) {
        return !cmpRect(rect0, rect1);
    }


    //--------------------------------------------------------------------------------
    // check memory ptr
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_GENFUNC bool isValid(const ExPtr<TYPE> &src, const int dim) {
        if (src.dim == 0 || src.ptr == NULL || src.dim != dim) return false;

        for (int i = 0; i < src.dim; i++) {
            if (src.dsize[i] == 0) return false;
        }
        return true;
    }

    template<typename TYPE>
    SP_GENFUNC bool isValid(const ExPtr<TYPE> &src) {
        return isValid(src, src.dim);
    }


    //--------------------------------------------------------------------------------
    // access ptr 1d (multi channel)
    //--------------------------------------------------------------------------------

    SP_GENFUNC const int acsid1(const int *dsize, const int d0) {
        const int id0 = maxval(0, minval(dsize[0] - 1, d0));
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
        const int id0 = maxval(0, minval(dsize[0] - 1, d0));
        const int id1 = maxval(0, minval(dsize[1] - 1, d1));
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
        const int id0 = maxval(0, minval(dsize[0] - 1, d0));
        const int id1 = maxval(0, minval(dsize[1] - 1, d1));
        const int id2 = maxval(0, minval(dsize[2] - 1, d2));
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
    // access ptr matrix
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
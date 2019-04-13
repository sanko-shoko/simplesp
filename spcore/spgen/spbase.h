//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BASE_H__
#define __SP_BASE_H__

#include "spcore/spcom.h"

namespace sp{

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
    SP_GENFUNC const int maxVal(const int a, const int b){
        return (a > b) ? a : b;
    }

    // get max value
    SP_GENFUNC const SP_REAL maxVal(const double a, const double b){
        return (a > b) ? a : b;
    }

    // get min value
    SP_GENFUNC const int minVal(const int a, const int b){
        return (a < b) ? a : b;
    }

    // get min value
    SP_GENFUNC const SP_REAL minVal(const double a, const double b){
        return (a < b) ? a : b;
    }

    // get uniform random value (-1.0, 1.0)
    SP_GENFUNC SP_REAL randValUnif(){
        const int maxv = 2000;
        return 2.0 * (rand() % (maxv + 1) + 1) / (maxv + 2) - 1.0;
    }

    // get gauss random value
    SP_GENFUNC SP_REAL randValGauss(){
        const SP_REAL a = (randValUnif() + 1.0) / 2.0;
        const SP_REAL b = (randValUnif() + 1.0) / 2.0;
        return sqrt(-2.0 * log(a)) * sin(2.0 * SP_PI * b);
    }


    //--------------------------------------------------------------------------------
    // compare
    //--------------------------------------------------------------------------------

    // check nearly equal
    SP_GENFUNC bool cmpVal(const SP_REAL a, const SP_REAL b, const SP_REAL t = SP_SMALL){
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

    SP_GENFUNC bool cmpVec2(const Vec2 &vec0, const Vec2 &vec1, const SP_REAL t = SP_SMALL){
        return cmpVal(vec0.x, vec1.x, t) & cmpVal(vec0.y, vec1.y, t);
    }
    SP_GENFUNC bool cmpVec3(const Vec3 &vec0, const Vec3 &vec1, const SP_REAL t = SP_SMALL) {
        return cmpVal(vec0.x, vec1.x, t) & cmpVal(vec0.y, vec1.y, t) & cmpVal(vec0.z, vec1.z, t);
    }

    SP_GENFUNC bool cmpCol(const Col3 &col0, const Col3 &col1) {
        return (col0.r == col1.r) & (col0.g == col1.g) & (col0.b == col1.b);
    }
    SP_GENFUNC bool cmpCol(const Col4 &col0, const Col4 &col1) {
        return (col0.r == col1.r) & (col0.g == col1.g) & (col0.b == col1.b) & (col0.a == col1.a);
    }

    SP_GENFUNC bool cmpRot(const Rot &rot0, const Rot &rot1, const SP_REAL t = SP_SMALL){
        bool ret = true;
        ret &= cmpVal(rot0.qx * sign(rot0.qw), rot1.qx * sign(rot1.qw), t);
        ret &= cmpVal(rot0.qy * sign(rot0.qw), rot1.qy * sign(rot1.qw), t);
        ret &= cmpVal(rot0.qz * sign(rot0.qw), rot1.qz * sign(rot1.qw), t);
        ret &= cmpVal(rot0.qw * sign(rot0.qw), rot1.qw * sign(rot1.qw), t);
        return ret;
    }

    SP_GENFUNC bool cmpPose(const Pose &pose0, const Pose &pose1, const SP_REAL tr = SP_SMALL, const SP_REAL tt = SP_SMALL){
        bool ret = true;
        ret &= cmpRot(pose0.rot, pose1.rot, tr);
        ret &= cmpVec3(pose0.trn, pose1.trn, tt);
        return ret;
    }


    //--------------------------------------------------------------------------------
    // convert value
    //--------------------------------------------------------------------------------

    template<typename TYPE> SP_GENFUNC void cnvVal(char &dst, const TYPE &src){
        dst = static_cast<char>(src + 0.5 - (src < 0));
    }

    template<typename TYPE> SP_GENFUNC void cnvVal(unsigned char &dst, const TYPE &src){
        dst = static_cast<unsigned char>((src + 0.5) * (src > 0));
    }

    template<typename TYPE> SP_GENFUNC void cnvVal(short &dst, const TYPE &src){
        dst = static_cast<short>(src + 0.5 - (src < 0));
    }

    template<typename TYPE> SP_GENFUNC void cnvVal(unsigned short &dst, const TYPE &src){
        dst = static_cast<unsigned short>((src + 0.5) * (src > 0));
    }

    template<typename TYPE> SP_GENFUNC void cnvVal(int &dst, const TYPE &src){
        dst = static_cast<int>(src + 0.5 - (src < 0));
    }

    template<typename TYPE> SP_GENFUNC void cnvVal(unsigned int &dst, const TYPE &src){
        dst = static_cast<unsigned int>((src + 0.5) * (src > 0));
    }

    template<typename TYPE> SP_GENFUNC void cnvVal(float &dst, const TYPE &src){
        dst = static_cast<float>(src);
    }

    template<typename TYPE> SP_GENFUNC void cnvVal(double &dst, const TYPE &src){
        dst = static_cast<SP_REAL>(src);
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
    template <typename TYPE> SP_GENFUNC void revByteOrder(TYPE *ptr, const int count) {
        const int n = sizeof(TYPE);
        if (sizeof(TYPE) == 1) return;

        for (int i = 0; i < count; i++) {
            Byte *tmp = (Byte*)&ptr[i];

            for (int j = 0; j < n / 2; j++) {
                swap(tmp[j], tmp[n - j - 1]);
            }
        }
    }

    // 
    template<typename TYPE>
    SP_GENFUNC TYPE getBigEndian(const TYPE &val) {
        TYPE dst = val;
        if (getByteOrder() != ByteOrder::BigEndian) revByteOrder(&dst, 1);
        return dst;
    }

    // 
    template<typename TYPE>
    SP_GENFUNC TYPE getLittleEndian(const TYPE &val) {
        TYPE dst = val;
        if (getByteOrder() != ByteOrder::LittleEndian) revByteOrder(&dst, 1);
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

    SP_GENFUNC void setBit(Byte &byte, const int p, const Byte b) {
        const Byte mask = 0x01 << p;
        byte = (b != 0) ? byte | mask : byte & ~mask;
    }

    template<typename TYPE>
    SP_GENFUNC void cnvBit(Byte *bytes, const int bsize, const TYPE *src, const int ssize, const TYPE thresh) {

        for (int i = 0; i < bsize; i++) {
            bytes[i] = 0;
        }

        for (int i = 0; i < ssize; i++) {
            if (src[i] > thresh) {
                setBit(bytes[i / 8], i % 8, 1);
            }
        }
    }
    
}

#endif
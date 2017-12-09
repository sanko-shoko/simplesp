//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
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
	template<typename TYPE>	SP_GENFUNC int sign(const TYPE val){
		return (val >= 0) ? +1 : -1;
	}

	// swap
	template<typename TYPE>	SP_GENFUNC void swap(TYPE &a, TYPE &b){
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
	template<typename TYPE>	SP_GENFUNC TYPE zero(){
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
	SP_GENFUNC const double maxVal(const double a, const double b){
		return (a > b) ? a : b;
	}

	// get min value
	SP_GENFUNC const int minVal(const int a, const int b){
		return (a < b) ? a : b;
	}

	// get min value
	SP_GENFUNC const double minVal(const double a, const double b){
		return (a < b) ? a : b;
	}

	// get uniform random value (-1.0, 1.0)
	SP_GENFUNC double randValUnif(){
		const int maxv = 2000;
		return 2.0 * (rand() % (maxv + 1) + 1) / (maxv + 2) - 1.0;
	}

	// get gauss random value
	SP_GENFUNC double randValGauss(){
		const double a = (randValUnif() + 1.0) / 2.0;
		const double b = (randValUnif() + 1.0) / 2.0;
		return sqrt(-2.0 * log(a)) * sin(2.0 * SP_PI * b);
	}


	//--------------------------------------------------------------------------------
	// compare
	//--------------------------------------------------------------------------------

	// check nearly equal
	SP_GENFUNC bool cmpVal(const double a, const double b){
		return ((a - b) < +SP_SMALL && (a - b) > -SP_SMALL) ? true : false;
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

	SP_GENFUNC bool cmpVec(const Vec2 &vec0, const Vec2 &vec1){
		return cmpVal(vec0.x, vec1.x) & cmpVal(vec0.y, vec1.y);
	}

	SP_GENFUNC bool cmpVec(const Vec3 &vec0, const Vec3 &vec1){
		return cmpVal(vec0.x, vec1.x) & cmpVal(vec0.y, vec1.y) & cmpVal(vec0.z, vec1.z);
	}

	SP_GENFUNC bool cmpRot(const Rot &rot0, const Rot &rot1){
		bool ret = true;
		ret &= cmpVal(rot0.qx * sign(rot0.qw), rot1.qx * sign(rot1.qw));
		ret &= cmpVal(rot0.qy * sign(rot0.qw), rot1.qy * sign(rot1.qw));
		ret &= cmpVal(rot0.qz * sign(rot0.qw), rot1.qz * sign(rot1.qw));
		ret &= cmpVal(rot0.qw * sign(rot0.qw), rot1.qw * sign(rot1.qw));
		return ret;
	}

	SP_GENFUNC bool cmpPose(const Pose &pose0, const Pose &pose1){
		bool ret = true;
		ret &= cmpRot(pose0.rot, pose1.rot);
		ret &= cmpVec(pose0.trn, pose1.trn);
		return ret;
	}


	//--------------------------------------------------------------------------------
	// convert value
	//--------------------------------------------------------------------------------

	template<typename TYPE> SP_GENFUNC void cnvVal(char &dst, const TYPE &src){
		dst = static_cast<char>((src > 0) ? (src + 0.5) : (src - 0.5));
	}

	template<typename TYPE> SP_GENFUNC void cnvVal(unsigned char &dst, const TYPE &src){
		dst = static_cast<unsigned char>((src > 0) ? (src + 0.5) : 0.0);
	}

	template<typename TYPE> SP_GENFUNC void cnvVal(short &dst, const TYPE &src){
		dst = static_cast<short>((src > 0) ? (src + 0.5) : (src - 0.5));
	}

	template<typename TYPE> SP_GENFUNC void cnvVal(unsigned short &dst, const TYPE &src){
		dst = static_cast<unsigned short>((src > 0) ? (src + 0.5) : 0.0);
	}

	template<typename TYPE> SP_GENFUNC void cnvVal(int &dst, const TYPE &src){
		dst = static_cast<int>((src > 0) ? (src + 0.5) : (src - 0.5));
	}

	template<typename TYPE> SP_GENFUNC void cnvVal(unsigned int &dst, const TYPE &src){
		dst = static_cast<unsigned int>((src > 0) ? (src + 0.5) : 0.0);
	}

	template<typename TYPE> SP_GENFUNC void cnvVal(float &dst, const TYPE &src){
		dst = static_cast<float>(src);
	}

	template<typename TYPE> SP_GENFUNC void cnvVal(double &dst, const TYPE &src){
		dst = static_cast<double>(src);
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
}

#endif
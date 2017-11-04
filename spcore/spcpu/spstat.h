//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STAT_H__
#define __SP_STAT_H__

#include "spcore/spcom.h"
#include "spcore/spcpu/spmop.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// sort
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC int compare_min(const void *a, const void *b){
		return (*static_cast<const TYPE*>(a) > *static_cast<const TYPE*>(b)) ? +1 : -1;
	}

	template<typename TYPE>
	SP_CPUFUNC int compare_max(const void *a, const void *b){
		return (*static_cast<const TYPE*>(a) > *static_cast<const TYPE*>(b)) ? -1 : +1;
	}

	template<typename TYPE>
	SP_CPUFUNC void sort(TYPE *mem, const int size, const bool minOrder = true){
		::qsort(mem, size, sizeof(TYPE), (minOrder) ? compare_min<TYPE> : compare_max<TYPE>);
	}

	template<typename TYPE>
	SP_CPUFUNC void sort(Mem<TYPE> &mem, const bool minOrder = true){
		sort(mem.ptr, mem.size(), minOrder);
	}


	//--------------------------------------------------------------------------------
	// max / min
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC TYPE maxVal(const Mem<TYPE> &mem){
		if (mem.size() == 0) return zero<TYPE>();

		TYPE maxv = mem[0];
		for (int i = 1; i < mem.size(); i++){
			maxv = maxVal(maxv, mem[i]);
		}
		return maxv;
	}

	template<typename TYPE>
	SP_CPUFUNC TYPE minVal(const Mem<TYPE> &mem){
		if (mem.size() == 0) return zero<TYPE>();

		TYPE minv = mem[0];
		for (int i = 0; i < mem.size(); i++){
			minv = minVal(minv, mem[i]);
		}
		return minv;
	}


	template<typename TYPE>
	SP_CPUFUNC int maxArg(const Mem<TYPE> &mem){
		double maxv = -SP_INFINITY;
		int ret = -1;
		for (int i = 0; i < mem.size(); i++){
			if (mem[i] > maxv){
				maxv = mem[i];
				ret = i;
			}
		}
		return ret;
	}

	template<typename TYPE>
	SP_CPUFUNC int minArg(const Mem<TYPE> &mem){
		double minv = +SP_INFINITY;
		int ret = -1;
		for (int i = 0; i < mem.size(); i++){
			if (mem[i] < minv){
				minv = mem[i];
				ret = i;
			}
		}
		return ret;
	}

	//--------------------------------------------------------------------------------
	// base
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC Mem<TYPE> sqVal(const Mem<TYPE> &mem){
		Mem<TYPE> dst(mem.dim, mem.dsize);

		for (int i = 0; i < mem.size(); i++){
			dst[i] = square(mem[i]);
		}
		return dst;
	}

	template<typename TYPE>
	SP_CPUFUNC Mem<TYPE> sqrtVal(const Mem<TYPE> &mem){
		Mem<TYPE> dst(mem.dim, mem.dsize);

		for (int i = 0; i < mem.size(); i++){
			dst[i] = sqrt(mem[i]);
		}
		return dst;
	}


	//--------------------------------------------------------------------------------
	// value
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC double sumVal(const Mem<TYPE> &mem){
		double sum = 0.0;

		for (int i = 0; i < mem.size(); i++){
			sum += mem[i];
		}
		return sum;
	}

	template<typename TYPE>
	SP_CPUFUNC double meanVal(const Mem<TYPE> &mem){
		return sumVal(mem) / mem.size();
	}


	template<typename TYPE>
	SP_CPUFUNC double sumSq(const Mem<TYPE> &mem){
		double sum = 0.0;

		for (int i = 0; i < mem.size(); i++){
			sum += square(mem[i]);
		}
		return sum;
	}

	template<typename TYPE>
	SP_CPUFUNC double meanSq(const Mem<TYPE> &mem){
		return sumSq(mem) / mem.size();
	}


	template<typename TYPE>
	SP_CPUFUNC double sumSqrt(const Mem<TYPE> &mem){
		double sum = 0.0;

		for (int i = 0; i < mem.size(); i++){
			sum += sqrt(mem[i]);
		}
		return sum;
	}

	template<typename TYPE>
	SP_CPUFUNC double meanSqrt(const Mem<TYPE> &mem){
		return sumSqrt(mem) / mem.size();
	}


	template<typename TYPE>
	SP_CPUFUNC double sumAbs(const Mem<TYPE> &mem){
		double sum = 0.0;

		for (int i = 0; i < mem.size(); i++){
			sum += fabs(static_cast<double>(mem[i]));
		}
		return sum;
	}

	template<typename TYPE>
	SP_CPUFUNC double meanAbs(const Mem<TYPE> &mem){
		return sumAbs(mem) / mem.size();
	}


	template<typename TYPE>
	SP_CPUFUNC TYPE medianVal(const Mem<TYPE> &mem){
		SP_ASSERT(mem.size() > 0);

		Mem<TYPE> tmp = mem;
		sort(tmp);

		return tmp[tmp.size() / 2];
	}


	//--------------------------------------------------------------------------------
	// matrix
	//--------------------------------------------------------------------------------

	SP_CPUFUNC Mat sumVal(const Mat &mat, const int axis){
		SP_ASSERT(axis == 0 || axis == 1);

		Mat sum((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
		sum.zero();

		const double *pMat = mat.ptr;

		for (int r = 0; r < mat.rows(); r++) {
			for (int c = 0; c < mat.cols(); c++) {
				sum[(axis == 0) ? c : r] += *pMat++;
			}
		}
		return sum;
	}

	SP_CPUFUNC Mat meanVal(const Mat &mat, const int axis){
		return sumVal(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
	}


	SP_CPUFUNC Mat sumSq(const Mat &mat, const int axis){
		SP_ASSERT(axis == 0 || axis == 1);

		Mat sum((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
		sum.zero();

		const double *pMat = mat.ptr;

		for (int r = 0; r < mat.rows(); r++){
			for (int c = 0; c < mat.cols(); c++){
				sum[(axis == 0) ? c : r] += square(*pMat++);
			}
		}
		return sum;
	}

	SP_CPUFUNC Mat meanSq(const Mat &mat, const int axis){
		return sumSq(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
	}


	SP_CPUFUNC Mat sumAbs(const Mat &mat, const int axis){
		SP_ASSERT(axis == 0 || axis == 1);

		Mat sum((axis == 1) ? mat.rows() : 1, (axis == 0) ? mat.cols() : 1);
		sum.zero();

		const double *pMat = mat.ptr;

		for (int r = 0; r < mat.rows(); r++){
			for (int c = 0; c < mat.cols(); c++){
				sum[(axis == 0) ? c : r] += fabs(*pMat++);
			}
		}
		return sum;
	}

	SP_CPUFUNC Mat meanAbs(const Mat &mat, const int axis){
		return sumAbs(mat, axis) / ((axis == 0) ? mat.rows() : mat.cols());
	}



	//--------------------------------------------------------------------------------
	// vector
	//--------------------------------------------------------------------------------

	template<typename VEC>
	SP_CPUFUNC VEC sumVec(const Mem<VEC> &vecs){
		VEC sum = zero<VEC>();

		for (int i = 0; i < vecs.size(); i++){
			sum += vecs[i];
		}
		return sum;
	}

	template<typename VEC>
	SP_CPUFUNC VEC meanVec(const Mem<VEC> &vecs){
		return sumVec(vecs) / vecs.size();
	}

	template<typename VEC>
	SP_CPUFUNC Mem<double> normVec(const Mem<VEC> &vecs){
		Mem<double> dst(vecs.dim, vecs.dsize);

		for (int i = 0; i < dst.size(); i++){
			dst[i] = normVec(vecs[i]);
		}
		return dst;
	}


	//--------------------------------------------------------------------------------
	// eval
	//--------------------------------------------------------------------------------
	
	SP_CPUFUNC double evalErr(const Mem1<double> &errs, const double thresh = 5.0) {
		double eval = 0.0;
		for (int i = 0; i < errs.size(); i++) {
			if (errs[i] < thresh) eval += 1.0;
		}
		return eval / errs.size();
	}

	SP_CPUFUNC double evalErr(const double err, const double thresh = 5.0) {
		return (err < thresh) ? 1.0 : 0.0;
	}

	
	template<typename TYPE>
	SP_CPUFUNC Mem1<TYPE> denoise(const Mem<TYPE> &src, const Mem<double> &errs, const double thresh = 5.0) {
		Mem1<TYPE> dst;
		dst.reserve(src.size());
		for (int i = 0; i < src.size(); i++) {
			if (errs[i] > thresh) continue;
			dst.push(src[i]);
		}
		return dst;
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
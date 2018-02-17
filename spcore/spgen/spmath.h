//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MATH_H__
#define __SP_MATH_H__

#include "spcore/spcom.h"
#include "spcore/spwrap.h"
#include "spcore/spgen/spbase.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// function
	//--------------------------------------------------------------------------------

	// gauss function
	SP_GENFUNC double funcGauss(const double x, const double mean, const double sigma){
		double ret = 0.0;

		if (fabs(sigma) > 0){
			ret = exp(-(x - mean) * (x - mean) / (2 * sigma * sigma)) / (sqrt(2 * SP_PI) * sigma);
		}
		return ret;
	}

	// tukey function
	SP_GENFUNC double funcTukey(const double x, const double t){
		double ret = 0.0;

		if (fabs(x) < t){
			const double v = 1.0 - (x * x) / (t * t);
			ret = v * v;
		}
		return ret;
	}

	// a * a
	SP_GENFUNC double square(const double a){
		return a * a;
	}

	// sqrt(a * a + b * b) without destructive underflow or overflow
	SP_GENFUNC double pythag(const double a, const double b){
		const double x = fabs(a);
		const double y = fabs(b);
		if (x > y){
			return x * sqrt(1.0 + (y / x) * (y / x));
		}
		else{
			return (y == 0.0) ? 0.0 : y * sqrt(1.0 + (x / y) * (x / y));
		}
	}

	// combination
	SP_GENFUNC int nCk(const int n, const int k) {
		int ret = 1;
		for (int i = 1; i <= k; i++){
			ret = ret * (n - i + 1) / i;
		}
		return ret;
	}


	//--------------------------------------------------------------------------------
	// mem
	//--------------------------------------------------------------------------------
	
	// set
	template <typename TYPE, typename TYPE0>
	SP_GENFUNC void setMem(TYPE *dst, const int size, const TYPE0 *mem0){
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i];
		}
	}

	// convert
	template <typename TYPE, typename TYPE0>
	SP_GENFUNC void cnvMem(TYPE *dst, const int size, const TYPE0 *mem0, const double scale = 1.0, const double base = 0.0) {
		for (int i = 0; i < size; i++) {
			cnvVal(dst[i], (mem0[i] - base) * scale);
		}
	}

	// addition
	template <typename TYPE, typename TYPE0, typename TYPE1>
	SP_GENFUNC void addMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i] + mem1[i];
		}
	}

	// subtraction
	template <typename TYPE, typename TYPE0, typename TYPE1>
	SP_GENFUNC void subMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i] - mem1[i];
		}
	}

	// multiple
	template <typename TYPE, typename TYPE0, typename TYPE1>
	SP_GENFUNC void mulMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i] * mem1[i];
		}
	}

	// division
	template <typename TYPE, typename TYPE0, typename TYPE1>
	SP_GENFUNC void divMem(TYPE *dst, const int size, const TYPE0 *mem0, const TYPE1 *mem1){
		for (int i = 0; i < size; i++){
			if (mem1[i] == 0.0) continue;
			dst[i] = mem0[i] / mem1[i];
		}
	}


	// set
	template <typename TYPE, typename ELEM>
	SP_GENFUNC void setElm(TYPE *dst, const int size, const ELEM &elm){
		for (int i = 0; i < size; i++){
			dst[i] = elm;
		}
	}

	// addition
	template <typename TYPE, typename TYPE0, typename ELEM>
	SP_GENFUNC void addElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i] + elm;
		}
	}

	// subtraction
	template <typename TYPE, typename TYPE0, typename ELEM>
	SP_GENFUNC void subElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i] - elm;
		}
	}

	// multiple
	template <typename TYPE, typename TYPE0, typename ELEM>
	SP_GENFUNC void mulElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i] * elm;
		}
	}

	// division
	template <typename TYPE, typename TYPE0, typename ELEM>
	SP_GENFUNC void divElm(TYPE *dst, const int size, const TYPE0 *mem0, const ELEM &elm){
		if (elm == 0.0) return;
		for (int i = 0; i < size; i++){
			dst[i] = mem0[i] / elm;
		}
	}


	//--------------------------------------------------------------------------------
	// matrix
	//--------------------------------------------------------------------------------

	// identity matrix
	SP_GENFUNC void eyeMat(double *dst, const int rows, const int cols){
		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				dst[r * cols + c] = (r == c) ? 1.0 : 0.0;
			}
		}
	}

	// zero matrix
	SP_GENFUNC void zeroMat(double *dst, const int rows, const int cols){
		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				dst[r * cols + c] = 0.0;
			}
		}
	}

	// extension matrix
	SP_GENFUNC void extMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0){
		eyeMat(dst, rows, cols);

		for (int r = 0; r < minVal(rows, rows0); r++){
			for (int c = 0; c < minVal(cols, cols0); c++){
				dst[r * cols + c] = mat0[r * cols0 + c];
			}
		}
	}

	// multiple
	SP_GENFUNC void mulMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0, const double *mat1, const int rows1, const int cols1){

		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				double &d = dst[r * cols + c];
				d = 0;
				for (int i = 0; i < cols0; i++){
					d += mat0[r * cols0 + i] * mat1[i * cols1 + c];
				}
			}
		}
	}


	// transpose
	SP_GENFUNC void trnMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0){
		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				dst[r * cols + c] = mat0[c * cols0 + r];
			}
		}
	}

	// covariance
	SP_GENFUNC void covMat(double *dst, const int rows, const int cols, const double *mat0, const int rows0, const int cols0){
		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				double &d = dst[r * cols + c];
				d = 0;
				for (int i = 0; i < rows0; i++){
					d += mat0[i * cols0 + r] * mat0[i * cols0 + c];
				}
			}
		}
	}

	// skew
	SP_GENFUNC void skewMat(double *dst, const int rows, const int cols, const Vec3 &vec){
		dst[0 * 3 + 0] = 0.0;
		dst[0 * 3 + 1] = -vec.z;
		dst[0 * 3 + 2] = +vec.y;

		dst[1 * 3 + 0] = +vec.z;
		dst[1 * 3 + 1] = 0.0;
		dst[1 * 3 + 2] = -vec.x;

		dst[2 * 3 + 0] = -vec.y;
		dst[2 * 3 + 1] = +vec.x;
		dst[2 * 3 + 2] = 0.0;
	}

	// norm
	SP_GENFUNC double normMat(const double *mat, const int rows, const int cols, const double *base = NULL){
		double norm = 0.0;
		for (int i = 0; i < rows * cols; i++){
			norm += (base == NULL) ? square(mat[i]) : square(mat[i] - base[i]);
		}
		return sqrt(norm);
	}


	//--------------------------------------------------------------------------------
	// matrix determinant
	//--------------------------------------------------------------------------------
	
	SP_GENFUNC double detMat22(const double *mat){
		return mat[0 * 2 + 0] * mat[1 * 2 + 1] - mat[0 * 2 + 1] * mat[1 * 2 + 0];
	}

	SP_GENFUNC double detMat33(const double *mat){
		const double v0 = mat[0 * 3 + 0] * (mat[1 * 3 + 1] * mat[2 * 3 + 2] - mat[2 * 3 + 1] * mat[1 * 3 + 2]);
		const double v1 = mat[0 * 3 + 1] * (mat[1 * 3 + 0] * mat[2 * 3 + 2] - mat[2 * 3 + 0] * mat[1 * 3 + 2]);
		const double v2 = mat[0 * 3 + 2] * (mat[1 * 3 + 0] * mat[2 * 3 + 1] - mat[2 * 3 + 0] * mat[1 * 3 + 1]);

		return v0 - v1 + v2;
	}

	SP_GENFUNC double detMat(const double *mat, const int rows, const int cols, double *buf){

		if (rows != cols) return 0.0;
		const int size = rows;

		for (int r = 0; r < size; r++){
			for (int c = 0; c < size; c++){
				buf[r * size + c] = mat[r * size + c];
			}
		}

		double dst = 1.0;
		for (int i = 0; i < size; i++){
			if (i == size - 1){
				dst *= buf[i * size + i];
				continue;
			}

			// partial pivoting
			{
				int pivot = i;
				double maxVal = 0.0;
				for (int r = i; r < size; r++){
					const double val = fabs(buf[r * size + i]);
					if (val > maxVal){
						maxVal = val;
						pivot = r;
					}
				}

				if (pivot > i){
					for (int c = 0; c < size; c++){
						swap(buf[i * size + c], buf[pivot * size + c]);
					}
					dst *= -1.0;
				}
			}

			// div
			{
				const double div = buf[i * size + i];
				if (fabs(div) < SP_SMALL) return 0.0;

				dst *= div;
				for (int c = i; c < size; c++){
					buf[i * size + c] /= div;
				}

				for (int r = i + 1; r < size; r++){
					const double scl = buf[r * size + i];
					for (int c = i + 1; c < size; c++){
						buf[r * size + c] -= buf[i * size + c] * scl;
					}
				}
			}
		}
		return dst;
	}

	//--------------------------------------------------------------------------------
	// matrix inverse
	//--------------------------------------------------------------------------------
	
	SP_GENFUNC bool invMat22(double *dst, const double *mat){
		const double det = detMat22(mat);
		if (fabs(det) < SP_SMALL) return false;

		dst[0 * 2 + 0] = +mat[1 * 2 + 1];
		dst[0 * 2 + 1] = -mat[0 * 2 + 1];
		dst[1 * 2 + 0] = -mat[1 * 2 + 0];
		dst[1 * 2 + 1] = +mat[0 * 2 + 0];

		mulElm(dst, 2 * 2, dst, 1.0 / det);
		return true;
	}

	SP_GENFUNC bool invMat33(double *dst, const double *mat){
		const double det = detMat33(mat);
		if (fabs(det) < SP_SMALL) return false;

		dst[0 * 3 + 0] = +(mat[1 * 3 + 1] * mat[2 * 3 + 2] - mat[1 * 3 + 2] * mat[2 * 3 + 1]);
		dst[0 * 3 + 1] = -(mat[0 * 3 + 1] * mat[2 * 3 + 2] - mat[0 * 3 + 2] * mat[2 * 3 + 1]);
		dst[0 * 3 + 2] = +(mat[0 * 3 + 1] * mat[1 * 3 + 2] - mat[0 * 3 + 2] * mat[1 * 3 + 1]);

		dst[1 * 3 + 0] = -(mat[1 * 3 + 0] * mat[2 * 3 + 2] - mat[1 * 3 + 2] * mat[2 * 3 + 0]);
		dst[1 * 3 + 1] = +(mat[0 * 3 + 0] * mat[2 * 3 + 2] - mat[0 * 3 + 2] * mat[2 * 3 + 0]);
		dst[1 * 3 + 2] = -(mat[0 * 3 + 0] * mat[1 * 3 + 2] - mat[0 * 3 + 2] * mat[1 * 3 + 0]);

		dst[2 * 3 + 0] = +(mat[1 * 3 + 0] * mat[2 * 3 + 1] - mat[1 * 3 + 1] * mat[2 * 3 + 0]);
		dst[2 * 3 + 1] = -(mat[0 * 3 + 0] * mat[2 * 3 + 1] - mat[0 * 3 + 1] * mat[2 * 3 + 0]);
		dst[2 * 3 + 2] = +(mat[0 * 3 + 0] * mat[1 * 3 + 1] - mat[0 * 3 + 1] * mat[1 * 3 + 0]);

		mulElm(dst, 3 * 3, dst, 1.0 / det);
		return true;
	}

	SP_GENFUNC bool invMat(double *dst, const double *mat, const int rows, const int cols, double *buf){

		if (rows != cols) return false;
		const int size = rows;

		for (int r = 0; r < size; r++){
			for (int c = 0; c < size; c++){
				dst[r * size + c] = (r == c) ? 1.0 : 0.0;
				buf[r * size + c] = mat[r * size + c];
			}
		}

		// Gauss-Jordan
		for (int i = 0; i < size; i++){
			// partial pivoting
			{
				int pivot = i;
				double maxVal = 0.0;
				for (int r = i; r < size; r++){
					const double val = fabs(buf[r * size + i]);
					if (val > maxVal){
						maxVal = val;
						pivot = r;
					}
				}

				if (pivot > i){
					for (int c = 0; c < size; c++){
						swap(dst[i * size + c], dst[pivot * size + c]);
						swap(buf[i * size + c], buf[pivot * size + c]);
					}
				}
			}

			// div
			{
				const double div = buf[i * size + i];
				if (fabs(div) < SP_SMALL) return false;

				for (int c = 0; c < size; c++){
					dst[i * size + c] /= div;
					buf[i * size + c] /= div;
				}

				for (int r = 0; r < size; r++){
					if (r == i) continue;

					const double scl = buf[r * size + i];
					for (int c = 0; c < size; c++){
						dst[r * size + c] -= dst[i * size + c] * scl;
						buf[r * size + c] -= buf[i * size + c] * scl;
					}
				}
			}
		}
		return true;
	}


	//--------------------------------------------------------------------------------
	// matrix eigen
	//--------------------------------------------------------------------------------

	SP_GENFUNC bool eigMat(double *eigVec, double *eigVal, const double *mat, const int rows, const int cols, const bool minOrder = true){
	
		if (rows < 2 || cols < 2 || rows != cols) return false;

		const int size = rows;

		for (int r = 0; r < size; r++){
			for (int c = 0; c < size; c++){
				eigVec[r * size + c] = (r == c) ? 1.0 : 0.0;
				eigVal[r * size + c] = mat[r * size + c];
			}
		}

		const int maxit = size * size;

		// jacobi algorithm
		for (int it = 0; it < maxit; it++){
			int p = 0, q = 0;
			double maxv = 0.0;

			for (int r = 0; r < size; r++){
				for (int c = r + 1; c < size; c++){

					const double val = fabs(eigVal[r * size + c]);
					if (val > maxv){
						maxv = val;
						p = r;
						q = c;
					}
				}
			}
			if (maxv < SP_SMALL) break;

			const double app = eigVal[p * size + p];
			const double apq = eigVal[p * size + q];
			const double aqq = eigVal[q * size + q];

			double sinv, cosv;
			{
				const double a = (app - aqq) / 2.0;
				const double b = -apq;

				// g = cos(2A) = |a| / sqrt(a * a + b * b)
				const double g = fabs(a) / pythag(a, b);

				sinv = sqrt((1.0 - g) / 2.0) * sign(a * b);
				cosv = sqrt((1.0 + g) / 2.0);
			}

			for (int i = 0; i < size; i++){
				if (i == p || i == q) continue;
				const double tmpa = cosv * eigVal[p * size + i] - sinv * eigVal[q * size + i];
				const double tmpb = sinv * eigVal[p * size + i] + cosv * eigVal[q * size + i];

				eigVal[i * size + p] = eigVal[p * size + i] = tmpa;
				eigVal[i * size + q] = eigVal[q * size + i] = tmpb;
			}
			{
				eigVal[p * size + p] = cosv * cosv * app + sinv * sinv * aqq - 2 * sinv * cosv * apq;
				eigVal[p * size + q] = sinv * cosv * (app - aqq) + (cosv * cosv - sinv * sinv) * apq;
				eigVal[q * size + p] = sinv * cosv * (app - aqq) + (cosv * cosv - sinv * sinv) * apq;
				eigVal[q * size + q] = sinv * sinv * app + cosv * cosv * aqq + 2 * sinv * cosv * apq;
			}

			for (int i = 0; i < size; i++){
				const double tmpa = cosv * eigVec[i * size + p] - sinv * eigVec[i * size + q];
				const double tmpb = sinv * eigVec[i * size + p] + cosv * eigVec[i * size + q];

				eigVec[i * size + p] = tmpa;
				eigVec[i * size + q] = tmpb;
			}
		}

		for (int r = 0; r < size; r++){
			for (int c = 0; c < size; c++){
				if (c == r) continue;

				eigVal[r * size + c] = 0.0;
			}
		}

		// sort
		for (int c = 0; c < size - 1; c++){
			int maxid = c;
			int minid = c;

			double maxv = 0.0;
			double minv = SP_INFINITY;

			for (int i = c; i < size; i++){
				const double val = ::fabs(eigVal[i * size + i]);
				if (val > maxv){
					maxv = val;
					maxid = i;
				}
				if (val < minv){
					minv = val;
					minid = i;
				}
			}

			const int select = (minOrder == true) ? minid : maxid;

			if (select != c){
				swap(eigVal[c * size + c], eigVal[select * size + select]);
				for (int r = 0; r < size; r++){
					swap(eigVec[r * size + c], eigVec[r * size + select]);
				}
			}
		}

		return true;
	}


	//--------------------------------------------------------------------------------
	// matrix svd (simgular value decomposition)
	//--------------------------------------------------------------------------------

	SP_GENFUNC bool svdMat(double *U, double *S, double *V, const double *mat, const int rows, const int cols, const bool minOrder = true){
		if (rows < 2 || cols < 2 || rows < cols) return false;

		for (int i = 0; i < rows * cols; i++){
			U[i] = mat[i];
		}
		for (int i = 0; i < cols * cols; i++){
			V[i] = 0.0;
			S[i] = 0.0;
		}
		double *Q = &S[0];
		double *R = &S[cols];

		// householder reduction to bidiagonal form
		for (int i = 0; i < cols; i++) {

			{
				double scale = 0.0;
				for (int r = i; r < rows; r++){
					scale += fabs(U[r * cols + i]);
				}

				double val = 0.0;
				if (scale > 0.0) {
					double s = 0.0;
					for (int r = i; r < rows; r++) {
						U[r * cols + i] /= scale;
						s += U[r * cols + i] * U[r * cols + i];
					}

					double f = U[i * cols + i];
					double g = -sign(f) * sqrt(s);
					U[i * cols + i] = f - g;

					double h = f * g - s;

					for (int j = i + 1; j < cols; j++) {
						s = 0.0;
						for (int r = i; r < rows; r++){
							s += U[r * cols + i] * U[r * cols + j];
						}
						f = s / h;
						for (int r = i; r < rows; r++){
							U[r * cols + j] += f * U[r * cols + i];
						}
					}
					for (int r = i; r < rows; r++){
						U[r * cols + i] *= scale;
					}
					val = scale * g;
				}
				Q[i] = val;
			}

			if (i < cols - 1) {
				double scale = 0.0;
				for (int c = i + 1; c < cols; c++){
					scale += fabs(U[i * cols + c]);
				}

				double val = 0.0;
				if (scale > 0.0){
					double s = 0.0;

					for (int c = i + 1; c < cols; c++) {
						U[i * cols + c] /= scale;
						s += U[i * cols + c] * U[i * cols + c];
					}
					double f = U[i * cols + i + 1];
					double g = -sign(f) * sqrt(s);
					double h = f * g - s;
					U[i * cols + i + 1] = f - g;

					for (int c = i + 1; c < cols; c++){
						R[c] = U[i * cols + c] / h;
					}

					for (int j = i + 1; j < rows; j++) {
						s = 0.0;
						for (int c = i + 1; c < cols; c++){
							s += U[j * cols + c] * U[i * cols + c];
						}
						for (int c = i + 1; c < cols; c++){
							U[j * cols + c] += s * R[c];
						}
					}
					for (int c = i + 1; c < cols; c++){
						U[i * cols + c] *= scale;
					}
					val = scale * g;
				}
				R[i + 1] = val;
			}
		}

		double unorm = 0.0;
		for (int i = 0; i < cols; i++) {
			unorm = maxVal(unorm, fabs(Q[i]) + fabs(R[i]));

		}

		// accumulation of right-hand transformations
		for (int i = cols - 1; i >= 0; i--) {
			const double g = R[i + 1];
			if (i < cols - 1){
				if (g){

					// double division to avoid possible underflow
					for (int j = i + 1; j < cols; j++){
						V[j * cols + i] = (U[i * cols + j] / U[i * cols + (i + 1)]) / g;
					}
					for (int j = i + 1; j < cols; j++) {
						double s = 0.0;
						for (int k = i + 1; k < cols; k++){
							s += U[i * cols + k] * V[k * cols + j];
						}
						for (int k = i + 1; k < cols; k++){
							V[k * cols + j] += s * V[k * cols + i];
						}
					}
				}
				for (int j = i + 1; j < cols; j++){
					V[i * cols + j] = V[j * cols + i] = 0.0;
				}
			}
			V[i * cols + i] = 1.0;
		}

		// accumulation of left-hand transformations
		for (int i = minVal(rows, cols) - 1; i >= 0; i--){ 
			
			for (int j = i + 1; j < cols; j++){
				U[i * cols + j] = 0.0;
			}

			double g = Q[i];
			if (g) {
				g = 1.0 / g;
				for (int j = i + 1; j < cols; j++) {
					double s = 0.0;
					for (int k = i + 1; k < rows; k++){
						s += U[k * cols + i] * U[k * cols + j];
					}
					double f = (s / U[i * cols + i]) * g;

					for (int k = i; k < rows; k++){
						U[k * cols + j] += f * U[k * cols + i];
					}
				}
				for (int j = i; j < rows; j++){
					U[j * cols + i] *= g;
				}
			}
			else{
				for (int j = i; j < rows; j++){
					U[j * cols + i] = 0.0;
				}
			}
			U[i * cols + i]++;
		}

		// diagonalization of the bidiagonal form
		const int maxit = 30;

		for (int k = cols - 1; k >= 0; k--){

			for (int it = 0; it < maxit; it++) {
				int flag = 1;

				int l = 0;
				int n = 0;

				// test for splitting
				for (l = k; l >= 0; l--) { 

					// Note that buf[0] is always zero
					n = l - 1; 
					if ((fabs(R[l]) + unorm) == unorm) {
						flag = 0;
						break;
					}
					if ((fabs(Q[n]) + unorm) == unorm){
						break;
					}
				}
				if (flag) {
					// cancellation of R[l], if l > 0
					double c = 0.0;
					double s = 1.0;
					for (int i = l; i <= k; i++) {
						double f = s * R[i];
						R[i] = c * R[i];

						if ((double)(fabs(f) + unorm) == unorm){
							break;
						}
						double g = Q[i];
						double h = pythag(f, g);
						Q[i] = h;
						h = 1.0 / h;
						c = g * h;
						s = -f * h;
						for (int j = 0; j < rows; j++) {
							const double y = U[j * cols + n];
							const double z = U[j * cols + i];
							U[j * cols + n] = y * c + z * s;
							U[j * cols + i] = z * c - y * s;
						}
					}
				}
				double z = Q[k];

				// convergence
				if (l == k) {
					// singular value is made nonnegative
					if (z < 0.0){
						Q[k] = -z;
						for (int j = 0; j < cols; j++){
							V[j * cols + k] = -V[j * cols + k];
						}
					}
					break;
				}

				if (it == maxit - 1){
					SP_PRINTD("no convergence in SVD iterations");
				}

				// shift from bottom 2-by-2 minor
				n = k - 1;
				double x = Q[l];
				double y = Q[n];
				double g = R[n];
				double h = R[k];
				double f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
				g = pythag(f, 1.0);
				f = ((x - z)*(x + z) + h*((y / (f + sign(f) * g)) - h)) / x;
				
				double c = 1.0;
				double s = 1.0;

				// next QR transformation
				for (int j = l; j <= n; j++) {
					int i = j + 1;
					g = R[i];
					y = Q[i];
					h = s*g;
					g = c*g;
					z = pythag(f, h);
					R[j] = z;
					c = f / z;
					s = h / z;
					f = x*c + g*s;
					g = g*c - x*s;
					h = y*s;
					y *= c;
					for (int jj = 0; jj < cols; jj++) {
						x = V[jj * cols + j];
						z = V[jj * cols + i];
						V[jj * cols + j] = x*c + z*s;
						V[jj * cols + i] = z*c - x*s;
					}
					z = pythag(f, h);
					Q[j] = z; /* Rotation can be arbitrary if z = 0. */
					if (z) {
						z = 1.0 / z;
						c = f*z;
						s = h*z;
					}
					f = c*g + s*y;
					x = c*y - s*g;
					for (int jj = 0; jj < rows; jj++) {
						y = U[jj * cols + j];
						z = U[jj * cols + i];
						U[jj * cols + j] = y*c + z*s;
						U[jj * cols + i] = z*c - y*s;
					}
				}
				R[l] = 0.0;
				R[k] = f;
				Q[k] = x;
			}
		}

		for (int i = 0; i < cols; i++){
			R[i] = 0.0;
		}
		for (int i = 0; i < cols; i++){
			swap(S[i * cols + i], Q[i]);
		}

		// sort
		for (int c = 0; c < cols - 1; c++){
			int maxid = c;
			int minid = c;

			double maxv = 0.0;
			double minv = SP_INFINITY;

			for (int i = c; i < cols; i++){
				const double val = S[i * cols + i];
				if (val > maxv){
					maxv = val;
					maxid = i;
				}
				if (val < minv){
					minv = val;
					minid = i;
				}
			}

			const int select = (minOrder == true) ? minid : maxid;

			if (select != c){
				swap(S[c * cols + c], S[select * cols + select]);
				for (int r = 0; r < rows; r++){
					swap(U[r * cols + c], U[r * cols + select]);
				}
				for (int r = 0; r < cols; r++){
					swap(V[r * cols + c], V[r * cols + select]);
				}
			}
		}
		return true;
	}

}

#endif
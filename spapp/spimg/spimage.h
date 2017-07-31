//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_IMAGE_H__
#define __SP_IMAGE_H__

#include "spcore/spcore.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// rescale 
	//--------------------------------------------------------------------------------

	template <typename TYPE, typename ELEM = TYPE>
	SP_CPUCALL void rescale(Mem<TYPE> &dst, const Mem<TYPE> &src, const double dscale0, const double dscale1){
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
	SP_CPUCALL void rescale(Mem<TYPE> &dst, const Mem<TYPE> &src){
		SP_ASSERT(isValid(2, src));

		const double dscale0 = static_cast<double>(dst.dsize[0]) / src.dsize[0];
		const double dscale1 = static_cast<double>(dst.dsize[1]) / src.dsize[1];

		rescale<TYPE, ELEM>(dst, src, dscale0, dscale1);
	}

	SP_CPUCALL void rescale(CamParam &dst, const CamParam &cam, const double dscale0, const double dscale1) {
		dst = cam;

		dst.dsize[0] = round(cam.dsize[0] * dscale0);
		dst.dsize[1] = round(cam.dsize[1] * dscale1);

		dst.fx *= dscale0;
		dst.fy *= dscale1;

		dst.cx *= dscale0;
		dst.cy *= dscale1;
	}


	//--------------------------------------------------------------------------------
	// pyramid down 
	//--------------------------------------------------------------------------------

	template <typename TYPE, typename ELEM = TYPE>
	SP_CPUCALL void pyrdown(Mem<TYPE> &dst, const Mem<TYPE> &src){
		SP_ASSERT(isValid(2, src));

		const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

		const int ch = sizeof(TYPE) / sizeof(ELEM);
		const int dsize0 = (tmp.dsize[0] + 1) / 2;
		const int dsize1 = (tmp.dsize[1] + 1) / 2;

		const int dsize[2] = { dsize0, dsize1 };
		dst.resize(2, dsize);

		for (int v = 0; v < dst.dsize[1]; v++){
			for (int u = 0; u < dst.dsize[0]; u++){
				const double su = 2 * u;
				const double sv = 2 * v;

				for (int c = 0; c < ch; c++){
					double sum = 0.0;
					sum += acs2<TYPE, ELEM>(tmp, su - 1, sv - 1, c) * 1.0;
					sum += acs2<TYPE, ELEM>(tmp, su + 0, sv - 1, c) * 2.0;
					sum += acs2<TYPE, ELEM>(tmp, su + 1, sv - 1, c) * 1.0;

					sum += acs2<TYPE, ELEM>(tmp, su - 1, sv + 0, c) * 2.0;
					sum += acs2<TYPE, ELEM>(tmp, su + 0, sv + 0, c) * 4.0;
					sum += acs2<TYPE, ELEM>(tmp, su + 1, sv + 0, c) * 2.0;

					sum += acs2<TYPE, ELEM>(tmp, su - 1, sv + 1, c) * 1.0;
					sum += acs2<TYPE, ELEM>(tmp, su + 0, sv + 1, c) * 2.0;
					sum += acs2<TYPE, ELEM>(tmp, su + 1, sv + 1, c) * 1.0;

					cnvVal(acs2<TYPE, ELEM>(dst, u, v, c), sum / 16.0);
				}
			}
		}
	}

	SP_CPUCALL void pyrdown(CamParam &dst, const CamParam &cam){
		rescale(dst, cam, 0.5, 0.5);
	}


	//--------------------------------------------------------------------------------
	// crop 
	//--------------------------------------------------------------------------------

	template <typename TYPE, typename ELEM = TYPE>
	SP_CPUCALL void crop(Mem<TYPE> &dst, const Mem<TYPE> &src, const Rect &rect, const double angle = 0.0){
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
	SP_CPUCALL void merge(Mem<TYPE> &dst, const Mem<TYPE> &src0, const Mem<TYPE> &src1, const bool horizon = true){
		SP_ASSERT(isValid(2, src0));
		SP_ASSERT(isValid(2, src1));

		const int dsize0 = (horizon == true) ? src0.dsize[0] + src1.dsize[0] : maxVal(src0.dsize[0], src1.dsize[0]);
		const int dsize1 = (horizon == true) ? maxVal(src0.dsize[1], src1.dsize[1]) : src0.dsize[1] + src1.dsize[1];
		
		const int dsize[2] = { dsize0, dsize1 };
		dst.resize(2, dsize);
		dst.zero();

		for (int v = 0; v < src0.dsize[1]; v++){
			for (int u = 0; u < src0.dsize[0]; u++){
				acs2(dst, u, v) = acs2(src0, u, v);
			}
		}

		const int offsetX = (horizon == true) ? src0.dsize[0] : 0;
		const int offsetY = (horizon == true) ? 0 : src0.dsize[1];

		for (int v = 0; v < src1.dsize[1]; v++){
			for (int u = 0; u < src1.dsize[0]; u++){
				acs2(dst, u + offsetX, v + offsetY) = acs2(src1, u, v);
			}
		}
	}


	//--------------------------------------------------------------------------------
	// invert
	//--------------------------------------------------------------------------------

	template<typename TYPE, typename ELEM = TYPE>
	SP_CPUCALL void invert(Mem<TYPE> &dst, const Mem<TYPE> &src) {
		SP_ASSERT(isValid(2, src));

		dst.resize(2, src.dsize);

		const int ch = sizeof(TYPE) / sizeof(ELEM);

		for (int v = 0; v < dst.dsize[1]; v++) {
			for (int u = 0; u < dst.dsize[0]; u++) {
				for (int c = 0; c < ch; c++) {
					acs2<TYPE, ELEM>(dst, u, v, c) = SP_BYTEMAX - acs2<TYPE, ELEM>(src, u, v, c);
				}
			}
		}
	}


	//--------------------------------------------------------------------------------
	// remap
	//--------------------------------------------------------------------------------

	template<typename TYPE, typename ELEM = TYPE>
	SP_CPUCALL void remap(Mem<TYPE> &dst, const Mem<TYPE> &src, const Mem<Vec2> &table, const bool useExt = false){
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
	SP_CPUCALL void warp(Mem<TYPE> &dst, const Mem<TYPE> &src, const Mat &mat){
		SP_ASSERT(isValid(2, src));

		const Mem<TYPE> &tmp = (&dst != &src) ? src : clone(src);

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
	SP_CPUCALL void cnvImg(Mem<TYPE0> &dst, const Mem<TYPE1> &src){
		SP_ASSERT(isValid(2, src));

		dst.resize(2, src.dsize);

		for (int i = 0; i < dst.size(); i++){
			cnvImg(dst[i], src[i]);
		}
	}

	SP_CPUCALL void cnvImgToGry(Mem<Byte> &dst, const Mem<Col3> &src) {
		SP_ASSERT(isValid(2, src));

		dst.resize(2, src.dsize);

		for (int i = 0; i < dst.size(); i++) {
			cnvImg(dst[i], src[i]);
		}
	}

	SP_CPUCALL void cnvImgToHSV(Mem<Vec3> &dst, const Mem<Col3> &src) {
		SP_ASSERT(isValid(2, src));

		dst.resize(2, src.dsize);

		for (int i = 0; i < dst.size(); i++) {
			cnvColToHSV(dst[i], src[i]);
		}
	}
		
	template <typename TYPE0, typename TYPE1>
	SP_CPUCALL void cnvDepthToImg(Mem<TYPE0> &dst, const Mem<TYPE1> &src, const double nearPlane = 100.0, const double farPlane = 10000.0){
		SP_ASSERT(isValid(2, src));

		dst.resize(2, src.dsize);
		dst.zero();

		for (int i = 0; i < dst.size(); i++){
			const double depth = extractDepth(src[i]);

			if (depth >= nearPlane && depth <= farPlane){
				cnvDepthToImg(dst[i], depth, nearPlane, farPlane);
			}
		}
	}

	template <typename TYPE>
	SP_CPUCALL void cnvNormalToImg(Mem<TYPE> &dst, const Mem<VecVN3> &src, const double nearPlane = 100.0, const double farPlane = 10000.0){
		SP_ASSERT(isValid(2, src));

		dst.resize(2, src.dsize);
		dst.zero();

		for (int i = 0; i < dst.size(); i++){
			const double depth = extractDepth(src[i]);

			if (depth >= nearPlane && depth <= farPlane){
				cnvNormalToImg(dst[i], src[i].nrm);
			}
		}
	}

	SP_CPUCALL void cnvLabelToImg(Mem<Col3> &dst, const Mem<int> &src){
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
	SP_CPUCALL void cnvPtrToImg(Mem<TYPE> &dst, const void *src, const int dsize0, const int dsize1, const int ch){

		switch (ch){
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
		default:
			dst.clear();
			break;
		}
	}

	template<typename TYPE>
	SP_CPUCALL void cnvImgToPtr(void *dst, const Mem<TYPE> &src, const int ch){
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
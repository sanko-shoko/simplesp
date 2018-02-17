//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_INTEGRAL_H__
#define __SP_INTEGRAL_H__

#include "spcore/spcore.h"


namespace sp{

	template<typename TYPE0, typename TYPE1, typename ELEM0 = TYPE0, typename ELEM1 = TYPE1>
	SP_CPUFUNC void makeIntegral(Mem<TYPE0> &sum, const Mem<TYPE1> &src) {
		SP_ASSERT((sizeof(TYPE0) / sizeof(ELEM0)) == (sizeof(TYPE1) / sizeof(ELEM1)));
		SP_ASSERT(isValid(2, src));

		sum.resize(2, src.dsize);

		const int ch = sizeof(TYPE0) / sizeof(ELEM0);

		for (int v = 0; v < sum.dsize[1]; v++) {
			for (int u = 0; u < sum.dsize[0]; u++) {
				for (int c = 0; c < ch; c++) {
					ELEM0 s = acs2<TYPE1, ELEM1>(src, u, v, c);
					if (u > 0) s += acs2<TYPE0, ELEM0>(sum, u - 1, v, c);
					if (v > 0) s += acs2<TYPE0, ELEM0>(sum, u, v - 1, c);
					if (u > 0 && v > 0) s -= acs2<TYPE0, ELEM0>(sum, u - 1, v - 1, c);

					acs2<TYPE0, ELEM0>(sum, u, v, c) = s;
				}
			}
		}
	}

	template <typename TYPE0, typename TYPE1, typename ELEM0 = TYPE0, typename ELEM1 = TYPE1>
	SP_CPUFUNC void boxFilterIntegral(Mem<TYPE0> &dst, const Mem<TYPE1> &src, const int winSize) {
		SP_ASSERT((sizeof(TYPE0) / sizeof(ELEM0)) == (sizeof(TYPE1) / sizeof(ELEM1)));
		SP_ASSERT(isValid(2, src));

		dst.resize(2, src.dsize);

		const int offset = winSize / 2;

		const int ch = sizeof(TYPE0) / sizeof(ELEM0);

		for (int v = 0; v < dst.dsize[1]; v++) {
			for (int u = 0; u < dst.dsize[0]; u++) {
				int x = u - offset;
				int y = v - offset;

				int xs = (x > 0) ? x : 0;
				int ys = (y > 0) ? y : 0;

				int xe = (x + winSize < dst.dsize[0]) ? x + winSize : dst.dsize[0];
				int ye = (y + winSize < dst.dsize[1]) ? y + winSize : dst.dsize[1];

				double div = (xe - xs) * (ye - ys);

				for (int c = 0; c < ch; c++) {
					ELEM1 sum = acs2<TYPE1, ELEM1>(src, xe - 1, ye - 1, c);
					if (xs > 0) sum -= acs2<TYPE1, ELEM1>(src, xs - 1, ye - 1, c);
					if (ys > 0) sum -= acs2<TYPE1, ELEM1>(src, xe - 1, ys - 1, c);
					if (xs > 0 && ys > 0) sum += acs2<TYPE1, ELEM1>(src, xs - 1, ys - 1, c);

					cnvVal(acs2<TYPE0, ELEM0>(dst, u, v, c), sum / div);
				}
			}
		}

	}
}

#endif
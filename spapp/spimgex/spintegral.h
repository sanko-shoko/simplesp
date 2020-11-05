//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_INTEGRAL_H__
#define __SP_INTEGRAL_H__

#include "spcore/spcore.h"


namespace sp{

    template<typename DST, typename SRC, typename DST_ELEM = DST, typename SRC_ELEM = SRC>
    SP_CPUFUNC void makeIntegral(Mem<DST> &sum, const Mem<SRC> &src) {
        SP_ASSERT((sizeof(DST) / sizeof(DST_ELEM)) == (sizeof(SRC) / sizeof(SRC_ELEM)));

        sum.resize(2, src.dsize);

        const int ch = sizeof(DST) / sizeof(DST_ELEM);

        for (int v = 0; v < sum.dsize[1]; v++) {
            for (int u = 0; u < sum.dsize[0]; u++) {
                for (int c = 0; c < ch; c++) {
                    DST_ELEM s = acs2<SRC, SRC_ELEM>(src, u, v, c);
                    if (u > 0) s += acs2<DST, DST_ELEM>(sum, u - 1, v, c);
                    if (v > 0) s += acs2<DST, DST_ELEM>(sum, u, v - 1, c);
                    if (u > 0 && v > 0) s -= acs2<DST, DST_ELEM>(sum, u - 1, v - 1, c);

                    acs2<DST, DST_ELEM>(sum, u, v, c) = s;
                }
            }
        }
    }

    template <typename DST, typename SRC, typename DST_ELEM = DST, typename SRC_ELEM = SRC>
    SP_CPUFUNC void boxFilterIntegral(Mem<DST> &dst, const Mem<SRC> &src, const int winSize) {
        SP_ASSERT((sizeof(DST) / sizeof(DST_ELEM)) == (sizeof(SRC) / sizeof(SRC_ELEM)));

        dst.resize(2, src.dsize);

        const int offset = winSize / 2;

        const int ch = sizeof(DST) / sizeof(DST_ELEM);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                int x = u - offset;
                int y = v - offset;

                int xs = (x > 0) ? x : 0;
                int ys = (y > 0) ? y : 0;

                int xe = (x + winSize < dst.dsize[0]) ? x + winSize : dst.dsize[0];
                int ye = (y + winSize < dst.dsize[1]) ? y + winSize : dst.dsize[1];

                SP_REAL div = (xe - xs) * (ye - ys);

                for (int c = 0; c < ch; c++) {
                    SRC_ELEM sum = acs2<SRC, SRC_ELEM>(src, xe - 1, ye - 1, c);
                    if (xs > 0) sum -= acs2<SRC, SRC_ELEM>(src, xs - 1, ye - 1, c);
                    if (ys > 0) sum -= acs2<SRC, SRC_ELEM>(src, xe - 1, ys - 1, c);
                    if (xs > 0 && ys > 0) sum += acs2<SRC, SRC_ELEM>(src, xs - 1, ys - 1, c);

                    acs2<DST, DST_ELEM>(dst, u, v, c) = cast<DST_ELEM>(sum / div);
                }
            }
        }

    }
}

#endif
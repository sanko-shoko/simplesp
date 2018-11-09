//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BLOB_H__
#define __SP_BLOB_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // blob filter
    //--------------------------------------------------------------------------------

    template <typename TYPE, typename TYPE0>
    SP_CPUFUNC void blobFilter(Mem<TYPE> &dst, const Mem<TYPE0> &src) {
        SP_ASSERT(isValid(2, src));

        dst.resize(2, src.dsize);

        const int w = src.dsize[0];
        const int h = src.dsize[1];

        const Mem<TYPE0> &tmp = (reinterpret_cast<const Mem<TYPE0>* >(&dst) != &src) ? src : clone(src);

        const TYPE0 *psrc = tmp.ptr;
        TYPE *pdst = dst.ptr;

        for (int v = 0; v < h; v++) {
            const int ys = (v == 0) ? 0 : -1;
            const int yc = 0;
            const int ye = (v == h - 1) ? 0 : +1;

            const int vs = v + ys;
            const int vc = v + yc;
            const int ve = v + ye;

            const TYPE0 *psrc0 = &psrc[(vs - 1) * w];
            const TYPE0 *psrc1 = &psrc[(vc + 0) * w];
            const TYPE0 *psrc2 = &psrc[(ve + 1) * w];

            TYPE *pdst0 = &pdst[(vs - 1) * w];
            TYPE *pdst1 = &pdst[(vc + 0) * w];
            TYPE *pdst2 = &pdst[(ve + 1) * w];

            const double nrm = 1.0 / square(8 * SP_BYTEMAX);

            for (int u = 0; u < w; u++) {
                const int xs = (u == 0) ? 0 : -1;
                const int xc = 0;
                const int xe = (u == w - 1) ? 0 : +1;

                const TYPE0 a00 = psrc0[xs];
                const TYPE0 a01 = psrc0[xc];
                const TYPE0 a02 = psrc0[xe];

                const TYPE0 a10 = psrc1[xs];
                const TYPE0 a11 = psrc1[xc];
                const TYPE0 a12 = psrc1[xe];

                const TYPE0 a20 = psrc2[xs];
                const TYPE0 a21 = psrc2[xc];
                const TYPE0 a22 = psrc2[xe];

                const double dxx = 2 * (a10 + 2 * a11 + a12) - (a00 + 2 * a01 + a02) - (a20 + 2 * a21 + a22);
                const double dyy = 2 * (a01 + 2 * a11 + a21) - (a00 + 2 * a10 + a20) - (a02 + 2 * a12 + a22);

                const double dxy = 4 * (a00 + a22) - 4 * (a02 + a20);

                const double s = nrm * (dxx * dyy - 0.9 * dxy * dxy);

                cnvVal(*pdst++, s);

                psrc0++;
                psrc1++;
                psrc2++;

                TYPE b00 = pdst0[xs];
                TYPE b01 = pdst0[xc];
                TYPE b02 = pdst0[xe];
                TYPE b10 = pdst1[xs];

                if (fabs(s) >= fabs(b00)) b00 = 0;
                if (fabs(s) >= fabs(b01)) b01 = 0;
                if (fabs(s) >= fabs(b02)) b02 = 0;
                if (fabs(s) >= fabs(b10)) b10 = 0;
            }
        }
    }

}

#endif
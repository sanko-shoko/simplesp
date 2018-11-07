//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_IMAGEF_H__
#define __SP_IMAGEF_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // rescale 
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void rescaleFast(Mem<Byte> &dst, const Mem<Byte> &src, const double dscale0, const double dscale1) {
        SP_ASSERT(isValid(2, src));

        const Mem<Byte> &tmp = (&dst != &src) ? src : clone(src);

        const int dsize0 = round(tmp.dsize[0] * dscale0);
        const int dsize1 = round(tmp.dsize[1] * dscale1);

        const int dsize[2] = { dsize0, dsize1 };
        dst.resize(2, dsize);

        //const Byte *pSrc = tmp.ptr;
        Byte *pDst = dst.ptr;

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                const double su = u / dscale0;
                const double sv = v / dscale1;

                *pDst++ = static_cast<Byte>(acs2<Byte>(tmp, su, sv) + 0.5);
            }
        }
    }

    SP_CPUFUNC void rescaleFast(Mem<Byte> &dst, const Mem<Byte> &src) {
        SP_ASSERT(isValid(2, src));

        const double dscale0 = static_cast<double>(dst.dsize[0]) / src.dsize[0];
        const double dscale1 = static_cast<double>(dst.dsize[1]) / src.dsize[1];

        rescaleFast(dst, src, dscale0, dscale1);
    }


    //--------------------------------------------------------------------------------
    // pyramid down 
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void pyrdownFast(Mem<Byte> &dst, const Mem<Byte> &src){
        SP_ASSERT(isValid(2, src));

        const Mem<Byte> &tmp = (&dst != &src) ? src : clone(src);

        const int dsize0 = (tmp.dsize[0] + 1) / 2;
        const int dsize1 = (tmp.dsize[1] + 1) / 2;

        const int dsize[2] = { dsize0, dsize1 };
        dst.resize(2, dsize);

        const Mem<short> buf(2, dst.dsize);

        const int tstep = tmp.dsize[0];
        const int dstep = dst.dsize[0];

        const Byte *ptmp = tmp.ptr;
        Byte *pdst = dst.ptr;
        short *pbuf = buf.ptr;

        {
            for (int v = 0; v < dst.dsize[1]; v++) {
                const int sv = 2 * v;

                {
                    const int u = 0;
                    const int su = 2 * u;

                    const Byte a = ptmp[sv * tstep + su + 0];
                    const Byte b = ptmp[sv * tstep + su + 0];
                    const Byte c = ptmp[sv * tstep + su + 1];

                    pbuf[v * dstep + u] = a + 2 * b + c;
                }
                {
                    const int u = dstep - 1;
                    const int su = 2 * u;

                    const Byte a = ptmp[sv * tstep + su - 1];
                    const Byte b = ptmp[sv * tstep + su + 0];
                    const Byte c = ptmp[sv * tstep + su + 0];

                    pbuf[v * dstep + u] = a + 2 * b + c;
                }

                for (int u = 1; u < dst.dsize[0] - 1; u++) {
                    const int su = 2 * u;

                    const Byte a = ptmp[sv * tstep + su - 1];
                    const Byte b = ptmp[sv * tstep + su + 0];
                    const Byte c = ptmp[sv * tstep + su + 1];

                    pbuf[v * dstep + u] = a + 2 * b + c;
                }
            }
        }
        {
            {
                const int v = 0;
                for (int u = 0; u < dst.dsize[0]; u++) {

                    const short a = pbuf[(v + 0) * dstep + u];
                    const short b = pbuf[(v + 0) * dstep + u];
                    const short c = pbuf[(v + 1) * dstep + u];

                    pdst[v * dstep + u] = static_cast<Byte>((a + 2 * b + c) / 16);
                }
            }

            {
                const int v = dstep - 1;
                for (int u = 0; u < dst.dsize[0]; u++) {
            
                    const short a = pbuf[(v - 1) * dstep + u];
                    const short b = pbuf[(v + 0) * dstep + u];
                    const short c = pbuf[(v + 0) * dstep + u];

                    pdst[v * dstep + u] = static_cast<Byte>((a + 2 * b + c) / 16);
                }
            }

            for (int v = 1; v < dst.dsize[1] - 1; v++) {

                for (int u = 0; u < dst.dsize[0]; u++) {

                    const short a = pbuf[(v - 1) * dstep + u];
                    const short b = pbuf[(v + 0) * dstep + u];
                    const short c = pbuf[(v + 1) * dstep + u];

                    pdst[v * dstep + u] = static_cast<Byte>((a + 2 * b + c) / 16);
                }
            }
        }
    }



}

#endif
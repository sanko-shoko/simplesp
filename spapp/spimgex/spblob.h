//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BLOB_H__
#define __SP_BLOB_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spfilter.h"

namespace sp {

    //--------------------------------------------------------------------------------
    // detect blob
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC void detectBlob(Mem1<Vec2> &pixs, const Mem2<TYPE> &img, const float thresh = 0.1f * SP_BYTEMAX) {
        SP_ASSERT(isValid(2, img));

        pixs.reserve(img.size());

        Mem<float> res(2, img.dsize);
        res.zero();

        const int w = img.dsize[0];
        const int h = img.dsize[1];

        const TYPE *pimg = img.ptr;
        float *pres = res.ptr;

        for (int v = 0; v < h; v++) {
            const int ys = (v <= 1) ? -v : -2;
            const int yc = 0;
            const int ye = (v >= h - 2) ? h - 1 - v : +2;

            const int vs = v + ys;
            const int vc = v + yc;
            const int ve = v + ye;

            const TYPE *pimg0 = &pimg[vs * w];
            const TYPE *pimg1 = &pimg[vc * w];
            const TYPE *pimg2 = &pimg[ve * w];

            float *pdst = &pres[vc * w];

            float val[8];

            for (int u = 0; u < w; u++) {
                const int xs = (u <= 1) ? -u : -2;
                const int xc = 0;
                const int xe = (u >= w - 2) ? w - 1 - u : +2;

                const int us = u + xs;
                const int uc = u + xc;
                const int ue = u + xe;

                const TYPE c = pimg1[uc];
                cnvVal(val[0], c - pimg0[us]);
                cnvVal(val[1], c - pimg0[uc]);
                cnvVal(val[2], c - pimg0[ue]);

                cnvVal(val[3], c - pimg1[us]);
                cnvVal(val[4], c - pimg1[ue]);

                cnvVal(val[5], c - pimg2[us]);
                cnvVal(val[6], c - pimg2[uc]);
                cnvVal(val[7], c - pimg2[ue]);

                const float a = (val[0] < val[1]) ? val[0] : val[1];
                const float b = (val[0] > val[1]) ? val[0] : val[1];
                float min0 = a;
                float min1 = b;
                float max0 = b;
                float max1 = a;

                for (int i = 2; i < 8; i++) {
                    if (val[i] < min0) { min1 = min0; min0 = val[i]; }
                    else if (val[i] < min1) { min1 = val[i]; }

                    if (val[i] > max0) { max1 = max0; max0 = val[i]; }
                    else if (val[i] > max1) { max1 = val[i]; }
                }

                float d = 0.f;
                if (min1 > 0) {
                    d = min1;
                }
                if (max1 < 0) {
                    d = max1;
                }
                cnvVal(*pdst++, d);
            }
        }

        const int margin = 1;
        for (int v = margin; v < h - margin; v++) {
            const int ys = (v == 0) ? 0 : -1;
            const int yc = 0;
            const int ye = (v == h - 1) ? 0 : +1;

            const int vs = v + ys;
            const int vc = v + yc;
            const int ve = v + ye;

            const float *pres0 = &pres[vs * w];
            const float *pres1 = &pres[vc * w];
            const float *pres2 = &pres[ve * w];

            for (int u = margin; u < w - margin; u++) {
                const int xs = (u == 0) ? 0 : -1;
                const int xc = 0;
                const int xe = (u == w - 1) ? 0 : +1;

                const int us = u + xs;
                const int uc = u + xc;
                const int ue = u + xe;

                const float d00 = pres0[us];
                const float d01 = pres0[uc];
                const float d02 = pres0[ue];

                const float d10 = pres1[us];
                const float d11 = pres1[uc];
                const float d12 = pres1[ue];

                const float d20 = pres2[us];
                const float d21 = pres2[uc];
                const float d22 = pres2[ue];

                const float d = d11;

                bool m = false;
                if (d > 0) {
                    if (d > d00 && d > d01 && d > d02 && d > d10 && d >= d12 && d >= d20 && d >= d21 && d >= d22) m = true;
                }
                else {
                    if (d < d00 && d < d01 && d < d02 && d < d10 && d <= d12 && d <= d20 && d <= d21 && d <= d22) m = true;
                }

                if (m == true && fabs(d) > thresh) {
                    pixs.push(getVec(u, v));
                }
            }
        }

    }

    template <typename TYPE>
    SP_CPUFUNC Mem1<Vec2> calcBlobDrc(const Mem2<TYPE> &img, const Vec2 &pix, const double sigma, const double pthresh = 0.8) {
        
        Mem1<Vec2> drcs;

        const int BINS = 36;
        Mem1<double> hist(BINS);

        const int x = round(pix.x);
        const int y = round(pix.y);

        // calc direction hist
        {
            hist.zero();

            const Rect rect = getRect2(img.dsize) - 1;

            const int radius = round(3.0 * sigma);

            const double sdiv = 1.0 / (2.0 * sigma * sigma);

            for (int ry = -radius; ry <= radius; ry++) {
                for (int rx = -radius; rx <= radius; rx++) {
                    const int ix = x + rx;
                    const int iy = y + ry;
                    if (isInRect2(rect, ix, iy) == false) continue;

                    const double dx = img(ix + 1, iy + 0) - img(ix - 1, iy + 0);
                    const double dy = img(ix + 0, iy + 1) - img(ix + 0, iy - 1);

                    const double angle = atan2(dy, dx);

                    int bin = round(angle * BINS / (2 * SP_PI));
                    if (bin < 0) bin += BINS;

                    hist[bin] += pythag(dx, dy) * exp(-(rx * rx + ry * ry) * sdiv);
                }
            }
        }

        Mem1<double> fhist(BINS);

        // filtering
        for (int i = 0; i < BINS; i++) {
            const double hi = hist(i);
            const double hp1 = hist(i + BINS - 1, true);
            const double hp2 = hist(i + BINS - 2, true);
            const double hn1 = hist(i + BINS + 1, true);
            const double hn2 = hist(i + BINS + 2, true);

            fhist[i] = (hi * 6.0 + (hn1 + hp1) * 4.0 + (hn2 + hp2)) / 16.0;
        }

        // detect peak
        const double thresh = maxVal(fhist) * pthresh;

        drcs.reserve(BINS);
        for (int i = 0; i < BINS; i++) {
            const double hi = fhist(i);
            const double hp1 = fhist(i + BINS - 1, true);
            const double hn1 = fhist(i + BINS + 1, true);

            if (hi > thresh && hi > maxVal(hp1, hn1)) {
                const double finei = i + 0.5 * (hp1 - hn1) / (hp1 + hn1 - 2 * hi);
                const double angle = finei * (2.0 * SP_PI / BINS);

                const Vec2 drc = getVec(cos(angle), sin(angle));
                drcs.push(drc);
            }
        }

        return drcs;
    }
}

#endif
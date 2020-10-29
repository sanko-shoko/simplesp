//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CONTOUR_H__
#define __SP_CONTOUR_H__

#include "spcore/spcore.h"


namespace sp{

    SP_CPUFUNC bool snake(Mem1<Vec2> &contour, const Mem2<Byte> &img, const Mem1<Vec2> &vtxs, const int unit = 10) {

        const int maxit = 200;

        Mem2<Byte> smth;
        bilateralFilter(smth, img, 2.0, 0.2 * SP_BYTEMAX);

        contour = vtxs;

        // snake main loop
        for (int it = 0; it < maxit; it++) {
            if (it % 10 == 0) {
                Mem1<Vec2> tmps = contour;
                for (int i = 0; i < tmps.size(); i++) {
                    const int ti = i + tmps.size();
                    const Vec2 &A = tmps[(ti + 0) % tmps.size()];
                    const Vec2 &B = tmps[(ti + 1) % tmps.size()];
                    const Vec2 &C = tmps[(ti - 1) % tmps.size()];
                    if (normVec(crsVec(getVec3(B - A, 0.0), getVec3(C - A, 0.0))) < 5) {
                        tmps.del(i--);
                    }
                }

                contour.clear();
                for (int i = 0; i < tmps.size(); i++) {
                    const Vec2 &A = tmps[(i + 0) % tmps.size()];
                    const Vec2 &B = tmps[(i + 1) % tmps.size()];
                    const Vec2 v = B - A;
                    contour.push(A);
                    const int num = floor(normVec(v) / unit);
                    for (int j = 0; j < num; j++) {
                        contour.push(A + v * (j + 1) / (num + 1));
                    }
                }
            }
            if (contour.size() < 4) {
                contour.clear();
                return false;
            }

            SP_REAL mean = 0.0;
            for (int i = 0; i < contour.size(); i++) {
                const int ti = i + contour.size();
                const Vec2 &A = contour[(ti + 0) % contour.size()];
                const Vec2 &B = contour[(ti + 1) % contour.size()];
                mean += normVec(A - B);
            }
            mean /= contour.size();

            const SP_REAL a = 1.0;
            const SP_REAL b = 1.0;
            const SP_REAL c = 100.0;

            const int w0 = 1;
            const int w1 = 1;

            for (int i = 0; i < contour.size(); i++) {
                const int ti = i + contour.size();
                const Vec2 &A = contour[(ti + 0) % contour.size()];
                const Vec2 &B = contour[(ti + 1) % contour.size()];
                const Vec2 &C = contour[(ti - 1) % contour.size()];
  
                SP_REAL eval = SP_INFINITY;

                Vec2 vec = getVec2(0.0, 0.0);
                for (int v = -w0; v <= +w0; v++) {
                    for (int u = -w0; u <= +w0; u++) {
                        const Vec2 tA = A + getVec2(u, v);
                        const int ix = round(tA.x);
                        const int iy = round(tA.y);

                        const SP_REAL e_len = sq(mean - normVec(C - tA));
                        const SP_REAL e_crv = sqVec(B + C - tA * 2.0);

                        int maxv = 0;
                        int minv = SP_BYTEMAX;
                        for (int y = -w1; y <= +w1; y++) {
                            for (int x = -w1; x <= +w1; x++) {
                                const int val = smth(ix + x, iy + y);
                                maxv = maxVal(maxv, val);
                                minv = minVal(minv, val);
                            }
                        }
                        const SP_REAL e_img = -static_cast<SP_REAL>(smth(ix, iy) - minv) / maxVal(maxv - minv, 20);

                        //const SP_REAL dx = fabs(img(ix + 1, iy) - img(ix - 1, iy));
                        //const SP_REAL dy = fabs(img(ix, iy + 1) - img(ix, iy - 1));
                        //const SP_REAL img = -(dx + dy);

                        const SP_REAL e = a * e_len + b * e_crv + c * e_img;
                        if (e < eval) {
                            eval = e;
                            vec = getVec2(u, v);
                        }
                    }
                }
                contour[i] = A + vec;
            }
        }

        return true;
    }

    SP_CPUFUNC bool snake(Mem1<Vec2> &contour, const Mem2<Col3> &img, const Mem1<Vec2> &vtxs, const int unit = 20) {
        Mem2<Byte> gry;
        cnvImg(gry, img);
        return snake(contour, gry, vtxs, unit);
    }
}

#endif
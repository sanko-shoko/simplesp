//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CONTOUR_H__
#define __SP_CONTOUR_H__

#include "spcore/spcore.h"


namespace sp{

    SP_CPUFUNC bool snake(Mem1<Vec2> &contour, const Mem2<Byte> &img, const Mem1<Vec2> &vtxs) {

        const int maxit = 100;

        Mem2<Byte> smth;
        gaussianFilter(smth, img, 3.0);

        const int block = 20;
        Mem2<double> map((img.dsize[0] + block - 1) / block, (img.dsize[1] + block - 1) / block);

        for (int v = 0; v < map.dsize[1]; v++) {
            for (int u = 0; u < map.dsize[0]; u++) {

                double sum = 0.0;
                for (int y = 0; y < block; y++) {
                    for (int x = 0; x < block; x++) {
                        const Byte val = smth(u + x, v + y);
                        sum += val;
                    }
                }
                const double mean = sum / (block * block);

                double sqsum = 0.0;
                for (int y = 0; y < block; y++) {
                    for (int x = 0; x < block; x++) {
                        const Byte val = smth(u + x, v + y);
                        sqsum += (val - mean) * (val - mean);
                    }
                }
                const double sigma = sqrt(sqsum / (block * block));

                map(u, v) = sigma;
            }
        }
        gaussianFilter3x3(map, map);

        const int unit = 20;

        contour = vtxs;

        // snake main loop
        for (int it = 0; it < maxit; it++) {
            Mem1<Vec2> tmps = contour;
            for (int i = 0; i < tmps.size(); i++) {
                const int ti = i + tmps.size();
                const Vec2 &A = tmps[(ti + 0) % tmps.size()];
                const Vec2 &B = tmps[(ti + 1) % tmps.size()];
                const Vec2 &C = tmps[(ti - 1) % tmps.size()];
                if (normVec(crsVec(B - A, C - A)) < 5) {
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

            const double a = 1.0;
            const double b = 1.0;
            const double c = 100.0;
            const int w = 2;

            for (int i = 0; i < contour.size(); i++) {
                const int ti = i + contour.size();
                const Vec2 &A = contour[(ti + 0) % contour.size()];
                const Vec2 &B = contour[(ti + 1) % contour.size()];
                const Vec2 &C = contour[(ti - 1) % contour.size()];
                const Vec2 U = unitVec(B - C);

                const double thresh = maxVal(1.0, acs2(map, A.x / block, A.y / block));

                double minv = SP_INFINITY;
                Vec2 vec = getVec(0.0, 0.0);
                for (int v = -w; v <= +w; v++) {
                    for (int u = -w; u <= +w; u++) {
                        const Vec2 tA = A + getVec(u, v);
                        const int ix = round(tA.x);
                        const int iy = round(tA.y);

                        const double len = a * sqVec(C - tA);
                        const double crv = b * sqVec(B + C - tA * 2.0);

                        const double dx = fabs(U.y) * fabs(smth(ix + 1, iy) - smth(ix - 1, iy));
                        const double dy = fabs(U.x) * fabs(smth(ix, iy + 1) - smth(ix, iy - 1));
                        const double dif = c * (dx + dy) / thresh;
                        const double e = len + crv - dif;
                        if (e < minv) {
                            minv = e;
                            vec = getVec(u, v);
                        }
                    }
                }
                contour[i] = A + vec;
            }
        }


        return true;
    }

    SP_CPUFUNC bool snake(Mem1<Vec2> &contour, const Mem2<Col3> &img, const Mem1<Vec2> &vtxs) {
        Mem2<Byte> gry;
        cnvImg(gry, img);
        return snake(contour, gry, vtxs);
    }
}

#endif
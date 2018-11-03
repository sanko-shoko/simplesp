//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// J.Canny,
// "A Computational Approach To Edge Detection",
// IEEE Transactions on Pattern Analysis and Machine Intelligence(PAMI), 1986


#ifndef __SP_CANNY_H__
#define __SP_CANNY_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spfilter.h"

namespace sp{

    SP_CPUFUNC void canny(Mem<Byte> &dst, const Mem<Byte> &src, const int low, const int higth) {

        dst.resize(2, src.dsize);
        dst.zero();

        // gradiend 
        Mem2<float> grad(src.dsize);

        // angle bin
        Mem2<Byte> bin(src.dsize);
        {
            Mem2<float> sobelX, sobelY;
            sobelFilterX(sobelX, src);
            sobelFilterY(sobelY, src);

            for (int v = 0; v < dst.dsize[1]; v++) {
                for (int u = 0; u < dst.dsize[0]; u++) {
                    const float gx = sobelX(u, v);
                    const float gy = sobelY(u, v);

                    grad(u, v) = static_cast<float>(pythag(gx, gy));
                    if (grad(u, v) < low) continue;

                    // angle (2*PI ~ 3*PI)
                    const double angle = (atan2(gy, gx) + 2 * SP_PI);

                    // bin (angle -> 0, 1, 2, 3)
                    bin(u, v) = round(angle / SP_PI * 4.0) % 4;
                }
            }
        }

        // non-maximum suppression
        {
            const int ref[][2] = {
                { +1, 0 },{ +1, +1 },{ 0, +1 },{ -1, +1 }
            };

            // dst (1 : grad >= low), (255 : grad >= hight)
            for (int v = 0; v < dst.dsize[1]; v++) {
                for (int u = 0; u < dst.dsize[0]; u++) {
                    if (grad(u, v) < low) continue;

                    const int *s = ref[bin(u, v)];
                    if (grad(u, v) > maxVal(grad(u - s[0], v - s[1]), grad(u + s[0], v + s[1]))) {
                        acs2(dst, u, v) = (grad(u, v) < higth) ? 1 : 255;
                    }
                }
            }
        }

        // hysteresis threshold
        {
            const int ref[][2] = {
                { +1, 0 },{ +1, +1 },{ 0, +1 },{ -1, +1 },
                { -1, 0 },{ -1, -1 },{ 0, -1 },{ +1, -1 }
            };

            for (int v = 0; v < dst.dsize[1]; v++) {
                for (int u = 0; u < dst.dsize[0]; u++) {
                    if (acs2(dst, u, v) != 1) continue;
                    acs2(dst, u, v) = 0;

                    // check edge (255) in 3*3 block
                    bool find = false;
                    for (int i = 0; i < 8; i++) {
                        const int x = u + ref[i][0];
                        const int y = v + ref[i][1];
                        if (acs2(dst, x, y) != 255) continue;

                        find = true;
                        break;
                    }
                    
                    if (find == false) continue;
                    acs2(dst, u, v) = 255;

                    int uu = u;
                    int vv = v;

                    // check candidate (1) in 3*3 block
                    for (int i = 0; i < 8; i++) {
                        const int x = uu + ref[i][0];
                        const int y = vv + ref[i][1];
                        if (acs2(dst, x, y) != 1) continue;

                        acs2(dst, x, y) = 255;
                        uu = x;
                        vv = y;

                        i = -1;
                    }
                }
            }
        }
    }


    SP_CPUFUNC void canny(Mem<Byte> &dst, const Mem<Col3> &src, const int low, const int higth) {
        Mem2<Byte> gry;
        cnvImg(gry, src);

        canny(dst, gry, low, higth);
    }

}

#endif
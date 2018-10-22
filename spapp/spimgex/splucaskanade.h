//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// BD.Lucas, T.Kanade,
// "An iterative image registration technique with an application to stereo vision",
// Proceedings of Imaging Understanding Workshop, 1981

#ifndef __SP_LUCASKANADE_H__
#define __SP_LUCASKANADE_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"
#include "spapp/spimg/spfilter.h"

namespace sp{

    SP_CPUFUNC void opticalFlowLK(Mem1<Vec2> &flows, Mem1<bool> &mask, const Mem2<Byte> &img0, const Mem2<Byte> &img1, const Mem1<Vec2> &pixs, const Mem1<double> &scls = Mem1<double>()) {
        SP_LOGGER_INSTANCE;
        SP_LOGGER_SET("opticalFlowLK");

        const int WIN_SIZE = 21;
        const double EIG_THRESH = 0.01;

        if (flows.size() != pixs.size()) {
            flows.resize(pixs.size());
            flows.zero();
        }

        mask.resize(pixs.size());
        setElm(mask, true);

        const int pynum = round(log2(minVal(img0.dsize[0], img0.dsize[1]) / 20.0));

        Mem1<Mem2<Byte> > pyimgs0(pynum);
        Mem1<Mem2<Byte> > pyimgs1(pynum);

        for (int p = 0; p < pynum; p++) {
            if (p == 0) {
                pyimgs0[p] = img0;
                pyimgs1[p] = img1;
            }
            else {
                pyrdown(pyimgs0[p], pyimgs0[p - 1]);
                pyrdown(pyimgs1[p], pyimgs1[p - 1]);
            }
        }

        const bool scalecheck = (scls.size() == pixs.size()) ? true : false;

        for(int p = pynum - 1; p >= 0; p--){
            const Mem2<Byte> &pyimg0 = pyimgs0[p];
            const Mem2<Byte> &pyimg1 = pyimgs1[p];
            const double scale = pow(0.5, p);

            Mem2<float> scharrX, scharrY;
            scharrFilterX(scharrX, pyimg1);
            scharrFilterY(scharrY, pyimg1);

            const Rect rect = getRect2(img1.dsize);
            const int offset = WIN_SIZE / 2;

            const double wscale = (p + 1) * (WIN_SIZE / 2);

            for (int i = 0; i < pixs.size(); i++) {
                if (mask[i] == false) continue;
                if (p != pynum - 1 && scalecheck == true && scls[i] > wscale) continue;

                // Ai = [dI/dx, dI/dy], A = [A0, A1, ... An-1]^T
                Mat A(WIN_SIZE * WIN_SIZE, 2);
                Mat I(WIN_SIZE * WIN_SIZE, 1);
                Mat AtA(2, 2);
                Mat AtB(2, 1);

                {
                    AtA.zero();
                    
                    const Vec2 pix0 = pixs[i] * scale;

                    for (int y = 0; y < WIN_SIZE; y++) {
                        for (int x = 0; x < WIN_SIZE; x++) {
                            const int i = y * WIN_SIZE + x;
                            const Vec2 v = getVec(x - offset, y - offset);

                            const Vec2 p0 = pix0 + v;
                            if (isInRect2(rect, p0.x, p0.y) == false) continue;

                            const double gx = acs2(scharrX, p0.x, p0.y) / SP_BYTEMAX;
                            const double gy = acs2(scharrY, p0.x, p0.y) / SP_BYTEMAX;
                            
                            A(i, 0) = gx;
                            A(i, 1) = gy;

                            I(i, 0) = acs2(pyimg1, p0.x, p0.y);

                            AtA(0, 0) += gx * gx;
                            AtA(0, 1) += gx * gy;
                            AtA(1, 0) += gy * gx;
                            AtA(1, 1) += gy * gy;
                        }
                    }

                    const double a = AtA(0, 0);
                    const double b = AtA(0, 1);
                    const double c = AtA(1, 1);
                    const double D = a * c - b * b;

                    const double mineig = (a + c - sqrt((a - c) * (a - c) + 4.0 * b * b)) / 2.0;

                    if (mineig / (WIN_SIZE * WIN_SIZE) < square(EIG_THRESH) || fabs(D) < SP_SMALL) {
                        if (p == 0) mask[i] = false;
                        continue;
                    }
                }

                const Mat invAtA = invMat(AtA);

                const int maxit = 2;
                for (int it = 0; it < maxit; it++) {
                    AtB.zero();

                    const Vec2 pix0 = (pixs[i] + flows[i]) * scale; 
                    const Vec2 pix1 = pixs[i] * scale;

                    for (int y = 0; y < WIN_SIZE; y++) {
                        for (int x = 0; x < WIN_SIZE; x++) {
                            const int i = y * WIN_SIZE + x;
                            const Vec2 v = getVec(x - offset, y - offset);

                            const Vec2 p0 = pix0 + v;
                            const Vec2 p1 = pix1 + v;
                            if (isInRect2(rect, p0.x, p0.y) == false || isInRect2(rect, p1.x, p1.y) == false) continue;

                            const double gx = A(i, 0);
                            const double gy = A(i, 1);

                            const double d = (I(i, 0) - acs2(pyimg0, p0.x, p0.y)) / SP_BYTEMAX;
                            AtB(0, 0) += gx * d;
                            AtB(1, 0) += gy * d;
                        }
                    }

                    const Mat result = invAtA * AtB;
                    if (result.size() == 0) {
                        if(p == 0) mask[i] = false;
                        break;
                    }

                    Vec2 delta = getVec(result[0], result[1]);

                    const double limit = 2.0;
                    if (normVec(delta) > 2.0) delta *= limit / normVec(delta);

                    flows[i] += delta / scale;

                    const Vec2 test = pixs[i] + flows[i];
                    if (isInRect2(rect, test.x, test.y) == false) {
                        mask[i] = false;
                        break;
                    }
                }
            }
        }
 
        for (int i = 0; i < flows.size(); i++) {
            if (mask[i] == false) flows[i] = getVec(0.0, 0.0);
        }
   }

    SP_CPUFUNC void opticalFlowLK(Mem1<Vec2> &flows, Mem1<bool> &masks, const Mem2<Col3> &img0, const Mem2<Col3> &img1, const Mem1<Vec2> &pixs, const Mem1<double> &scls = Mem1<double>()) {
        Mem2<Byte> gry0, gry1;
        cnvImg(gry0, img0);
        cnvImg(gry1, img1);

        opticalFlowLK(flows, masks, gry0, gry1, pixs, scls);
    }
}

#endif
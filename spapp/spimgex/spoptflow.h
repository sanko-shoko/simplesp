//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// BD.Lucas, T.Kanade,
// "An iterative image registration technique with an application to stereo vision",
// Proceedings of Imaging Understanding Workshop, 1981

#ifndef __SP_OPTFLOW_H__
#define __SP_OPTFLOW_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spfilter.h"

namespace sp{


    SP_CPUFUNC void opticalFlowLK(Mem1<Vec2> &flows, Mem1<bool> &mask, const Mem2<Byte> &img0, const Mem2<Byte> &img1, const Mem1<Vec2> &pixs, const Mem1<double> &scls = Mem1<double>()) {

        const int wsize = 19;
        const int whalf = wsize / 2;

        const double EIG_THRESH = 0.01 * SP_BYTEMAX;

        Mem1<double> errs;

        // clear
        if (flows.size() != pixs.size()) {
            flows.resize(pixs.size());
            flows.zero();
        }
        {
            mask.resize(pixs.size());
            setElm(mask, true);

            errs.resize(pixs.size());
            errs.zero();
        }


        // pyramid num
        const int pynum = round(log2(minVal(img0.dsize[0], img0.dsize[1]) / 10.0));

        Mem1<Mem2<Byte> > pyimgs0(pynum);
        Mem1<Mem2<Byte> > pyimgs1(pynum);
        {
            SP_LOGGER_SET("pyrdown");
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
        }

        Mem1<Mem2<float> > dXs(pynum);
        Mem1<Mem2<float> > dYs(pynum);
        {
            SP_LOGGER_SET("filter");
            for (int p = 0; p < pynum; p++) {
                scharrFilterX3x3(dXs[p], pyimgs1[p]);
                scharrFilterY3x3(dYs[p], pyimgs1[p]);
            }
        }


        {
            SP_LOGGER_SET("lk");
            for (int i = 0; i < pixs.size(); i++) {

                for (int p = pynum - 1; p >= 0; p--) {
                    const double scale = pow(0.5, p);

                    const Mem2<Byte> &pyimg0 = pyimgs0[p];
                    const Mem2<Byte> &pyimg1 = pyimgs1[p];

                    const Rect rect0 = getRect2(pyimg0.dsize);
                    const Rect rect1 = getRect2(pyimg1.dsize);

                    Mem2<float> &dX = dXs[p];
                    Mem2<float> &dY = dYs[p];

                    // check status
                    {
                        if (mask[i] == false) continue;
                      
                        const bool sclCheck = (scls.size() == pixs.size()) ? true : false;
                        if (sclCheck == true && p < pynum - 1) {
                            //if (scls[i] * scale < whalf) continue;
                        }

                        const Vec2 pix1 = pixs[i] * scale;
                        const Vec2 pix0 = pix1 + flows[i] * scale;

                        if (isInRect2(rect0, pix0.x, pix0.y) == false || isInRect2(rect1, pix1.x, pix1.y) == false) {
                            mask[i] = false;
                            continue;
                        }
                    }

                    // Ai = [dI/dx, dI/dy], A = [A0, A1, ... An-1]^T
                    Mat A(wsize * wsize, 2);
                    Mat I(wsize * wsize, 1);
                    Mat AtA(2, 2);
                    Mat AtB(2, 1);

                    {
                        AtA.zero();

                        int cnt = 0;
                        {
                            const Vec2 pix1 = pixs[i] * scale;
                            const int ipix1x = round(pix1.x);
                            const int ipix1y = round(pix1.y);

                            for (int y = 0; y < wsize; y++) {
                                for (int x = 0; x < wsize; x++) {
                                    const int i = y * wsize + x;

                                    const int i1x = ipix1x + (x - whalf);
                                    const int i1y = ipix1y + (y - whalf);
                                    if (isInRect2(rect1, i1x, i1y) == false) continue;

                                    const double gx = acs2(dX, i1x, i1y);
                                    const double gy = acs2(dY, i1x, i1y);

                                    A(i, 0) = gx;
                                    A(i, 1) = gy;

                                    AtA(0, 0) += gx * gx;
                                    AtA(0, 1) += gx * gy;
                                    AtA(1, 0) += gy * gx;
                                    AtA(1, 1) += gy * gy;

                                    I(i, 0) = acs2(pyimg1, i1x, i1y);
                                    cnt++;
                                }
                            }
                        }

                        const double a = AtA(0, 0);
                        const double b = AtA(0, 1);
                        const double c = AtA(1, 1);
                        const double D = a * c - b * b;

                        const double mineig = (a + c - sqrt((a - c) * (a - c) + 4.0 * b * b)) / 2.0;

                        if (mineig / cnt < square(EIG_THRESH) || fabs(D) < SP_SMALL) {
                            if(p == 0) mask[i] = false;
                            continue;
                        }
                    }

                    const Mat invAtA = invMat(AtA);

                    const int maxit = 2;
                    for (int it = 0; it < maxit; it++) {
                        AtB.zero();

                        int cnt = 0;
                        {
                            const Vec2 pix1 = pixs[i] * scale;
                            const int ipix1x = round(pix1.x);
                            const int ipix1y = round(pix1.y);

                            const Vec2 pix0 = getVec(ipix1x, ipix1y) + flows[i] * scale;

                            double esum = 0.0;

                            for (int y = 0; y < wsize; y++) {
                                for (int x = 0; x < wsize; x++) {
                                    const int i = y * wsize + x;

                                    const Vec2 p0 = pix0 + getVec(x - whalf, y - whalf);
                                    const int i1x = ipix1x + (x - whalf);
                                    const int i1y = ipix1y + (y - whalf);

                                    if (isInRect2(rect0, p0.x, p0.y) == false || isInRect2(rect1, i1x, i1y) == false) continue;

                                    const double gx = A(i, 0);
                                    const double gy = A(i, 1);

                                    const double d = I(i, 0) - acs2(pyimg0, p0.x, p0.y);
                                    AtB(0, 0) += gx * d;
                                    AtB(1, 0) += gy * d;

                                    cnt++;
                                    esum += fabs(d);
                                }
                            }

                            errs[i] = esum / cnt;
                        }

                        const Mat result = invAtA * AtB;
                        if (result.size() == 0) {
                            if (p == 0) mask[i] = false;
                            break;
                        }

                        Vec2 delta = getVec(result[0], result[1]);
                        const double norm = normVec(delta);

                        const double limit = 2.0;
                        if (norm > limit) delta *= limit / norm;

                        flows[i] += delta / scale;
                    }

                    {
                        const Vec2 test = (pixs[i] + flows[i]) * scale;

                        if (isInRect2(rect0, test.x, test.y) == false) {
                            mask[i] = false;
                            break;
                        }
                    }
               }
            }
        }
 
        if (cntVal(mask, true) > 0) {
            const double sigma = 1.4826 * medianVal(filter(errs, mask));

            for (int i = 0; i < mask.size(); i++) {
                if (errs[i] > 3.0 * sigma) mask[i] = false;
            }
        }


        for (int i = 0; i < mask.size(); i++) {
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
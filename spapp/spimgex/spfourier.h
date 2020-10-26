//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FOURIER_H__
#define __SP_FOURIER_H__

#include "spcore/spcore.h"


namespace sp{

    namespace _fourier {
        
        SP_CPUFUNC void calcHor(Mat &dstRe, Mat &dstIm, const Mat &srcRe, const Mat srcIm = Mat(), const bool flag = true) {

            const int M = srcRe.dsize[0];
            
            const int sign = (flag == true) ? +1 : -1;
            const int offset = M / 2;
            const bool useIm = (srcIm.size() != 0) ? true : false;

            dstRe.resize(srcRe.dsize);
            dstIm.resize(srcRe.dsize);

            for (int y = 0; y < srcRe.dsize[1]; y++) {
                for (int x = 0; x < srcRe.dsize[0]; x++) {

                    SP_REAL sumr = 0.0;
                    SP_REAL sumi = 0.0;

                    for (int k = 0; k < srcRe.dsize[0]; k++) {
                        const SP_REAL w = (flag == true) ? k * (x - offset) : (k - offset) * x;

                        const SP_REAL r = acs2(srcRe, k, y);
                        const SP_REAL a = sign * 2 * SP_PI * w / M;

                        if (useIm == false) {
                            sumr += +r * cos(a);
                            sumi += -r * sin(a);
                        }
                        else {
                            const SP_REAL i = acs2(srcIm, k, y);
                            sumr += +r * cos(a) + i * sin(a);
                            sumi += -r * sin(a) + i * cos(a);
                        }
                    }

                    if (flag == true) {
                        acs2(dstRe, x, y) = sumr;
                        acs2(dstIm, x, y) = sumi;
                    }
                    else {
                        acs2(dstRe, x, y) = sumr / M;
                        acs2(dstIm, x, y) = sumi / M;
                    }
                }
            }
        }

        SP_CPUFUNC void calcVer(Mat &dstRe, Mat &dstIm, const Mat &srcRe, const Mat &srcIm = Mat(), bool flag = true) {

            calcHor(dstRe, dstIm, trnMat(srcRe), trnMat(srcIm), flag);

            dstRe = trnMat(dstRe);
            dstIm = trnMat(dstIm);
        }
    }

    using namespace _fourier;

    template<typename TYPE>
    SP_CPUFUNC void dft(Mem<SP_REAL> &re, Mem<SP_REAL> &im, const Mem<TYPE> &img) {
        SP_ASSERT(checkPtr(img, 2));

        Mat mat;
        cnvMem(mat, img);

        Mat horRe, horIm;
        calcHor(horRe, horIm, mat);

        Mat verRe, verIm;
        calcVer(verRe, verIm, horRe, horIm);

        re = verRe;
        im = verIm;
    }

    template<typename TYPE>
    SP_CPUFUNC void idft(Mem<TYPE> &img, const Mem<SP_REAL> &re, const Mem<SP_REAL> &im) {
        SP_ASSERT(checkPtr(re, 2));
        SP_ASSERT(checkPtr(im, 2));
        SP_ASSERT(cmp(re.dsize, im.dsize, 2));

        Mat mRe, mIm;
        cnvMem(mRe, re);
        cnvMem(mIm, im);

        Mat horRe, horIm;
        calcHor(horRe, horIm, mRe, mIm, false);

        Mat verRe, verIm;
        calcVer(verRe, verIm, horRe, horIm, false);

        cnvMem(img, verRe);
    }


    SP_CPUFUNC void poc(Mem<SP_REAL> &dst, const Mem<SP_REAL> &re0, const Mem<SP_REAL> &im0, const Mem<SP_REAL> &re1, const Mem<SP_REAL> &im1) {
        const int *dsize = re0.dsize;

        Mem2<SP_REAL> re(dsize);
        Mem2<SP_REAL> im(dsize);

        for (int i = 0; i < re0.size(); i++) {
            const SP_REAL vr = re0[i] * re1[i] + im0[i] * im1[i];
            const SP_REAL vi = im0[i] * re1[i] - re0[i] * im1[i];
            re[i] = vr / (pythag(vr, vi) + SP_SMALL);
            im[i] = vi / (pythag(vr, vi) + SP_SMALL);
        }
        idft(dst, re, im);
    }
}

#endif
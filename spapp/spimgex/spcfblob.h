//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spimgex/spfeature.h"

//--------------------------------------------------------------------------------
// cfblob
//--------------------------------------------------------------------------------

namespace sp {

    class CFBlob {

    public:

        struct short2 {
            short x, y;
        };

        class ImgSet {

        public:
            Mem2<Byte> img;
            Mem2<short2> di1;
            Mem2<short2> di2;
        };


    private:

        Mem1<Ftr> m_ftrs;

        Mem1<ImgSet> m_imgsets;

    private:

        double BLOB_CONTRAST = 0.01;

    public:

        CFBlob() {
        }

        CFBlob(const CFBlob &cfblob) {
            *this = cfblob;
        }
        CFBlob& operator = (const CFBlob &cfblob) {
            m_ftrs = cfblob.m_ftrs;
            return *this;
        }


        //--------------------------------------------------------------------------------
        // parameter
        //--------------------------------------------------------------------------------

        void setParam(const double contrast = 0.01) {
            BLOB_CONTRAST = contrast;
        }


        //--------------------------------------------------------------------------------
        // data
        //--------------------------------------------------------------------------------

        const Mem1<Ftr>* getFtrs() const {
            return (m_ftrs.size() > 0) ? &m_ftrs : NULL;
        }

        template<typename T>
        static Mem1<Ftr> getFtrs(const Mem2<T> &img, const double contrast = 0.01) {
            CFBlob cfblob;
            cfblob.setParam(contrast);
            cfblob.execute(img);
            return (cfblob.getFtrs() != NULL) ? *cfblob.getFtrs() : Mem1<Ftr>();
        }


        //--------------------------------------------------------------------------------
        // execute detection and description
        //--------------------------------------------------------------------------------

        bool execute(const void *img, const int dsize0, const int dsize1, const int ch) {

            Mem2<Byte> gry;
            cnvPtrToImg(gry, img, dsize0, dsize1, ch);
            return _execute(gry);
        }

        bool execute(const Mem2<Col3> &img) {

            Mem2<Byte> gry;
            cnvImg(gry, img);
            return _execute(gry);
        }

        bool execute(const Mem2<Byte> &img) {

            return _execute(img);
        }

    private:

        bool _execute(const Mem2<Byte> &img) {

            // clear data
            {
                m_ftrs.clear();
            }

            try {
                if (img.size() == 0) throw "image size";

                makeImgSet(m_imgsets, img);

                detect(m_ftrs, m_imgsets);
            }
            catch (const char *str) {
                SP_PRINTF("Test.execute [%s]\n", str);

                return false;
            }

            return true;
        }

    private:

        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------

        void makeImgSet(Mem1<ImgSet> &imgsets, const Mem2<Byte> &img) {
            SP_LOGGER_SET("makeImgSet");

            const int pynum = 6;
            imgsets.resize(pynum);
            {
                SP_LOGGER_SET("pyrdown");
                imgsets[0].img = img;

                for (int s = 1; s < pynum; s++) {
                    pyrdown(imgsets[s].img, imgsets[s - 1].img);
                }
            }
            {
                SP_LOGGER_SET("ss");

                for (int s = 0; s < pynum; s++) {
                    const Mem2<Byte> &img = imgsets[s].img;
                    Mem2<short2> &di1 = imgsets[s].di1;
                    Mem2<short2> &di2 = imgsets[s].di2;

                    di1.resize(img.dsize);
                    di2.resize(img.dsize);

                    di1.zero();
                    di2.zero();

                    const int w = img.dsize[0];
                    const int h = img.dsize[1];
                    const int m = 1;

                    const Byte *sptr = img.ptr;
                    for (int v = m; v < h - m; v++) {
                        short2 *ptr1 = &di1(m, v);
                        short2 *ptr2 = &di2(m, v);

                        for (int u = m; u < w - m; u++) {
                            const Byte xa = sptr[(v + 0) * w + (u - 1)];
                            const Byte xb = sptr[(v + 0) * w + (u + 1)];
                            const Byte ya = sptr[(v - 1) * w + (u + 0)];
                            const Byte yb = sptr[(v + 1) * w + (u + 0)];
                            const Byte cc = sptr[(v + 0) * w + (u + 0)];

                            ptr1->x = (xb - xa);
                            ptr1->y = (yb - ya);
                            ptr2->x = (xb - cc) - (cc - xa);
                            ptr2->y = (yb - cc) - (cc - ya);

                            ptr1++;
                            ptr2++;
                        }
                    }
                }
            }
            for (int s = 0; s < pynum; s++) {
                SP_HOLDER_SET(strFormat("pyimg%d", s).c_str(), imgsets[s].img);
            }
        }

        bool detect(Mem1<Ftr> &ftrs, const Mem1<ImgSet> &imgsets) {
            SP_LOGGER_SET("detect");

            const int pynum = imgsets.size();

            Mem1<Mem1<Ftr> > pyfts(pynum - 1);
            {
                SP_LOGGER_SET("procCoarse");
                for (int s = 0; s < pynum - 1; s++) {
                    procCoarse(pyfts[s], imgsets[s + 1].img, 15);

                    for (int i = 0; i < pyfts[s].size(); i++) {
                        pyfts[s][i].pix *= 2.0;
                        pyfts[s][i].scl = 2.0;
                    }

                    SP_HOLDER_SET(strFormat("pyfts%d", s).c_str(), pyfts[s]);
                }
            }

            ftrs.clear();
            {
                SP_LOGGER_SET("procFine");
                for (int s = 0; s < pynum - 1; s++) {
                    SP_HOLDER_SET(strFormat("refine%d", s).c_str(), pyfts[s]);
                    procFine(pyfts[s], imgsets[s]);

                    SP_HOLDER_SET(strFormat("refine2%d", s).c_str(), pyfts[s]);
              
                    const double ss = pow(2.0, s);
                    for (int i = 0; i < pyfts[s].size(); i++) {
                        Ftr ftr = pyfts[s][i];
                        ftr.pix *= ss;
                        ftr.scl *= ss;
                        ftrs.push(ftr);
                    }
                }
                SP_HOLDER_SET("output", ftrs);
            }

            return true;
        }


        template <typename TYPE>
        void procCoarse(Mem1<Ftr> &ftrs, const Mem2<TYPE> &img, const float thresh = 0.1f * SP_BYTEMAX) {
            SP_ASSERT(isValid(2, img));

            const int w = img.dsize[0];
            const int h = img.dsize[1];

            ftrs.reserve(img.size() / 4);

            Mem2<float> res(img.dsize);
            res.zero();

            const TYPE *pimg = img.ptr;
            float *pres = res.ptr;

            //laplacianFilter(res, img, 1.0);
            //laplacianFilter5x5(res, img);
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
                        Ftr *ftr = ftrs.extend();
                        ftr->pix = getVec(u, v);
                    }
                }
            }

        }

        bool procFine(Mem1<Ftr> &ftrs, const ImgSet &imgset) {

            const Mem2<Byte> &img = imgset.img;
            const Mem2<short2> &di1 = imgset.di1;
            const Mem2<short2> &di2 = imgset.di2;

            const double k = 4.0;
            const int rdiv = 18;
            const int itmax = 20;

            Mem1<Vec2> rvec;
            for (int r = 0; r < rdiv; r++) {
                const double p = r * 2.0 * SP_PI / rdiv;
                const Vec2 v = getVec(cos(p), sin(p));
                rvec.push(v);
            }

            const Rect rect = getRect2(img.dsize) - 2;
            
            const int w = img.dsize[0];
            const int h = img.dsize[1];

            for (int i = 0; i < ftrs.size(); i++) {

                for (int it = 0; it < itmax; it++) {
                    const Vec2 pix = ftrs[i].pix;
                    const double scl = ftrs[i].scl;

                    Vec2 ap = getVec(0.0, 0.0);
                    double as = 0.0;
                    
                    int cnt = 0;
                    for (int r = 0; r < rdiv; r++) {
                        const Vec2 n = rvec[r];
                        const Vec2 v = scl * n;
                        const Vec2 a = pix + v;
                        const int ax = round(a.x);
                        const int ay = round(a.y);
                        if (isInRect2(rect, ax, ay) == false) continue;

                        Vec2 d1, d2;
                        d1.x = acs2<short2, short>(di1, a.x, a.y, 0);
                        d1.y = acs2<short2, short>(di1, a.x, a.y, 1);
                        d2.x = acs2<short2, short>(di2, a.x, a.y, 0);
                        d2.y = acs2<short2, short>(di2, a.x, a.y, 1);
                        //short2 d1, d2;
                        //d1 = acs2(di1, ax, ay);
                        //d2 = acs2(di2, ax, ay);

                        Vec2 delta;
                        delta.x = d2.x * sign(d1.x);
                        delta.y = d2.y * sign(d1.y);

                        ap += delta;
                        as += dotVec(n, delta);
                        cnt++;
                    }
                    if (cnt == 0) break;

                    ap *= k / (SP_BYTEMAX * cnt);
                    as *= k / (SP_BYTEMAX * cnt);

                    ftrs[i].pix += ap;
                    ftrs[i].scl += as;
                 }
            }

            return true;
        }

    };
}
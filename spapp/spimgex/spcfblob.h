//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
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

        class MyFtr : public Ftr {

        public:
            int pyid;
            int stat;

            MyFtr() :Ftr() {
                pyid = 0;
                stat = 0;
            }
        };

        struct short2 {
            short x, y;
        };

        class ImgSet {

        public:
            Mem2<Byte> img;
            Mem2<short> dog;
            Mem2<short2> di1;
            Mem2<short2> di2;
        };


    private:

        Mem1<Ftr> m_ftrs;

        Mem1<ImgSet> m_imgsets;

    private:

        SP_REAL BLOB_CONTRAST = 0.01;

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

        void setParam(const SP_REAL contrast) {
            BLOB_CONTRAST = contrast;
        }


        //--------------------------------------------------------------------------------
        // data
        //--------------------------------------------------------------------------------

        const Mem1<Ftr>* getFtrs() const {
            return (m_ftrs.size() > 0) ? &m_ftrs : NULL;
        }

        template<typename T>
        static Mem1<Ftr> getFtrs(const Mem2<T> &img, const SP_REAL contrast = 0.01) {
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
            SP_LOGGER_SET("CFBlob.execute");

            // clear data
            {
                m_ftrs.clear();
            }

            try {
                if (img.size() == 0) throw "image size";

                makeImgSet(m_imgsets, img);

                Mem1<MyFtr> myfts;

                // detect features
                if (detect(myfts, m_imgsets) == false) throw "no featues";

                // descript features
                descript(m_ftrs, myfts, m_imgsets);
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
                    Mem2<Byte> g1, g2;
                    gaussianFilter3x3(g1, imgsets[s].img);
                    gaussianFilter3x3(g2, g1);

                    subMem(imgsets[s].dog, g2, g1);
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
                char str[SP_STRMAX];
                sprintf(str, "pyimg%d", s);
                SP_HOLDER_SET(str, imgsets[s].img);
            }
        }

        bool detect(Mem1<MyFtr> &myftrs, const Mem1<ImgSet> &imgsets) {
            SP_LOGGER_SET("CFBlob.detect");

            const int pynum = imgsets.size();

            myftrs.clear();
            {
                SP_LOGGER_SET("procCoarse");
                procCoarse(myftrs, imgsets, 10);

                SP_HOLDER_SET("myftrs1", myftrs);
            }

            {
                SP_LOGGER_SET("procFine");
                procFine(myftrs, imgsets);
                SP_HOLDER_SET("myftrs2", myftrs);

                for (int i = 0; i < myftrs.size(); i++) {
                    MyFtr &myftr = myftrs[i];
                    myftr.pix *= (1 << myftr.pyid);
                    myftr.scl *= (1 << myftr.pyid);
                }
                SP_HOLDER_SET("myftrs3", myftrs);
            }

            return true;
        }
        
        void descript(Mem1<Ftr> &ftrs, const Mem1<MyFtr> &myfts, const Mem1<ImgSet> &imgsets) {
            SP_LOGGER_SET("CFBlob.descript");

            ftrs.reserve(myfts.size() * 2);

            for (int i = 0; i < myfts.size(); i++) {

                const MyFtr &myft = myfts[i];
                if (myft.stat < 0) continue;

                const Mem2<Byte> &img = imgsets[myft.pyid].img;

                const Vec2 pix = myft.pix / (1 << myft.pyid);
                const SP_REAL scl = myft.scl / (1 << myft.pyid);

                Ftr ftr = myft;

                Mem1<Vec2> drcs;
                calcFtrDrc(drcs, img, pix, scl);

                for (int a = 0; a < drcs.size(); a++) {
                    ftr.drc = drcs[a];
                    calcFtrDsc(ftr.dsc, img, pix, ftr.drc, scl);
                    ftrs.push(ftr);
                }
            }
        }

        void procCoarse(Mem1<MyFtr> &ftrs, const Mem1<ImgSet> &imgsets, const float thresh = 0.1f * SP_BYTEMAX) {
            ftrs.reserve(1000);

            for (int p = 0; p < imgsets.size() - 1; p++) {

                const Mem2<Byte> &img = imgsets[p + 1].img;
                const Mem2<short> &dog = imgsets[p].dog;

                const int dsize0 = img.dsize[0];
                const int dsize1 = img.dsize[1];

                Mem2<short> res(img.dsize);
                res.zero();

                const Byte *pimg = img.ptr;
                short *pres = res.ptr;

                const int m = 2;

                for (int v = m; v < dsize1 - m; v++) {

                    const int v0 = v - 2;
                    const int v1 = v + 0;
                    const int v2 = v + 2;

                    const Byte *pimg0 = &pimg[v0 * dsize0];
                    const Byte *pimg1 = &pimg[v1 * dsize0];
                    const Byte *pimg2 = &pimg[v2 * dsize0];

                    short *pdst = &pres[v1 * dsize0];

                    short val[8];

                    for (int u = m; u < dsize0 - m; u++) {

                        const int u0 = u - 2;
                        const int u1 = u + 0;
                        const int u2 = u + 2;

                        const short c = (2 * pimg1[u1] + pimg1[u1 - dsize0] + pimg1[u1 + dsize0] + pimg1[u1 - 1] + pimg1[u1 + 1]) / 6;
                        val[0] = c - pimg0[u0];
                        val[1] = c - pimg0[u1];
                        val[2] = c - pimg0[u2];

                        val[3] = c - pimg1[u0];
                        val[4] = c - pimg1[u2];

                        val[5] = c - pimg2[u0];
                        val[6] = c - pimg2[u1];
                        val[7] = c - pimg2[u2];
                        const short a = (val[0] < val[1]) ? val[0] : val[1];
                        const short b = (val[0] > val[1]) ? val[0] : val[1];
                        short min0 = a;
                        short min1 = b;
                        short max0 = b;
                        short max1 = a;

                        for (int i = 2; i < 8; i++) {
                            if (val[i] < min0) { min1 = min0; min0 = val[i]; }
                            else if (val[i] < min1) { min1 = val[i]; }

                            if (val[i] > max0) { max1 = max0; max0 = val[i]; }
                            else if (val[i] > max1) { max1 = val[i]; }
                        }

                        short d = 0;
                        if (min1 > 0) {
                            d = min1;
                        }
                        if (max1 < 0) {
                            d = max1;
                        }
                        pdst[u1] = d;
                    }
                }

                for (int v = m; v < dsize1 - m; v++) {

                    const int vs = v - 1;
                    const int vc = v + 0;
                    const int ve = v + 1;

                    const short *pres0 = &pres[vs * dsize0];
                    const short *pres1 = &pres[vc * dsize0];
                    const short *pres2 = &pres[ve * dsize0];

                    for (int u = m; u < dsize0 - m; u++) {

                        const int us = u - 1;
                        const int uc = u + 0;
                        const int ue = u + 1;

                        const short d00 = pres0[us];
                        const short d01 = pres0[uc];
                        const short d02 = pres0[ue];

                        const short d10 = pres1[us];
                        const short d11 = pres1[uc];
                        const short d12 = pres1[ue];

                        const short d20 = pres2[us];
                        const short d21 = pres2[uc];
                        const short d22 = pres2[ue];

                        const short d = d11;

                        int cst = 0;
                        if (d > 0) {
                            if (d > d00 && d > d01 && d > d02 && d > d10 && d >= d12 && d >= d20 && d >= d21 && d >= d22) cst = +1;
                        }
                        else {
                            if (d < d00 && d < d01 && d < d02 && d < d10 && d <= d12 && d <= d20 && d <= d21 && d <= d22) cst = -1;
                        }

                        if (cst != 0 && fabs(d) > thresh) {
                            MyFtr *ftr = ftrs.extend();
                            ftr->pix = getVec2(u, v) * 2.0;
                            ftr->scl = 2.0;
                            ftr->cst = cst;
                            ftr->pyid = p;
                        }
                    }
                }
            }
        }

        void procFine(Mem1<MyFtr> &ftrs, const Mem1<ImgSet> &imgsets) {
            for (int i = 0; i < ftrs.size(); i++) {
                const int p = ftrs[i].pyid;

                const Mem2<Byte> &img = imgsets[p].img;
                const Mem2<short2> &di1 = imgsets[p].di1;
                const Mem2<short2> &di2 = imgsets[p].di2;

                const SP_REAL k = 4.0;
                const int rdiv = 18;
                const int itmax = 20;

                Mem1<Vec2> rvec;
                for (int r = 0; r < rdiv; r++) {
                    const SP_REAL p = r * 2.0 * SP_PI / rdiv;
                    const Vec2 v = getVec2(cos(p), sin(p));
                    rvec.push(v);
                }

                const Rect2 rect = getRect2(img.dsize) - 2;

                //{
                //    const Vec2 pix = ftrs[i].pix;
                //    const SP_REAL scl = ftrs[i].scl;

                //    Vec3 result;
                //    SP_REAL maxv = 0.0;
                //    const int r = 3;
                //    for (int s = -r; s <= +r; s++) {
                //        for (int y = -1; y <= +1; y++) {
                //            for (int x = -1; x <= +1; x++) {

                //                const Vec2 tpix = pix + getVec3(x, y) * 0.5;
                //                const SP_REAL tscl = scl + s * 0.5;

                //                SP_REAL mm = 0.0;
                //                for (int r = 0; r < rdiv; r++) {
                //                    const Vec2 n = rvec[r];
                //                    const Vec2 v = tscl * n;
                //                    const Vec2 a = tpix + v;
                //                    const int ax = round(a.x);
                //                    const int ay = round(a.y);
                //                    if (inRect2(rect, ax, ay) == false) continue;

                //                    Vec2 d1, d2;
                //                    d1.x = acs2<short2, short>(di1, a.x, a.y, 0);
                //                    d1.y = acs2<short2, short>(di1, a.x, a.y, 1);
                //                    const SP_REAL dd = fabs(dotVec(n, d1));

                //                    mm += dd;
                //                }
                //                if (mm > maxv) {
                //                    maxv = mm;
                //                    result = getVec3(x * 0.5, y * 0.5, s * 0.5);
                //                }
                //            }
                //        }
                //    }
                //    ftrs[i].pix += getVec3(result.x, result.y);
                //    ftrs[i].scl += result.z;
                //}

                for (int it = 0; it < itmax; it++) {
                    const Vec2 pix = ftrs[i].pix;
                    const SP_REAL scl = ftrs[i].scl;

                    Vec2 ap = getVec2(0.0, 0.0);
                    SP_REAL as = 0.0;

                    int cnt = 0;
                    for (int r = 0; r < rdiv; r++) {
                        const Vec2 n = rvec[r];
                        const Vec2 v = scl * n;
                        const Vec2 a = pix + v;
                        const int ax = round(a.x);
                        const int ay = round(a.y);
                        if (inRect2(rect, ax, ay) == false) continue;

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

                    ftrs[i].stat = 0;
                    if (ftrs[i].scl < 1.5) {
                        ftrs[i].stat = -1;
                    }
                }
                {
                    const Vec2 pix = ftrs[i].pix;
                    const SP_REAL scl = ftrs[i].scl;

                    Vec2 ap = getVec2(0.0, 0.0);
                    SP_REAL as = 0.0;

                    SP_REAL minv = SP_INFINITY;
                    for (int r = 0; r < rdiv / 2; r++) {
                        const Vec2 n0 = rvec[r];
                        const Vec2 v0 = scl * n0;
                        const Vec2 n1 = rvec[r + rdiv / 2];
                        const Vec2 v1 = scl * n1;

                        const Vec2 a = pix + v0;
                        const Vec2 b = pix + v1;
                        const int ax = round(a.x);
                        const int ay = round(a.y);
                        const int bx = round(b.x);
                        const int by = round(b.y);
                        if (inRect2(rect, ax, ay) == false) continue;
                        if (inRect2(rect, bx, by) == false) continue;

                        Vec2 ad1, bd1;
                        ad1.x = acs2<short2, short>(di1, a.x, a.y, 0);
                        ad1.y = acs2<short2, short>(di1, a.x, a.y, 1);
                        bd1.x = acs2<short2, short>(di1, b.x, b.y, 0);
                        bd1.y = acs2<short2, short>(di1, b.x, b.y, 1);

                        const SP_REAL s0 = dotVec(n0, ad1);
                        const SP_REAL s1 = dotVec(n1, bd1);
                        const SP_REAL s = maxval(fabs(s0), fabs(s1));
                        minv = minval(minv, s);
                    }

                    if (minv < 11) {
                        ftrs[i].stat = -1;
                    }
                }
            }
        }
       
        template <typename TYPE>
        void calcFtrDrc(Mem1<Vec2> &drcs, const Mem2<TYPE> &img, const Vec2 &pix, const SP_REAL scl) {

            const int BINS = 36;
            Mem1<SP_REAL> hist(BINS);

            const int x = round(pix.x);
            const int y = round(pix.y);

            // calc direction hist
            {
                hist.zero();

                const Rect2 rect = getRect2(img.dsize) - 1;

                const int radius = round(3.0 * scl);

                const SP_REAL sdiv = 1.0 / (2.0 * scl * scl);

                for (int ry = -radius; ry <= radius; ry++) {
                    for (int rx = -radius; rx <= radius; rx++) {
                        const int ix = x + rx;
                        const int iy = y + ry;
                        if (inRect2(rect, ix, iy) == false) continue;

                        const SP_REAL dx = img(ix + 1, iy + 0) - img(ix - 1, iy + 0);
                        const SP_REAL dy = img(ix + 0, iy + 1) - img(ix + 0, iy - 1);

                        const SP_REAL angle = atan2(dy, dx);

                        int bin = round(angle * BINS / (2 * SP_PI));
                        if (bin < 0) bin += BINS;

                        hist[bin] += pythag(dx, dy) * exp(-(rx * rx + ry * ry) * sdiv);
                    }
                }
            }

            Mem1<SP_REAL> fhist(BINS);

            // filtering
            for (int i = 0; i < BINS; i++) {
                const SP_REAL hi = hist(i);
                const SP_REAL hp1 = hist(i + BINS - 1, true);
                const SP_REAL hp2 = hist(i + BINS - 2, true);
                const SP_REAL hn1 = hist(i + BINS + 1, true);
                const SP_REAL hn2 = hist(i + BINS + 2, true);

                fhist[i] = (hi * 6.0 + (hn1 + hp1) * 4.0 + (hn2 + hp2)) / 16.0;
            }

            // detect peak
            const SP_REAL phresh = maxval(fhist) * 0.8;

            drcs.reserve(BINS);
            for (int i = 0; i < BINS; i++) {
                const SP_REAL hi = fhist(i);
                const SP_REAL hp1 = fhist(i + BINS - 1, true);
                const SP_REAL hn1 = fhist(i + BINS + 1, true);

                if (hi > phresh && hi > maxval(hp1, hn1)) {
                    const SP_REAL finei = i + 0.5 * (hp1 - hn1) / (hp1 + hn1 - 2 * hi);
                    const SP_REAL angle = finei * (2.0 * SP_PI / BINS);

                    const Vec2 drc = getVec2(cos(angle), sin(angle));
                    drcs.push(drc);
                }
            }
        }

        template <typename TYPE>
        void calcFtrDsc(Dsc &dsc, const Mem2<TYPE> &img, const Vec2 &pix, const Vec2 &drc, const SP_REAL scl) {

            const int DSC_BINS = 8;
            const int DSC_BLKS = 4;
            const SP_REAL DCS_SCL_FCTR = 3.0;

            Mem3<SP_REAL> hist(DSC_BLKS + 2, DSC_BLKS + 2, DSC_BINS + 1);
            hist.zero();

            const Rect2 rect = getRect2(img.dsize) - 1;

            const SP_REAL block = DCS_SCL_FCTR * scl;

            const int radius = round(block * sqrt(2.0) * (DSC_BLKS + 1) * 0.5);

            const SP_REAL sdiv = 1.0 / (2.0 * DSC_BLKS * DSC_BLKS);

            const SP_REAL tcos = +drc.x;
            const SP_REAL tsin = -drc.y;

            for (int ry = -radius; ry <= radius; ry++) {
                for (int rx = -radius; rx <= radius; rx++) {
                    const int ix = round(pix.x + rx);
                    const int iy = round(pix.y + ry);
                    if (inRect2(rect, ix, iy) == false) continue;

                    const SP_REAL tx = (tcos * rx - tsin * ry) / block;
                    const SP_REAL ty = (tsin * rx + tcos * ry) / block;

                    const SP_REAL bx = tx + (DSC_BLKS - 1) * 0.5 + 1.0;
                    const SP_REAL by = ty + (DSC_BLKS - 1) * 0.5 + 1.0;

                    if (bx > 0 && bx < DSC_BLKS && by > 0 && by < DSC_BLKS) {

                        const SP_REAL dx = img(ix + 1, iy + 0) - img(ix - 1, iy + 0);
                        const SP_REAL dy = img(ix + 0, iy + 1) - img(ix + 0, iy - 1);

                        const SP_REAL ndx = tcos * dx - tsin * dy;
                        const SP_REAL ndy = tsin * dx + tcos * dy;

                        SP_REAL angle = atan2(ndy, ndx);
                        if (angle < 0) angle += 2 * SP_PI;

                        const SP_REAL ba = angle * (DSC_BINS / (2 * SP_PI));

                        const int ibx = floor(bx);
                        const int iby = floor(by);
                        const int iba = floor(ba);

                        const SP_REAL abx = bx - ibx;
                        const SP_REAL aby = by - iby;
                        const SP_REAL aba = ba - iba;

                        const SP_REAL val = pythag(dx, dy) * exp(-(tx * tx + ty * ty) * sdiv);

                        hist(ibx + 0, iby + 0, iba + 0) += val * (1 - abx) * (1 - aby) * (1 - aba);
                        hist(ibx + 0, iby + 0, iba + 1) += val * (1 - abx) * (1 - aby) * (0 + aba);
                        hist(ibx + 0, iby + 1, iba + 0) += val * (1 - abx) * (0 + aby) * (1 - aba);
                        hist(ibx + 0, iby + 1, iba + 1) += val * (1 - abx) * (0 + aby) * (0 + aba);
                        hist(ibx + 1, iby + 0, iba + 0) += val * (0 + abx) * (1 - aby) * (1 - aba);
                        hist(ibx + 1, iby + 0, iba + 1) += val * (0 + abx) * (1 - aby) * (0 + aba);
                        hist(ibx + 1, iby + 1, iba + 0) += val * (0 + abx) * (0 + aby) * (1 - aba);
                        hist(ibx + 1, iby + 1, iba + 1) += val * (0 + abx) * (0 + aby) * (0 + aba);
                    }
                }
            }

            Mem1<SP_REAL> ddsc(DSC_BLKS * DSC_BLKS * DSC_BINS);
            {
                int cnt = 0;
                for (int iby = 1; iby < DSC_BLKS + 1; iby++) {
                    for (int ibx = 1; ibx < DSC_BLKS + 1; ibx++) {
                        hist(ibx, iby, 0) += hist(ibx, iby, DSC_BINS);

                        for (int k = 0; k < DSC_BINS; k++) {
                            ddsc[cnt++] = hist(ibx, iby, k);
                        }
                    }
                }
            }

            {
                const SP_REAL DSC_MAG_THR = 0.2;

                const SP_REAL sq = sumSq(ddsc);
                const SP_REAL thresh = sqrt(sq) * DSC_MAG_THR;

                for (int k = 0; k < ddsc.size(); k++) {
                    ddsc[k] = minval(ddsc[k], thresh);
                }
            }

            {
                const SP_REAL sq = sumSq(ddsc);
                const SP_REAL nrm = 1.0 / maxval(sqrt(sq), SP_SMALL);

                const int dim = ddsc.size();

                const int bsize = dim / 8;
                const int fsize = dim * sizeof(float);

                dsc.dim = dim;
                dsc.type = Dsc::DSC_CFBlob;
                dsc.bin.resize(bsize);
                dsc.val.resize(fsize);

                // float
                float *val = reinterpret_cast<float*>(dsc.val.ptr);
                for (int k = 0; k < dim; k++) {
                    val[k] = static_cast<float>(ddsc[k] * nrm);
                }

                // binary
                Byte *bin = reinterpret_cast<Byte*>(dsc.bin.ptr);
                const float thresh = static_cast<float>(1.0 / sqrt(dim));
                cnvBit(bin, bsize, val, dim, thresh);
            }
        }
    };
}
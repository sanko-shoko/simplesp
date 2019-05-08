//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// D. Lowe,
// "Distinctive image features from scale-invariant keypoints",
// International Journal of Computer Vision, 60, 2 2004


#ifndef __SP_SIFT_H__
#define __SP_SIFT_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spimgex/spfeature.h"


namespace sp{

    class SIFT{

    public:

        class MyFtr : public Ftr{

        public:
            int pyid;
            int lyid;

            MyFtr() :Ftr() {
                pyid = 0;
                lyid = 0;
            }
        };

        class ImgSet {

        public:
            Mem1<Mem2<float> > imgs;
            Mem1<Mem2<float> > dogs;
        };

    private:
 
        Mem1<Ftr> m_ftrs;

        Mem1<ImgSet> m_imgsets;

    private:

        double BLOB_CONTRAST = 0.01;
        double EDGE_THRESH = 10.0;

    public:

        SIFT() {
        }

        SIFT(const SIFT &sift) {
            *this = sift;
        }

        SIFT& operator = (const SIFT &sift) {
            m_ftrs = sift.m_ftrs;
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
            SIFT sift;
            sift.setParam(contrast);
            sift.execute(img);
            return (sift.getFtrs() != NULL) ? *sift.getFtrs() : Mem1<Ftr>();
        }


        //--------------------------------------------------------------------------------
        // execute detection and description
        //--------------------------------------------------------------------------------

        bool execute(const void *img, const int dsize0, const int dsize1, const int ch){

            Mem2<Byte> gry;
            cnvPtrToImg(gry, img, dsize0, dsize1, ch);
            return _execute(gry);
        }

        bool execute(const Mem2<Col3> &img){

            Mem2<Byte> gry;
            cnvImg(gry, img);
            return _execute(gry);
        }

        bool execute(const Mem2<Byte> &img){

            return _execute(img);
        }


    private:

        bool _execute(const Mem2<Byte> &img){
            SP_LOGGER_SET("SIFT.execute");

            // clear data
            {
                m_ftrs.clear();
                m_imgsets.clear();
            }

            try{
                if (img.size() == 0) throw "image size";

                // make Imgs & DoGs
                makeImgSet(m_imgsets, img);

                Mem1<MyFtr> myfts;

                // detect features
                if (detect(myfts, m_imgsets) == false) throw "no featues";

                // descript features
                descript(m_ftrs, myfts, m_imgsets);

            }
            catch (const char *str){
                SP_PRINTF("SIFT.execute [%s]\n", str);

                return false;
            }

            return true;
        }

     private:

         SP_REAL INIT_SIGMA = 0.5;
         SP_REAL BASE_SIGMA = 1.6;

         int LAYERS = 3;
         SP_REAL LAYER_STEP = pow(2.0, 1.0 / LAYERS);


        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------

        void makeImgSet(Mem1<ImgSet> &imgsets, const Mem2<Byte> &img){
            Mem2<float> imgf;
            cnvMem(imgf, img, 1.0 / 255.0);

            const int pynum = round(log2(minval(img.dsize[0], img.dsize[1]) / (8.0 * BASE_SIGMA)));

            imgsets.resize(pynum);

            // octaves
            for (int p = 0; p < pynum; p++) {
                Mem1<Mem2<float> > &imgs = imgsets[p].imgs;
                Mem1<Mem2<float> > &dogs = imgsets[p].dogs;

                imgs.resize(LAYERS + 3);
                dogs.resize(LAYERS + 2);

                if (p == 0){
                    const SP_REAL crnt = BASE_SIGMA;
                    const SP_REAL prev = INIT_SIGMA;
                    const SP_REAL dsig = sqrt(crnt * crnt - prev * prev);

                    gaussianFilter(imgs[0], imgf, dsig);
                }
                else{
                    pyrdown(imgs[0], imgsets[p - 1].imgs[LAYERS]);
                }

                for (int s = 1; s < LAYERS + 3; s++){
                    const SP_REAL crnt = BASE_SIGMA * pow(LAYER_STEP, s);
                    const SP_REAL prev = BASE_SIGMA * pow(LAYER_STEP, s - 1);
                    const SP_REAL dsig = sqrt(crnt * crnt - prev * prev);

                    gaussianFilter(imgs[s], imgs[s - 1], dsig);
                }

                for (int s = 0; s < LAYERS + 2; s++){
                    subMem(dogs[s], imgs[s + 1], imgs[s]);
                }
            }
        }

        bool detect(Mem1<MyFtr> &ftrs, const Mem1<ImgSet> &imgsets){
            SP_LOGGER_SET("SIFT.detect");

            ftrs.reserve(1000);

            for (int p = 0; p < imgsets.size(); p++){
                const Mem1<Mem2<float> > &imgs = imgsets[p].imgs;
                const Mem1<Mem2<float> > &dogs = imgsets[p].dogs;

                const int m = round(4.0 * BASE_SIGMA);

                for (int s = 1; s < LAYERS + 1; s++){
                    for (int y = m; y < dogs[s].dsize[1] - m; y++){
                        for (int x = m; x < dogs[s].dsize[0] - m; x++){

                            const float base = dogs[s](x, y);

                            // check contrast
                            if (fabs(base) < BLOB_CONTRAST) continue;

                            bool npeak = true;
                            bool ppeak = true;
                            for (int ws = -1; ws <= 1; ws++){
                                for (int wy = -1; wy <= 1; wy++){
                                    for (int wx = -1; wx <= 1; wx++){
                                        if (wx == 0 && wy == 0 && ws == 0) continue;

                                        const float val = dogs[s + ws](x + wx, y + wy);
                                        npeak &= val > base;
                                        ppeak &= val < base;
                                        if (npeak == false && ppeak == false) goto _exit;
                                    }
                                }
                            }
                        _exit:;

                            if (npeak || ppeak){
                                Vec3 vec = getVec3(x, y, s);

                                if (calcFtrRefine(vec, imgsets[p].dogs) == false) continue;

                                const SP_REAL SIG_FCTR = 1.5;

                                MyFtr ftr;

                                ftr.pix = getVec2(vec.x, vec.y) * (1 << p);
                                ftr.scl = SIG_FCTR * BASE_SIGMA * pow(LAYER_STEP, vec.z) * (1 << p);
                                ftr.cst = dogs[s](x, y);

                                ftr.pyid = p;
                                ftr.lyid = s;

                                ftrs.push(ftr);
                            }
                        }
                    }
                }
            }

            return (ftrs.size() != 0) ? true : false;
        }

        void descript(Mem1<Ftr> &ftrs, const Mem1<MyFtr> &myfts, const Mem1<ImgSet> &imgsets){
            SP_LOGGER_SET("SIFT.descript");

            ftrs.reserve(myfts.size() * 2);

            for (int i = 0; i < myfts.size(); i++){
 
                const MyFtr &myft = myfts[i];

                const Mem2<float> &img = imgsets[myft.pyid].imgs[myft.lyid];

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

        bool calcFtrRefine(Vec3 &vec, const Mem1<Mem2<float> > &dogs) {

            const int x = round(vec.x);
            const int y = round(vec.y);
            const int s = round(vec.z);

            const Mem2<float> &dog0 = dogs[(s - 1) + 0];
            const Mem2<float> &dog1 = dogs[(s - 1) + 1];
            const Mem2<float> &dog2 = dogs[(s - 1) + 2];

            const SP_REAL val = dog1(x, y);

            const SP_REAL dxx = (dog1(x + 1, y + 0) + dog1(x - 1, y + 0) - val * 2.0f);
            const SP_REAL dyy = (dog1(x + 0, y + 1) + dog1(x + 0, y - 1) - val * 2.0f);
            const SP_REAL dss = (dog2(x + 0, y + 0) + dog0(x + 0, y + 0) - val * 2.0f);

            const SP_REAL dxy = (dog1(x + 1, y + 1) - dog1(x - 1, y + 1) - dog1(x + 1, y - 1) + dog1(x - 1, y - 1)) * 0.25f;
            const SP_REAL dxs = (dog2(x + 1, y + 0) - dog2(x - 1, y + 0) - dog0(x + 1, y + 0) + dog0(x - 1, y + 0)) * 0.25f;
            const SP_REAL dys = (dog2(x + 0, y + 1) - dog2(x + 0, y - 1) - dog0(x + 0, y + 1) + dog0(x + 0, y - 1)) * 0.25f;

            const SP_REAL dv2[3 * 3] = { dxx, dxy, dxs, dxy, dyy, dys, dxs, dys, dss };

            SP_REAL dst[3 * 3];
            if (invMat33(dst, dv2) == false) return false;

            const SP_REAL dx = (dog1(x + 1, y + 0) - dog1(x - 1, y + 0)) * 0.5f;
            const SP_REAL dy = (dog1(x + 0, y + 1) - dog1(x + 0, y - 1)) * 0.5f;
            const SP_REAL ds = (dog2(x + 0, y + 0) - dog0(x + 0, y + 0)) * 0.5f;

            const SP_REAL dv[3] = { dx, dy, ds };

            SP_REAL delta[3] = { 0 };
            mulMat(delta, 3, 1, dst, 3, 3, dv, 3, 1);

            if (fabs(delta[0]) >= 0.5 || fabs(delta[1]) >= 0.5 || fabs(delta[2]) >= 0.5) return false;

            const SP_REAL det = dxx * dyy - dxy * dxy;
            const SP_REAL tr = dxx + dyy;

            // check edge
            if (det <= 0) return false;
            if (tr * tr / det > EDGE_THRESH) return false;

            vec = getVec3(x - delta[0], y - delta[1], s - delta[2]);

            return true;
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

                const Rect rect = getRect2(img.dsize) - 1;

                const int radius = round(3.0 * scl);

                const double sdiv = 1.0 / (2.0 * scl * scl);

                for (int ry = -radius; ry <= radius; ry++) {
                    for (int rx = -radius; rx <= radius; rx++) {
                        const int ix = x + rx;
                        const int iy = y + ry;
                        if (inRect2(rect, ix, iy) == false) continue;

                        const double dx = img(ix + 1, iy + 0) - img(ix - 1, iy + 0);
                        const double dy = img(ix + 0, iy + 1) - img(ix + 0, iy - 1);

                        const double angle = atan2(dy, dx);

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

            const Rect rect = getRect2(img.dsize) - 1;

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

                    const SP_REAL bx = tx + (DSC_BLKS - 1) * 0.5f + 1.0f;
                    const SP_REAL by = ty + (DSC_BLKS - 1) * 0.5f + 1.0f;

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
                dsc.type = Dsc::DSC_SIFT;
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

#endif
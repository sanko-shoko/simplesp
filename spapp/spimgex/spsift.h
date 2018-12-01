//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
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

        class MyFeature : public Ftr{

        public:
            int octave;
            int layer;
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

            // clear data
            {
                m_ftrs.clear();
                m_imgsets.clear();
            }

            try{
                if (img.size() == 0) throw "image size";

                Mem2<float> imgf;
                cnvMem(imgf, img, 1.0 / 255.0);

                // make Imgs & DoGs
                makeImgSet(m_imgsets, imgf);

                Mem1<MyFeature> myfts;

                // detect features
                if(detect(myfts, m_imgsets) == false) throw "no featues";

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

         double INIT_SIGMA = 0.5;
         double BASE_SIGMA = 1.6;

         int LAYERS = 3;
         double LAYER_STEP = pow(2.0, 1.0 / LAYERS);


        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------

        void makeImgSet(Mem1<ImgSet> &imgsets, const Mem2<float> &img){

            const int octaves = round(log2(minVal(img.dsize[0], img.dsize[1]) / (8.0 * BASE_SIGMA)));

            imgsets.resize(octaves);

            for (int o = 0; o < octaves; o++){
                Mem1<Mem2<float> > &imgs = imgsets[o].imgs;
                Mem1<Mem2<float> > &dogs = imgsets[o].dogs;

                imgs.resize(LAYERS + 3);
                dogs.resize(LAYERS + 2);

                if (o == 0){
                    const double crnt = BASE_SIGMA;
                    const double prev = INIT_SIGMA;
                    const double dsig = sqrt(crnt * crnt - prev * prev);

                    gaussianFilter(imgs[0], img, dsig);
                }
                else{
                    pyrdown(imgs[0], imgsets[o - 1].imgs[LAYERS]);
                }

                for (int s = 1; s < LAYERS + 3; s++){
                    const double crnt = BASE_SIGMA * pow(LAYER_STEP, s);
                    const double prev = BASE_SIGMA * pow(LAYER_STEP, s - 1);
                    const double dsig = sqrt(crnt * crnt - prev * prev);

                    gaussianFilter(imgs[s], imgs[s - 1], dsig);
                }

                for (int s = 0; s < LAYERS + 2; s++){
                    subMem(dogs[s], imgs[s + 1], imgs[s]);
                }
            }
        }

        bool detect(Mem1<MyFeature> &ftrs, const Mem1<ImgSet> &imgsets){

            ftrs.clear();
            for (int o = 0; o < imgsets.size(); o++){
                const Mem1<Mem2<float> > &imgs = imgsets[o].imgs;
                const Mem1<Mem2<float> > &dogs = imgsets[o].dogs;

                for (int s = 1; s < LAYERS + 1; s++){
                    const int margin = round(4.0 * BASE_SIGMA);

                    for (int y = margin; y < dogs[s].dsize[1] - margin; y++){
                        for (int x = margin; x < dogs[s].dsize[0] - margin; x++){

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
                                Vec3 vec = getVec(x, y, s);

                                if (calcFtRefine(vec, imgsets[o]) == false) continue;

                                const double SIG_FCTR = 1.5;

                                MyFeature ftr;

                                ftr.pix = getVec(vec.x, vec.y) * (1 << o);
                                ftr.scl = SIG_FCTR * BASE_SIGMA * pow(LAYER_STEP, vec.z) * (1 << o);
                                ftr.cst = dogs[s](x, y);

                                ftr.octave = o;
                                ftr.layer = s;

                                ftrs.push(ftr);
                            }
                        }
                    }
                }
            }

            return (ftrs.size() != 0) ? true : false;
        }

        void descript(Mem1<Ftr> &ftrs, const Mem1<MyFeature> &myfts, const Mem1<ImgSet> &imgsets){

            ftrs.reserve(myfts.size() * 2);

            for (int i = 0; i < myfts.size(); i++){
 
                const MyFeature &myft = myfts[i];

                const Mem2<float> &img = imgsets[myft.octave].imgs[myft.layer];

                const Vec2 pix = myft.pix / (1 << myft.octave);
                const double scl = myft.scl / (1 << myft.octave);

                Ftr ftr = myft;

                const double PEAK_THRESH = 0.8;

                Mem1<Vec2> drcs;
                calcFtDrc(drcs, img, pix, scl, PEAK_THRESH);
        
                for (int a = 0; a < drcs.size(); a++) {
                    ftr.drc = drcs[a];
                    calcFtDsc(ftr.dsc, img, pix, ftr.drc, scl);
                    ftrs.push(ftr);
                }
            }
        }

        bool calcFtRefine(Vec3 &vec, const ImgSet &imgset) {

            const Mem1<Mem2<float>> &imgs = imgset.imgs;
            const Mem1<Mem2<float>> &dogs = imgset.dogs;

            const int x = round(vec.x);
            const int y = round(vec.y);
            const int s = round(vec.z);

            // refine position
            {
                const Mem2<float> &dog0 = dogs[(s - 1) + 0];
                const Mem2<float> &dog1 = dogs[(s - 1) + 1];
                const Mem2<float> &dog2 = dogs[(s - 1) + 2];

                const double val = dog1(x, y);

                const double dxx = (dog1(x + 1, y + 0) + dog1(x - 1, y + 0) - val * 2.0);
                const double dyy = (dog1(x + 0, y + 1) + dog1(x + 0, y - 1) - val * 2.0);
                const double dss = (dog2(x + 0, y + 0) + dog0(x + 0, y + 0) - val * 2.0);

                const double dxy = (dog1(x + 1, y + 1) - dog1(x - 1, y + 1) - dog1(x + 1, y - 1) + dog1(x - 1, y - 1)) * 0.25;
                const double dxs = (dog2(x + 1, y + 0) - dog2(x - 1, y + 0) - dog0(x + 1, y + 0) + dog0(x - 1, y + 0)) * 0.25;
                const double dys = (dog2(x + 0, y + 1) - dog2(x + 0, y - 1) - dog0(x + 0, y + 1) + dog0(x + 0, y - 1)) * 0.25;

                const double dv2[3 * 3] = { dxx, dxy, dxs, dxy, dyy, dys, dxs, dys, dss };

                double dst[3 * 3];
                if (invMat33(dst, dv2) == false) return false;

                const double dx = (dog1(x + 1, y + 0) - dog1(x - 1, y + 0)) * 0.5;
                const double dy = (dog1(x + 0, y + 1) - dog1(x + 0, y - 1)) * 0.5;
                const double ds = (dog2(x + 0, y + 0) - dog0(x + 0, y + 0)) * 0.5;

                const double dv[3] = { dx, dy, ds };

                double delta[3] = { 0 };
                mulMat(delta, 3, 1, dst, 3, 3, dv, 3, 1);

                if (fabs(delta[0]) >= 0.5 || fabs(delta[1]) >= 0.5 || fabs(delta[2]) >= 0.5) return false;

                const double det = dxx * dyy - dxy * dxy;
                const double tr = dxx + dyy;

                // check edge
                if (det <= 0) return false;
                if (tr * tr / det > EDGE_THRESH) return false;

                vec.x = x - delta[0];
                vec.y = y - delta[1];
                vec.z = s - delta[2];
            }

            return true;
        }
        
        template <typename TYPE>
        void calcFtDrc(Mem1<Vec2> &drcs, const Mem2<TYPE> &img, const Vec2 &pix, const double scl, const double pthresh = 0.8) {

            const int BINS = 36;
            Mem1<double> hist(BINS);

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
        }

        template <typename TYPE>
        void calcFtDsc(Dsc &dsc, const Mem2<TYPE> &img, const Vec2 &pix, const Vec2 &drc, const double scl) {

            const int DSC_BINS = 8;
            const int DSC_BLKS = 4;

            Mem3<double> hist(DSC_BLKS + 2, DSC_BLKS + 2, DSC_BINS + 1);
            hist.zero();

            const Rect rect = getRect2(img.dsize) - 1;

            const double kx = pix.x;
            const double ky = pix.y;
            const double ks = scl;

            const double DCS_SCL_FCTR = 3.0;
            const double block = DCS_SCL_FCTR * ks;

            const int radius = round(block * sqrt(2.0) * (DSC_BLKS + 1) * 0.5);

            const double sdiv = 1.0 / (2.0 * DSC_BLKS * DSC_BLKS);

            const double tcos = +drc.x;
            const double tsin = -drc.y;

            for (int ry = -radius; ry <= radius; ry++) {
                for (int rx = -radius; rx <= radius; rx++) {
                    const int ix = round(kx + rx);
                    const int iy = round(ky + ry);
                    if (isInRect2(rect, ix, iy) == false) continue;

                    const double tx = (tcos * rx - tsin * ry) / block;
                    const double ty = (tsin * rx + tcos * ry) / block;

                    const double bx = tx + (DSC_BLKS - 1) * 0.5 + 1.0;
                    const double by = ty + (DSC_BLKS - 1) * 0.5 + 1.0;

                    if (bx > 0 && bx < DSC_BLKS && by > 0 && by < DSC_BLKS) {

                        const double dx = img(ix + 1, iy + 0) - img(ix - 1, iy + 0);
                        const double dy = img(ix + 0, iy + 1) - img(ix + 0, iy - 1);

                        const double ndx = tcos * dx - tsin * dy;
                        const double ndy = tsin * dx + tcos * dy;

                        double angle = atan2(ndy, ndx);
                        if (angle < 0) angle += 2 * SP_PI;

                        const double ba = angle * (DSC_BINS / (2 * SP_PI));

                        const int ibx = floor(bx);
                        const int iby = floor(by);
                        const int iba = floor(ba);

                        const double abx = bx - ibx;
                        const double aby = by - iby;
                        const double aba = ba - iba;

                        const double val = pythag(dx, dy) * exp(-(tx * tx + ty * ty) * sdiv);

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

            Mem1<double> ddsc(DSC_BLKS * DSC_BLKS * DSC_BINS);
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
                const double DSC_MAG_THR = 0.2;

                const double sq = sumSq(ddsc);
                const double thresh = sqrt(sq) * DSC_MAG_THR;

                for (int k = 0; k < ddsc.size(); k++) {
                    ddsc[k] = minVal(ddsc[k], thresh);
                }
            }

            {
                const double sq = sumSq(ddsc);
                const double nrm = 1.0 / maxVal(sqrt(sq), SP_SMALL);

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
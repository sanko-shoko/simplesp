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
        SP_LOGGER_INSTANCE;

        struct KeyPoint{
            Vec2 pix;
            Vec2 drc;
            int octave;
            double layer;
            double scale;
            double contrast;
        };

        SIFT(){
        }
        
        SIFT(const SIFT &sift) {
            *this = sift;
        }
        SIFT& operator = (const SIFT &sift) {
            m_fts = sift.m_fts;
            return *this;
        }

    private:

        Mem1<Feature> m_fts;

    public:

        //--------------------------------------------------------------------------------
        // data
        //--------------------------------------------------------------------------------

        const Mem1<Feature>* getFeatrue() const {
            return (m_fts.size() > 0) ? &m_fts : NULL;
        }


        //--------------------------------------------------------------------------------
        // execute detection and description
        //--------------------------------------------------------------------------------

        bool execute(const void *src, const int dsize0, const int dsize1, const int ch){

            Mem2<Byte> gry;
            cnvPtrToImg(gry, src, dsize0, dsize1, ch);
            return _execute(gry);
        }

        bool execute(const Mem2<Col3> &src){

            Mem2<Byte> gry;
            cnvImg(gry, src);
            return _execute(gry);
        }

        bool execute(const Mem2<Byte> &src){

            return _execute(src);
        }

    private:

        bool _execute(const Mem2<Byte> &src){
            SP_LOGGER_SET("-execute");

            // clear data
            {
                m_fts.clear();
            }

            try{
                if (src.size() == 0) throw "image size";

                Mem2<float> img;
                cnvMem(img, src, 1.0 / 255.0);

                // make DoG
                makeDoG(img);

                // detect key points
                Mem1<KeyPoint> keys = detect();
                if (keys.size() == 0) throw "no keys";

                // descript features
                m_fts = descript(keys);
            }
            catch (const char *str){
                SP_PRINTF("SIFT.execute [%s]\n", str);

                return false;
            }

            return true;
        }

    private:
        double BLOB_CONTRAST = 0.01;
        double EDGE_THRESH = 10.0;

        double INIT_SIGMA = 0.5;
        double BASE_SIGMA = 1.6;

        int LAYERS = 3;
        double LAYER_STEP = pow(2.0, 1.0 / LAYERS);

        Mem1<Mem1<Mem2<float> > > m_dogsList;
        Mem1<Mem1<Mem2<float> > > m_imgsList;

        void makeDoG(const Mem2<float> &img){

            const int octaves = round(log2(minVal(img.dsize[0], img.dsize[1]) / (8.0 * BASE_SIGMA)));

            m_dogsList.resize(octaves);
            m_imgsList.resize(octaves);

            for (int o = 0; o < octaves; o++){
                Mem1<Mem2<float>> &dogs = m_dogsList[o];
                Mem1<Mem2<float>> &imgs = m_imgsList[o];

                dogs.resize(LAYERS + 2);
                imgs.resize(LAYERS + 3);

                if (o == 0){
                    const double crnt = BASE_SIGMA;
                    const double prev = INIT_SIGMA;
                    const double dsig = sqrt(crnt * crnt - prev * prev);

                    gaussianFilter(imgs[0], img, dsig);
                }
                else{
                    pyrdown(imgs[0], m_imgsList[o - 1][LAYERS]);
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

        Mem1<KeyPoint> detect(){
            Mem1<KeyPoint> keyPnts;

            for (int o = 0; o < m_dogsList.size(); o++){
                const Mem1<Mem2<float>> &dogs = m_dogsList[o];

                for (int s = 1; s < LAYERS + 1; s++){
                    const int margin = round(4.0 * BASE_SIGMA);

                    for (int y = margin; y < dogs[s].dsize[1] - margin; y++){
                        for (int x = margin; x < dogs[s].dsize[0] - margin; x++){

                            const float base = dogs[s](x, y);

                            bool npeak = true;
                            bool ppeak = true;
                            for (int ws = -1; ws <= 1; ws++){
                                for (int wy = -1; wy <= 1; wy++){
                                    for (int wx = -1; wx <= 1; wx++){
                                        const float val = dogs[s + ws](x + wx, y + wy);
                                        if (val < base) npeak = false;
                                        if (val > base) ppeak = false;
                                    }
                                }
                            }

                            if (npeak || ppeak){
                                addKeyPoint(keyPnts, o, x, y, s);
                            }
                        }
                    }
                }
            }

            return keyPnts;
        }

        void addKeyPoint(Mem1<KeyPoint> &keyPnts, const int o, const int x, const int y, const int s){

            const Mem1<Mem2<float>> &dogs = m_dogsList[o];
            const Mem1<Mem2<float>> &imgs = m_imgsList[o];

            double fx = x;
            double fy = y;
            double fs = s;

            // refine position
            {
                const Mem2<float> &dog0 = dogs[(s - 1) + 0];
                const Mem2<float> &dog1 = dogs[(s - 1) + 1];
                const Mem2<float> &dog2 = dogs[(s - 1) + 2];

                const double val = dog1(x, y);

                // check contrast
                if (fabs(val) < BLOB_CONTRAST) return;

                const double dxx = (dog1(x + 1, y + 0) + dog1(x - 1, y + 0) - val * 2.0);
                const double dyy = (dog1(x + 0, y + 1) + dog1(x + 0, y - 1) - val * 2.0);
                const double dss = (dog2(x + 0, y + 0) + dog0(x + 0, y + 0) - val * 2.0);

                const double dxy = (dog1(x + 1, y + 1) - dog1(x - 1, y + 1) - dog1(x + 1, y - 1) + dog1(x - 1, y - 1)) * 0.25;
                const double dxs = (dog2(x + 1, y + 0) - dog2(x - 1, y + 0) - dog0(x + 1, y + 0) + dog0(x - 1, y + 0)) * 0.25;
                const double dys = (dog2(x + 0, y + 1) - dog2(x + 0, y - 1) - dog0(x + 0, y + 1) + dog0(x + 0, y - 1)) * 0.25;

                const double dv2[3 * 3] = { dxx, dxy, dxs, dxy, dyy, dys, dxs, dys, dss };

                double dst[3 * 3];
                if (invMat33(dst, dv2) == false) return;

                const double dx = (dog1(x + 1, y + 0) - dog1(x - 1, y + 0)) * 0.5;
                const double dy = (dog1(x + 0, y + 1) - dog1(x + 0, y - 1)) * 0.5;
                const double ds = (dog2(x + 0, y + 0) - dog0(x + 0, y + 0)) * 0.5;

                const double dv[3] = { dx, dy, ds };

                double delta[3] = { 0 };
                mulMat(delta, 3, 1, dst, 3, 3, dv, 3, 1);

                if (fabs(delta[0]) >= 0.5 || fabs(delta[1]) >= 0.5 || fabs(delta[2]) >= 0.5) return;

                const double det = dxx * dyy - dxy * dxy;
                const double tr = dxx + dyy;

                // check edge
                if (det <= 0) return;
                if (tr * tr / det > EDGE_THRESH) return;

                fx = x - delta[0];
                fy = y - delta[1];
                fs = s - delta[2];
            }

            KeyPoint keyPnt;

            keyPnt.pix = getVec(fx, fy) * (1 << o);
            // keyPnt.drc = next ->

            keyPnt.octave = o;
            keyPnt.layer = fs;
            keyPnt.scale = 2.0 * BASE_SIGMA * pow(LAYER_STEP, fs) * (1 << o);
            keyPnt.contrast = dogs[s](x, y);


            const int ORI_BINS = 36;
            Mem1<double> hist(ORI_BINS);

            // calc orientation hist
            {
                hist.zero();

                const Mem2<float> &img = imgs[s];
                const Rect rect = getRect2(img.dsize) - 1;

                const double ORI_SIG_FCTR = 1.5;
                const double sigma = ORI_SIG_FCTR * fs;
                const int radius = round(3.0 * sigma);

                const double sdiv = 1.0 / (2.0 * sigma * sigma);


                for (int ry = -radius; ry <= radius; ry++){
                    for (int rx = -radius; rx <= radius; rx++)    {
                        const int ix = x + rx;
                        const int iy = y + ry;
                        if (isInRect2(rect, ix, iy) == false) continue;

                        const double dx = img(ix + 1, iy + 0) - img(ix - 1, iy + 0);
                        const double dy = img(ix + 0, iy + 1) - img(ix + 0, iy - 1);

                        const double angle = atan2(dy, dx);

                        int bin = round(angle * ORI_BINS / (2 * SP_PI));
                        if (bin < 0) bin += ORI_BINS;

                        hist[bin] += pythag(dx, dy) * exp(-(rx * rx + ry * ry) * sdiv);
                    }
                }
            }

            Mem1<double> fhist(ORI_BINS);

            // filtering
            for (int i = 0; i < ORI_BINS; i++){
                const double hi = hist(i);
                const double hp1 = hist(i + ORI_BINS - 1, true);
                const double hp2 = hist(i + ORI_BINS - 2, true);
                const double hn1 = hist(i + ORI_BINS + 1, true);
                const double hn2 = hist(i + ORI_BINS + 2, true);

                fhist[i] = (hi * 6.0 + (hn1 + hp1) * 4.0 + (hn2 + hp2)) / 16.0;
            }

            // detect peak
            const double ORI_PEAK_THRESH = 0.8;
            const double thresh = maxVal(fhist) * ORI_PEAK_THRESH;
    
            for (int i = 0; i < ORI_BINS; i++){
                const double hi = fhist(i);
                const double hp1 = fhist(i + ORI_BINS - 1, true);
                const double hn1 = fhist(i + ORI_BINS + 1, true);

                if (hi > thresh && hi > maxVal(hp1, hn1)){
                    const double finei = i + 0.5 * (hp1 - hn1) / (hp1 + hn1 - 2 * hi);
                    const double angle = finei * (2.0 * SP_PI / ORI_BINS);

                    keyPnt.drc = getVec(cos(angle), sin(angle));

                    keyPnts.push(keyPnt);
                }
            }
        }

        Mem1<Feature> descript(const Mem1<KeyPoint> &keys){
            Mem1<Feature> fts(keys.size());

            const int DSC_BINS = 8;
            const int DSC_BLKS = 4;

            for (int i = 0; i < keys.size(); i++){

                Mem3<double> hist(DSC_BLKS + 2, DSC_BLKS + 2, DSC_BINS + 1);
                hist.zero();

                const KeyPoint &key = keys[i];
                const Mem2<float> &img = m_imgsList[key.octave][round(key.layer)];
                const Rect rect = getRect2(img.dsize) - 1;

                const double kx = key.pix.x / (1 << key.octave);
                const double ky = key.pix.y / (1 << key.octave);

                const double DCS_SCL_FCTR = 3.0;
                const double scale = BASE_SIGMA * pow(LAYER_STEP, key.layer);
                const double block = DCS_SCL_FCTR * scale;

                const int radius = round(block * sqrt(2.0) * (DSC_BLKS + 1) * 0.5);
                
                const double sdiv = 1.0 / (2.0 * DSC_BLKS * DSC_BLKS);

                const double tcos = +key.drc.x;
                const double tsin = -key.drc.y;

                for (int ry = -radius; ry <= radius; ry++){
                    for (int rx = -radius; rx <= radius; rx++){
                        const int ix = round(kx + rx);
                        const int iy = round(ky + ry);
                        if (isInRect2(rect, ix, iy) == false) continue;

                        const double tx = (tcos * rx - tsin * ry) / block;
                        const double ty = (tsin * rx + tcos * ry) / block;

                        const double bx = tx + (DSC_BLKS - 1) * 0.5 + 1.0;
                        const double by = ty + (DSC_BLKS - 1) * 0.5 + 1.0;

                        if (bx > 0 && bx < DSC_BLKS && by > 0 && by < DSC_BLKS){

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

                Mem1<double> dsc(DSC_BLKS * DSC_BLKS * DSC_BINS);
                {
                    int cnt = 0;
                    for (int iby = 1; iby < DSC_BLKS + 1; iby++){
                        for (int ibx = 1; ibx < DSC_BLKS + 1; ibx++){
                            hist(ibx, iby, 0) += hist(ibx, iby, DSC_BINS);

                            for (int k = 0; k < DSC_BINS; k++){
                                dsc[cnt++] = hist(ibx, iby, k);
                            }
                        }
                    }
                }

                {
                    const double DSC_MAG_THR = 0.2;

                    const double sq = sumSq(dsc);
                    const double thresh = sqrt(sq) * DSC_MAG_THR;

                    for (int k = 0; k < dsc.size(); k++){
                        dsc[k] = minVal(dsc[k], thresh);
                    }
                }

                {
                    const double sq = sumSq(dsc);
                    const double nrm = 1.0 / maxVal(sqrt(sq), SP_SMALL);

                    fts[i].pix = key.pix;
                    fts[i].drc = key.drc;
                    fts[i].scl = key.scale;

                    fts[i].dsc.resize(dsc.size() * sizeof(float));

                    float *ptr = reinterpret_cast<float*>(fts[i].dsc.ptr);
                    for (int k = 0; k < dsc.size(); k++){
                        ptr[k] = static_cast<float>(dsc[k] * nrm);
                    }
                }
            }

            return fts;
        }
    };


}

#endif
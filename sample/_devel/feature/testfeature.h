
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spimgex/spfeature.h"
#include "spapp/spimgex/spblob.h"

//--------------------------------------------------------------------------------
// feature ex
//--------------------------------------------------------------------------------

namespace sp {

    class FastBlob {

    public:
        class MyFeature : public Feature {

        public:
            int octave;
        };

    private:

        Mem1<Feature> m_fts;

    private:

        double BLOB_CONTRAST = 0.01;

    public:

        FastBlob() {
        }

        FastBlob(const FastBlob &test) {
            *this = test;
        }
        FastBlob& operator = (const FastBlob &test) {
            m_fts = test.m_fts;
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

        const Mem1<Feature>* getFeatures() const {
            return (m_fts.size() > 0) ? &m_fts : NULL;
        }

        template<typename T>
        static Mem1<Feature> getFeatures(const Mem2<T> &img, const double contrast = 0.01) {
            Test test;
            test.setParam(contrast);
            test.execute(img);
            return (test.getFeatures() != NULL) ? *test.getFeatures() : Mem1<Feature>();
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
        double m_testScale = 0.7;

    private:

        bool _execute(const Mem2<Byte> &img) {

            // clear data
            {
                m_fts.clear();
            }

            try {
                if (img.size() == 0) throw "image size";

                detect(img);
            }
            catch (const char *str) {
                SP_PRINTF("Test.execute [%s]\n", str);

                return false;
            }

            return true;
        }

    private:

        bool detect(const Mem2<Byte> &img) {
            SP_LOGGER_SET("detect");

            const int pynum = 7;
            Mem1<Mem2<Byte> > pimgs(pynum);

            {
                SP_LOGGER_SET("pyrdown");
                pimgs[0] = img;
                for (int s = 0; s < pynum - 1; s++) {
                    //rescale(pimgs[s + 1], pimgs[s], 0.5, 0.5);
                    //gaussianFilter3x3Fast(pimgs[s], pimgs[s]);
                    pyrdown(pimgs[s + 1], pimgs[s]);
                    //saveBMP("test.bmp", pimgs[s]);
                }
            }
            Mem1<Mem1<Feature> > fts(pynum);
            {
                SP_LOGGER_SET("detectBlob");
                for (int s = 0; s < pynum - 1; s++) {
                    Mem1<Feature> tmp;
                    detectBlob(tmp, pimgs[s + 1], 15);

                    const int t = maxVal(0, s - 1);
                    for (int i = 0; i < tmp.size(); i++) {
                        tmp[i].pix *= pow(2, (s + 1) - t);
                        tmp[i].scl = pow(2, (s + 1) - t);
                        fts[t].push(tmp[i]);
                    }
                }
            }

            Mem1<MyFeature> keys;
            Mem1<MyFeature> refs;
            keys.reserve(img.size());
            refs.reserve(img.size());
            {
                SP_LOGGER_SET("refine");
                for (int s = 0; s < pynum - 1; s++) {

                    for (int i = 0; i < fts[s].size(); i++) {
                        MyFeature key;
                        key.pix = fts[s][i].pix;
                        key.scl = fts[s][i].scl;
                        key.octave = s;
                        keys.push(key);

                        if (refine(key, pimgs[s], fts[s][i].pix, fts[s][i].scl) == true) {
                            //key.pix *= scale / a;
                            //key.scl *= scale / a;
                            refs.push(key);
                        }
                        else {
                            refs.push(key);
                        }
                    }
                }
            }
            //refs = keys;
            m_fts.resize(refs.size());
            for (int i = 0; i < m_fts.size(); i++) {
                m_fts[i] = refs[i];
            }
            printf("twet");
            SP_ONCE(
            SP_HOLDER_SET("keys", keys);
            SP_HOLDER_SET("refs", refs);

            for (int s = 0; s < pynum - 1; s++) {
                SP_HOLDER_SET(strFormat("test%d", s).c_str(), pimgs[s]);
            }
            );

            return true;
        }

        bool refine(Feature &key, const Mem2<Byte> &img, const Vec2 &uv, const double ss) {
            const int u = round(uv.x);
            const int v = round(uv.y);

            const int wsize = round(3 * ss) + 1;
            const int half = wsize / 2;

            int base = round(ss) - 1;

            const double m = (img(u - base, v - base) + img(u + base, v - base) + img(u - base, v + base) + img(u + base, v + base)) / 4.0;
            double sum = 0.0;
            for (int y = -base; y <= base; y++) {
                for (int x = -base; x <= base; x++) {
                    sum += img(u - x, v - y);
                }
            }
            sum /= square(2 * base + 1);

            // y = a * exp(-X^2 / (2 * c^2)) + b
            double a = img(u, v) - sum;
            double b = sum;
            double c = ss - 1;
            Vec2 flow = getVec(0, 0);

            Mat weight(wsize, wsize);
  
            for (int it = 0; it < 7; it++) {
                Mat tw(wsize * wsize, wsize * wsize);
                tw.zero();
                for (int y = 0; y < wsize; y++) {
                    for (int x = 0; x < wsize; x++) {
                        const int i = y * wsize + x;
                        const Vec2 d = getVec(x - half, y - half);

                        const double r = c * 1;
                        const double n = exp(-sqVec(d) / (2 * r * r));

                        weight(x, y) = n;
                        tw(i, i) = n;
                    }
                }

                // Ai = [dI/dx, dI/dy], A = [A0, A1, ... An-1]^T
                Mat A(wsize * wsize, 5);
                Mat B(wsize * wsize, 1);
                Mat I(wsize * wsize, 1);
                for (int y = 0; y < wsize; y++) {
                    for (int x = 0; x < wsize; x++) {
                        const int i = y * wsize + x;
                        const Vec2 d = getVec(x - half, y - half);

                        const double n = exp(-sqVec(d) / (2 * c * c));

                        const double gx = a * n * (-(x - half) / (c * c));
                        const double gy = a * n * (-(y - half) / (c * c));

                        const double ga = n;
                        const double gb = 1.0;
                        const double gc = a * n * (1.5 * sqVec(d) / (c * c * c));
                        A(i, 0) = gx;
                        A(i, 1) = gy;
                        A(i, 2) = ga;
                        A(i, 3) = gb;
                        A(i, 4) = gc;

                        I(i, 0) = a * n + b;
                    }
                }


                const Vec2 pix = getVec(u, v) + flow;

                Mem1<double> errs(wsize * wsize);
                for (int y = 0; y < wsize; y++) {
                    for (int x = 0; x < wsize; x++) {
                        const int i = y * wsize + x;
                        const Vec2 v = getVec(x - half, y - half);

                        const Vec2 p0 = pix + v;

                        const double gx = A(i, 0);
                        const double gy = A(i, 1);

                        const double ga = A(i, 2);
                        const double gb = A(i, 3);
                        const double gc = A(i, 4);

                        double d = I(i, 0) - acs2(img, p0.x, p0.y);

                        const double nv = fabs(d);
                        d = d * minVal(5.0, nv) / (nv + SP_SMALL);

                        errs[i] = fabs(d);
                        B(i, 0) = d;
                    }
                }
                //solveEq(result, A, B, errs);
                const Mat At = trnMat(A);
                const Mat AtW = calcAtWeight(A, errs, 1.0) * tw;
                const Mat invAtA = invMat(AtW * A);
                const Mat AtB = AtW * B;

                Mat result;
                result = invMat(AtW * A) * AtW * B;
                if (result.size() == 0) {
                    return false;
                }
                {
                    Vec2 delta = getVec(result[0], result[1]);
                    const double norm = normVec(delta);
                    flow += delta*minVal(1.0, norm) / norm;
                }
    
                {
                    double da = result[2];
                    double db = result[3];
                    double dc = result[4];
                    //printf("%lf \n", delta);
                    a -= da;
                    b -= db;
                    c -= dc;
                    const double norm = fabs(c);
                    c = c * maxVal(1.5, minVal(5.0, norm)) / norm;

                    //if (c > 3.0) return false;
                }
            }

            key.pix = (getVec(u, v) + flow);
            key.scl = c;

            return true;
        }

    };
}
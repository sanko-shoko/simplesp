
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spfilter.h"

//--------------------------------------------------------------------------------
// feature ex
//--------------------------------------------------------------------------------

namespace sp {
    class Test {

    public:

        struct KeyPoint {
            Vec2 pix;
            Vec2 drc;
            int pyrid;
            double scale;
            double contrast;
        };

    private:

        Mem1<Feature> m_fts;

    private:

        double BLOB_CONTRAST = 0.01;

    public:

        Test() {
        }

        Test(const Test &test) {
            *this = test;
        }
        Test& operator = (const Test &test) {
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
            SP_LOGGER_START("detect");

            Mem1<Vec2> pixs;
            pixs.reserve(img.size());

            Mem2<int> map(img.dsize);
            map.zero();

            const int pynum = 5;
            Mem1<Mem2<Byte> > pimgs(pynum);
            Mem1<Mem2<float> > laps(pynum);
            Mem1<Mem2<bool> > msks(pynum);

            Mem1<Mem2<float> > sobelXs(pynum);
            Mem1<Mem2<float> > sobelYs(pynum);

            SP_LOGGER_START("filter");
            for (int s = 0; s < pynum; s++) {
                pyrdownFast(pimgs[s], (s == 0) ? img : pimgs[s - 1]);
                //gaussianFilter3x3Fast(pyimgs[s], pyimgs[s]);
                blobFilter(laps[s], msks[s], pimgs[s]);

            }
            SP_LOGGER_STOP("filter");

            Mem1<KeyPoint> keys;
            Mem1<KeyPoint> refs;
            keys.reserve(img.size());
            refs.reserve(img.size());

            for (int s = 0; s < pynum; s++) {
                const Mem2<float> &lap = laps[s];
                const Mem2<bool> &msk = msks[s];
                
                const float *plap = lap.ptr;
                const bool *pmsk = msk.ptr;

                const int step = lap.dsize[0];
                const double scale = pow(2, s + 1);

                for (int v = 0; v < lap.dsize[1]; v++) {
                    for (int u = 0; u < lap.dsize[0]; u++) {

                        const bool m = pmsk[v * step + u];
                        const float c = plap[v * step + u];

                        if (m == true && fabs(c) >= 0.1 * (SP_BYTEMAX)) {
                            KeyPoint key;
                            key.pix = getVec(u, v) * scale;
                            key.scale = scale;
                            key.pyrid = s;
                            keys.push(key);

                            if (refine(key, pimgs, s, u, v) == true) {
                                refs.push(key);
                            }
                            else {
                                refs.push(key);
                            }
                        }
                    }
                }
            }
            SP_LOGGER_STOP("detect");

            SP_HOLDER_SET("keys", keys);
            SP_HOLDER_SET("refs", refs);

            for (int s = 0; s < pynum; s++) {
                SP_HOLDER_SET(strFormat("test%d", s).c_str(), pimgs[s]);
            }
            return true;
        }

        bool refine(KeyPoint &key, const Mem1<Mem2<Byte> > &imgs, const int s, const int u, const int v) {
            const Mem2<Byte> &img = imgs[s];

            const int winSize = 7;
            const int half = winSize / 2;

            const double m = (img(u - 2, v - 2) + img(u + 2, v - 2) + img(u - 2, v + 2) + img(u + 2, v + 2)) / 4.0;

            // y = a * exp(-X^2 / (2 * c^2)) + b
            double a = img(u, v) - m;
            double b = m;
            double c = 1.0;
            Vec2 flow = getVec(0, 0);

            for (int ii = 0; ii < 4; ii++) {

                {
                    // Ai = [dI/dx, dI/dy], A = [A0, A1, ... An-1]^T
                    Mat A(winSize * winSize, 2);
                    Mat I(winSize * winSize, 1);
                    Mat AtA(2, 2);
                    Mat AtB(2, 1);

                    AtB.zero();
                    AtA.zero();

                    for (int y = 0; y < winSize; y++) {
                        for (int x = 0; x < winSize; x++) {
                            const int i = y * winSize + x;
                            const Vec2 d = getVec(x - half, y - half);

                            const double n = a * exp(-sqVec(d) / (2 * c * c));

                            const double gx = n * (-(x - half) / (c * c)) / SP_BYTEMAX;
                            const double gy = n * (-(y - half) / (c * c)) / SP_BYTEMAX;

                            A(i, 0) = gx;
                            A(i, 1) = gy;

                            const double p = n + b;
                            I(i, 0) = p;

                            AtA(0, 0) += gx * gx;
                            AtA(0, 1) += gx * gy;
                            AtA(1, 0) += gy * gx;
                            AtA(1, 1) += gy * gy;
                        }
                    }

                    const Mat invAtA = invMat(AtA);

                    const Vec2 pix = getVec(u, v) + flow;

                    for (int y = 0; y < winSize; y++) {
                        for (int x = 0; x < winSize; x++) {
                            const int i = y * winSize + x;
                            const Vec2 v = getVec(x - half, y - half);

                            const Vec2 p0 = pix + v;

                            const double gx = A(i, 0);
                            const double gy = A(i, 1);

                            double d = (I(i, 0) - acs2(img, p0.x, p0.y)) / SP_BYTEMAX;

                            const double nv = fabs(d);
                            d = d * minVal(10.0 / SP_BYTEMAX, nv) / (nv + SP_SMALL);

                            AtB(0, 0) += gx * d;
                            AtB(1, 0) += gy * d;
                        }
                    }

                    const Mat result = invAtA * AtB;
                    if (result.size() == 0) {
                        return false;
                    }

                    Vec2 delta = getVec(result[0], result[1]);
                    const double norm = normVec(delta);

                    flow += delta * minVal(1.0, norm) / norm;
                }

                if(ii < 1){
                    // Ai = [dI/dx, dI/dy], A = [A0, A1, ... An-1]^T
                    Mat A(winSize * winSize, 3);
                    Mat I(winSize * winSize, 1);
                    Mat AtA(3, 3);
                    Mat AtB(3, 1);
                    AtA.zero();
                    AtB.zero();

                    for (int y = 0; y < winSize; y++) {
                        for (int x = 0; x < winSize; x++) {
                            const int i = y * winSize + x;
                            const Vec2 d = getVec(x - half, y - half);

                            const double n = a * exp(-sqVec(d) / (2 * c * c));

                            const double ga = n / SP_BYTEMAX;
                            const double gb = 1.0 / SP_BYTEMAX;
                            const double gc = n * (1.5 * sqVec(d) / (c * c * c)) / SP_BYTEMAX;

                            A(i, 0) = ga;
                            A(i, 1) = gb;
                            A(i, 2) = gc;

                            AtA(0, 0) += ga * ga;
                            AtA(0, 1) += ga * gb;
                            AtA(0, 2) += ga * gc;

                            AtA(1, 0) += gb * ga;
                            AtA(1, 1) += gb * gb;
                            AtA(1, 2) += gb * gc;

                            AtA(2, 0) += gc * ga;
                            AtA(2, 1) += gc * gb;
                            AtA(2, 2) += gc * gc;

                            const double p = n + b;
                            I(i, 0) = p;
                        }
                    }

                    const Mat invAtA = invMat(AtA);

                    const Vec2 pix = getVec(u, v) + flow;

                    for (int y = 0; y < winSize; y++) {
                        for (int x = 0; x < winSize; x++) {
                            const int i = y * winSize + x;
                            const Vec2 v = getVec(x - half, y - half);

                            const Vec2 p0 = pix + v;

                            const double ga = A(i, 0);
                            const double gb = A(i, 1);
                            const double gc = A(i, 2);

                            const double d = (I(i, 0) - acs2(img, p0.x, p0.y)) / SP_BYTEMAX;
                            AtB(0, 0) += ga * d;
                            AtB(1, 0) += gb * d;
                            AtB(2, 0) += gc * d;
                        }
                    }

                    const Mat result = invAtA * AtB;
                    if (result.size() == 0) {
                        printf("size%d ", 1);
                        return false;
                    }

                    double da = result[0];
                    double db = result[1];
                    double dc = result[2];
                    //printf("%lf \n", delta);
                    a -= da;
                    b -= db;
                    c -= dc;
                }
            }

            const double scale = pow(2, s + 1);
            key.pix = (getVec(u, v) + flow) * scale;
            key.scale = scale * c;
            key.pyrid = s;

            return true;
        }

    };
}
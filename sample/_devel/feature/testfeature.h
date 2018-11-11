
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
           
            return true;
        }

    };
}
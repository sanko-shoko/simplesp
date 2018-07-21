//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_PROJECTOR_H__
#define __SP_PROJECTOR_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"

namespace sp{

    SP_CPUFUNC Mem2<Byte> genRandomPattern(const int dsize[], const double rate, const int seed = 0){
        Mem2<Byte> ptn(dsize);
        ptn.zero();

        srand(seed);
        for (int v = 0; v < ptn.dsize[1]; v++) {
            for (int u = 0; u < ptn.dsize[0]; u++) {
                if ((rand() % 101) < rate * 100) {
                    ptn(u, v) = SP_BYTEMAX;
                }
            }
        }

        return ptn;
    }

    SP_CPUFUNC Mem2<Byte> genRandomPattern(const int dsize0, const int dsize1, const double rate, const int seed = 0) {
        int dsize[2] = {dsize0, dsize1};
        return genRandomPattern(dsize, rate, seed);
    }

    class StructuredLight {

    protected:
        int m_dsize[2];
        int m_axis;

    public:

        void init(const int dsize0, const int dsize1, const int axis) {
            m_dsize[0] = dsize0;
            m_dsize[1] = dsize1;
            m_axis = axis;
        }

        virtual bool isValid() const {
            return (m_dsize[0] > 0 && m_dsize[1] > 0) ? true : false;
        }

        virtual int getCodeNum() const {
            return 0;
        }
        
        Mem2<Byte> getPlain(const Byte val) const {
            SP_ASSERT(isValid() == true);

            Mem2<Byte> img(m_dsize);
            setElm(img, val);

            return img;
        }

    };

    class GrayCode : public StructuredLight {

    public:
        GrayCode() {
            StructuredLight::init(0, 0, 0);
        }

        GrayCode(const int dsize0, const int dsize1, const int axis) {
            init(dsize0, dsize1, axis);
        }

        void init(const int dsize0, const int dsize1, const int axis) {
            StructuredLight::init(dsize0, dsize1, axis);
        }

        virtual int getCodeNum() const {
            return ceil(log(m_dsize[m_axis]) / log(2.0));
        }

        Mem1<Mem2<Byte> > encode() const{
            SP_ASSERT(isValid() == true);

            Mem1<Mem2<Byte> > imgs;

            const int num = getCodeNum();

            for (int i = 0; i < num; i++) {
                Mem2<Byte> img(m_dsize);

                if (m_axis == 0) {
                    for (int u = 0; u < m_dsize[0]; u++) {
                        const Byte bit = getBits(num, u)[i];
                        for (int v = 0; v < m_dsize[1]; v++) {
                            img(u, v) = bit * SP_BYTEMAX;
                        }
                    }
                }
                else {
                    for (int v = 0; v < m_dsize[1]; v++) {
                        const Byte bit = getBits(num, v)[i];
                        for (int u = 0; u < m_dsize[0]; u++) {
                            img(u, v) = bit * SP_BYTEMAX;
                        }
                    }
                }
                imgs.push(img);
            }

            return imgs;
        }

        Mem2<double> decode(const Mem1<Mem2<Byte> > &imgs, const Mem2<Byte> &wimg, const Mem2<Byte> &bimg, const int thresh = 10) const {
            SP_ASSERT(isValid() == true);

            Mem2<double> map(imgs[0].dsize);
            setElm(map, -1.0);

            const int num = getCodeNum();

            Mem1<Byte> bits(num);

            for (int i = 0; i < map.size(); i++) {

                const Byte w = wimg[i];
                const Byte b = bimg[i];
                if (w - b > thresh) {
                    for (int j = 0; j < num; j++) {
                        bits[j] = (2 * imgs[j][i] > w + b) ? 1 : 0;
                    }
                    map[i] = getIndex(bits);
                }
            }

            return map;
        }


    private:

        Mem1<Byte> getBits(const int size, const int index) const {
            Mem1<Byte> bits(size);

            for (int i = 0; i < size; i++) {
                const int t = round(pow(2, size - i));

                bits[i] = ((index + t / 2) / t) % 2;
            }
            return bits;
        }

        int getIndex(const Mem1<Byte> &bits) const {
            const int size = bits.size();

            int index = 0;

            Byte v = 0;
            for (int i = 0; i < size; i++) {
                const int t = round(pow(2, size - i));

                v = v ^ bits[i];
                index += v * (t / 2);
            }

            return index;
        }
    };


    class PhaseShift : public StructuredLight {
    private:
        int m_period;

    public:
        PhaseShift() {
            StructuredLight::init(0, 0, 0);
            m_period = 0;
        }

        PhaseShift(const int dsize0, const int dsize1, const int axis, const int period = 16) {
            init(dsize0, dsize1, axis, period);
        }

        void init(const int dsize0, const int dsize1, const int axis, const int period = 16) {
            StructuredLight::init(dsize0, dsize1, axis);
            m_period = period;
        }

        virtual int getCodeNum() const {
            return 3;
        }

        Mem1<Mem2<Byte> > encode() const {
            SP_ASSERT(isValid() == true);

            Mem1<Mem2<Byte> > imgs;

            const double shift[3] = { -2.0 / 3.0 * SP_PI, 0.0, +2.0 / 3.0 * SP_PI };

            for (int i = 0; i < 3; i++) {
                Mem2<Byte> img(m_dsize);

                if (m_axis == 0) {
                    for (int u = 0; u < m_dsize[0]; u++) {
                        const double s = sin(u * 2.0 * SP_PI / m_period + shift[i]);
                        for (int v = 0; v < m_dsize[1]; v++) {
                            img(u, v) = round((s * SP_BYTEMAX + SP_BYTEMAX) / 2.0);
                        }
                    }
                }
                else {
                    for (int v = 0; v < m_dsize[1]; v++) {
                        const double s = sin(v * 2.0 * SP_PI / m_period + shift[i]);
                        for (int u = 0; u < m_dsize[0]; u++) {
                            img(u, v) = round((s * SP_BYTEMAX + SP_BYTEMAX) / 2.0);
                        }
                    }
                }
                imgs.push(img);
            }

            return imgs;
        }

        Mem2<double> decode(const Mem1<Mem2<Byte> > &imgs, const Mem2<Byte> &wimg, const Mem2<Byte> &bimg, const int thresh = 10) const {
            SP_ASSERT(isValid() == true);

            Mem2<double> map(imgs[0].dsize);
            setElm(map, -1.0);

            for (int i = 0; i < map.size(); i++) {
                const Byte w = wimg[i];
                const Byte b = bimg[i];
                if (w - b > thresh) {
                    const int v0 = imgs[0][i];
                    const int v1 = imgs[1][i];
                    const int v2 = imgs[2][i];

                    const double div = 2 * v1 - (v0 + v2);
                    //if (fabs(div) < SP_SMALL) continue;

                    double p = atan2(sqrt(3.0) * (v0 - v2), div);
                    if (p < -SP_PI / 2) {
                        p += 2 * SP_PI;
                    }
                    p += SP_PI / 2.0;
                    map[i] = p * m_period / (2.0 * SP_PI);
                }
            }

            return map;
        }

        Mem2<double> decode(const Mem1<Mem2<Byte> > &imgs, const Mem2<Byte> &wimg, const Mem2<Byte> &bimg, const Mem2<double> &gcmap, const int thresh = 10) const {
            SP_ASSERT(isValid() == true);

            const Mem2<double> &psmap = decode(imgs, wimg, bimg, thresh);

            Mem2<double> map(psmap.dsize);
            setElm(map, -1.0);

            for (int i = 0; i < psmap.size(); i++) {
                if (psmap[i] < 0.0 || gcmap[i] < 0.0) continue;

                const int p0 = (round(gcmap[i]) / m_period) * m_period;
                const int p1 = p0 - m_period;

                if (fabs(p0 + psmap[i] - gcmap[i]) < fabs(p1 + psmap[i] - gcmap[i])) {
                    map[i] = p0 + psmap[i];
                }
                else {
                    map[i] = p1 + psmap[i];
                }
            }
            return map;
        }
    };

}
#endif
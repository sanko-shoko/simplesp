//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// K.He, J.Sun, and X.Tang, 
// "Guided Image Filtering", 
// IEEE Transactions on Pattern Analysis and Machine Intelligence(PAMI), 2013


#ifndef __SP_GUIDEDFILTER_H__
#define __SP_GUIDEDFILTER_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"
#include "spapp/spimgex/spintegral.h"


namespace sp{

    class Guide1 {
    public:
        Mem2<SP_REAL> I;
        Mem2<SP_REAL> I2;
        Mem2<SP_REAL> mean_I;
        Mem2<SP_REAL> mean_I2;
        Mem1<SP_REAL> inv;

        template<typename TYPE>
        Guide1(const Mem<TYPE> &src, const int winSize, const SP_REAL epsilon) {
            set(src, winSize, epsilon);
        }

        template<typename TYPE>
        void set(const Mem<TYPE> &src, const int winSize, const SP_REAL epsilon) {
            SP_ASSERT(checkPtr(src, 2));

            I.resize(src.dsize);
            I2.resize(src.dsize);

            for (int i = 0; i < src.size(); i++) {
                const TYPE &g = src[i];

                I[i] = SP_RCAST(g);
                I2[i] = SP_RCAST(g * g);
            }

            boxFilter(mean_I, I, winSize);
            boxFilter(mean_I2, I2, winSize);

            inv.resize(src.size());
            for (int i = 0; i < src.size(); i++) {
                const double var = mean_I2[i] - mean_I[i] * mean_I[i];
                inv[i] = SP_RCAST(1.0 / (var + epsilon));
            }
        }
    };

    class Guide3 {
    public:
        Mem2<Vec3> I;
        Mem2<Vec3> I2;
        Mem2<Vec3> Ic;
        Mem2<Vec3> mean_I;
        Mem2<Vec3> mean_I2;
        Mem2<Vec3> mean_Ic;
        Mem1<SP_REAL> inv;

        Guide3(const Mem<Col3> &src, const int winSize, const SP_REAL epsilon) {
            set(src, winSize, epsilon);
        }

        void set(const Mem<Col3> &src, const int winSize, const SP_REAL epsilon) {
            SP_ASSERT(checkPtr(src, 2));

            I.resize(src.dsize);
            I2.resize(src.dsize);
            Ic.resize(src.dsize);

            for (int i = 0; i < src.size(); i++) {
                const Col3 &g = src[i];

                I[i].x = SP_RCAST(g.r);
                I[i].y = SP_RCAST(g.g);
                I[i].z = SP_RCAST(g.b);

                I2[i].x = SP_RCAST(g.r * g.r);
                I2[i].y = SP_RCAST(g.g * g.g);
                I2[i].z = SP_RCAST(g.b * g.b);

                Ic[i].x = SP_RCAST(g.r * g.g);
                Ic[i].y = SP_RCAST(g.g * g.b);
                Ic[i].z = SP_RCAST(g.b * g.r);
            }

            boxFilter<Vec3, SP_REAL>(mean_I, I, winSize);
            boxFilter<Vec3, SP_REAL>(mean_I2, I2, winSize);
            boxFilter<Vec3, SP_REAL>(mean_Ic, Ic, winSize);

            inv.resize(src.size() * 3 * 3);
            for (int i = 0; i < src.size(); i++) {
                SP_REAL var[3 * 3];
                var[0 * 3 + 0] = mean_I2[i].x - mean_I[i].x * mean_I[i].x;
                var[1 * 3 + 1] = mean_I2[i].y - mean_I[i].y * mean_I[i].y;
                var[2 * 3 + 2] = mean_I2[i].z - mean_I[i].z * mean_I[i].z;

                var[0 * 3 + 1] = var[1 * 3 + 0] = mean_Ic[i].x - mean_I[i].x * mean_I[i].y;
                var[1 * 3 + 2] = var[2 * 3 + 1] = mean_Ic[i].y - mean_I[i].y * mean_I[i].z;
                var[2 * 3 + 0] = var[0 * 3 + 2] = mean_Ic[i].z - mean_I[i].z * mean_I[i].x;

                var[0 * 3 + 0] += epsilon;
                var[1 * 3 + 1] += epsilon;
                var[2 * 3 + 2] += epsilon;

                invMat33(&inv[i * 3 * 3], var);
            }
        }
    };


    template<typename TYPE>
    SP_CPUFUNC void guidedFilter(Mem<TYPE> &dst, const Mem<TYPE> &src, const Guide1 &guide, const int winSize) {
        SP_ASSERT(checkPtr(src, 2));

        Mem2<SP_REAL> p(src.dsize);
        Mem2<SP_REAL> Ip(src.dsize);

        for (int i = 0; i < src.size(); i++) {
            const TYPE &v = src[i];
            const SP_REAL &g = guide.I[i];
            p[i] = v;
            Ip[i] = g * v;
        }

        Mem2<SP_REAL> mean_p;
        boxFilter(mean_p, p, winSize);

        Mem2<SP_REAL> mean_Ip;
        boxFilter(mean_Ip, Ip, winSize);

        Mem2<SP_REAL> a(src.dsize);
        Mem2<SP_REAL> b(src.dsize);

        for (int i = 0; i < src.size(); i++) {
            const SP_REAL cov = mean_Ip[i] - guide.mean_I[i] * mean_p[i];

            a[i] = guide.inv[i] * cov;
            b[i] = mean_p[i] - a[i] * guide.mean_I[i];
        }

        boxFilter(a, a, winSize);
        boxFilter(b, b, winSize);

        dst.resize(2, src.dsize);
        for (int i = 0; i < src.size(); i++) {
            dst[i] = cast<TYPE>(a[i] * guide.I[i] + b[i]);
        }
    }

    SP_CPUFUNC void guidedFilter(Mem<Byte> &dst, const Mem<Byte> &src, const Guide3 &guide, const int winSize) {
        SP_ASSERT(checkPtr(src, 2));

        Mem2<SP_REAL> p(src.dsize);

        Mem2<Vec3> Ip(src.dsize);

        for (int i = 0; i < src.size(); i++) {
            const Byte &v = src[i];
            const Vec3 &g = guide.I[i];

            p[i] = v;

            Ip[i].x = g.x * v;
            Ip[i].y = g.y * v;
            Ip[i].z = g.z * v;
        }

        Mem2<SP_REAL> mean_p;
        boxFilter(mean_p, p, winSize);

        Mem2<Vec3> mean_Ip;
        boxFilter<Vec3, SP_REAL>(mean_Ip, Ip, winSize);

        Mem2<Vec3> a(src.dsize);
        Mem2<SP_REAL> b(src.dsize);

        for (int i = 0; i < src.size(); i++) {

            SP_REAL cov[3];
            cov[0] = (mean_Ip[i].x - guide.mean_I[i].x * mean_p[i]);
            cov[1] = (mean_Ip[i].y - guide.mean_I[i].y * mean_p[i]);
            cov[2] = (mean_Ip[i].z - guide.mean_I[i].z * mean_p[i]);

            SP_REAL tmp[3];
            mulMat(tmp, 3, 1, &guide.inv[i * 3 * 3], 3, 3, cov, 3, 1);

            a[i].x = tmp[0];
            a[i].y = tmp[1];
            a[i].z = tmp[2];
            b[i] = mean_p[i] - tmp[0] * guide.mean_I[i].x - tmp[1] * guide.mean_I[i].y - tmp[2] * guide.mean_I[i].z;
        }

        boxFilter<Vec3, SP_REAL>(a, a, winSize);
        boxFilter(b, b, winSize);

        dst.resize(2, src.dsize);
        for (int i = 0; i < src.size(); i++) {
            const Vec3 &g = guide.I[i];
            dst[i] = cast<Byte>(minval(255.0, a[i].x * g.x + a[i].y * g.y + a[i].z * g.z + b[i]));
        }
    }

    SP_CPUFUNC void guidedFilter(Mem<Col3> &dst, const Mem<Col3> &src, const Guide3 &guide, const int winSize) {
        SP_ASSERT(checkPtr(src, 2));

        Mem2<Vec3> p(src.dsize);

        Mem2<Vec3> Ip[3];
        Ip[0].resize(src.dsize);
        Ip[1].resize(src.dsize);
        Ip[2].resize(src.dsize);

        for (int i = 0; i < src.size(); i++) {
            const Col3 &v = src[i];
            const Vec3 &g = guide.I[i];

            p[i].x = v.r;
            p[i].y = v.g;
            p[i].z = v.b;

            Ip[0][i].x = g.x * v.r;
            Ip[0][i].y = g.y * v.r;
            Ip[0][i].z = g.z * v.r;

            Ip[1][i].x = g.x * v.g;
            Ip[1][i].y = g.y * v.g;
            Ip[1][i].z = g.z * v.g;

            Ip[2][i].x = g.x * v.b;
            Ip[2][i].y = g.y * v.b;
            Ip[2][i].z = g.z * v.b;
        }

        Mem2<Vec3> mean_p;
        boxFilter<Vec3, SP_REAL>(mean_p, p, winSize);

        Mem2<Vec3> mean_Ip[3];
        for (int c = 0; c < 3; c++) {
            boxFilter<Vec3, SP_REAL>(mean_Ip[c], Ip[c], winSize);
        }

        Mem2<Vec3> a[3];
        a[0].resize(src.dsize);
        a[1].resize(src.dsize);
        a[2].resize(src.dsize);

        Mem2<SP_REAL> b[3];
        b[0].resize(src.dsize);
        b[1].resize(src.dsize);
        b[2].resize(src.dsize);

        for (int i = 0; i < src.size(); i++) {

            SP_REAL mp[3];
            mp[0] = mean_p[i].x;
            mp[1] = mean_p[i].y;
            mp[2] = mean_p[i].z;

            for (int c = 0; c < 3; c++) {
                SP_REAL cov[3];
                cov[0] = (mean_Ip[c][i].x - guide.mean_I[i].x * mp[c]);
                cov[1] = (mean_Ip[c][i].y - guide.mean_I[i].y * mp[c]);
                cov[2] = (mean_Ip[c][i].z - guide.mean_I[i].z * mp[c]);

                SP_REAL tmp[3];
                mulMat(tmp, 3, 1, &guide.inv[i * 3 * 3], 3, 3, cov, 3, 1);

                a[c][i].x = tmp[0];
                a[c][i].y = tmp[1];
                a[c][i].z = tmp[2];
                b[c][i] = mp[c] - tmp[0] * guide.mean_I[i].x - tmp[1] * guide.mean_I[i].y - tmp[2] * guide.mean_I[i].z;
            }
        }

        for (int c = 0; c < 3; c++) {
            boxFilter<Vec3, SP_REAL>(a[c], a[c], winSize);
            boxFilter(b[c], b[c], winSize);
        }

        dst.resize(2, src.dsize);
        for (int i = 0; i < src.size(); i++) {
            const Vec3 &g = guide.I[i];
            dst[i].r = cast<Byte>(minval(255.0, a[0][i].x * g.x + a[0][i].y * g.y + a[0][i].z * g.z + b[0][i]));
            dst[i].g = cast<Byte>(minval(255.0, a[1][i].x * g.x + a[1][i].y * g.y + a[1][i].z * g.z + b[1][i]));
            dst[i].b = cast<Byte>(minval(255.0, a[2][i].x * g.x + a[2][i].y * g.y + a[2][i].z * g.z + b[2][i]));
        }
    }


    SP_CPUFUNC void guidedFilter(Mem<Byte> &dst, const Mem<Byte> &src, const int winSize, const SP_REAL epsilon) {
        SP_ASSERT(checkPtr(src, 2));

        const Guide1 guide(src, winSize, epsilon);
        guidedFilter(dst, src, guide, winSize);
    }

    SP_CPUFUNC void guidedFilter(Mem<Col3> &dst, const Mem<Col3> &src, const int winSize, const SP_REAL epsilon) {
        SP_ASSERT(checkPtr(src, 2));

        const Guide3 guide(src, winSize, epsilon);
        guidedFilter(dst, src, guide, winSize);
    }

}

#endif
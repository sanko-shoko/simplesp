//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STEREO_H__
#define __SP_STEREO_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spblock.h"


namespace sp{

    class StereoBase {
    private:

        // image size
        int m_dsize[2];

        // max / min disparity
        int m_maxDisp, m_minDisp;

        // window size
        int m_winSize;

        // camera parameter
        CamParam m_cam[2];

        // camera baseline
        SP_REAL m_baseLine;

        // disparity map
        Mem2<float> m_dispMap[2];

        // eval map
        Mem2<float> m_evalMap[2];

        // depth map
        Mem2<Vec3> m_depthMap[2];

    public:

        enum Order{
            StereoL = +1,
            StereoR = -1
        };

    public:

        StereoBase() {
            m_cam[0] = getCamParam(0, 0);
            m_cam[1] = getCamParam(0, 0);
            m_baseLine = 1.0;

            m_maxDisp = 10;
            m_minDisp = 0;

            m_winSize = 21;
        }


        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        void setCam(const CamParam &camL, const CamParam &camR, const SP_REAL baseLine = 1.0) {
            m_cam[0] = camL;
            m_cam[1] = camR;
            m_baseLine = baseLine;
        }

        void setRange(const int maxDisp, const int minDisp) {
            m_maxDisp = maxDisp;
            m_minDisp = minDisp;
        }

        void setWinSize(const int winSize) {
            m_winSize = winSize;
        }

        Mem2<float>& getDispMap(const Order &order) {
            return m_dispMap[order == Order::StereoL ? 0 : 1];
        }

        Mem2<Vec3>& getDepthMap(const Order &order) {
            return m_depthMap[order == Order::StereoL ? 0 : 1];
        }


        //--------------------------------------------------------------------------------
        // execute stereo matching
        //--------------------------------------------------------------------------------

        virtual bool execute(const void *srcL, const void *srcR, const int dsize0, const int dsize1, const int ch) {
            Mem2<Byte> gryL, gryR;
            cnvPtrToImg(gryL, srcL, dsize0, dsize1, ch);
            cnvPtrToImg(gryR, srcR, dsize0, dsize1, ch);
            return _execute(gryL, gryR);
        }

        virtual bool execute(const Mem2<Byte> &srcL, const Mem2<Byte> &srcR) {
            return _execute(srcL, srcR);
        }

        virtual bool execute(const Mem2<Col3> &srcL, const Mem2<Col3> &srcR) {
            Mem2<Byte> gryL, gryR;
            cnvImg(gryL, srcL);
            cnvImg(gryR, srcR);
            return _execute(gryL, gryR);
        }

    protected:

        virtual bool _execute(const Mem2<Byte> &srcL, const Mem2<Byte> &srcR) {

            m_dsize[0] = srcL.dsize[0];
            m_dsize[1] = srcL.dsize[1];
            {
                correspBM(m_dispMap[0], m_evalMap[0], srcL, srcR, StereoL);
                correspBM(m_dispMap[1], m_evalMap[1], srcR, srcL, StereoR);

                consistencyCheck(m_dispMap[0], m_evalMap[0], m_dispMap[1], m_evalMap[1], 2.0, StereoL);
                consistencyCheck(m_dispMap[1], m_evalMap[1], m_dispMap[0], m_evalMap[0], 2.0, StereoR);
            }

            if (cmp(2, m_cam[0].dsize, m_dsize) == true && cmp(2, m_cam[1].dsize, m_dsize) == true) {
                cnvDispToDepth(m_depthMap[0], m_dispMap[0], m_evalMap[0], StereoL);
                cnvDispToDepth(m_depthMap[1], m_dispMap[1], m_evalMap[1], StereoR);
            }
            return true;
        }

        Rect2 getDispRect(const int order) {
            const int layers = m_maxDisp - m_minDisp + 1;

            Rect2 rect = getRect2(m_dsize);

            const int maxv = (order > 0) ? m_maxDisp : -m_minDisp;
            const int minv = (order > 0) ? m_minDisp : -m_maxDisp;

            const int lx = maxval(+m_winSize + maxv - layers / 2, 0);
            const int rx = maxval(+m_winSize - minv - layers / 2, 0);

            rect.dbase[0] += lx;
            rect.dsize[0] -= lx + rx;

            return rect;
        }

        void correspBM(Mem2<float> &dispMap, Mem2<float> &evalMap, const Mem2<Byte> &src, const Mem2<Byte> &ref, const int order) {

            const Rect2 rect = getDispRect(order);

            dispMap.resize(m_dsize);
            dispMap.zero();
            evalMap.resize(m_dsize);
            evalMap.zero();

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int v = rect.dbase[1]; v < rect.dbase[1] + rect.dsize[1]; v++) {
                for (int d = m_minDisp; d <= m_maxDisp; d++) {

                    int sad = -1;
                    for (int u = rect.dbase[0]; u < rect.dbase[0] + rect.dsize[0]; u++) {
                        float &disp = dispMap(u, v);
                        float &eval = evalMap(u, v);

                        sad = calcSAD(src, ref, u, v, d, order, sad);
                        const int e = (SP_BYTEMAX * m_winSize * m_winSize) - sad;

                        if (e > eval) {
                            eval = static_cast<float>(e);
                            disp = static_cast<float>(d);
                        }
                    }
                }
            }
        }

        void consistencyCheck(Mem2<float> &dispMap0, Mem2<float> &evalMap0, const Mem2<float> &dispMap1, const Mem2<float> &evalMap1, const double thresh, const int order) {

            const Rect2 rect = getDispRect(order);

            for (int v = rect.dbase[1]; v < rect.dbase[1] + rect.dsize[1]; v++) {
                for (int u = rect.dbase[0]; u < rect.dbase[0] + rect.dsize[0]; u++) {
                    const float &disp0 = dispMap0(u, v);
                    const float &disp1 = dispMap1(u - order * round(disp0), v);

                    if (fabs(disp0 - disp1) > thresh) {
                        evalMap0(u, v) = 0.0f;
                    }
                }
            }
        }

        void cnvDispToDepth(Mem2<Vec3> &dst, const Mem2<float> &dispMap, const Mem2<float> &evalMap, const int order) {

            dst.resize(m_dsize);
            dst.zero();

            const CamParam &cam = (order > 0) ? m_cam[0] : m_cam[1];

            const Rect2 rect = getDispRect(order);

            const double f = m_cam[0].fx;
            const double b = m_baseLine;
            const double cx0 = (order > 0) ? m_cam[0].cx : m_cam[1].cx;
            const double cx1 = (order > 0) ? m_cam[1].cx : m_cam[0].cx;

            for (int v = rect.dbase[1]; v < rect.dbase[1] + rect.dsize[1]; v++) {
                for (int u = rect.dbase[0]; u < rect.dbase[0] + rect.dsize[0]; u++) {
                    const Vec2 npx = invCam(cam, getVec2(u, v));
                    const float disp = dispMap(u, v);
                    const float eval = evalMap(u, v);
                    if (eval > 0.0) {
                        const double depth = f * b / (disp - (cx0 - cx1));
                        dst(u, v) = getVec3(npx.x, npx.y, 1.0) * depth;
                    }
                }
            }
        }

    protected:

        int calcSAD(const Mem2<Byte> &src, const Mem2<Byte> &ref, const int x, const int y, const int d, const int order, const int pre = -1) {

            const int offset = m_winSize / 2;

            int sad;

            if (pre < 0) {
                sad = 0;

                for (int wy = 0; wy < m_winSize; wy++) {
                    for (int wx = 0; wx < m_winSize; wx++) {
                        const int v0 = src(x + wx - offset, y + wy - offset);
                        const int v1 = ref(x + wx - offset - order * d, y + wy - offset);
                        sad += abs(v0 - v1);
                    }
                }
            }
            else {
                sad = pre;
                
                int wx;
                
                wx = -1;
                for (int wy = 0; wy < m_winSize; wy++) {
                    const int v0 = src(x + wx - offset, y + wy - offset);
                    const int v1 = ref(x + wx - offset - order * d, y + wy - offset);
                    sad -= abs(v0 - v1);
                }

                wx = m_winSize - 1;
                for (int wy = 0; wy < m_winSize; wy++) {
                    const int v0 = src(x + wx - offset, y + wy - offset);
                    const int v1 = ref(x + wx - offset - order * d, y + wy - offset);
                    sad += abs(v0 - v1);
                }
            }

            return sad;
        }

    };

}

#endif
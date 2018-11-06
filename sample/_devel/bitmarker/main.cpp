﻿#define SP_USE_DEBUG 1
#define SP_USE_CONSOLE 1

#include "simplesp.h"
#include "spex/sptw.h"
#include "spex/spcv.h"

using namespace sp;

class BitMarkerGUI : public TwWindow {
    Mem1<BitMarkerParam> m_mrks;

    BitMarker m_bitMarker;

    Mem2<Col3> m_crntImg;

    struct UIFlag {
        bool start;

        bool dispMinGauss;
        bool dispMinLab;
        bool dispContour;
        bool dispCorner;
        bool test;

        UIFlag() {
            memset(this, 0, sizeof(UIFlag));
        }
    }m_ui;

private:

    virtual void init() {
        if (1) {
            
            m_mrks = getBitMarkerParam(0, 3, 50.0, 4, 3, 5.0);
        }
        else {

            cv::Mat cvneko = cv::imread(SP_DATA_DIR  "/marker/neko.bmp");
            Mem2<Col3> neko;
            cvCnvImg(neko, cvneko);

            BitMarkerParam mrk(neko, 50.0);

            m_mrks.clear();
            m_mrks.push(mrk);
        }
        m_bitMarker.addMrks(m_mrks);

        m_ui.start = true;

        TwBar *bar = TwNewBar("TweakBar");

        TwAddButton(bar, "start", TwButtonCB(BitMarkerGUI, start), this, "");

        TwAddSeparator(bar, NULL, "");

#if SP_USE_DEBUG
        TwAddVarRW(bar, "dispMinGauss", TW_TYPE_BOOL8, &m_ui.dispMinGauss, "");
        TwAddVarRW(bar, "dispMinLab", TW_TYPE_BOOL8, &m_ui.dispMinLab, "");
        TwAddVarRW(bar, "dispContour", TW_TYPE_BOOL8, &m_ui.dispContour, "");
        TwAddVarRW(bar, "dispCorner", TW_TYPE_BOOL8, &m_ui.dispCorner, "");
        TwAddVarRW(bar, "test", TW_TYPE_BOOL8, &m_ui.test, "");
#endif

    }

    void start() {
        m_ui.start ^= true;
    }

    // capture next image
    void nextImg() {
        if (1) {
            // usb camera
            static cv::VideoCapture cap;
            cvCaptureImg(m_crntImg, cap);
        }
        else {
            cvCnvImg(m_crntImg, cv::imread(SP_DATA_DIR  "/marker/cap_neko.bmp"));
            cvCnvImg(m_crntImg, cv::imread("test.bmp"));
        }
    }

    virtual void display() {
        if (m_ui.start) {
            nextImg();
        }

        {
            glLoadView2D(m_crntImg.dsize, m_viewPos, m_viewScale);
            glRenderImg(m_crntImg);
        }

        if (1 || m_ui.test) {
            m_bitMarker.execute(m_crntImg);

            glLoadView3D(m_bitMarker.getCam(), m_viewPos, m_viewScale);

            for (int i = 0; i < m_bitMarker.size(); i++) {
                if (m_bitMarker.getPose(i) == NULL) continue;

                glLoadMatrix(*m_bitMarker.getPose(i));
                glLineWidth(4.0f);

                glBegin(GL_LINES);
                glAxis(30.0);
                glEnd();
            }

            glLoadView2D(m_crntImg.dsize, m_viewPos, m_viewScale);
            for (int i = 0; i < m_bitMarker.size(); i++) {
                if (m_bitMarker.getPose(i) == NULL) continue;

                glPointSize(static_cast<float>(5));
                const Mem1<Vec2> &cpixs = *m_bitMarker.getCrspPixs(i);
                glBegin(GL_POINTS);
                glColor(sp::getCol(0, 255, 0));
                for (int j = 0; j < cpixs.size(); j++) {
                    glVertex(cpixs[j]);
                }

                glColor(sp::getCol(0, 0, 255));
                const Mem1<Vec3> &cobjs = *m_bitMarker.getCrspObjs(i);
                for (int j = 0; j < cobjs.size(); j++) {
                    glVertex(mulCamD(m_bitMarker.getCam(), prjVec(*m_bitMarker.getPose(i) * cobjs[j])));
                }
                glEnd();
            }

        }
        else {

            Pose pose;
            //if (calcBitMarkerArrayPose(pose, m_crntImg, getCamParam(m_crntImg.dsize), m_mrks) == true) {
            //    glLoadView3D(getCamParam(m_crntImg.dsize), m_viewPos, m_viewScale);
            //    glLoadMatrix(pose);
            //    glLineWidth(4.0f);
            //    glBegin(GL_LINES);
            //    glAxis(60.0);
            //    glEnd();
            //}
        }

#if SP_USE_DEBUG

        if (m_ui.dispMinGauss) {
            const sp::Mem2<unsigned char> *minImg = SP_HOLDER_GET(m_bitMarker, "minImg", const sp::Mem2<unsigned char>);
            if (minImg) {
                const double scale = static_cast<double>(m_crntImg.dsize[0]) / minImg->dsize[0];
                glLoadView2D(minImg->dsize, m_viewPos, scale * m_viewScale);
                glRenderImg(*minImg);
            }
        }
        if (m_ui.dispMinLab) {
            const sp::Mem2<int> *labelMap = SP_HOLDER_GET(m_bitMarker, "labelMap", const sp::Mem2<int>);
            if (labelMap) {
                glPointSize(static_cast<float>(1.2 * m_viewScale));
                const double scale = static_cast<double>(m_crntImg.dsize[0]) / labelMap->dsize[0];
                glLoadView2D(labelMap->dsize, m_viewPos, scale * m_viewScale);

                glBegin(GL_POINTS);
                for (int v = 0; v < labelMap->dsize[1]; v++) {
                    for (int u = 0; u < labelMap->dsize[0]; u++) {
                        if ((*labelMap)(u, v) < 0) continue;
                        glColor((*labelMap)(u, v));
                        glVertex(getVec(u, v));
                    }
                }
                glEnd();
            }
        }
        if (m_ui.dispContour) {
            const sp::Mem1<sp::Mem1<sp::Vec2> > *contours = SP_HOLDER_GET(m_bitMarker, "contours", const sp::Mem1<sp::Mem1<sp::Vec2> >);
            if (contours) {
                glColor(sp::getCol(0, 255, 0));
                glPointSize(static_cast<float>(2 * m_viewScale));

                glLoadView2D(m_crntImg.dsize, m_viewPos, m_viewScale);

                glBegin(GL_POINTS);
                for (int i = 0; i < contours->size(); i++) {
                    for (int j = 0; j < (*contours)[i].size(); j++) {
                        glVertex((*contours)[i][j]);
                    }
                }
                glEnd();
            }
        }

        if (m_ui.dispCorner) {
            const sp::Mem1<sp::Mem1<sp::Vec2> > *corners = SP_HOLDER_GET(m_bitMarker, "corners", const sp::Mem1<sp::Mem1<sp::Vec2> >);
            if (corners) {
                glColor(sp::getCol(0, 255, 0));
                glPointSize(static_cast<float>(2 * m_viewScale));

                glLoadView2D(m_crntImg.dsize, m_viewPos, m_viewScale);

                glBegin(GL_POINTS);
                for (int i = 0; i < corners->size(); i++) {
                    for (int j = 0; j < (*corners)[i].size(); j++) {
                        glVertex((*corners)[i][j]);
                    }
                }
                glEnd();
            }
        }

#endif

    }
};

int main(){

    BitMarkerGUI win;
    win.execute("bitmarker", 800, 600);

    return 0;
}
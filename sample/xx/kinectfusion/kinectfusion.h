﻿#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/sprs.h"

using namespace sp;

class KinectFusionGUI : public BaseWindow{

    // realsense 
    RealSense m_rs;

    // Kinect Fusion
    KinectFusion m_kfusion;

    // start flag
    bool m_start;

private:

    void help() {
        printf("'s' key : start kinect fusion\n");
        printf("'r' key : reset kinect fusion\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init(){
        m_start = false;

        m_rs.init(0);
        m_rs.enableDepth();

        help();
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_key[GLFW_KEY_S] == 1) {
            m_start = true;
        }
        if (m_key[GLFW_KEY_R] == 1) {
            m_kfusion.reset();
            m_start = false;
        }
    }

    virtual void display() {

        const double nearPlane = 100;
        const double farPlane = 2000;

        m_rs.capture();
        //{
        //    static int id = 0;
        //    if (m_rs.testLoad("data", id++) == false){
        //        id = 0;
        //    }
        //}
        const Mem2<SP_REAL> *depth = m_rs.getDepth();
        if (depth == NULL) return;

        // make min data for computational cost
        CamParam minCam;
        Mem2<SP_REAL> minDepth;
        {
            pyrdown(minCam, *m_rs.getDepthCam());
            pyrdownDepth(minDepth, *depth);
        }

        // kinect fusion
        {
            if (m_start == false) {
                Mem1<double> zlist;
                zlist.reserve(minDepth.size());
                for (int i = 0; i < minDepth.size(); i++) {
                    if (minDepth[i] > 0.0) zlist.push(minDepth[i]);
                }

                const double meanv = mean(zlist);

                m_kfusion.init(120, meanv / 100.0, minCam, getPose(getVec3(0.0, 0.0, meanv)));
            }
            else {
                m_kfusion.execute(minDepth);
            }
        }

        // render
        {
            Mem2<Col3> view;
            if (m_start == false) {
                cnvDepthToImg(view, minDepth, nearPlane, farPlane);
            }
            else {
                cnvNormalToImg(view, *m_kfusion.getCast(), nearPlane, farPlane);
            }

            glLoadView2D(m_kfusion.getCam(), m_viewPos, m_viewScale * 2.0);
            glTexImg(view);

            glLoadView3D(m_kfusion.getCam(), m_viewPos, m_viewScale * 2.0);
            glLoadMatrix(*m_kfusion.getPose());

            glBegin(GL_LINES);
            glCube(m_kfusion.getMap()->dsize[0] * m_kfusion.getMap()->unit);
            glEnd();
        }
    }


};

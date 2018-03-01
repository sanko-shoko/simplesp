#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/sprs.h"

using namespace sp;

class RSCaptureGUI : public BaseWindow{

    // realsense 
    RealSense m_rs;

    // view pose
    Pose m_view;

    // base pose
    Pose m_base;

    // view mode
    bool m_view3d;

private:

    void help() {
        printf("'v' key : switch view mode\n");
        printf("'r' key : reset view pose\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init(){
        m_view3d = false;

        m_rs.init(0);

        m_rs.enableDepth();
        //m_rs.enableColor();

        m_view = zeroPose();
        
        m_base = getPose(getVec(0.0, 0.0, -500.0));

        help();
    }

    virtual void display(){

        const double nearPlane = 100;
        const double farPlane = 2000;

        m_rs.capture();
        //{
        //    static int id = 0;
        //    if (m_rs.testLoad("data", id++) == false){
        //        id = 0;
        //    }
        //}
        const Mem2<Col3> *color = m_rs.getColor();
        const Mem2<double> *depth = m_rs.getDepth();

        if (m_view3d == false){
            // view 2D

            if (color != NULL){
                glLoadView2D(*m_rs.getColorCam(), m_viewPos, m_viewScale);
                glRenderImg(*color);
            }

            if (depth != NULL){
                glLoadView2D(*m_rs.getDepthCam(), m_viewPos, m_viewScale);
                static Mem2<Col3> img;
                cnvDepthToImg(img, *depth, nearPlane, farPlane);
                glRenderImg(img);
            }
        }
        else{
            // view 3D

            if (depth != NULL){
                glLoadView3D(*m_rs.getDepthCam(), m_viewPos, m_viewScale);

                // render points
                glLoadMatrix(m_view);

                glPointSize(1.f);
                glBegin(GL_POINTS);
                for (int v = 0; v < depth->dsize[1]; v++){
                    for (int u = 0; u < depth->dsize[0]; u++){
                        const double d = (*depth)(u, v);
                        if (d == 0.0) continue;
                        
                        Col3 col;
                        cnvDepthToCol(col, d, nearPlane, farPlane);
                        glColor(col);

                        const Vec3 pnt = prjVec(invCam(*m_rs.getDepthCam(), getVec(u, v))) * d;
                        glVertex(pnt);
                    }
                }
                glEnd();

                // render axis
                glClear(GL_DEPTH_BUFFER_BIT);
                glLoadMatrix(m_view * invPose(m_base));

                glLineWidth(2.f);
                glBegin(GL_LINES);
                glAxis(100.0);
                glEnd();
            }
        }

        if (m_keyAction[GLFW_KEY_V] == 1){
            m_view3d ^= true;
        }
        if (m_keyAction[GLFW_KEY_R] == 1){
            m_view = zeroPose();
        }

        // save image
        {
            static int cnt = 0;
            const int limit = 300;

            if (m_keyAction[GLFW_KEY_S] == 1){
                cnt = limit;
            }
            if (m_keyAction[GLFW_KEY_N] == 1){
                cnt = 1;
            }

            if (cnt > 0){
                static int id = 0;

                if (color != NULL){
                    char name[256];
                    sprintf(name, "color%03d.bmp", id);
                    saveBMP(*color, name);
                }
                if (depth != NULL){
                    char name[256];
                    sprintf(name, "depth%03d.bin", id);
                    saveMem(*depth, name);
                }
                id++;
                cnt++;
            }
            if (cnt > 300){
                cnt = 0;
            }
        }
    }

    virtual void mousePos(double x, double y) {
        if (m_view3d == false || m_rs.getDepthCam() == NULL) return;

        controlPose(m_view, m_mouse, *m_rs.getDepthCam(), m_viewScale, m_base);
    }

    virtual void mouseScroll(double x, double y) {
        if (m_view3d == false || m_rs.getDepthCam() == NULL) return;

        controlPose(m_view, m_mouse, *m_rs.getDepthCam(), m_viewScale, m_base);
    }

};

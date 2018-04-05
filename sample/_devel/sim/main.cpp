#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

#define AXIS 0

class ActiveStereoGUI : public BaseWindow {

    // camera & projector param
    CamParam m_cam, m_prj;

    // image
    Mem2<Col3> m_img;

    // object mesh model
    Mem1<Mesh3> m_model;

    Pose m_pose;

private:

    void help() {
        printf("'a' key : capture simulation image\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        help();

        {
            m_cam = getCamParam(640, 480);

            m_img.resize(m_cam.dsize);
            m_img.zero();

            m_prj = getCamParam(320, 240);
        }

        {
            loadPlane(m_model, 2000.0);

            m_pose = getPose(getRotAngleY(SP_PI * 10.0 / 180.0) * getRotAngleX(SP_PI * 10.0 / 180.0) * getRot(1.0, 0.0, 0.0, 0.0), getVec(0.0, 0.0, 5000.0));

            print(m_pose.rot * getVec(0, 0, 1));
        }
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] == 1) {
            Mem2<Byte> cap;

            ::srand(0);
            Mem2<Byte> ptn(m_prj.dsize);
            for (int i = 0; i < ptn.size(); i++) {
                ptn[i] = (::rand() % 5 == 0) ? 200 : 10;
            }

            Mem2<Byte> imgL, imgR;

            renderPattern(cap, m_cam, m_pose, m_model, zeroPose(), m_prj, ptn);
            cnvImg(m_img, cap);
            cnvImg(imgL, cap);

            const double baseLine = 50.0;
            const Pose stereo = getPose(getVec(baseLine, 0.0, 0.0));
            renderPattern(cap, m_cam, invPose(stereo) * m_pose, m_model, stereo, m_prj, ptn);
            cnvImg(imgR, cap);

            Mem2<VecPN3> map;
            renderVecPN(map, m_cam, m_pose, m_model);

            const int size = 100;
            const Rect rect = getRect2((m_cam.dsize[0] - size) / 2, (m_cam.dsize[1] - size) / 2, size, size);

            const int range = 40;
            const int base = 5000;

            saveBMP(imgL, "imgL.bmp");
            saveBMP(imgR, "imgR.bmp");

            const Vec3 nrm = m_pose.rot * getVec(0.0, 0.0, 1.0);
            for (int d = base - range; d <= base + range; d++) {

                double sum = 0.0;
                int cnt = 0;
                for (int v = 0; v < m_cam.dsize[1]; v++) {
                    for (int u = 0; u < m_cam.dsize[0]; u++) {
                        if (isInRect2(rect, u, v) == false) continue;

                        const Vec3 vec = extVec(invCam(m_cam, getVec(u, v)), 1.0);

                        const double depth = dotVec(getVec(0.0, 0.0, d), nrm) / dotVec(vec, nrm);

                        const Vec3 m = map(u, v).pos;
                        //if (m.z > 0) {
                        //    printf("%lf %lf", m.z, depth);
                        //    getchar();
                        //}
                        const double disp = m_cam.fx * baseLine / depth;

                        const double err = acs2(imgL, u, v) - acs2(imgR, u - disp, (double)v);
                        sum += err * err;
                        cnt++;
                    }
                }
                printf("%d : %lf\n", d, ::sqrt(sum / cnt));
            }

        }
    }


    virtual void display() {

        // view 2D
        glLoadView2D(m_cam, m_viewPos, m_viewScale);
        glRenderImg(m_img);

        // view 3D
        glLoadView3D(m_cam, m_viewPos, m_viewScale);
        renderOutline();
  
    }

    void renderOutline() {

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        {
            glLoadMatrix(m_pose);

            glEnable(GL_STENCIL_TEST);

            glClearStencil(0);
            glClear(GL_STENCIL_BUFFER_BIT);

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            // fill stencil
            {
                glStencilFunc(GL_ALWAYS, 1, 0xFFFF);
                glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                glColor4d(0.0, 0.0, 0.0, 0.0);

                glBegin(GL_TRIANGLES);
                for (int i = 0; i < m_model.size(); i++) {
                    glMesh(m_model[i]);
                }
                glEnd();
            }

            // draw outline
            {
                glStencilFunc(GL_NOTEQUAL, 1, 0xFFFF);
                glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(2.0f);
                glColor3d(1.0, 1.0, 1.0);

                glBegin(GL_TRIANGLES);
                for (int i = 0; i < m_model.size(); i++) {
                    glMesh(m_model[i]);
                }
                glEnd();
            }
        }
        glPopAttrib();

        glClear(GL_DEPTH_BUFFER_BIT);
    }

    virtual void mousePos(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

    virtual void mouseScroll(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

};


int main(){

    ActiveStereoGUI win;
    win.execute("sim", 800, 600);

    return 0;
}
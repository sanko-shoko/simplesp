#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

#define AXIS 0

class ActiveStereoGUI : public BaseWindow {

    // camera & projector param
    CamParam m_cam, m_prj;

    // cam to prj pose
    Pose m_cam2prj;

    // image
    Mem2<Col3> m_img;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object to cam pose
    Pose m_pose;

    // view pose
    Pose m_view;

    // GrayCode
    GrayCode m_graycode;
    PhaseShift m_phaseshift;

    int m_capture;

    bool m_view3d;

    // pnts
    Mem1<Vec3> m_pnts;

    // capture images
    Mem2<Byte> m_wimg, m_bimg;
    Mem1<Mem2<Byte> > m_gcimgs;
    Mem1<Mem2<Byte> > m_psimgs;

private:

    void help() {
        printf("'a' key : capture simulation image\n");
        printf("'s' key : decode gray code\n");
        printf("'d' key : decode gray code + phase shift\n");
        printf("'f' key : switch view mode (2d <-> 3d)\n");
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

            if (AXIS == 0) {
                m_cam2prj = getPose(getRotAngleY(+10.0 / 180.0 * SP_PI), getVec(-100.0, 0.0, 0.0));
            }
            else {
                m_cam2prj = getPose(getRotAngleX(+10.0 / 180.0 * SP_PI), getVec(0.0, 100.0, 0.0));
            }

            m_graycode.init(m_prj.dsize[0], m_prj.dsize[1], AXIS);
            m_phaseshift.init(m_prj.dsize[0], m_prj.dsize[1], AXIS);
            m_capture = -1;
        }

        {
            if (loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply", m_model) == false) {

                // if could not find stanford bunny, load dummy model
                loadGeodesicDorm(m_model, 100.0, 1);
            }

            m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));

            m_view3d = false;
            m_view = zeroPose();
        }
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_capture >= 0) return;

        if (m_keyAction[GLFW_KEY_A] == 1) {
            m_capture = 0;
        }
        if (m_keyAction[GLFW_KEY_S] == 1) {
            decodeGC();
        }
        if (m_keyAction[GLFW_KEY_D] == 1) {
            decodeGCPS();
        }
        if (m_keyAction[GLFW_KEY_F] == 1) {
            m_view3d ^= true;
        }
    }

    void calcPnt3d(const Mem2<double> &map) {
        m_img.zero();
        for (int i = 0; i < m_img.size(); i++) {
            if (map[i] < 0.0) continue;
            cnvPhaseToCol(m_img[i], map[i] / m_prj.dsize[AXIS]);
        }

        m_pnts.clear();
        for (int v = 0; v < map.dsize[1]; v++) {
            for (int u = 0; u < map.dsize[0]; u++) {
                if (map(u, v) < 0.0) continue;

                Vec3 pnt;
                if (AXIS == 0) {
                    if (calcPnt3dX(pnt, zeroPose(), m_cam, getVec(u, v), m_cam2prj, m_prj, map(u, v)) == true) {
                        m_pnts.push(pnt);
                    }
                }
                else {
                    if (calcPnt3dY(pnt, zeroPose(), m_cam, getVec(u, v), m_cam2prj, m_prj, map(u, v)) == true) {
                        m_pnts.push(pnt);
                    }
                }
            }
        }
    }

    void decodeGC() {

        const Mem2<double> gcmap = m_graycode.decode(m_gcimgs, m_wimg, m_bimg);

        calcPnt3d(gcmap);
    }

    void decodeGCPS() {

        const Mem2<double> gcmap = m_graycode.decode(m_gcimgs, m_wimg, m_bimg);
        const Mem2<double> psmap = m_phaseshift.decode(m_psimgs, m_wimg, m_bimg, gcmap);

        calcPnt3d(psmap);
    }

    void capture() {

        static Mem1<Mem2<Byte> > ptns;
        static Mem1<Mem2<Byte> > caps;

        if (m_capture == 0) {
            Mem1<Mem2<Byte> > gcptns = m_graycode.encode();
            Mem1<Mem2<Byte> > psptns = m_phaseshift.encode();

            ptns.clear();
            ptns.push(m_graycode.getPlain(255));
            ptns.push(m_graycode.getPlain(0));
            ptns.push(gcptns);
            ptns.push(psptns);

            caps.clear();
            caps.resize(ptns.size());
        }

        if (m_capture < ptns.size()) {
            renderPattern(caps[m_capture], m_cam, m_pose, m_model, m_cam2prj, m_prj, ptns[m_capture]);
            cnvImg(m_img, caps[m_capture]);

            m_capture++;
        }
        else {
            m_wimg = caps[0];
            m_bimg = caps[1];

            m_gcimgs = caps.part(2, m_graycode.getCodeNum());
            m_psimgs = caps.part(2 + m_graycode.getCodeNum(), m_phaseshift.getCodeNum());

            m_capture = -1;
        }

    }

    virtual void display() {
        {
            static double pre = 0.0;
            const double dif = static_cast<double>(clock() - pre) / CLOCKS_PER_SEC;

            if (m_capture >= 0 && dif > 0.1) {
                pre = clock();
                capture();
            }
        }

        if (m_view3d == false) {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glTexImg(m_img);

            // view 3D
            glLoadView3D(m_cam, m_viewPos, m_viewScale);
            
            glLoadMatrix(m_pose);
            glRenderOutline(m_model);
        }
        else {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            // render points
            glLoadMatrix(m_view);

            glPointSize(1.f);
            glColor3d(0.5, 0.5, 1.0);
            glBegin(GL_POINTS);
            for (int i = 0; i < m_pnts.size(); i++) {
                glVertex(m_pnts[i]);
            }
            glEnd();
        }
    }

    virtual void mousePos(double x, double y) {
        if (m_capture >= 0) return;

        if (m_view3d == false) {
            controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
        }
        else {
            controlPose(m_view, m_mouse, m_wcam, m_viewScale, invPose(m_pose));
        }
    }

    virtual void mouseScroll(double x, double y) {
        if (m_capture >= 0) return;

        if (m_view3d == false) {
            controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
        }
        else {
            controlPose(m_view, m_mouse, m_wcam, m_viewScale, invPose(m_pose));
        }
    }

};


int main(){

    ActiveStereoGUI win;
    win.execute("render", 800, 600);

    return 0;
}
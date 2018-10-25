#define SP_USE_THREAD 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

using namespace sp;

class SLAMGUI : public BaseWindow {

    SLAM m_slam;

    // cam pose (object to cam pose)
    Pose m_pose;

    // axis pose (cam to axis pose)
    Pose m_axis;

    CamParam m_cam;
    Mem2<Col3> m_img;
 
    // thread;
    Thread m_thread;

private:

    void help() {
        printf("'a' key : update\n");
        printf("'s' key : reset\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {
        help();

        const double distance = 20.0;
        m_pose = getPose(getVec(0.0, 0.0, +distance));
        m_axis = getPose(getVec(0.0, 0.0, -distance));

    }

    void reset() {
        loadText(SP_DATA_DIR "/image/shiba.txt", m_cam);
        m_slam.init(m_cam);
    }

    void update(){
        m_slam.updateMap(m_img);
    }

    void addView() {
        //m_slam.addView(m_img, &m_cam);
    }

    void capture() {
        static cv::VideoCapture cap(0);
        cv::Mat cvimg;
        cap >> cvimg;

        cvCnvImg(m_img, cvimg);
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] >= 1) {
            m_thread.run<SLAMGUI, &SLAMGUI::addView>(this);
        }

        if (m_keyAction[GLFW_KEY_S] == 1) {
            m_thread.run<SLAMGUI, &SLAMGUI::reset>(this);
        }
    }

    virtual void display() {
        {
            capture();
        }
        {
            m_thread.run<SLAMGUI, &SLAMGUI::update>(this, false);
        }
        {
            const double scale = 0.3 * m_wcam.dsize[0] / m_img.dsize[0];
            const Vec2 offset = getVec(m_wcam.dsize[0], m_wcam.dsize[1]) * (scale * 0.5 - 0.5);
            glLoadView2D(m_wcam, offset, scale);

            glRenderImg(m_img);
        }
 
    }

    virtual void mousePos(double x, double y) {
        Pose delta = zeroPose();
        controlPose(delta, m_mouse, m_wcam, m_viewScale, m_axis);
        m_pose = delta * m_pose;
    }

    virtual void mouseScroll(double x, double y) {
        Pose delta = zeroPose();
        controlPose(delta, m_mouse, m_wcam, m_viewScale, m_axis);
        m_pose.trn.z += delta.trn.z;
        m_axis.trn.z -= delta.trn.z;
    }

};


int main() {

    SLAMGUI win;
    win.execute("slam", 800, 600);

    return 0;
}
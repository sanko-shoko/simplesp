#define SP_USE_THREAD 1
#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

using namespace sp;

class SfMGUI : public BaseWindow {

    SfM m_sfm;

    // cam pose (object to cam pose)
    Pose m_pose;

    // axis pose (cam to axis pose)
    Pose m_axis;

    CamParam m_cam;
    Mem2<Col3> m_img;
 
    // thread;
    Thread m_thread;

    // 
    float m_err;

    // 
    int m_addcnt;

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

        m_err = 1.0f;

        m_addcnt = 0;
    }

    void reset() {
        loadText(SP_DATA_DIR "/image/shiba.txt", m_cam);
        m_sfm.init();
        m_addcnt = 0;
    }

    void update(){
        m_sfm.update();
    }

    void addView() {

        m_sfm.addView(m_img, &m_cam);
    }

    void capture() {
        static cv::VideoCapture cap(0);
        cv::Mat cvimg;
        cap >> cvimg;

        cvCnvImg(m_img, cvimg);
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] >= 1) {
            m_thread.run<SfMGUI, &SfMGUI::addView>(this);
        }

        if (m_keyAction[GLFW_KEY_S] == 1) {
            m_thread.run<SfMGUI, &SfMGUI::reset>(this);
        }
    }

    virtual void display() {
        // test window
        //if (ImGui::Begin("param", NULL, ImGuiWindowFlags_NoResize)) {

        //    ImGui::SetWindowPos(ImVec2(10, 10), ImGuiCond_Once);
        //    ImGui::SetWindowSize(ImVec2(300, 300), ImGuiCond_Always);

        //    if (ImGui::Button("Button")) {
        //        printf("Button\n");
        //    }

        //    ImGui::InputFloat("m_err", &m_err, 0.01f, 0.1f);

        //    ImGui::End();
        //}

        {
            capture();
        }
        {
            m_thread.run<SfMGUI, &SfMGUI::update>(this, false);
        }

        {
            const double scale = 0.3 * m_wcam.dsize[0] / m_img.dsize[0];
            const Vec2 offset = getVec(m_wcam.dsize[0], m_wcam.dsize[1]) * (scale * 0.5 - 0.5);
            glLoadView2D(m_wcam, offset, scale);

            glRenderImg(m_img);
        }
        {
            // view 3D
            glLoadView3D(m_wcam, m_viewPos, m_viewScale);

            // render points
            if (m_sfm.getMPnts() != NULL) {
                const Mem1<MapPoint> &mpnts = *m_sfm.getMPnts();

                glPointSize(4.f);

                glLoadMatrix(m_pose);

                glBegin(GL_POINTS);
                for (int i = 0; i < mpnts.size(); i++) {
                    if (mpnts[i].err > static_cast<double>(m_err)) continue;
                    glColor(mpnts[i].col);
                    glVertex(mpnts[i].pos);
                }
                glEnd();
            }

            // render cam
            if (m_sfm.size() > 0) {

                glLineWidth(2.f);

                glColor3d(0.5, 0.5, 0.8);

                for (int i = 0; i < m_sfm.size(); i++) {
                    const View *view = m_sfm.getView(i);

                    if (view == NULL || view->state != View::POSE_VALID) continue;
                    glLoadMatrix(m_pose * invPose(view->pose));

                    glBegin(GL_LINES);
                    glCam(view->cam, 0.03 * m_pose.trn.z);
                    glEnd();
                }
            }

            // render axis
            {
                glLoadMatrix(invPose(m_axis) * m_pose.rot);

                glLineWidth(2.f);
                glBegin(GL_LINES);
                glAxis(0.05 * m_pose.trn.z);
                glEnd();
            }
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

    SfMGUI win;
    win.execute("sfm", 800, 600);

    return 0;
}
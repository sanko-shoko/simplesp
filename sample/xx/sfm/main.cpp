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
    CalibTool m_ctool;

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

        printf("option \n");
        printf("'x' key : add detected points for calibration \n");
        printf("'c' key : execute calibration (i >= 3) \n");
    }

    virtual void init() {
        help();

        const double distance = 20.0;
        m_pose = getPose(getVec(0.0, 0.0, +distance));
        m_axis = getPose(getVec(0.0, 0.0, -distance));

        m_err = 1.0f;

        m_addcnt = 0;
        reset();
    }

    void reset() {
        //loadText(SP_DATA_DIR "/image/shiba.txt", m_cam);
        m_cam = getCamParam(0, 0);
        m_sfm.clear();
        m_addcnt = 0;

        m_ctool.clear();
    }

    void update(){
        m_sfm.update();
    }

    void addView() {
        //char path[256];
        //if (1) {
        //    for (int i = 0; i < 1; i++) {
        //        sprintf(path, "test%03d.bmp", m_addcnt++);
        //        Mem2<Col3> img;
        //        loadBMP(path, img);
        //        m_sfm.addView(m_cam, img);
        //    }
        //    //m_img = img;
        //}
        //else {
        //    sprintf(path, "test%03d.bmp", m_addcnt++);
        //    Mem2<Col3> img = m_backup;
        //    saveBMP(path, img);
        //    m_sfm.addView(m_cam, img);
        //}

        CamParam cam;
        if (m_cam.dsize[0] * m_cam.dsize[1] > 0) {
            cam = m_cam;
        }
        else {
            cam = getCamParam(m_img.dsize);
        }

        m_sfm.addView(cam, m_img);
    }

    void capture() {
        static cv::VideoCapture cap(0);
        cv::Mat cvimg;
        cap >> cvimg;

        cvCnvImg(m_img, cvimg);
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] == 1) {
            m_thread.run<SfMGUI, &SfMGUI::addView>(this);
        }

        if (m_keyAction[GLFW_KEY_S] == 1) {
            m_thread.run<SfMGUI, &SfMGUI::reset>(this);
        }

        {
            if (m_keyAction[GLFW_KEY_X] == 1) {
                const DotMarkerParam mrk(5, 5, 30.0);
                m_ctool.addImg(mrk, m_img);
            }

            if (m_keyAction[GLFW_KEY_C] == 1) {
                if (m_ctool.execute() == true) {
                    m_cam = *m_ctool.getCam();
                }
            }
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
            {
                glPointSize(4.f);

                glLoadMatrix(m_pose);

                glBegin(GL_POINTS);
                for (int i = 0; i < m_sfm.msize(); i++) {
                    const MapPnt *pnt = m_sfm.getMPnt(i);
                    if (pnt->err > m_err) continue;
                    glColor(pnt->col);
                    glVertex(pnt->pos);
                }
                glEnd();
            }

            // render cam
            {
                glLineWidth(2.f);

                glColor3d(0.5, 0.5, 0.8);

                for (int i = 0; i < m_sfm.vsize(); i++) {
                    const View *view = m_sfm.getView(i);

                    if (view == NULL || view->valid == false) continue;
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
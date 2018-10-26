#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class SfMGUI : public BaseWindow {

    SfM m_sfm;

    // cam pose (object to cam pose)
    Pose m_pose;

    // axis pose (cam to axis pose)
    Pose m_axis;

    // 
    float m_err;

private:

    void help() {
        printf("'a' key : update\n");
        printf("'s' key : reset\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {
        help();

        const double distance = 10.0;
        m_pose = getPose(getVec(0.0, 0.0, +distance));
        m_axis = getPose(getVec(0.0, 0.0, -distance));

        m_err = 1.0f;
        ImGui::GetIO().IniFilename = NULL;
        reset();
    }

    void reset(){
        m_sfm.init();

        CamParam cam;
        loadText(SP_DATA_DIR "/image/shiba.txt", cam);

        for (int i = 0; i < 7; i++) {
            char path[512];
            sprintf(path, SP_DATA_DIR "/image/shiba%02d.bmp", i);

            Mem2<Col3> img;
            loadBMP(path, img);

            m_sfm.addView(img, &cam);
        }
        m_sfm.update();
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] >= 1) {
            m_sfm.update();
        }

        if (m_keyAction[GLFW_KEY_S] == 1) {
            reset();
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


int main(){

    SfMGUI win;
    win.execute("sfm", 800, 600);

    return 0;
}
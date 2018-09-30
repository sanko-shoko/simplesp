#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class ICPGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // model
    Mem1<Mesh3> m_model;

    Pose m_pose;

    // data A
    Mem<VecPN3> m_dataA;

    // data B
    Mem<VecPN3> m_dataB;

private:

    void help() {
        printf("dataA (point cloud) controlled by mouse\n");
        printf("'a' key : render dataB (point cloud)\n");
        printf("'s' key : render dataB (depth map)\n");
        printf("'d' key : calc ICP (dataA <-> dataB)\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {
        help();

        m_cam = getCamParam(640, 480);

        if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
        m_dataA = getModelPoint(m_model);
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] == 1) {
            m_dataB.resize(2, m_cam.dsize);

            m_dataB.zero();
            renderVecPN(m_dataB, m_cam, m_pose, m_model);

            const double distance = getModelDistance(m_model, m_cam);
            const double radius = getModelRadius(m_model);
            cnvDepthToImg(m_img, m_dataB, distance - 2 * radius, distance + 2 * radius);
        }

        if (m_keyAction[GLFW_KEY_S] == 1) {
            const int dsize[1] = { m_dataA.size() };
            m_dataB.resize(1, dsize);

            for (int i = 0; i < m_dataA.size(); i++) {
                m_dataB[i] = m_pose * m_dataA[i];
            }
        }

        if (m_keyAction[GLFW_KEY_D] > 0) {

            // point to point
            if (m_dataB.dim == 1) {
                calcICP(m_pose, m_dataB, m_dataA, 1);
            }

            // point to 2d map
            if (m_dataB.dim == 2) {
                calcICP(m_pose, m_cam, m_dataB, m_dataA, 1);
            }
        }
    }

    virtual void display() {

        // render dataB
        {
            if (m_dataB.dim == 1) {
                glLoadView3D(m_cam, m_viewPos, m_viewScale);
                glPointSize(5.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.2f, 0.7f);
                for (int i = 0; i < m_dataB.size(); i++) {
                    glVertex(m_dataB[i].pos);
                }
                glEnd();
            }

            if (m_dataB.dim == 2) {
                glLoadView2D(m_cam, m_viewPos, m_viewScale);
                glRenderImg(m_img);
            }
        }

        // render dataA
        {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glClear(GL_DEPTH_BUFFER_BIT);
            {
                glLoadMatrix(m_pose);

                // render points
                glPointSize(3.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.7f, 0.2f);
                for (int i = 0; i < m_dataA.size(); i++) {
                    glVertex(m_dataA[i].pos);
                }
                glEnd();
            }

            renderAxis();
        }
    }

    void renderAxis() {
        glLoadMatrix(m_pose);

        glLineWidth(2.f);
        glBegin(GL_LINES);
        glAxis(100.0);
        glEnd();
    }

    virtual void mousePos(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

    virtual void mouseScroll(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

};

int main(){

    ICPGUI win;
    win.execute("render", 800, 600);

    return 0;
}
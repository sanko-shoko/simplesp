#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class ModelTrackGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // map
    Mem2<VecPN3> m_map;

    // model
    Mem1<Mesh3> m_model;

    // pose
    Pose m_pose;

    // pose model
    Mem1<PoseModel> m_pmodels;

    // mode
    int m_mode;

private:

    void help() {
        printf("'r' key : render\n");
        printf("'c' key : track 2d\n");
        printf("'v' key : track 3d\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {
        help();

        m_mode = 0;

        m_cam = getCamParam(640, 480);

        if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        const double distance = getModelDistance(m_model, m_cam);

        printf("please wait...\n");
        const int level = 2;
        m_pmodels = getPoseModel(m_model, distance, level);

        m_pose = getPose(getVec(0.0, 0.0, distance));
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_R] == 1) {
            m_map.zero();
            renderVecPN(m_map, m_cam, m_pose, m_model);

            cnvNormalToImg(m_img, m_map);
        }

        if (m_keyAction[GLFW_KEY_C] > 0) {
            if (m_img.size() == 0) return;
            m_mode = 0;

            Mem2<Byte> gry;
            cnvImg(gry, m_img);

            track2D(m_pose, gry, m_cam, m_pmodels, 50, 1);
        }

        if (m_keyAction[GLFW_KEY_V] > 0) {
            if (m_map.size() == 0) return;
            m_mode = 1;

            track3D(m_pose, m_map, m_cam, m_pmodels, 1);
        }
    }

    virtual void display() {

        // render image
        {

            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glRenderImg(m_img);
        }

        // render model
        {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glLoadMatrix(m_pose);

            glClear(GL_DEPTH_BUFFER_BIT);
            {

                glRenderOutline(m_model);
            }

            glClear(GL_DEPTH_BUFFER_BIT);
            {
                // render points
                glPointSize(5.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.7f, 0.2f);

                const int id = findPoseModel(m_pmodels, m_pose);

                if (m_mode == 0) {
                    for (int i = 0; i < m_pmodels[id].edges.size(); i++) {
                        glVertex(m_pmodels[id].edges[i].pos);
                    }
                }
                else {
                    for (int i = 0; i < m_pmodels[id].pnts.size(); i++) {
                        glVertex(m_pmodels[id].pnts[i].pos);
                    }
                }
                glEnd();
            }

            glClear(GL_DEPTH_BUFFER_BIT);
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

    ModelTrackGUI win;
    win.execute("modeltrack", 800, 600);

    return 0;
}
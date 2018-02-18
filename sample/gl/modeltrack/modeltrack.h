#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class ModelTrackGUI : public BaseWindow{

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // model
    Mem1<Mesh> m_model;

    // pose
    Pose m_pose;

    // pose model
    Mem1<PoseModel> m_pmodels;

private:

    void help() {
        printf("'n' key : render normal\n");
        printf("'c' key : calc track\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init(){
        help();

        m_cam = getCamParam(640, 480);

        if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false){

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        printf("please wait...\n");
        const double distance = getModelDistance(m_model, m_cam);
        m_pmodels = getPoseModel(m_model, distance);

        m_pose = getPose(getVec(0.0, 0.0, distance));

    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_N] == 1) {
            Mem2<VecPN3> map;
            renderVecPN(map, m_cam, m_pose, m_model);

            cnvNormalToImg(m_img, map);
        }

        if (m_keyAction[GLFW_KEY_C] > 0) {
            if (m_img.size() == 0) return;

            Mem2<Byte> gry;
            cnvImg(gry, m_img);

            track2D(m_pose, gry, m_cam, m_pmodels, 50, 1);
        }
    }

    virtual void display(){

        // render dataB
        {

            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glRenderImage(m_img);
        }

        // render dataA
        {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glClear(GL_DEPTH_BUFFER_BIT);
            {
                glLoadMatrix(m_pose);

                glRenderOutline(m_model);

                // render points
                glPointSize(5.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.7f, 0.2f);

                const int id = findPoseModel(m_pmodels, m_pose);
                for (int i = 0; i < m_pmodels[id].edges.size(); i++) {
                    glVertex(m_pmodels[id].edges[i].pos);
                }
                glEnd();
                
                // render axis
                glLineWidth(2.f);
                glBegin(GL_LINES);
                glAxis(100.0);
                glEnd();
            }
        }

    }

    virtual void mousePos(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

    virtual void mouseScroll(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

};

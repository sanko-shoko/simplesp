#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class ModelTrackGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;
    Mem2<Byte> m_gry;

    Mem2<double> m_depth;

    // model
    Mem1<Mesh3> m_model;

    double m_radius;
    double m_distance;

    // object to cam pose
    Pose m_pose;

    // estimated pose
    Pose m_est;
    
    // pose model
    Mem1<PoseModel> m_pmodels;

    // mode (2D / 3D)
    int m_mode;

    bool m_start;

private:

    void help() {
        printf("'s' key : start & stop moving\n");
        printf("'d' key : track 2d (edge based)\n");
        printf("'f' key : track 3d (depth based)\n");
        printf("'r' key : reset pose\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {
        help();

        m_mode = -1;

        m_cam = getCamParam(640, 480);

        SP_ASSERT(loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply"));

        m_radius = getModelRadius(m_model);
        m_distance = getModelDistance(m_model, m_cam);

        printf("please wait...\n");
        const int level = 2;
        m_pmodels = getPoseModel(m_model, m_distance, level);

        m_pose = getPose(getVec(0.0, 0.0, m_distance));

        m_start = true;
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_R] == 1) {
            m_est = m_pose;
        }
        if (m_keyAction[GLFW_KEY_S] == 1) {
            m_start = (m_start == false) ? true : false;
        }

        if (m_keyAction[GLFW_KEY_D] >= 1) {
            if (m_mode < 0) m_est = m_pose;
            m_mode = 0;
            track2D(m_est, m_gry, m_cam, m_pmodels, 50, 1);
        }
        if (m_keyAction[GLFW_KEY_F] >= 1) {
            if (m_mode < 0) m_est = m_pose;
            m_mode = 1;
            Mem2<Vec3> map;
            cnvDepthToVec(map, m_cam, m_depth);
            track3D(m_est, map, m_cam, m_pmodels, 1);
        }

    }

    virtual void display() {
        // render
        {
            static double s = 0.0;

            if (m_start == true) {
                m_pose *= getRotAngle(getVec(+::sin(s), +::cos(s), +0.0), -0.02);
                m_pose.trn.x = ::cos(s * 2.0) * m_radius * 0.4;
                m_pose.trn.y = ::sin(s * 2.0) * m_radius * 0.4;

                s += 0.01;
            }

            Mem2<VecPN3> map;
            renderVecPN(map, m_cam, m_pose, m_model);

            cnvNormalToImg(m_img, map);
            cnvImg(m_gry, m_img);

            cnvVecPNToDepth(m_depth, map);
        }

        // render image
        {
            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glRenderImg(m_img);
        }

        // tracking
        if (m_start == true) {
            if (m_mode == 0) {
                track2D(m_est, m_gry, m_cam, m_pmodels, 50, 3);
            }
            if (m_mode == 1) {
                Mem2<Vec3> map;
                cnvDepthToVec(map, m_cam, m_depth);
                track3D(m_est, map, m_cam, m_pmodels, 3);
            }
        }

        // render model
        if(m_mode >= 0){
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glLoadMatrix(m_est);

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

                const int id = findPoseModel(m_pmodels, m_est);

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
        }

    }

    virtual void mousePos(double x, double y) {
        controlPose(m_est, m_mouse, m_wcam, m_viewScale);
    }

    virtual void mouseScroll(double x, double y) {
        controlPose(m_est, m_mouse, m_wcam, m_viewScale);
    }

};


int main(){

    ModelTrackGUI win;
    win.execute("modeltrack", 800, 600);

    return 0;
}
#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class TrackGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object surface points
    Mem1<VecPN3> m_pnts;

    // object to cam pose
    Pose m_pose;

    Mem2<VecPN3> m_map;

    double m_near, m_far;

    Track3DRF track3d;


private:

    void help() {
        printf("'d' key : render depth\n");
        printf("'n' key : render normal\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        m_cam = getCamParam(640, 480);

        help();

        SP_ASSERT(loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply"));

        m_pnts = getModelPoint(m_model);
        m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));


        const double distance = getModelDistance(m_model, m_cam);
        const double radius = getModelRadius(m_model);

        m_map.resize(m_cam.dsize);
        m_map.zero();

        m_near = distance - 2 * radius;
        m_far = distance + 2 * radius;
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_keyAction[GLFW_KEY_L] == 1) {
            track3d.train(m_model);
            m_pose.rot = getGeomPose(1, 0, 100).rot;
        }
        if (m_keyAction[GLFW_KEY_R] == 1) {
            m_map.zero();
            renderVecPN(m_map, m_cam, m_pose, m_model);
        }

        if (m_keyAction[GLFW_KEY_A] == 1) {
            Mem2<double> depth(m_map.dsize);
            cnvVecPNToDepth(depth, m_map);

            track3d.updatePose(m_pose, m_cam, depth);
        }
    }

    virtual void display() {

        {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);

            Mem2<Byte> img;
            cnvDepthToImg(img, m_map, m_near, m_far);
            glRenderImg(img);
        }
        {
            // view 3D
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            {
                glLoadMatrix(m_pose);

                glPointSize(3.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.7f, 0.2f);
                for (int i = 0; i < m_pnts.size(); i++) {
                    glVertex(m_pnts[i].pos);
                }
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



int main(){

	TrackGUI win;
	win.execute("track3d", 800, 600);

	return 0;
}
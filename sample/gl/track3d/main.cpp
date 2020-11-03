#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class TrackGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // object mesh model
    Mem1<Mesh3> m_model;

    double m_radius;
    double m_distance;

    // object to cam pose
    Pose m_pose;

    // estimated pose
    Pose m_est;

    Mem2<double> m_depth;

    TrackRF m_tracker;

    bool m_start;

private:

    void help() {
        printf("'t' key : training\n");
        printf("'s' key : start & stop moving\n");
        printf("'a' key : estimate one step\n");
        printf("\n");
    }

    virtual void init() {

        help();

        m_cam = getCamParam(640, 480);

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        SP_ASSERT(m_model.size() > 0);

        m_radius = getModelRadius(m_model);
        m_distance = getModelDistance(m_model, m_cam);

        m_pose = getPose(getVec3(0.0, 0.0, m_distance));
 
        m_depth.resize(m_cam.dsize);
        m_depth.zero();

        m_start = true;
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_key[GLFW_KEY_T] == 1) {
            if (m_tracker.valid() == false) {
                m_tracker.train(m_model);
            }
            m_est = m_pose;
        }
        if (m_key[GLFW_KEY_S] == 1) {
            m_start = (m_start == false) ? true : false;
        }
        if (m_key[GLFW_KEY_A] >= 1) {
            if (m_tracker.valid() == true) {
                m_tracker.execute(m_est, m_cam, m_depth);
            }
        }
    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render depth map
        {
            static double s = 0.0;
            
            if (m_start == true) {
                m_pose *= getRotAngle(getVec3(+::sin(s), +::cos(s), +0.0), -0.02);
                m_pose.pos.x = ::cos(s * 2.0) * m_radius * 0.4;
                m_pose.pos.y = ::sin(s * 2.0) * m_radius * 0.4;

                s += 0.01;
            }

            m_depth.zero();
            renderDepth(m_depth, m_cam, m_pose, m_model);

            Mem2<double> tmp(m_cam.dsize);
            tmp.zero();

            for (int u = 0; u < m_cam.dsize[0]; u++) {
                for (int v = 0; v < m_cam.dsize[1] * 2 / 10; v++) {
                    tmp(u, v) = m_distance - 1.8 * m_radius;
                }
                for (int v = m_cam.dsize[1] * 4 / 10; v < m_cam.dsize[1] * 6 / 10; v++) {
                    tmp(u, v) = m_distance + 1.8 * m_radius;
                }
                for (int v = m_cam.dsize[1] * 8 / 10; v < m_cam.dsize[1] * 10 / 10; v++) {
                    tmp(u, v) = m_distance - 1.8 * m_radius;
                }
            }
            for (int i = 0; i < m_depth.size(); i++) {
                if (tmp[i] > 0.0 && (m_depth[i] == 0.0 || m_depth[i] > tmp[i])) {
                    m_depth[i] = tmp[i];
                }
            }
        }

        // tracking
        if (m_tracker.valid() == true && m_start == true) {
            m_tracker.execute(m_est, m_cam, m_depth, 2);
        }

        {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);

            glTexDepth<Byte>(m_depth, m_distance - 2 * m_radius, m_distance + 2 * m_radius);
        }

        {
            // view 3D
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glLoadMatrix(m_est);

            glRenderOutline(m_model);
            glEnd();
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

	TrackGUI win;
	win.execute("tracker", 800, 600);

	return 0;
}
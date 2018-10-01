#define SP_USE_DEBUG 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class KinectFusionGUI : public BaseWindow {

private:

    // Kinect Fusion
    KinectFusion m_kfusion;

    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object to cam pose
    Pose m_pose;

private:

    void help() {
        printf("'r' key : reset kinect fusion\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_keyAction[GLFW_KEY_R] == 1) {
            m_kfusion.reset();
        }
    }

    virtual void init() {

        help();

        m_cam = getCamParam(320, 240);
        m_viewScale *= 1.5;
        //m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();

        if (loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply", m_model) == false) {

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
    }

    virtual void display() {
        Mem2<double> depth;

        // render depth
        {
            static double s = 0.0;
            m_pose *= getRotAngle(getVec(+0.0, +::sin(s), +::cos(s)), 0.02);
            s += 0.01;

            renderDepth(depth, m_cam, m_pose, m_model);
        }

        // kinect fusion
        {

            if (m_kfusion.track() == false) {
                const double radius = getModelRadius(m_model);
                m_kfusion.setMap(200, radius / 100.0, m_pose);
                m_kfusion.setCam(m_cam);
            }

            m_kfusion.execute(depth);
        }

        // render
        {
            glLoadView2D(m_kfusion.getCam(), m_viewPos, m_viewScale);
            glRenderNormal<Byte>(m_kfusion.getCast());


            glLoadView3D(m_kfusion.getCam(), m_viewPos, m_viewScale);
            glLoadMatrix(*m_kfusion.getPose());

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glBegin(GL_QUADS);
            glCube(m_kfusion.getMap().dsize[0] * m_kfusion.getMap().unit);
            glEnd();
        }

        // render depth
        {
            const double scale = 0.3;

            const Vec2 offset = getVec(m_cam.dsize[0], m_cam.dsize[1]) * (0.5 - scale * 0.5);
            glLoadView2D(m_cam, m_viewPos - offset * m_viewScale, m_viewScale * scale);

            glRenderDepth(depth, m_pose.trn.z - 500.0, m_pose.trn.z + 500.0);
        }

    }
};


int main(){

    KinectFusionGUI win;
    win.execute("kinectfusion", 800, 600);

    return 0;
}
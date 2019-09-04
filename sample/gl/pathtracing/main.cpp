#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class PathTracingGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // image
    Mem2<Col4> m_img;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object to cam pose
    Pose m_pose;

    PathTrace m_pt;

private:

    void help() {
        printf("\n");
    }

    virtual void init() {

        help();

        m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        if (m_model.size() == 0) {
            // if could not find stanford bunny, load dummy model
            m_model = loadGeodesicDorm(100.0, 1);
        }

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        m_model.push(m_model + getVec3(0, 0, 100));
        SP_ASSERT(m_model.size() > 0);

        m_pose = getPose(getVec3(0.0, 0.0, getModelDistance(m_model, m_cam)));


        static Mem1<Material*> pmats;
        pmats.resize(m_model.size());
        static Material mat;
        mat.dif = getCol4(255, 100, 100, 255);
        for (int i = 0; i < pmats.size(); i++) {
            pmats[i] = &mat;
        }

        static Mem1<Vec3> lights;
        lights.push(getVec3(0.0, 0.0, -1000.0));

        m_pt.set(lights);
        m_pt.set(m_cam, m_pose, true);
        m_pt.add(m_model, pmats);
        m_pt.build();

    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_key[GLFW_KEY_D] == 1 || m_key[GLFW_KEY_N] == 1) {
        }

    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        m_pt.set(m_cam, m_pose, true);
        m_pt.update();

        m_pt.makeImg(m_img, 1.0, NULL);
        {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glTexImg(m_img);
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
    PathTracingGUI win;
    win.execute("pathtracing", 800, 600);

    return 0;
}
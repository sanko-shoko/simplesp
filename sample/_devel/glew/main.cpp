#define SP_USE_GLEW 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class RenderGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object to cam pose
    Pose m_pose;

    FrameBuffer m_fb;

private:

    void help() {
        printf("'a' key : render image\n");
        printf("'s' key : render depth\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        help();

        m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();

        m_fb.resize(m_cam.dsize);

        if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        const double distance = getModelDistance(m_model, m_cam);
        const double radius = getModelRadius(m_model);

        m_fb.bind();
        display();
        m_fb.unbind();

        if (m_keyAction[GLFW_KEY_A] == 1) {
            m_fb.readImg(m_img);
        }
        if (m_keyAction[GLFW_KEY_S] == 1) {

            Mem2<double> depth;
            m_fb.readDepth(depth);
            cnvDepthToImg(m_img, depth, distance - 2 * radius, distance + 2 * radius);
        }
    }

    virtual void display() {

        {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glRenderImg(m_img);
        }

        {
            // view 3D
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            renderSurface();
            renderAxis();
        }
    }

    void renderSurface() {
        glLoadMatrix(m_pose);

        const GLfloat diffuse[] = { 0.4f, 0.5f, 0.5f, 1.0f };
        glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);

        glRenderSurface(m_model);
    }

    void renderAxis() {
        glDisable(GL_DEPTH_TEST);

        glLoadMatrix(m_pose);

        glLineWidth(2.f);
        glBegin(GL_LINES);
        glAxis(100.0);
        glEnd();
        
        glEnable(GL_DEPTH_TEST);
    }

    virtual void mousePos(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

    virtual void mouseScroll(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

};


int main() {

    RenderGUI win;
    win.execute("render", 800, 600);

    return 0;
}
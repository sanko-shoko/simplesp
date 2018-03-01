#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class VoxelGUI : public BaseWindow{

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // model
    Mem1<Mesh> m_model;

    Voxel m_voxel;

    // pose
    Pose m_pose;


private:

    void help() {
        printf("'r' key : \n");
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

        const double radius = getModelRadius(m_model);
        const double distance = getModelDistance(m_model, m_cam);

        cnvVoxel(m_voxel, m_model, 4.0);

        m_pose = getPose(getVec(0.0, 0.0, distance));
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_R] == 1) {
        }
    }

    virtual void display(){

        // render image
        {

            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glRenderImg(m_img);
        }

        // render model
        {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glClear(GL_DEPTH_BUFFER_BIT);
            {
                glLoadMatrix(m_pose);

                glRenderVoxel(m_voxel);

                glRenderOutline(m_model);

                // render points
                glPointSize(5.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.7f, 0.2f);

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

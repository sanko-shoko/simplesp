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
        printf("'s' key : output ply\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        help();

        if (0) {
            m_cam = getCamParam(640, 480);
        }
        else {
            m_cam = getCamParam(320, 240);
            m_viewScale *= 1.5;
        }

        m_img.resize(m_cam.dsize);
        m_img.zero();

        if (loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply", m_model) == false) {

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_keyAction[GLFW_KEY_R] == 1) {
            m_kfusion.reset();
        }
        if (m_keyAction[GLFW_KEY_S] == 1) {
            Mem1<Mesh3> model;
            cnvVoxelToModel(model, *m_kfusion.getMap());
            savePLY("model.ply", model);
        }
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

            if (m_kfusion.valid() == false) {
                const double radius = getModelRadius(m_model) * 1.1;

                // voxel size
                const int vsize = 200;

                // voxel unit length [mm]
                const double unit = (2 * radius) / vsize;

                // voxel pose
                const Pose pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));

                m_kfusion.init(vsize, unit, m_cam, pose);
            }

            m_kfusion.execute(depth);
            SP_LOGGER_PRINT(NULL);
        }

        // render
        {
            glLoadView2D(m_kfusion.getCam(), m_viewPos, m_viewScale);
            glTexNormal<Byte>(*m_kfusion.getCast());


            glLoadView3D(m_kfusion.getCam(), m_viewPos, m_viewScale);
            glLoadMatrix(*m_kfusion.getPose());

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glBegin(GL_QUADS);
            glCube(m_kfusion.getMap()->dsize[0] * m_kfusion.getMap()->unit);
            glEnd();
        }

        // render depth
        if(0){
            const double scale = 0.3;

            const Vec2 offset = getVec(m_cam.dsize[0], m_cam.dsize[1]) * (0.5 - scale * 0.5);
            glLoadView2D(m_cam, m_viewPos - offset * m_viewScale, m_viewScale * scale);

            glTexDepth(depth, m_pose.trn.z - 500.0, m_pose.trn.z + 500.0);
        }
        else {
            Mem2<Col3> img;
            cnvDepthToImg(img, depth, m_pose.trn.z - 500.0, m_pose.trn.z + 500.0);

            glShowImg(this, "input depth map", img);
        }

    }
};


int main(){

    KinectFusionGUI win;
    win.execute("kinectfusion", 800, 600);

    return 0;
}
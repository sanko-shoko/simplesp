#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class VoxelGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // model
    Mem1<Mesh3> m_model;

    // view pose
    Pose m_view;

    // multi camerea poses
    Mem1<Pose> m_poses;

    // multi camerea pameras
    Mem1<CamParam> m_cams;

    // voxel
    Voxel<> m_voxel;

    // reconstruct
    Mem1<Mesh3> m_modelR;

    int m_mode;

private:

    void help() {
        printf("'v' key : execute visual hull\n");
        printf("'s' key : switch mode (original model <-> reconstructed model)\n");
        printf("\n");
    }

    virtual void init() {
        help();

        m_mode = 0;

        m_cam = getCamParam(640, 480);

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        if (m_model.size() == 0) {
            // if could not find stanford bunny, load dummy model
            m_model = loadGeodesicDorm(100.0, 1);
        }

        const double radius = getModelRadius(m_model);
        const double distance = getModelDistance(m_model, m_cam);

        m_view = getPose(getVec3(0.0, 0.0, 3 * distance)) * getRotAngleX(SP_PI);

        const int level = 0;
        for (int i = 0; i < getGeodesicMeshNum(level); i += 2) {
            const Pose pose = getGeodesicPose(level, i, distance);

            m_poses.push(pose);
            m_cams.push(m_cam);
        }
    }
    void genSilhouette(Mem1<Mem2<Byte> > &imgs, const Mem1<Mesh3> &model, const  Mem1<CamParam> &cams, const Mem1<Pose> &poses) {
        imgs.clear();
        for (int i = 0; i < poses.size(); i++) {

            Mem2<VecPN3> map;
            renderVecPN(map, cams[i], poses[i], model);

            Mem2<Byte> img(map.dsize);
            for (int i = 0; i < img.size(); i++) {
                img[i] = (map[i].pos.z > 0) ? SP_BYTEMAX : 0;
            }
            imgs.push(img);
        }

    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_key[GLFW_KEY_V] == 1) {
            Mem1<Mem2<Byte> > imgs;
            genSilhouette(imgs, m_model, m_cams, m_poses);

            for (int i = 0; i < imgs.size(); i++) {
                char str[256];
                sprintf(str, "img%03d.bmp", i);
                saveBMP(str, imgs[i]);
            }

            visualHull(m_voxel, imgs, m_cams, m_poses);

            cnvVoxelToMesh(m_modelR, m_voxel);

            m_mode = 1;
        }

        if (m_key[GLFW_KEY_S] == 1) {
            m_mode = 1 - m_mode;
        }

    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render model
        {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glClear(GL_DEPTH_BUFFER_BIT);
            {
                glLoadMatrix(m_view);

                if (m_mode == 0) {
                    glRenderSurface(m_model);
                }
                else {
                    glRenderSurface(m_modelR);
                }
                //glRenderOutline(m_model);
            }

            // render cam
            {
                glLineWidth(2.f);
                glColor3d(0.5, 0.5, 0.8);

                for (int i = 0; i < m_poses.size(); i++) {
                    glLoadMatrix(m_view * invPose(m_poses[i]));

                    glBegin(GL_LINES);
                    glCam(m_cam, 50);
                    glEnd();
                }
            }
        }

    }

    virtual void mousePos(double x, double y) {
        controlPose(m_view, m_mouse, m_wcam, m_viewScale);
    }

    virtual void mouseScroll(double x, double y) {
        controlPose(m_view, m_mouse, m_wcam, m_viewScale);
    }

};


int main(){

    VoxelGUI win;
    win.execute("visualhull", 800, 600);

    return 0;
}
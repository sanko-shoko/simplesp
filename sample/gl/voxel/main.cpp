#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

void glRenderVoxel(const Voxel<> &voxel) {

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    {
        glPushMatrix();
        glLoadIdentity();

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        GLfloat lightPos[4] = { 0.f, 0.f, -1000.f, 1.f };
        glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
        glPopMatrix();

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        const Vec3 cent = voxel.center();

        for (int z = 0; z < voxel.vmap.dsize[2]; z++) {
            for (int y = 0; y < voxel.vmap.dsize[1]; y++) {
                for (int x = 0; x < voxel.vmap.dsize[0]; x++) {
                    const char &val = voxel.vmap(x, y, z);
                    if (val < 0) continue;

                    const Vec3 mpos = getVec3(x, y, z);
                    const Vec3 cpos = ((mpos - cent) * voxel.unit);

                    glPushMatrix();
                    glMultMatrix(getPose(cpos));

                    glCube(voxel.unit);

                    glPopMatrix();
                }
            }
        }
    }
    glPopAttrib();
}

class VoxelGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // model
    Mem1<Mesh3> m_model;

    // voxel
    Voxel<> m_voxel;

    // pose
    Pose m_pose;

    int m_mode;

private:

    void help() {
        printf("'s' key : switch mode (voxel <-> mesh)\n");
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

        cnvMeshToVoxel(m_voxel, m_model, 4.0);

        // Marching cubes
        cnvVoxelToMesh(m_model, m_voxel);
        savePLY("model.ply", m_model);

        const double distance = getModelDistance(m_model, m_cam);
        m_pose = getPose(getVec3(0.0, 0.0, distance));
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_key[GLFW_KEY_S] == 1) {
            m_mode ^= 1;
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
                glLoadMatrix(m_pose);

                if (m_mode == 0) {
                    glRenderVoxel(m_voxel);
                }
                else {
                    glRenderSurface(m_model);
                }
                //glRenderOutline(m_model);
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

    VoxelGUI win;
    win.execute("voxel", 800, 600);

    return 0;
}
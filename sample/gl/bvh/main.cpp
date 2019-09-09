//#define SP_USE_DEBUG 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class BVHGUI : public BaseWindow {

private:

    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object to cam pose
    Pose m_pose;

    BVH m_bvh;

    int m_level;
    bool m_stop;
private:

    void help() {
        printf("'a' key : level--\n");
        printf("'s' key : level++\n");
        printf("\n");
    }

    virtual void init() {
        help();
        m_level = 1;
        m_stop = false;
        m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        SP_ASSERT(m_model.size() > 0);

        m_pose = getPose(getVec3(0.0, 0.0, getModelDistance(m_model, m_cam)));
        m_bvh.addMeshes(m_model);
        m_bvh.build();

    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_key[GLFW_KEY_S] == 1) {
            m_level = maxval(0, m_level + 1);
        }
        if (m_key[GLFW_KEY_A] == 1) {
            m_level = maxval(0, m_level - 1);
        }
        if (m_key[GLFW_KEY_D] == 1) {
            m_stop ^= true;
        }
    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(0){
            Mem2<SP_REAL> depth;
            depth.resize(m_cam.dsize);
            depth.zero();

            Mat pmat = getMat(m_pose);
            Pose ipose = invPose(m_pose);
            Mat irmat = getMat(ipose.rot);

            for (int v = 0; v < depth.dsize[1]; v++) {
                for (int u = 0; u < depth.dsize[0]; u++) {
                    const Vec2 prj = invCam(m_cam, getVec2(u, v));
                    const Vec3 vec = unitVec(getVec3(prj.x, prj.y, 1.0));
                    VecPD3 ray;
                    ray.pos = ipose.trn;
                    ray.drc = irmat * (vec);

                    BVH::Hit hit;
                    if (m_bvh.trace(hit, ray, 0, 1500.0)) {
                        depth(u, v) = (pmat * hit.vec.pos).z;
                    }
                }
            }
            glLoadView2D(m_cam);
            glTexDepth(depth, maxval(m_pose.trn.z - 500.0, 10.0), m_pose.trn.z + 500.0);
        }
        else{
            glLoadView3D(m_cam);
            glLoadMatrix(m_pose);

            glRenderSurface(m_model);

            const Mem1<const BVH::Node*> nodes = m_bvh.getNodes(m_level);

            glLineWidth(2.0);
            for (int i = 0; i < nodes.size(); i++) {
                glColor(getCol3(i + nodes.size()));
                const Vec3 A = nodes[i]->box.pos[0];
                const Vec3 B = nodes[i]->box.pos[1];

                glBegin(GL_LINE_LOOP);
                glVertex(getVec3(A.x, A.y, A.z)); glVertex(getVec3(B.x, A.y, A.z)); glVertex(getVec3(B.x, B.y, A.z)); glVertex(getVec3(A.x, B.y, A.z));
                glEnd();

                glBegin(GL_LINE_LOOP);
                glVertex(getVec3(A.x, A.y, B.z)); glVertex(getVec3(B.x, A.y, B.z)); glVertex(getVec3(B.x, B.y, B.z)); glVertex(getVec3(A.x, B.y, B.z));
                glEnd();

                glBegin(GL_LINES);
                glVertex(getVec3(A.x, A.y, A.z)); glVertex(getVec3(A.x, A.y, B.z));
                glVertex(getVec3(B.x, A.y, A.z)); glVertex(getVec3(B.x, A.y, B.z));
                glVertex(getVec3(A.x, B.y, A.z)); glVertex(getVec3(A.x, B.y, B.z));
                glVertex(getVec3(B.x, B.y, A.z)); glVertex(getVec3(B.x, B.y, B.z));
                glEnd();
            }
        }
    }

    virtual void mousePos(double x, double y) {
        if (controlPose(m_pose, m_mouse, m_wcam, m_viewScale) == true) {
        }
    }

    virtual void mouseScroll(double x, double y) {
        if (controlPose(m_pose, m_mouse, m_wcam, m_viewScale) == true) {
        }
    }
};


int main() {

    BVHGUI win;
    win.execute("bvh", 800, 600);

    return 0;
}
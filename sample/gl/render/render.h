#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class RenderGUI : public BaseWindow{

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // object mesh model
    Mem1<Mesh> m_model;

    // object surface points
    Mem1<VecPN3> m_pnts;

    // object to cam pose
    Pose m_pose;

    int m_mode;

private:

    void help() {
        printf("'d' key : render depth\n");
        printf("'n' key : render normal\n");
        printf("'m' key : switch render mode (->points ->meshes ->outline)\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init(){

        help();

        m_mode = 0;

        m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();
    
        if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false){

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }
        
        m_pnts = getModelPoint(m_model);
        m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_D] == 1|| m_keyAction[GLFW_KEY_N] == 1) {
            const double distance = getModelDistance(m_model, m_cam);
            const double radius = getModelRadius(m_model);

            Mem2<VecPN3> map;
            renderVecPN(map, m_cam, m_pose, m_model);

            if (m_keyAction[GLFW_KEY_D] == 1) {
                cnvDepthToImg(m_img, map, distance - 2 * radius, distance + 2 * radius);
            }
            if (m_keyAction[GLFW_KEY_N] == 1) {
                cnvNormalToImg(m_img, map, distance - 2 * radius, distance + 2 * radius);
            }
        }

        if (m_keyAction[GLFW_KEY_M] == 1) {
            if (++m_mode >= 3) m_mode = 0;
        }
    }

    virtual void display(){

        {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glRenderImage(m_img);
        }

        {
            // view 3D
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            switch (m_mode) {
            case 0: renderPoint(); break;
            case 1: renderSurface(); break;
            case 2: renderOutline(); break;
            default: break;
            }

            renderAxis();
        }
    }

    void renderPoint() {

        {
            glLoadIdentity();

            glPointSize(3.f);
            glBegin(GL_POINTS);
            glColor3f(0.2f, 0.7f, 0.2f);

            for (int i = 0; i < m_pnts.size(); i++) {
                // X_C = R * X_M + T
                glVertex(m_pose.rot * m_pnts[i].pos + m_pose.trn);
            }
            glEnd();
        }

        {
            //glLoadMatrix(m_pose);

            //glPointSize(3.f);
            //glBegin(GL_POINTS);
            //glColor3f(0.2f, 0.7f, 0.2f);

            //for (int i = 0; i < m_pnts.size(); i++) {
            //    glVertex(m_pnts[i].pos);
            //}
            //glEnd();
        }
    }

    void renderSurface() {
        glLoadMatrix(m_pose);

        const GLfloat diffuse[] = { 0.4f, 0.5f, 0.5f, 1.0f };
        glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);

        glRenderSurface(m_model);
    }

    void renderOutline() {
        glLoadMatrix(m_pose);

        glRenderOutline(m_model);
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


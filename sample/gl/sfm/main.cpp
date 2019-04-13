#define SFM_DEBUG 0

#include "simplesp.h"
#include "spex/spgl.h"

#if SFM_DEBUG
#include "spex/spcv.h"
#endif

using namespace sp;

class SfMGUI : public BaseWindow {

    SfM m_sfm;

    // cam pose (object to cam pose)
    Pose m_pose;

    // axis pose (cam to axis pose)
    Pose m_axis;

    // 
    float m_err;

    double m_cscale;

private:

    void help() {
        printf("'a' key : update\n");
        printf("'s' key : reset\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {
        help();

        const double distance = 20.0;
        m_pose = getPose(getVec3(0.0, 0.0, +distance));
        m_axis = getPose(getVec3(0.0, 0.0, -distance));

        m_err = 3.0f;

        m_cscale = 0.03;

        reset();
    }

    void reset() {
        m_sfm.clear();

#if SFM_DEBUG
        return;
#endif

        for (int i = 0; i < 7; i++) {
            CamParam cam;
            Mem2<Col3> img;
            if (loadShiba(cam, img, i) == false) break;

            m_sfm.addView(cam, img);
        }
        m_sfm.update();
    }

    bool loadShiba(CamParam &cam, Mem2<Col3> &img, const int i) {
        if (i >= 7) return false;
        loadText(SP_DATA_DIR "/image/shiba.txt", cam);

        char path[512];
        sprintf(path, SP_DATA_DIR "/image/shiba%02d.bmp", i);
        loadBMP(path, img);

        return true;
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] >= 1) {
            m_sfm.update();
        }

        if (m_keyAction[GLFW_KEY_S] == 1) {
            reset();
        }

        if (m_keyAction[GLFW_KEY_D] == 1) {
            m_cscale += 0.01;
            if (m_cscale > 0.03) m_cscale = 0.01;
        }

#if SFM_DEBUG
        if (m_keyAction[GLFW_KEY_T] == 1) {
            add();
        }
#endif

    }

    virtual void display() {

        // view 3D
        glLoadView3D(m_wcam, m_viewPos, m_viewScale);

        // render points
        {
            glPointSize(4.f);

            glLoadMatrix(m_pose);

            glBegin(GL_POINTS);
            for (int i = 0; i < m_sfm.msize(); i++) {
                const MapPnt *pnt = m_sfm.getMPnt(i);
                if (pnt->err > m_err) continue;
                glColor(pnt->col);
                glVertex(pnt->pos);
            }
            glEnd();
        }

        // render cam
        {
            glLineWidth(2.f);

            glColor3d(0.5, 0.5, 0.8);

            for (int i = 0; i < m_sfm.vsize(); i++) {
                const View *view = m_sfm.getView(i);

                if (view == NULL || view->valid == false) continue;
                glLoadMatrix(m_pose * invPose(view->pose));

                glCam(view->cam, m_cscale * m_pose.trn.z);
            }
        }

        // render axis
        {
            glLoadMatrix(invPose(m_axis) * m_pose.rot);

            glLineWidth(2.f);
            glAxis(0.05 * m_pose.trn.z);
        }
    }

    virtual void mousePos(double x, double y) {
        Pose delta = zeroPose();
        controlPose(delta, m_mouse, m_wcam, m_viewScale, m_axis);
        m_pose = delta * m_pose;
    }

    virtual void mouseScroll(double x, double y) {
        Pose delta = zeroPose();
        controlPose(delta, m_mouse, m_wcam, m_viewScale, m_axis);
        m_pose.trn.z += delta.trn.z;
        m_axis.trn.z -= delta.trn.z;
    }


    //--------------------------------------------------------------------------------
    // option
    //--------------------------------------------------------------------------------

    bool loadTsukuba(CamParam &cam, Mem2<Col3> &img, const int i) {

        const double f = 615.0;
        cam = getCamParam(640, 480, f, f);

        char path[512];
        sprintf(path, SP_DATA_DIR "/tsukuba/NewTsukubaStereoDataset/illumination/fluorescent/left/tsukuba_fluorescent_L_%05d.png", i + 1);

        bool ret = false;
#if SFM_DEBUG
        ret = cvLoadImg(path, img);
#endif

        return ret;
    }

    void add() {
        const int v = m_sfm.vsize();
        CamParam cam;
        Mem2<Col3> img;

        static int cnt = 0;

        int scale = (v < 15) ? 15 : 4;
        cnt += scale;
        if (loadTsukuba(cam, img, cnt) == false) return;
        const Pose *hint = m_sfm.getlatestPose();

        m_sfm.addView(cam, img, hint);
        m_sfm.update();
    }

};


int main() {

    SfMGUI win;
    win.execute("sfm", 800, 600);

    return 0;
}
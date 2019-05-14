#define SP_USE_THREAD 1
#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

using namespace sp;

class SfMGUI : public BaseWindow {

private:

    SfM m_sfm;

    // cam pose (object to cam pose)
    Pose m_pose;

    // axis pose (cam to axis pose)
    Pose m_axis;

    CamParam m_cam;
    Mem2<Col3> m_img;
 
    // calibration tool;
    CalibTool m_ctool;

    // thread;
    Thread m_thread;

    // 
    float m_err;

    // 
    int m_addcnt;

private:

    void help() {
        printf("'a' key : add image\n");
        printf("'s' key : reset\n");
        printf("'ESC' key : exit\n");
        printf("\n");

        printf("option \n");
        printf("'x' key : add detected points for calibration \n");
        printf("'c' key : execute calibration (i >= 3) \n");
    }

    virtual void init() {
        help();
        reset();
    }

    void reset() {
        m_cam = getCamParam(0, 0);
        //loadText(SP_DATA_DIR "/image/shiba.txt", m_cam);

        m_sfm.clear();

        const double distance = 20.0;
        m_pose = getPose(getVec3(0.0, 0.0, +distance));
        m_axis = getPose(getVec3(0.0, 0.0, -distance));

        m_err = 3.0f;

        m_addcnt = 0;

        m_ctool.clear();
    }

    void update(){
        m_sfm.update();
    }

    void addView() {
        CamParam cam;
        if (m_cam.dsize[0] * m_cam.dsize[1] > 0) {
            cam = m_cam;
        }
        else {
            cam = getCamParam(m_img.dsize);
        }
        
        //char path[256];
        //if (0) {
        //    for (int i = 0; i < 1; i++) {
        //        sprintf(path, "img%03d.bmp", m_addcnt++);
        //        Mem2<Col3> img;
        //        loadBMP(path, img);
        //        m_sfm.addView(cam, img);
        //        m_addcnt++;
        //    }
        //    return;
        //}
        //else {
        //    sprintf(path, "img%03d.bmp", m_addcnt++);
        //    saveBMP(path, m_img);
        //}

        m_sfm.addView(cam, m_img);
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_key[GLFW_KEY_A] == 1) {
            m_thread.run<SfMGUI, &SfMGUI::addView>(this);
        }

        if (m_key[GLFW_KEY_S] == 1) {
            m_thread.run<SfMGUI, &SfMGUI::reset>(this);
        }

        {
            if (m_key[GLFW_KEY_X] == 1) {
                const DotMarkerParam mrk(5, 5, 30.0);
                m_ctool.addImg(mrk, m_img);
            }

            if (m_key[GLFW_KEY_C] == 1) {
                if (m_ctool.execute() == true) {
                    m_cam = *m_ctool.getCam();
                }
            }
        }
    }

    virtual void display() {
        {
            static cv::VideoCapture cap(0);
            cvCaptureImg(m_img, cap);

            glShowImg(this, "input image", m_img);
        }

        {
            m_thread.run<SfMGUI, &SfMGUI::update>(this, false);
        }
        //{
        //    const double scale = 0.3 * m_wcam.dsize[0] / m_img.dsize[0];
        //    const Vec2 offset = getVec3(m_wcam.dsize[0], m_wcam.dsize[1]) * (scale * 0.5 - 0.5);
        //    glLoadView2D(m_wcam, offset, scale);

        //    glTexImg(m_img);
        //}
        {
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

                    glBegin(GL_LINES);
                    glCam(view->cam, 0.03 * m_pose.trn.z);
                    glEnd();
                }
            }

            // render axis
            {
                glLoadMatrix(invPose(m_axis) * m_pose.rot);

                glLineWidth(2.f);
                glBegin(GL_LINES);
                glAxis(0.05 * m_pose.trn.z);
                glEnd();
            }
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

};

int main() {

    SfMGUI win;
    win.execute("sfm", 800, 600);

    return 0;
}
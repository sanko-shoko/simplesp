#define SP_USE_DEBUG 1
#define SP_USE_THREAD 1

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

using namespace sp;

class VideoGUI : public BaseWindow {

private:

    Mem2<Col3> *m_img;

    virtual void init();
    virtual void keyFun(int key, int scancode, int action, int mods);

    virtual void display();
};

class SLAMGUI : public BaseWindow {
    friend VideoGUI;

private:

    SLAM m_slam;

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
    bool m_upflag;

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

        static VideoGUI video;
        video.create("video", 640, 480, this);
        addSubWindow(&video);

    }

    void reset() {
        m_cam = getCamParam(0, 0);

        m_slam.clear();

        const double distance = 20.0;
        m_pose = getPose(getVec(0.0, 0.0, +distance));
        m_axis = getPose(getVec(0.0, 0.0, -distance));

        m_err = 3.0f;

        m_upflag = false;

        m_ctool.clear();
    }

    void update() {
        if (m_upflag == false) return;

        m_slam.updatePose(m_img);
        
        m_thread.run<SLAMGUI, &SLAMGUI::updateMap>(this, false);

    }

    void addView() {
        Mem2<Col3> img = m_img;
        m_slam.addView(img);
    }

    void updateMap() {
        Mem2<Col3> img = m_img;
        m_slam.updateMap(img);
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_A] == 1) {
            CamParam cam;
            if (m_cam.dsize[0] * m_cam.dsize[1] > 0) {
                cam = m_cam;
            }
            else {
                cam = getCamParam(m_img.dsize);
            }
            m_slam.clear();
            m_slam.setCam(cam);
            m_slam.setBase(m_img, zeroPose());

            m_upflag = true;
        }

        if (m_keyAction[GLFW_KEY_S] == 1) {
            m_upflag = false;
            m_thread.run<SLAMGUI, &SLAMGUI::reset>(this);
        }

        // calibration
        {
            if (m_keyAction[GLFW_KEY_X] == 1) {
                const DotMarkerParam mrk(5, 5, 30.0);
                m_ctool.addImg(mrk, m_img);
            }

            if (m_keyAction[GLFW_KEY_C] == 1) {
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

            //glShowImg(this, "test", m_img);


            const Mem2<Col3> *wimg = SP_HOLDER_GET("warp image", Mem2<Col3>);
            if (wimg) {
                glShowImg(this, "warped image", *wimg);
            }
        }

        {
            update();
        }
        {
            // view 3D
            glLoadView3D(m_wcam, m_viewPos, m_viewScale);

            // render points
            {
                glPointSize(4.f);

                glLoadMatrix(m_pose);

                glBegin(GL_POINTS);
                for (int i = 0; i < m_slam.msize(); i++) {
                    const MapPnt *pnt = m_slam.getMPnt(i);
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

                for (int i = 0; i < m_slam.vsize(); i++) {
                    const View *view = m_slam.getView(i);

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

void VideoGUI::init() {
    SLAMGUI *p = static_cast<SLAMGUI*>(m_parent);
    m_img = &p->m_img;
}
void VideoGUI::keyFun(int key, int scancode, int action, int mods) {
    SLAMGUI *p = static_cast<SLAMGUI*>(m_parent);
    p->_keyFun(key, scancode, action, mods);
}
void VideoGUI::display(){
    SLAMGUI *p = static_cast<SLAMGUI*>(m_parent);
    SLAM *slam = &p->m_slam;
    if (slam->getMask() != NULL) {
        const Mem1<bool> &mask = *slam->getMask();
        const Mem1<Vec2> &bases = *slam->getBases();
        const Mem1<Vec2> &crsps = *slam->getCrsps();

        for (int i = 0; i < mask.size(); i++) {
            if (mask[i] == false) continue;

            const Vec2 pix0 = bases[i];
            const Vec2 pix1 = crsps[i];
            const Vec2 flow = pix1 - pix0;

            const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
            const double norm = normVec(flow) / 50.0;

            Col3 col;
            cnvHSVToCol(col, getVec(angle + SP_PI, minVal(1.0, norm), 1.0));

            renderLine(*m_img, pix0, pix1, col, 2);
        }
    }

    if (slam->getPose() != NULL) {
        const Pose base = getPose(getVec(0.0, 0.0, 20.0));

        renderAxis(*m_img, slam->getCam(), *slam->getPose() * base, 2.0, 2);
        renderGrid3d(*m_img, slam->getCam(), *slam->getPose() * base, 6.0, 2, getCol(100, 200, 200), 2);
    }

    glLoadView2D(m_img->dsize, m_viewPos, m_viewScale);
    glTexImg(*m_img);
}

int main() {

    SLAMGUI win;
    win.execute("slam", 800, 600);

    return 0;
}
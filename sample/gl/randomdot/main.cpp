#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class ActiveStereoGUI : public BaseWindow {

    // camera & projector param
    CamParam m_cam, m_prj;

    // rectified param
    RectParam m_rects[2];

    // cam to prj pose
    Pose m_cam2prj;

    // image
    Mem2<Col3> m_img;

    // patern
    Mem2<Byte> m_ptn;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object to cam pose
    Pose m_pose;

    // view pose
    Pose m_view;

    bool m_view3d;

    // pnts
    Mem1<Vec3> m_pnts;

private:

    void help() {
        printf("'a' key : capture simulation image\n");
        printf("'s' key : stereo matching\n");
        printf("'f' key : switch view mode (2d <-> 3d)\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        help();

        {
            m_cam = getCamParam(640, 480);

            m_img.resize(m_cam.dsize);
            m_img.zero();

            m_prj = getCamParam(320, 240);

            m_cam2prj = getPose(getRotAngleY(+10.0 / 180.0 * SP_PI), getVec3(-100.0, 0.0, 0.0));

            m_ptn = genRandomPattern(m_prj.dsize, 0.15);
        }

        {
            m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
            if (m_model.size() == 0) {
                // if could not find stanford bunny, load dummy model
                m_model = loadGeodesicDorm(100.0, 1);
            }

            m_pose = getPose(getVec3(0.0, 0.0, getModelDistance(m_model, m_cam)));

            m_view3d = false;
            m_view = zeroPose();
        }
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_key[GLFW_KEY_A] == 1) {
            cnvImg(m_img, capture());
        }
        if (m_key[GLFW_KEY_S] == 1) {
            stereoMatching();
        }
        if (m_key[GLFW_KEY_F] == 1) {
            m_view3d ^= true;
        }
    }

    Mem2<Byte> capture() {
        Mem2<Byte> cap;
        renderPattern(cap, m_cam, m_pose, m_model, m_cam2prj, m_prj, m_ptn);
        return cap;
    }

    void stereoMatching() {

        CamParam cams[2];
        Mem2<Byte> imgs[2];
        {
            // imgL
            {
                imgs[0] = capture();
                cams[0] = m_cam;
            }

            // imgR (rescale to camera image)
            {
                const double scale0 = (static_cast<double>(m_cam.dsize[0]) / m_prj.dsize[0]);
                const double scale1 = (static_cast<double>(m_cam.dsize[1]) / m_prj.dsize[1]);

                rescale(cams[1], m_prj, scale0, scale1);
                rescale(imgs[1], m_ptn, scale0, scale1);
            }
        }


        // rectification
        Mem2<Vec2> tables[2];
        {
            rectify(m_rects[1], m_rects[0], cams[1], m_cam2prj, cams[0], zeroPose(), (cams[0].fx + cams[0].fy) / 2.0);

            for (int i = 0; i < 2; i++) {
                makeRemapTable(tables[i], m_rects[i]);
            }
        }

        Mem2<Byte> rimgs[2];
        {
            remap(rimgs[0], imgs[0], tables[0]);
            remap(rimgs[1], imgs[1], tables[1]);

            saveBMP("rimg0.bmp", rimgs[0]);
            saveBMP("rimg1.bmp", rimgs[1]);
        }

        {
            const int maxDisp = 80;
            const int minDisp = 0;

            StereoBase estimator;
            estimator.setRange(maxDisp, minDisp);
            estimator.setCam(m_rects[0].cam, m_rects[1].cam, normVec(m_cam2prj.trn));
            estimator.setWinSize(15);

            // matching
            {
                estimator.execute(rimgs[0], rimgs[1]);
            }

            Mem2<Byte> mask(rimgs[0].dsize);
            // mask
            {
                mask.zero();

                Mem2<Byte> tmp;
                maxFilter(tmp, rimgs[0], 15);

                for (int i = 0; i < mask.size(); i++) {
                    if (tmp[i] > 100) {
                        mask[i] = SP_BYTEMAX;
                    }
                }
            }

            // output
            {
                cnvDispToImg(m_img, estimator.getDispMap(StereoBase::StereoL), maxDisp, minDisp);

                for (int i = 0; i < mask.size(); i++) {
                    if (mask[i] == 0) {
                        m_img[i] = getCol3(0, 0, 0);
                    }
                }
                saveBMP("disp.bmp", m_img);
                
                const Mem2<Vec3> depthMap = estimator.getDepthMap(StereoBase::StereoL);

                m_pnts.clear();
                for (int i = 0; i < depthMap.size(); i++) {
                    if (depthMap[i].z > 0.0 && mask[i] > 0) {
                        m_pnts.push(depthMap[i]);
                    }
                }
            }
        }
    }

    virtual void display() {
        if (m_view3d == false) {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glTexImg(m_img);

            // view 3D
            glLoadView3D(m_cam, m_viewPos, m_viewScale);
            
            glLoadMatrix(m_pose);
            glRenderOutline(m_model);
        }
        else {
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glLoadView3D(m_rects[0].cam, m_viewPos, m_viewScale);

            // render points
            glLoadMatrix(m_view);

            glPointSize(1.f);
            glColor3d(0.5, 0.5, 1.0);
            glBegin(GL_POINTS);
            for (int i = 0; i < m_pnts.size(); i++) {
                glVertex(m_pnts[i]);
            }
            glEnd();
        }
    }

    virtual void mousePos(double x, double y) {

        if (m_view3d == false) {
            controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
        }
        else {
            controlPose(m_view, m_mouse, m_wcam, m_viewScale, invPose(m_pose));
        }
    }

    virtual void mouseScroll(double x, double y) {

        if (m_view3d == false) {
            controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
        }
        else {
            controlPose(m_view, m_mouse, m_wcam, m_viewScale, invPose(m_pose));
        }
    }

};


int main(){

    ActiveStereoGUI win;
    win.execute("randomdot", 800, 600);

    return 0;
}
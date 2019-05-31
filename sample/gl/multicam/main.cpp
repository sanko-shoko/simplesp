#define SP_USE_DEBUG 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class VoxelGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // view pose
    Pose m_view;

    // multi camerea poses
    Mem1<Pose> m_poses;

    Mem1<Pose> m_ests;

    Mem1<Vec3> m_pnts;

    Mem1<Pose> m_boards;

    int m_id;

private:

    void help() {
        printf("'a' key : switch board\n");
        printf("'s' key : calibration\n");
        printf("\n");
    }

    virtual void init() {
        help();

        m_id = 0;

        m_cam = getCamParam(640, 480);


        const double distance = 1000.0;
        m_view = getPose(getVec3(0.0, 0.0, 3 * distance));

        {
            const int cnum = 10;
            for (int i = 0; i < cnum; i++) {
                const Pose pose = getRotAngleX(+20.0 * SP_PI / 180.0) * getPose(getRotAngleY(i * 2 * SP_PI / cnum), getVec3(0.0, +0.4 * distance, distance));

                m_poses.push(pose);
            }
        }

        {
            const int onum = 50;
            const double size = 200.0;
            for (int i = 0; i < onum; i++) {
                const Vec3 pos = randuVec3(size, 0.0, size) + getVec3(0.0, -1.0, 0.0);

                const Pose pose = invPose(getPose(invRot(getRotDirection(pos)), pos));
                //const Pose pose = getPose(pos);
                m_boards.push(pose);

                if (i == 0) {
                    print(m_poses[0] * invPose(pose));
                }
            }
        }

        {
            // set marker info
            const int dsize[2] = { 4, 3 };
            const int block = 3;
            const double length = 30.0;
            const double interval = 5.0;

            const Mem1<BitMarkerParam> mrks = getBitMarkerParam(0, block, length, dsize[0], dsize[1], interval);

            Mem1<Vec3> unit = extVec(grid(2, 2), 0) * length;
            unit -= meanVec(unit);

            m_pnts.clear();
            for (int i = 0; i < mrks.size(); i++) {
                m_pnts.push(invPose(mrks[i].offset) * unit);
            }
        }
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_key[GLFW_KEY_A] == 1) {
            m_id++;
            if (m_id >= m_boards.size()) m_id = 0;
        }

        if (m_key[GLFW_KEY_S] == 1) {
            Mem1<Mem1<Mem1<Vec2> > > pixs;
            Mem1<Mem1<Mem1<Vec2> > > objs;
            Mem1<CamParam> cams;
            
            genDataset(cams, pixs, objs, m_boards, m_pnts);


            Mem1<Pose> poses;
            calibMultiCam(poses, cams, pixs, objs);

            Pose base = m_poses[0];
            m_ests.clear();
            for (int i = 0; i < cams.size(); i++) {

                m_ests.push(poses[i] * base);
            }
        }

    }

    void genDataset(Mem1<CamParam> &cams, Mem1<Mem1<Mem1<Vec2> > > &pixs, Mem1<Mem1<Mem1<Vec2> > > &objs, const Mem1<Pose> &boards, const Mem1<Vec3> &pnts) {

        for (int i = 0; i < m_poses.size(); i++) {

            CamParam cam = getCamParam(640, 480);

            Mem1<Mem1<Vec2> > spixs;
            Mem1<Mem1<Vec2> > sobjs;
            for (int j = 0; j < boards.size(); j++) {
                Mem1<Vec2> tpixs;
                Mem1<Vec2> tobjs;
                const Pose pose = m_poses[i] * invPose(boards[j]);

                const Vec3 nrm = pose.rot * getVec3(0.0, 0.0, 1.0);
                if (nrm.z > 0.3) {
                    for (int k = 0; k < pnts.size(); k++) {
                        const double noise = 0.5;
                        const Vec2 pix = mulCamD(cam, prjVec(pose * pnts[k])) + randgVec2(noise, noise);
                     
                        tpixs.push(pix);
                        tobjs.push(getVec2(pnts[k].x, pnts[k].y));
                        ///print(noise);
                    }
                }
                spixs.push(tpixs);
                sobjs.push(tobjs);

                Mem2<Byte> img(640, 480);
                img.zero();

                //printf("pnts %d\n", tpixs.size());
                //for (int k = 0; k < tpixs.size(); k++) {
                //    renderPoint(img, tpixs[k], BYTE(255), 1);
                //}

            }

            pixs.push(spixs);
            objs.push(sobjs);
            cams.push(cam);
        }
    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render model
        {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);
            glDisable(GL_DEPTH_TEST);

            {
                glPointSize(3.f);

                glLoadMatrix(m_view * invPose(m_boards[m_id]));

                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.7f, 0.2f);

                for (int i = 0; i < m_pnts.size(); i++) {
                    glVertex(m_pnts[i]);
                }
                glEnd();

                glAxis(50.0);
            }

            // render cam
            {

                glLineWidth(2.f);

                for (int i = 0; i < m_poses.size(); i++) {
                    glLoadMatrix(m_view * invPose(m_poses[i]));

                    const Vec3 nrm = (m_poses[i] * invPose(m_boards[m_id])).rot * getVec3(0.0, 0.0, 1.0);
                    if (nrm.z > 0.3) {
                        glColor3d(0.5, 0.5, 0.8);
                    }
                    else {
                        glColor3d(0.2, 0.2, 0.4);
                    }

                    glCam(m_cam, 50);
                }

                for (int i = 0; i < m_ests.size(); i++) {
                    glLoadMatrix(m_view * invPose(m_ests[i]));

                    glColor3d(0.5, 0.8, 0.5);
                    glCam(m_cam, 50);
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
    win.execute("multicam", 800, 600);

    return 0;
}
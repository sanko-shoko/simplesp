#include "simplesp.h"
#include "spex/spcv.h"
#include "spex/spgl.h"

using namespace sp;

#define MARKER_DSIZE0 5
#define MARKER_DSIZE1 5
#define MARKER_DISTANCE 30.0

class DotMarkerGUI : public BaseWindow{

    // model
    Mem1<Mesh3> m_model;

    // marker detector
    DotMarker m_dotMarker;

    // diminish flag
    bool m_diminish;

private:

    void help() {
        printf("'d' key : diminish marker\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init(){

        help();

        if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        m_dotMarker.setMrk(DotMarkerParam(MARKER_DSIZE0, MARKER_DSIZE1, MARKER_DISTANCE));

        m_diminish = false;
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_keyAction[GLFW_KEY_D] == 1) {
            m_diminish ^= true;
        }

        // calibration
        {
            static Mem1<Mem1<Vec2> > pixsList, objsList;

            if (m_keyAction[GLFW_KEY_A] == 1 && m_dotMarker.getPose() != NULL) {
                printf("add detected points (i = %d)\n", pixsList.size());
                pixsList.push(*m_dotMarker.getCrspPixs());
                objsList.push(*m_dotMarker.getCrspObjs());

            }

            if (m_keyAction[GLFW_KEY_C] == 1) {
                CamParam cam;
                print(m_dotMarker.getCam());
                const double rms = calibCam(cam, m_dotMarker.getCam().dsize[0], m_dotMarker.getCam().dsize[1], pixsList, objsList);
                if (rms >= 0.0) {
                    printf("rms %lf\n", rms);
                    saveText("cam.txt", cam);
                    m_dotMarker.setCam(cam);
                }
                print(m_dotMarker.getCam());
            }
        }
    }

    virtual void display(){
        static Mem2<Col3> img;

        // capture image
        {
            static cv::VideoCapture cap;
            if (cvCaptureImg(img, cap) == false){

                // if no camera
                if (img.size() == 0) {
                    loadBMP(SP_DATA_DIR "/marker/dotmarker.bmp", img);
                }
            }
        }

        // detect dot marker
        m_dotMarker.execute(img);

        if (m_diminish == true) {
            m_dotMarker.diminish(img);
        }

        {
            glLoadView2D(getCamParam(img.dsize), m_viewPos, m_viewScale);
            glRenderImg(img);
        }

        if (m_dotMarker.getPose() != NULL) {
            
            // cam           /
            //       model  / board (5 deg)
            //             /  

            const Pose boardToCamPose = *m_dotMarker.getPose();
            const Pose worldToBoardPose = getPose(getRotAngleX(5 * SP_PI / 180.0));
            const Pose worldToModelPose = getPose(getRotAngleX(180 * SP_PI / 180.0), getVec(0.0, 30.0, 0.0));

            const Pose modelToCamPose = boardToCamPose * worldToBoardPose * invPose(worldToModelPose);
            
            glLoadView3D(m_dotMarker.getCam(), m_viewPos, m_viewScale);

            // light
            {
                glLoadMatrix(zeroPose());

                const GLfloat lightPos[4] = { 0.f, 0.f, -1000.f, 0.f };
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
            }

            // model
            {
                glLoadMatrix(modelToCamPose);

                const GLfloat diffuse[] = { 0.4f, 0.5f, 0.5f, 1.0f };
                glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
                for (int i = 0; i < m_model.size(); i++) {
                    glBegin(GL_TRIANGLES);
                    glNormal(getMeshNrm(m_model[i]));
                    glMesh(m_model[i]);
                    glEnd();
                }
            }
        }

    }


};

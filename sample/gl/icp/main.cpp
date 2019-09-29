#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

static bool ShowText(const char *text, const ImVec2 &pos, const ImVec4 &col = ImVec4(1.f, 1.f, 1.f, 1.f), const float scale = 1.f) {

    char name[32] = { 0 };
    const int maxv = 100;
    for (int i = 0; i < maxv; i++) {
        sprintf(name, "##showtext%04d", i);
        const ImGuiWindow* window = ImGui::FindWindowByName(name);
        if (window == NULL || window->Active == false) {
            break;
        }
    }

    ImGui::PushStyleColor(ImGuiCol_Text, col);
    {
        ImGui::Begin(name, NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground);
        const float backup = ImGui::GetFontSize();
        ImGui::SetWindowFontScale(scale);
        ImGui::SetWindowPos(pos, ImGuiCond_Always);

        ImGui::Text(text);

        ImGui::SetWindowFontScale(backup);

        ImGui::End();
    }
    ImGui::PopStyleColor(1);
    return true;
}

class ICPGUI : public BaseWindowIMGUI {

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // model
    Mem1<Mesh3> m_model;

    // data A
    Mem<VecPD3> m_dataA;
    Pose m_poseA;

    // data B (target)
    Mem<VecPD3> m_dataB;
    Pose m_poseB;

    int m_it;

private:

    void help() {
        printf("[points] : controlled by mouse\n");
        printf("'a' key : render target (points)\n");
        printf("'s' key : render target (depth map)\n");
        printf("'d' key : calc ICP\n");
        printf("\n");
    }

    virtual void init() {
        help();

        m_cam = getCamParam(640, 480);

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        SP_ASSERT(m_model.size() > 0);

        m_poseA = getPose(getVec3(0.0, 0.0, getModelDistance(m_model, m_cam)));
        m_dataA = getModelPoint(m_model);

        reset();
    }

    void reset() {
        m_it = 0;
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        if (m_key[GLFW_KEY_A] == 1) {
            m_dataB = Mem1<VecPD3>(m_dataA.size());

            for (int i = 0; i < m_dataB.size(); i++) {
                m_dataB[i] = m_poseA * m_dataA[i];
            }
            m_poseB = m_poseA;
            reset();
        }
        
        if (m_key[GLFW_KEY_S] == 1) {
            m_dataB = Mem2<VecPD3>(m_cam.dsize);

            m_dataB.zero();
            renderVecPD(m_dataB, m_cam, m_poseA, m_model);

            const double distance = getModelDistance(m_model, m_cam);
            const double radius = getModelRadius(m_model);
            cnvDepthToImg(m_img, m_dataB, distance - 2 * radius, distance + 2 * radius);

            m_poseB = m_poseA;
            reset();
        }
 
        if (m_key[GLFW_KEY_D] > 0 && m_dataB.size() > 0) {

            // point to point
            if (m_dataB.dim == 1) {
                calcICP(m_poseA, m_dataB, m_dataA, 1);
            }

            // point to 2d map
            if (m_dataB.dim == 2) {
                calcICP(m_poseA, m_cam, m_dataB, m_dataA, 1);
            }
            m_it++;
        }
    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render dataB
        {
            if (m_dataB.dim == 1) {
                glLoadView3D(m_cam, m_viewPos, m_viewScale);
                glPointSize(5.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.2f, 0.7f);
                for (int i = 0; i < m_dataB.size(); i++) {
                    glVertex(m_dataB[i].pos);
                }
                glEnd();
            }

            if (m_dataB.dim == 2) {
                glLoadView2D(m_cam, m_viewPos, m_viewScale);
                glTexImg(m_img);
            }
        }

        // render dataA
        {
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            glClear(GL_DEPTH_BUFFER_BIT);
            {
                glLoadMatrix(m_poseA);

                // render points
                glPointSize(3.f);
                glBegin(GL_POINTS);
                glColor3f(0.2f, 0.7f, 0.2f);
                for (int i = 0; i < m_dataA.size(); i++) {
                    glVertex(m_dataA[i].pos);
                }
                glEnd();
            }

            {
                glLoadMatrix(m_poseA);

                glLineWidth(2.f);
                glBegin(GL_LINES);
                glAxis(50.0);
                glEnd();
            }
        }

        {
            const Mat vmat = glGetViewMat(m_cam.dsize, m_viewPos, m_viewScale);

            if (m_dataA.size() > 0) {
                const Vec2 pix = vmat * mulCam(m_cam, prjVec(m_poseA.trn));
                const string str = string("data A (points)");
                ShowText(str.c_str(), ImVec2(float(pix.x + 100.0), float(pix.y - 120.0)), ImVec4(1.f, 1.f, 0.f, 1.f), 1.4f);
            }
            if(m_dataB.size() > 0){
                const Vec2 pix = vmat * mulCam(m_cam, prjVec(m_poseB.trn));
                const string str = string("data B ") + ((m_dataB.dim == 1) ? "(points)" : "(depth map)");

                ShowText(str.c_str(), ImVec2(float(pix.x - 220.0), float(pix.y + 120.0)), ImVec4(0.f, 1.f, 1.f, 1.f), 1.4f);
            }
            if (m_it > 0) {
                const string str = "icp iteration :" + to_string(m_it);
                ShowText(str.c_str(), ImVec2(90.f, 70.f), ImVec4(1.f, 1.f, 1.f, 1.f), 1.4f);
            }

        }
 
    }

    virtual void mousePos(double x, double y) {
        if (controlPose(m_poseA, m_mouse, m_wcam, m_viewScale) == true) {
            reset();
        }
    }

    virtual void mouseScroll(double x, double y) {
        if (controlPose(m_poseA, m_mouse, m_wcam, m_viewScale) == true) {
            reset();
        }
    }

};

int main(){

    ICPGUI win;
    win.execute("icp", 800, 600);

    return 0;
}
#ifndef __GTMAKER__
#define __GTMAKER__

#include "gtutil.h"

using namespace sp;

class GTMakerGUI : public BaseWindow {

private:
    DataBase m_database;

    // selected image
    Mem2<Col3> m_img;

    // selected id
    int m_selectid;

    // focused gt
    GT *m_focus;
    
    // editor mode
    enum Mode {
        M_Rect = 0, // Rectangle
        M_Ordr = 1, // Order
        M_Cont = 2  // Contour
    };
    Mode m_mode;

    // ui state
    enum State {
        S_Base = 0,
        S_Init = 1,
        S_Edit = 2
    };
    State m_state;

public:

    GTMakerGUI() {
        ImGui::GetIO().IniFilename = NULL;

        reset();

        m_database.gtNames.push("dog");
        m_database.gtNames.push("cat");

    }

    void reset() {

        m_focus = NULL;
        m_state = S_Base;
        m_mode = M_Rect;
    }

private:

    void select(const int id) {
        m_selectid = maxVal(0, minVal(m_database.imNames.size() - 1, id));

        const string path = m_database.imDir + "\\" + m_database.imNames[m_selectid];
        SP_ASSERT(cvLoadImg(m_img, path.c_str()));

        reset();

        adjustImg();
    }

    void setMode(Mode mode) {
        m_mode = mode;
        init();
    }

    void adjustImg() {
        if (m_img.size() == 0) return;

        m_viewPos = getVec(100.0, 10.0);
        m_viewScale = 0.92 * minVal(static_cast<double>(m_wcam.dsize[0] - 180) / m_img.dsize[0], static_cast<double>(m_wcam.dsize[1]) / m_img.dsize[1]);
    }


private:

    virtual void init() {
        initRect();
        initCont();
    }

    //--------------------------------------------------------------------------------
    // ui
    //--------------------------------------------------------------------------------
  
    virtual void display() {

        if (ImGui::BeginMainMenuBar()) {

            if (ImGui::BeginMenu("file")) {

                if (ImGui::MenuItem("open image dir") && m_database.open_imDir()) {
                    select(0);
                }
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        {
            glLoadView2D(m_img.dsize, m_viewPos, m_viewScale);
            glRenderImg(m_img);
        }

        if (m_database.isValid() == true) {
            
            dispData();

            //const ImVec4 col(0.8f, 0.8f, 0.8f, 0.8f);
            //ImGui::PushStyleColor(ImGuiCol_WindowBg, col);

            switch (m_mode) {
            case M_Rect: menuRect(); break;
            case M_Cont: menuCont(); break;
            }
            //ImGui::PopStyleColor();

            switch (m_mode) {
            case M_Rect: dispRect(); break;
            case M_Cont: dispCont(); break;
            }
        }
    }

    void dispData();


    //--------------------------------------------------------------------------------
    // call back
    //--------------------------------------------------------------------------------

    virtual void windowSize(int width, int height) {
        if (m_database.isValid() == false) return;

        adjustImg();
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
        if (m_database.isValid() == false) return;

        if (m_keyAction[GLFW_KEY_A] > 0) {
            select(m_selectid - 1);
        }
        if (m_keyAction[GLFW_KEY_S] > 0) {
            select(m_selectid + 1);
        }
    }

    virtual void mouseButton(int button, int action, int mods) {
        if (m_database.isValid() == false) return;

        switch (m_mode) {
        case M_Rect: mouseButtonRect(button, action, mods); break;
        case M_Cont: mouseButtonCont(button, action, mods); break;
        }
    }

    virtual void mousePos(double x, double y) {
        if (m_database.isValid() == false) return;

        switch (m_mode) {
        case M_Rect: mousePosRect(x, y); break;
        case M_Cont: mousePosCont(x, y); break;
        }
    }

    virtual void mouseScroll(double x, double y) {
    }
    

    // rectangle
    void initRect();
    void menuRect();
    void dispRect();
    void mouseButtonRect(int button, int action, int mods);
    void mousePosRect(double x, double y);

    // order
    void initOrdr();
    void menuOrdr();
    void dispOrdr();
    void mouseButtonOrdr(int button, int action, int mods);
    void mousePosOrdr(double x, double y);

    // contour
    void initCont();
    void menuCont();
    void dispCont();
    void mouseButtonCont(int button, int action, int mods);
    void mousePosCont(double x, double y);


    //--------------------------------------------------------------------------------
    // others
    //--------------------------------------------------------------------------------

    int findNearPos(const Mem1<Vec2> &pnst, const Vec2 &pix);
    int findNearLine(const Mem1<Vec2> &pnst, const Vec2 &pix);

 };

#endif
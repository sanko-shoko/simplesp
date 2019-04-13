//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GLWIN_H__
#define __SP_GLWIN_H__

#if SP_USE_GLEW
#define GLEW_STATIC
#include "GL/glew.h"
#endif

#if SP_USE_IMGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "imgui_internal.h"
#endif

#include "GLFW/glfw3.h"

#include "spex/spgltex.h"

#include "spcore/spcore.h"

#include <string>
#include <chrono>
#include <map>

namespace sp {

    //--------------------------------------------------------------------------------
    // mouse
    //--------------------------------------------------------------------------------

#define GLFW_DOUBLECLICK 3

    class Mouse {

    public:

        // cursor position and move
        Vec2 pos, move;

        // drag
        Vec2 drag;

        // scroll value
        double scroll;

        // button
        int bDownL, bDownR, bDownM;

 
        Mouse() {
            reset();
        }

        void reset() {
            memset(this, 0, sizeof(Mouse));
        }

        void setButton(const int button, const int action, const int mods) {
            int _action = action;

            if (_action == GLFW_PRESS) {
                using namespace std::chrono;

                static system_clock::time_point prev = system_clock::now();
                const system_clock::time_point crnt = system_clock::now();
                const double diff = duration <double, std::milli>(crnt - prev).count();
                if (diff > 10.0 && diff < 400.0) {
                    _action = GLFW_DOUBLECLICK;
                }
                prev = crnt;
            }

            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                bDownL = _action;
            }
            if (button == GLFW_MOUSE_BUTTON_RIGHT) {
                bDownR = _action;
            }
            if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
                bDownM = _action;
            }

            if (_action == 0) {
                drag = getVec2(0.0, 0.0);
            }
        }

        void setPos(const double x, const double y) {

            if (bDownL || bDownR || bDownM) {
                move = getVec2(x, y) - pos;
                drag += move;
            }
            pos = getVec2(x, y);
        }

        void setScroll(const double x, const double y) {
            scroll = y;
        }

    };


    //--------------------------------------------------------------------------------
    // control
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool controlView(Vec2 &viewpPos, double &viewScale, const Mouse &mouse) {
        bool ret = false;
        if (mouse.bDownL && normVec(mouse.move) > 0.0) {
            viewpPos += mouse.move;
            ret = true;
        }

        if (mouse.scroll != 0) {
            viewpPos *= (1.0 + mouse.scroll * 0.1);
            viewScale *= (1.0 + mouse.scroll * 0.1);
            ret = true;
        }
        return ret;
    }

    SP_CPUFUNC bool controlPose(Pose &pose, const Mouse &mouse, const CamParam &cam, const double viewScale, const Pose base = zeroPose()) {
        bool ret = false;

        Pose cpose = pose * invPose(base);
        if (cpose.trn.z < 0.0) return false;

        if (mouse.bDownM && normVec(mouse.move) > 0.0) {
            cpose.trn.x += SP_CAST(mouse.move.x * cpose.trn.z / (cam.fx * viewScale));
            cpose.trn.y += SP_CAST(mouse.move.y * cpose.trn.z / (cam.fy * viewScale));
            ret = true;
        }

        if (mouse.bDownL && normVec(mouse.move) > 0.0) {
            cpose.rot = getRotAngle(getVec3(+mouse.move.y, -mouse.move.x, 0.0), 0.01 * normVec(mouse.move)) * cpose.rot;
            ret = true;
        }

        if (mouse.scroll != 0) {
            cpose.trn -= unitVec(cpose.trn) * (cpose.trn.z * mouse.scroll * 0.02);
            ret = true;
        }

        pose = cpose * base;
        return ret;
    }


    //--------------------------------------------------------------------------------
    // base window
    //--------------------------------------------------------------------------------
  
    class BaseWindow {

    protected:

        virtual void windowSize(int width, int height) {
        }

        virtual void mouseButton(int button, int action, int mods) {
        }

        virtual void mousePos(double x, double y) {
        }

        virtual void mouseScroll(double x, double y) {
        }

        virtual void keyFun(int key, int scancode, int action, int mods) {
        }

        virtual void charFun(unsigned int charInfo) {
        }

        virtual void drop(int num, const char **paths) {
        }

        virtual void focus(int focused) {
        }

        virtual void init() {
        }

        virtual void display() {
        }

 
    protected:

        // window ptr
        GLFWwindow *m_win;

        // parent window
        BaseWindow *m_parent;

        // child windows
        Mem1<BaseWindow*> m_cwins;

    public:

        // keybord action
        char m_keyAction[400];

        // view position
        Vec2 m_viewPos;

        // view scale
        double m_viewScale;

        // mouse event class
        Mouse m_mouse;

        // window cam
        CamParam m_wcam;

        // back gournd color
        const Col4 *m_bgcol;

        // control view flag
        bool m_space;

        // escape flag
        bool m_escape;

    public:

        BaseWindow() {
            m_win = NULL;
            m_parent = NULL;

            m_viewPos = getVec2(0.0, 0.0);
            m_viewScale = 1.0;

            const static Col4 col = getCol(24, 32, 32, 255);
            m_bgcol = &col;

            memset(m_keyAction, 0, sizeof(m_keyAction));
            
            m_space = true;
            m_escape = true;
        }


        //--------------------------------------------------------------------------------
        // main window
        //--------------------------------------------------------------------------------

        bool create(const char *name, const int width, const int height, BaseWindow *parent = NULL) {

            if (m_win == NULL) {
                // glfw create window
                m_win = glfwCreateWindow(width, height, name, NULL, NULL);
            }
            if (m_win == NULL) {
                SP_PRINTF(" Can't create GLFW window.\n");
                glfwTerminate();
                return false;
            }

            m_wcam = getCamParam(width, height);
            m_parent = parent;

            // glfw make context
            glfwMakeContextCurrent(m_win);
            glfwSwapInterval(1); // vsync

#if SP_USE_GLEW
            // glew init
            SP_ASSERT(glewInit() == GLEW_OK);
#endif

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                // imgui init
                IMGUI_CHECKVERSION();
                ImGui::CreateContext();
                SP_ASSERT(ImGui_ImplGlfw_InitForOpenGL(m_win, true) == true);
                SP_ASSERT(ImGui_ImplOpenGL2_Init() == true);
                ImGui::GetIO().IniFilename = NULL;
            }
#endif
 
            init();

            if (parent != NULL) {
                parent->addSubWindow(this);
            }

            return true;
        }

        void execute(const char *name, const int width, const int height) {

            // glfw init
            SP_ASSERT(glfwInit());

            glfwWindowHint(GLFW_SAMPLES, 8);
            glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

            SP_ASSERT(create(name, width, height) == true);

            while (!glfwWindowShouldClose(m_win) && (m_escape == false || !glfwGetKey(m_win, GLFW_KEY_ESCAPE))) {

                if (main() == false) break;

                if (findFocused() == NULL) glfwPollEvents();
            }

            // glfw terminate
            glfwTerminate();

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                ImGui_ImplOpenGL2_Shutdown();
                ImGui_ImplGlfw_Shutdown();
                ImGui::DestroyContext();
            }
#endif
        }


        //--------------------------------------------------------------------------------
        // sub window
        //--------------------------------------------------------------------------------

        void addSubWindow(BaseWindow *bw) {
            for (int i = 0; i < m_cwins.size(); i++) {
                if (m_cwins[i] == bw) {
                    return;
                }
            }
            m_cwins.push(bw);
        }

        void delSubWindow(BaseWindow *bw) {
            for (int i = 0; i < m_cwins.size(); i++) {
                if (m_cwins[i] == bw) {
                    m_cwins.del(i);
                    break;
                }
            }
        }

 
    protected:

        //--------------------------------------------------------------------------------
        // main loop
        //--------------------------------------------------------------------------------

        bool main() {

            if (glfwWindowShouldClose(m_win) || (m_escape == true && glfwGetKey(m_win, GLFW_KEY_ESCAPE))) {
                glfwDestroyWindow(m_win);
                m_win = NULL;
                return false;
            }

            // glfw set context
            glfwMakeContextCurrent(m_win);

            // glfw set event callbacks
            setCallback(m_win);

            // check events
            if (glfwGetWindowAttrib(m_win, GLFW_FOCUSED)) {
                glfwPollEvents();
            }

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                ImGui_ImplOpenGL2_NewFrame();
                ImGui_ImplGlfw_NewFrame();
                ImGui::NewFrame();
            }
#endif
            if (m_bgcol != NULL) {
                glClearColor(m_bgcol->r / 255.f, m_bgcol->g / 255.f, m_bgcol->b / 255.f, m_bgcol->a / 255.f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            }
            
            display();

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                ImGui::Render();
                ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
            }
#endif

            glfwSwapBuffers(m_win);

            for (int i = 0; i < m_cwins.size(); i++) {
                if (m_cwins[i]->main() == false) {
                    m_cwins.del(i);
                }
            }

            return true;
        }


        //--------------------------------------------------------------------------------
        // focus
        //--------------------------------------------------------------------------------
        
        bool isFocused() {
            return glfwGetWindowAttrib(m_win, GLFW_FOCUSED);
        }

        GLFWwindow* findFocused() {
            if (glfwGetWindowAttrib(m_win, GLFW_FOCUSED)) {
                return m_win;
            }
            for (int i = 0; i < m_cwins.size(); i++) {
                GLFWwindow *w = m_cwins[i]->findFocused();
                if (w != NULL) return w;
            }
            return NULL;
        }

    public:

        //--------------------------------------------------------------------------------
        // event process
        //--------------------------------------------------------------------------------

        void _windowSize(int width, int height) {
            m_wcam = getCamParam(width, height);

            ::glViewport(0, 0, width, height);

            windowSize(width, height);
        }

        void _mouseButton(int button, int action, int mods) {

            m_mouse.setButton(button, action, mods);

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                if ((m_space == false || m_keyAction[GLFW_KEY_SPACE] == 0) && ImGui::GetIO().WantCaptureMouse) {
                    ImGui_ImplGlfw_MouseButtonCallback(NULL, button, action, mods);
                    return;
                }
            }
#endif

            mouseButton(button, action, mods);
        }

        void _mousePos(double x, double y) {

            m_mouse.setPos(x, y);

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                if ((m_space == false || m_keyAction[GLFW_KEY_SPACE] == 0) && ImGui::GetIO().WantCaptureMouse) {
                    return;
                }
            }
#endif

            // control view
            if (m_space == true && m_keyAction[GLFW_KEY_SPACE] > 0) {
                controlView(m_viewPos, m_viewScale, m_mouse);
                return;
            }

            mousePos(x, y);
        }

        void _mouseScroll(double x, double y) {

            m_mouse.setScroll(x, y);

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                if ((m_space == false || m_keyAction[GLFW_KEY_SPACE] == 0) && ImGui::GetIO().WantCaptureMouse) {
                    ImGui_ImplGlfw_ScrollCallback(NULL, x, y);
                    m_mouse.setScroll(0.0, 0.0);
                    return;
                }
            }
#endif

            // control view
            if (m_space == true && m_keyAction[GLFW_KEY_SPACE] > 0) {
                controlView(m_viewPos, m_viewScale, m_mouse);
                m_mouse.setScroll(0.0, 0.0);
                return;
            }

            mouseScroll(x, y);
            m_mouse.setScroll(0.0, 0.0);
        }

        void _keyFun(int key, int scancode, int action, int mods) {
            if (key < 0) return;

            m_keyAction[key] = static_cast<char>(action);

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                static bool prev = false;
                if (ImGui::GetIO().WantCaptureKeyboard == true || (action == 0 && prev == true)) {
                    prev = ImGui::GetIO().WantCaptureKeyboard;
                    ImGui_ImplGlfw_KeyCallback(NULL, key, scancode, action, mods);
                    return;
                }
                prev = false;
            }
#endif

            keyFun(key, scancode, action, mods);
        }

        void _charFun(unsigned int charInfo) {

#if SP_USE_IMGUI
            if (m_parent == NULL) {
                if (ImGui::GetIO().WantCaptureKeyboard == true) {
                    ImGui_ImplGlfw_CharCallback(NULL, charInfo);
                    return;
                }
            }
#endif

            charFun(charInfo);
        }

        void _drop(int num, const char **paths) {
            drop(num, paths);
        }

        void _focus(int focused) {
            focus(focused);
        }

    protected:

        //--------------------------------------------------------------------------------
        // callback function
        //--------------------------------------------------------------------------------

        void setCallback(GLFWwindow *window) {
            // set my ptr
            glfwSetWindowUserPointer(window, this);

            glfwSetWindowSizeCallback(window, windowSizeCB);

            glfwSetMouseButtonCallback(window, mouseButtonCB);
            glfwSetCursorPosCallback(window, mousePosCB);
            glfwSetScrollCallback(window, mouseScrollCB);

            glfwSetKeyCallback(window, keyFunCB);
            glfwSetCharCallback(window, charFunCB);

            glfwSetDropCallback(window, dropCB);
            glfwSetWindowFocusCallback(window, focusCB);
        }

        static BaseWindow* getThisPtr(GLFWwindow *window) {
            return static_cast<BaseWindow*>(glfwGetWindowUserPointer(window));
        }

        static void windowSizeCB(GLFWwindow *window, int width, int height) {
            getThisPtr(window)->_windowSize(width, height);
        }
        static void mouseButtonCB(GLFWwindow *window, int button, int action, int mods) {
            getThisPtr(window)->_mouseButton(button, action, mods);
        }
        static void mousePosCB(GLFWwindow *window, double x, double y) {
            getThisPtr(window)->_mousePos(x, y);
        }
        static void mouseScrollCB(GLFWwindow *window, double x, double y) {
            getThisPtr(window)->_mouseScroll(x, y);
        }
        static void keyFunCB(GLFWwindow* window, int key, int scancode, int action, int mods) {
            getThisPtr(window)->_keyFun(key, scancode, action, mods);
        }
        static void charFunCB(GLFWwindow* window, unsigned int charInfo) {
            getThisPtr(window)->_charFun(charInfo);
        }
        static void dropCB(GLFWwindow *window, int num, const char **paths) {
            getThisPtr(window)->_drop(num, paths);
        }
        static void focusCB(GLFWwindow *window, int focused) {
            getThisPtr(window)->_focus(focused);
        }
    };


    template<typename TYPE>
    class ImgWindow : public BaseWindow {


    private:

        Mem2<TYPE> m_img;

        virtual void display() {
            if (m_img.size() > 0) {
                if (cmpSize(2, m_img.dsize, m_wcam.dsize) == false) {
                    glfwSetWindowSize(m_win, m_img.dsize[0], m_img.dsize[1]);
                }
                glLoadView2D(m_img.dsize, m_viewPos, m_viewScale);
                glTexImg(m_img);
            }
        };

        virtual void keyFun(int key, int scancode, int action, int mods) {
            BaseWindow *p = static_cast<BaseWindow*>(m_parent);
            p->_keyFun(key, scancode, action, mods);
        }

    public:

        ImgWindow() {
        }

        void set(const Mem2<TYPE> &img) {
            m_img = img;
        }

    };

    template<typename TYPE>
    SP_CPUFUNC void glShowImg(BaseWindow *win, const char *name, const Mem2<TYPE> &img) {
        static Mem1<const char*> names;
        static MemP<ImgWindow<TYPE> > pool;

        ImgWindow<TYPE> *ptr = NULL;
        for (int i = 0; i < names.size(); i++) {
            if (strcmp(names[i], name) == 0) {
                ptr = &pool[i];
                break;
            }
        }
        if (ptr == NULL) {
            ptr = pool.malloc();
            ptr->create(name, img.dsize[0], img.dsize[1], win);
            names.push(name);
        }

        ptr->set(img);
    }
}

#endif

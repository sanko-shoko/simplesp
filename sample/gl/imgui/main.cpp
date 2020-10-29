#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class SampleGUI : public BaseWindowIMGUI {

private:

    void help() {
        printf("\n");
    }

    virtual void init() {
        help();
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // test window
        {
            ImGui::Begin("Test Window", NULL, ImGuiWindowFlags_NoResize);

            ImGui::SetWindowPos(ImVec2(50, 50), ImGuiCond_Once);
            ImGui::SetWindowSize(ImVec2(300, 300), ImGuiCond_Always);

            static int iVal = 0;
            ImGui::InputInt("InputInt", &iVal, 1, 100, ImGuiInputTextFlags_EnterReturnsTrue);
            printf("%d\n", iVal);

            ImGui::End();
        }

        //{
        //    static bool show_test_window = true;
        //    ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
        //    ImGui::ShowTestWindow(&show_test_window);
        //}
    }

    virtual void mousePos(double x, double y) {
    }

    virtual void mouseScroll(double x, double y) {
    }

};


int main() {

    SampleGUI win;
    win.execute("sample", 800, 600);

    return 0;
}


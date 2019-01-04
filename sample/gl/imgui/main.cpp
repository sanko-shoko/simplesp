#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class SampleGUI : public BaseWindow {

private:

    void help() {
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {
        help();
    }

    virtual void keyFun(int key, int scancode, int action, int mods) {
    }

    virtual void display() {

        // test window
        {
            ImGui::Begin("Test Window", NULL, ImGuiWindowFlags_NoResize);

            ImGui::SetWindowPos(ImVec2(50, 50), ImGuiCond_Once);
            ImGui::SetWindowSize(ImVec2(300, 300), ImGuiCond_Always);

            if (ImGui::Button("Button")) {
                printf("Button\n");
            }

            static bool check = true;
            ImGui::Checkbox("Check", &check);

            static int radio = 0;
            ImGui::RadioButton("a", &radio, 0); ImGui::SameLine();
            ImGui::RadioButton("b", &radio, 1); ImGui::SameLine();
            ImGui::RadioButton("c", &radio, 2); ImGui::SameLine();
            ImGui::Text("Radio");

            static int id = 1;
            const char* items[] = { "AAA", "BBB", "CCC", "DDD", "EEE", "FFF", "GGG" };
            if (ImGui::ListBox("ListBox", &id, items, IM_ARRAYSIZE(items), 4)) {
                printf("Select %s\n", items[id]);
            }

            static char text[128] = "Hello, world!";
            ImGui::InputText("InputText", text, IM_ARRAYSIZE(text));

            static int iVal = 0;
            ImGui::InputInt("InputInt", &iVal, 1, 100);

            static float fVal = 0.0f;
            ImGui::InputFloat("InputFloat", &fVal, 0.1f, 100.0f);

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


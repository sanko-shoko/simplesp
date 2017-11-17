#define SP_USE_IMGUI 1

#include "gtutil.h"
#include "rectmode.h"

using namespace sp;

class GTMakerGUI : public BaseWindow {

	BaseMode *m_base;
	Mem2<Col3> m_img;
private:

	void help() {
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init() {
		help();

		m_img.resize(640, 480);
		setElm(m_img, getCol(50, 50, 50));


		selectMode(0);
	}

	virtual void action() {
	}

	void selectMode(const int mode) {
		static RectMode rectmode(this, &m_img);

		switch (mode) {
		case 0: m_base = &rectmode; break;
		}
	}

	virtual void display() {
		m_base->display();

		// test window
		if (ImGui::Begin("Test Window", NULL, ImGuiWindowFlags_NoResize)) {

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

	}
	virtual void mouseButton(int button, int action, int mods) {
		m_base->mouseButton(button, action, mods);
	}
	virtual void mousePos(double x, double y) {
		m_base->mousePos(x, y);
	}

	virtual void mouseScroll(double x, double y) {
		m_base->mouseScroll(x, y);
	}

};


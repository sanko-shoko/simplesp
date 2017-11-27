#define SP_USE_IMGUI 1

#include "gtutil.h"
#include "rectmode.h"

using namespace sp;

class GTMakerGUI : public BaseWindow {

	BaseMode *m_base;

private:

	void help() {
		//printf("1. open image directory\n");
		//printf("2. edit labels (dog, cat, ...)\n");
		//printf("3. set rectangles (left click and drag)\n");
		//printf("\n");

		//printf("[shortcut]\n");
		//printf("a key : image ++\n");
		//printf("s key : image --\n");
		//printf("space key + mouse : adjust image size and position\n");
		//printf("\n");
	}

	virtual void init() {
		help();

		ImGui::GetIO().IniFilename = NULL;

		BaseMode::init(this);

		selectMode(0);
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {

		if (m_keyAction[GLFW_KEY_A] > 0) {
			if (m_base->select(BaseMode::m_selectid + 1)) {
				adjustImg();
			}
		}
		if (m_keyAction[GLFW_KEY_S] > 0) {
			if (m_base->select(BaseMode::m_selectid - 1)) {
				adjustImg();
			}
		}
	}

	void selectMode(const int mode) {
		static RectMode rectmode;

		switch (mode) {
		case 0: m_base = &rectmode; break;
		}
	}

	void adjustImg() {
		const Mem<Col3> &img = BaseMode::m_img;
		if (img.size() == 0) return;

		m_viewPos = getVec(100.0, 10.0);
		m_viewScale = 0.92 * minVal(static_cast<double>(m_wcam.dsize[0] - 180) / img.dsize[0], static_cast<double>(m_wcam.dsize[1]) / img.dsize[1]);
	}

	virtual void display() {

		if (ImGui::BeginMainMenuBar()) {

			if (ImGui::BeginMenu("file")) {

				if (ImGui::MenuItem("open image dir")) {
					if (BaseMode::open() == true) {
						m_base->select(0);
						adjustImg();
					}
				}
				{
					m_base->menu("file");
				}

				ImGui::EndMenu();
			}

			ImGui::EndMainMenuBar();
		}

		if (BaseMode::m_imNames.size() != 0) {

			if (ImGui::Begin("dataset", NULL, ImGuiWindowFlags_Block)) {
				int &selectid = BaseMode::m_selectid;

				ImGui::SetWindowPos(ImVec2(15, 35), ImGuiCond_Always);
				ImGui::SetWindowSize(ImVec2(190, 70), ImGuiCond_Always);

				if (selectid >= 0) {
					ImGui::Text(BaseMode::m_imNames[selectid].c_str());
				}
				ImGui::PushItemWidth(110);

				if (ImGui::InputInt("", &selectid, 1, 100)) {
					m_base->select(selectid);
					adjustImg();
				}
				ImGui::PopItemWidth();
				{
					ImGui::SameLine();
					ImGui::Text("/ %d", BaseMode::m_imNames.size());
				}
				ImGui::End();
			}
		}

		if (BaseMode::m_img.size() != 0) {
			BaseMode::m_vmat = getViewMat(BaseMode::m_img.dsize[0], BaseMode::m_img.dsize[1], m_viewPos, m_viewScale);
		}

		m_base->display();
	}

	virtual void windowSize(int width, int height) {
		adjustImg();
	}

	virtual void mouseButton(int button, int action, int mods) {
		if (m_base != NULL) {
			m_base->mouseButton(button, action, mods);
		}
	}

	virtual void mousePos(double x, double y) {
		if (m_base != NULL) {
			m_base->mousePos(x, y);
		}
	}

	virtual void mouseScroll(double x, double y) {
		if (m_base != NULL) {
			m_base->mouseScroll(x, y);
		}
	}

};


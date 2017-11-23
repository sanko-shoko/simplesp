#define SP_USE_IMGUI 1

#include "gtutil.h"
#include "rectmode.h"

using namespace sp;

class GTMakerGUI : public BaseWindow {

	BaseMode *m_base;

private:

	void help() {
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init() {
		help();
		
		BaseMode::init(this);

		selectMode(0);
	}

	virtual void action() {
	}

	void selectMode(const int mode) {
		static RectMode rectmode;

		switch (mode) {
		case 0: m_base = &rectmode; break;
		default: m_base = NULL; break;
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

				if (ImGui::MenuItem("open dir")) {
					if (BaseMode::open() == true) {
						m_base->select(0);
						adjustImg();
					}
				}

				m_base->menu("file");

				ImGui::EndMenu();
			}

			ImGui::EndMainMenuBar();
		}

		const Mem1<string> &names = BaseMode::m_names;
		if (names.size() != 0) {

			if (ImGui::Begin("dataset", NULL, ImGuiWindowFlags_Block)) {
				static int imgid = 0;

				ImGui::SetWindowPos(ImVec2(20, 40), ImGuiCond_Always);
				ImGui::SetWindowSize(ImVec2(180, 60), ImGuiCond_Always);

				ImGui::Text(BaseMode::m_names[imgid].c_str());

				ImGui::PushItemWidth(110);

				if (ImGui::InputInt("", &imgid, 1, 100)) {
					imgid = maxVal(imgid, 0);
					imgid = minVal(imgid, BaseMode::m_names.size() - 1);
					m_base->select(imgid);
					adjustImg();
				}
				{
					ImGui::SameLine();
					ImGui::Text("/%d", BaseMode::m_names.size());
				}
				ImGui::End();
			}
		}

		const Mem2<Col3> &img = BaseMode::m_img;
		if (img.size() != 0) {
			BaseMode::m_vmat = getViewMat(img.dsize[0], img.dsize[1], m_viewPos, m_viewScale);
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


#ifndef __GTMAKER__
#define __GTMAKER__

#include "gtutil.h"

using namespace sp;

class GTMakerGUI : public BaseWindow {

private:
	DataBase m_database;

	Mem2<Col3> m_img;

	int m_selectid;

	RectGT *m_focus;
	Vec2 *m_base;

	bool m_usePaint;

	enum Mode {
		Base = -1,
		Paint = 0
	};

	Mode m_mode;

public:

	GTMakerGUI() {
		reset();

		m_database.gtNames.push("dog");
		m_database.gtNames.push("cat");

		//m_usePaint = true;
	}

	void reset() {
		m_focus = NULL;
		m_base = NULL;

		m_mode = Mode::Base;
	}

private:

	void select(const int id) {
		m_selectid = maxVal(0, minVal(m_database.imNames.size() - 1, id));

		const string path = m_database.imDir + "\\" + m_database.imNames[m_selectid];
		SP_ASSERT(cvLoadImg(m_img, path.c_str()));

		reset();

		adjustImg();
	}

	void adjustImg() {
		if (m_img.size() == 0) return;

		m_viewPos = getVec(100.0, 10.0);
		m_viewScale = 0.92 * minVal(static_cast<double>(m_wcam.dsize[0] - 180) / m_img.dsize[0], static_cast<double>(m_wcam.dsize[1]) / m_img.dsize[1]);
	}


private:

	virtual void init() {

		ImGui::GetIO().IniFilename = NULL;

	}

	virtual void display() {

		if (ImGui::BeginMainMenuBar()) {

			if (ImGui::BeginMenu("file")) {

				if (ImGui::MenuItem("open image dir") && m_database.open_imDir()) {
					select(0);
				}
				ImGui::EndMenu();
			}

			//if (ImGui::BeginMenu("option")) {

			//	if (ImGui::MenuItem("paint", NULL, m_usePaint)) {
			//		m_usePaint ^= true;
			//	}
			//	ImGui::EndMenu();
			//}

			ImGui::EndMainMenuBar();
		}

		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
			glRenderImage(m_img);
		}

		dispRectGT();

		dispDataBase();


	}

	void dispRectGT();
	void dispDataBase();


	virtual void windowSize(int width, int height) {
		if (m_database.isValid() == false) return;

		adjustImg();
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {
		if (m_database.isValid() == false) return;

		if (m_keyAction[GLFW_KEY_A] > 0) {
			select(m_selectid + 1);
		}
		if (m_keyAction[GLFW_KEY_S] > 0) {
			select(m_selectid - 1);
		}
	}

	virtual void mouseButton(int button, int action, int mods) {
		if (m_database.isValid() == false) return;

		switch (m_mode) {
		case Mode::Base: mousebuttonRect(button, action, mods); break;
		case Mode::Paint: mousebuttonPaint(button, action, mods); break;
		}
	}

	void mousebuttonRect(int button, int action, int mods);
	void mousebuttonPaint(int button, int action, int mods);

	virtual void mousePos(double x, double y) {
		if (m_database.isValid() == false) return;
	
		switch (m_mode) {
		case Mode::Base: mousePosRect(x, y); break;
		case Mode::Paint: mousePosPaint(x, y); break;
		}
	}

	void mousePosRect(double x, double y);
	void mousePosPaint(double x, double y);

	virtual void mouseScroll(double x, double y) {
	}

};

#endif
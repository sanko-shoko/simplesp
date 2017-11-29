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

private:
	void dispRectGT();

	void dispDataBase();

public:

	GTMakerGUI() {
		reset();

		m_database.gtNames.push("dog");
		m_database.gtNames.push("cat");
	}

	void reset() {
		m_focus = NULL;
		m_base = NULL;
	}

private:

	void select(const int id) {
		m_selectid = maxVal(0, minVal(m_database.imNames.size() - 1, id));

		const string path = m_database.imDir + "\\" + m_database.imNames[m_selectid];
		SP_ASSERT(cvLoadImg(m_img, path.c_str()));
		reset();
		m_database.gtsList.resize(m_database.imNames.size());

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

			ImGui::EndMainMenuBar();
		}

		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
			glRenderImage(m_img);
		}

		dispRectGT();

		dispDataBase();
	}

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

		const Mat vmat = getViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
		const Vec2 pix = invMat(vmat) * m_mouse.pos;

		MemP<RectGT> &gts = m_database.gtsList[m_selectid];

		static Vec2 base;
		m_base = (m_mouse.bDownL == 1) ? &base : NULL;

		static RectGT newgt;

		const double thresh = 10.0 / m_viewScale;

		if (m_focus != &newgt && m_focus != NULL) {
			if (m_mouse.bDownL == 1) {
				const Vec2 a = getVec(m_focus->rect.dbase[0], m_focus->rect.dbase[1]);
				const Vec2 b = getVec(m_focus->rect.dsize[0] - 1, m_focus->rect.dsize[1] - 1);

				Vec2 p[4];
				p[0] = a + getVec(0.0, 0.0);
				p[1] = a + getVec(b.x, 0.0);
				p[2] = a + getVec(b.x, b.y);
				p[3] = a + getVec(0.0, b.y);

				double minv = thresh;
				for (int i = 0; i < 4; i++) {
					const double norm = normVec(p[i] - pix);
					if (norm < minv) {
						minv = norm;
						base = p[(i + 2) % 4];
					}
				}
				if (minv < thresh) {
					return;
				}

			}
			if (m_mouse.bDownL == 0) {
				m_focus->rect = andRect(m_focus->rect, getRect2(m_img.dsize));
				return;
			}
		}

		{
			if (m_focus != &newgt && m_mouse.bDownL == 1) {

				m_focus = &newgt;
				m_focus->rect = getRect2(pix);
				m_focus->label = -1;

				base = pix;
				return;
			}

			if (m_focus == &newgt && m_mouse.bDownL == 0) {

				if (minVal(m_focus->rect.dsize[0], m_focus->rect.dsize[1]) > thresh) {
					RectGT *gt = gts.malloc();
					*gt = *m_focus;
					m_focus = gt;
					m_focus->rect = andRect(m_focus->rect, getRect2(m_img.dsize));
				}
				else {
					m_focus = NULL;
				}
				return;
			}
		}
	}
	virtual void mousePos(double x, double y) {
		if (m_database.isValid() == false) return;

		const Mat vmat = getViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
		const Vec2 pix = invMat(vmat) * m_mouse.pos;

		if (m_focus != NULL && m_base != NULL) {
			m_focus->rect = orRect(getRect2(pix), getRect2(*m_base));
		}
	}

	virtual void mouseScroll(double x, double y) {
	}

};

#endif
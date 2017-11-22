#ifndef __RECTMODE__
#define __RECTMODE__

#include "gtutil.h"

using namespace sp;

struct RectGT {
	Rect rect;
	int label;
};

class RectMode : public BaseMode {
private:
	char *combolist[1000];

	Mem1<MemP<RectGT>> m_database;

	Mem1<LabelInfo> m_labelinfo;

	MemP<RectGT> *m_gts;

	RectGT *m_make;
	RectGT *m_edit;
	RectGT *m_focus;
	Vec2 *m_act;

private:

	void displayRect(const RectGT &gt) {

		glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);
		
		Col3 col[2];
		col[0] = getCol(0, 100, 100);
		col[1] = getCol(80, 180, 180);
	
		for (int i = 0; i < 2; i++) {
			glLineWidth(i == 0 ? 3.0f : 1.0f);
			glBegin(GL_LINES);
			glColor(col[i]);
			glRect(gt.rect);
			glEnd();
		}
	}

	void displayRectFocus(const RectGT &gt) {

		glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);

		Col3 col[2];
		col[0] = getCol(160, 160, 100);
		col[1] = getCol(240, 240, 180);

		for (int i = 0; i < 2; i++) {
			glLineWidth(i == 0 ? 3.0f : 1.0f);
			glBegin(GL_LINES);
			glColor(col[i]);
			glRect(gt.rect);
			glEnd();
		}
		for (int i = 0; i < 2; i++) {
			glPointSize(i == 0 ? 5.0f : 3.0f);
			glBegin(GL_POINTS);
			glColor(col[i]);
			glRect(gt.rect);
			glEnd();
		}
	}

	void displayLabel(RectGT &gt) {

		glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);

		char str[SP_STRMAX];
		sprintf(str, "RectGT %p", &gt);

		if (ImGui::Begin(str, NULL, ImGuiWindowFlags_Block)) {
			const Vec2 offset = getVec(0.0, -40.0);

			ImGui::PushItemWidth(100);
			{
				const Vec2 vec = m_vmat * getVec(gt.rect.dbase[0], gt.rect.dbase[1]) + offset;

				ImGui::SetWindowPos(ImVec2(static_cast<float>(vec.x), static_cast<float>(vec.y)), ImGuiCond_Always);
				ImGui::SetWindowSize(ImVec2(180 , 35), ImGuiCond_Always);
			}

			ImGui::Text(".");

			ImGui::SameLine();
			ImGui::Combo("", &gt.label, combolist, m_labelinfo.size());

			ImGui::SameLine();

			bool del = false;
			if (ImGui::Button("del")) {
				if (&gt == m_focus) {
					m_focus = NULL;
				}
				del = true;
				m_gts->free(&gt);
			}

			if (del == false && ImGui::IsWindowHovered() && m_parent->m_mouse.bDownL) {
				m_focus = &gt;
			}

			ImGui::End();
		}
	}

public:

	RectMode(){
		m_gts = NULL;

		m_labelinfo.push(LabelInfo("dog"));
		m_labelinfo.push(LabelInfo("cat"));
	}

	virtual void select(const int id) {
		if (id < 0) return;

		string path = m_folder + "\\" + m_names[id];
		SP_ASSERT(cvLoadImg(m_img, path.c_str()));

		m_database.resize(m_names.size());
		m_gts = &m_database[id];
	
		m_make = NULL;
		m_edit = NULL;
		m_focus = NULL;
		m_act = NULL;
	}

	virtual void save() {
		if (m_gts == NULL) return;

		char str[SP_STRMAX];
		sprintf(str, "rect_%s", getTimeStamp().c_str());

		const char *path = tinyfd_saveFileDialog("save", str, 0, NULL, NULL);
		if (path == NULL) return;

		mkdir(path);
	}

	virtual void display() {
		if (m_img.size() == 0 || m_gts == NULL) return;

		m_vmat = getViewMat(m_img.dsize[0], m_img.dsize[1], m_parent->m_viewPos, m_parent->m_viewScale);


		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);
			glRenderImage(m_img);
		}

		if (m_make != NULL && minVal(m_make->rect.dsize[0], m_make->rect.dsize[1]) > 5) {
			displayRect(*m_make);
		}

		for (int i = 0; i < m_gts->size(); i++) {
			if (&(*m_gts)[i] == m_focus) continue;

			displayRect((*m_gts)[i]);
			displayLabel((*m_gts)[i]);
		}

		if (m_focus != NULL) {
			displayRectFocus(*m_focus);
			displayLabel(*m_focus);
		}


		if (ImGui::Begin("gt info", NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove)) {

			ImGui::SetWindowPos(ImVec2(20, 110), ImGuiCond_Always);
			ImGui::SetWindowSize(ImVec2(180, static_cast<float>(m_parent->m_wcam.dsize[1] - 135)), ImGuiCond_Always);

			ImGui::Text("labels");

			Mem1<LabelInfo> tmp;
			for (int i = 0; i < m_labelinfo.size(); i++) {
				char str[SP_STRMAX];
				sprintf(str, "label%d", i);
				ImGui::BeginChild(str, ImVec2(0, 24));

				ImGui::PushItemWidth(100);
				ImGui::InputText("", m_labelinfo[i].name, SP_STRMAX);
				ImGui::SameLine();

				if (ImGui::Button("del")) {
				}
				else {
					tmp.push(m_labelinfo[i]);
				}
				ImGui::EndChild();
			}

			m_labelinfo = tmp;
			{
				ImGui::BeginChild("", ImVec2(0, 24));

				ImGui::PushItemWidth(100);

				if (ImGui::Button("add")) {
					m_labelinfo.push(LabelInfo(""));
				}
				ImGui::EndChild();
			}

			ImGui::End();
		}

		{
			for (int i = 0; i < m_labelinfo.size(); i++) {
				combolist[i] = m_labelinfo[i].name;
			}
		}
	}

	virtual void mouseButton(int button, int action, int mods) {
		if (m_gts == NULL) return;
		
		const Vec2 pix = invMat(m_vmat) * m_parent->m_mouse.pos;

		if (m_parent->m_mouse.bDownL == 1) {
			static Vec2 act;
			act = pix;
			m_act = &act;
		}
		else {
			m_act = NULL;
		}

		if (m_focus != NULL) {
			if (m_edit == NULL && m_parent->m_mouse.bDownL == 1) {
				const Vec2 a = getVec(m_focus->rect.dbase[0], m_focus->rect.dbase[1]);
				const Vec2 b = getVec(m_focus->rect.dsize[0], m_focus->rect.dsize[1]);

				Vec2 p[4];
				p[0] = a + getVec(0.0, 0.0);
				p[1] = a + getVec(b.x, 0.0);
				p[2] = a + getVec(b.x, b.y);
				p[3] = a + getVec(0.0, b.y);

				for (int i = 0; i < 4; i++) {
					if (normVec(p[i] - pix) * m_parent->m_viewScale < 10) {
						*m_act = p[(i + 2) % 4];
						m_edit = m_focus;
					}
				}
			}
			if (m_edit != NULL && m_parent->m_mouse.bDownL == 0) {
				m_edit = NULL;
			}

		}
		if (m_edit == NULL) {
			if (m_make == NULL && m_parent->m_mouse.bDownL == 1) {
				static RectGT gt;

				m_make = &gt;
				m_make->rect = getRect2(pix);
				m_make->label = -1;
			}

			if (m_make != NULL && m_parent->m_mouse.bDownL == 0 && minVal(m_make->rect.dsize[0], m_make->rect.dsize[1]) > 10) {
				RectGT *gt = m_gts->malloc();
			
				*gt = *m_make;
				m_make = NULL;
				m_focus = gt;
			}
		}


	}

	virtual void mousePos(double x, double y) {
		if (m_gts == NULL) return;

		const Vec2 pix = invMat(m_vmat) * m_parent->m_mouse.pos;

		if (m_make != NULL && m_act != NULL) {
			m_make->rect = orRect(getRect2(pix), getRect2(*m_act));
		}

		if (m_edit != NULL && m_act != NULL) {
			m_edit->rect = orRect(getRect2(pix), getRect2(*m_act));
		}
	}

	virtual void mouseScroll(double x, double y) {
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {
	}

	virtual void charFun(unsigned int charInfo) {
	}

};


#endif
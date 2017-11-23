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

	void displayRect(const RectGT &gt, const Col3 &col, bool focus = false) {

		for (int i = 0; i < 2; i++) {
			glLineWidth(i == 0 ? 3.0f : 1.0f);
			glBegin(GL_LINES);
			glColor(i == 0 ? col / 2.0 : col);
			glRect(gt.rect);
			glEnd();
		}
		if (focus == false) return;

		for (int i = 0; i < 2; i++) {
			glPointSize(i == 0 ? 5.0f : 3.0f);
			glBegin(GL_POINTS);
			glColor(i == 0 ? col / 2.0 : col);
			glRect(gt.rect);
			glEnd();
		}
	}

	void displayLabel(RectGT &gt) {

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

	void delLabel(const int id) {
		m_labelinfo.del(id);

		for (int i = 0; i < m_database.size(); i++) {
			MemP<RectGT> &gts = m_database[i];

			for (int j = 0; j < gts.size(); j++) {
				RectGT &gt = gts[j];
				if (gt.label == id) {
					gt.label = -1;
				}
				if (gt.label > id) {
					gt.label--;
				}
			}
		}
	}

public:

	RectMode(){
		m_gts = NULL;

		m_labelinfo.push(LabelInfo("dog"));
		m_labelinfo.push(LabelInfo("cat"));
	}

	virtual void select(const int id) {
		static int backup = -1;
		if (id < 0 || backup == id) return;
		backup = id;

		const string path = m_imdir + "\\" + m_names[id];
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

		const string dir = m_gtdir + "\\" + "rect";
		mkdir(dir.c_str());

		{
			File file((dir + "\\" + "_labels.txt").c_str(), "w");

			file.printf("%d, \n", m_labelinfo.size());

			for (int i = 0; i < m_labelinfo.size(); i++) {
				file.printf("%s, \n", m_labelinfo[i].name);
			}
		}
		for(int i = 0; i < m_database.size(); i++){
			MemP<RectGT> &gts = m_database[i];
			if (gts.size() == 0) continue;

			File file((dir + "\\" + m_names[i] + ".txt").c_str(), "w");

			file.printf("%d, \n", gts.size());

			for (int j = 0; j < gts.size(); j++) {
				const RectGT &gt = gts[j];
				file.printf("%d, %d, ", j, gt.label);
				file.printf("%d, %d, %d, %d, ", gt.rect.dbase[0], gt.rect.dbase[1], gt.rect.dsize[0], gt.rect.dsize[1]);
				file.printf("\n");
			}
		}
	}

	virtual void load() {
		const char *path = tinyfd_selectFolderDialog("select gt folder", NULL);
		if (path == NULL) return;

		const string dir = path;
		const string tmp = dir + "\\" + "rect";

		const Mem1<string> names = getFileList(tmp.c_str(), "txt");

		{
			File file((tmp + "\\" + "_labels.txt").c_str(), "r");

			int num;
			file.scanf("%d, \n", &num);

			m_labelinfo.resize(num);
			for (int i = 0; i < m_labelinfo.size(); i++) {
				char str[SP_STRMAX];
				file.gets(str);

				::strcpy(m_labelinfo[i].name, strSplit(str)[0].c_str());
			}
		}

		for (int i = 0; i < m_names.size(); i++) {
			MemP<RectGT> &gts = m_database[i];
			gts.clear();

			string name;
			for (int j = 0; j < names.size(); j++) {
				if (names[j] == "_labels.txt" || names[j] != (m_names[i] + ".txt")) continue;

				name = names[j];
				break;
			}
			if (name.size() == 0) continue;

			File file((tmp + "\\" + m_names[i] + ".txt").c_str(), "r");
			int num;
			file.scanf("%d, \n", &num);

			for (int j = 0; j < num; j++) {
				int buf;
				RectGT &gt = *gts.malloc();
				gt.rect.dim = 2;
				file.scanf("%d, %d, ", &buf, &buf);
				file.scanf("%d, %d, %d, %d, ", &gt.rect.dbase[0], &gt.rect.dbase[1], &gt.rect.dsize[0], &gt.rect.dsize[1]);
				file.scanf("\n");
			}

		}

	}

	virtual void menu(const char *name) {
		if (m_gts == NULL) return;

		if (::strcmp(name, "file") == 0) {
			if (ImGui::MenuItem("save gt")) {
				save();
			}
			if (ImGui::MenuItem("load gt")) {
				load();
			}
		}
	}

	virtual void display() {
		if (m_img.size() == 0 || m_gts == NULL) return;

		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);
			glRenderImage(m_img);
		}

		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);

			if (m_make != NULL && minVal(m_make->rect.dsize[0], m_make->rect.dsize[1]) > 5) {
				displayRect(*m_make, getCol(80, 180, 180));
			}

			for (int i = 0; i < m_gts->size(); i++) {
				if (&(*m_gts)[i] == m_focus) continue;
				displayRect((*m_gts)[i], getCol(80, 180, 180));
			}

			if (m_focus != NULL) {
				displayRect(*m_focus, getCol(240, 240, 180), true);
			}
		}

		{
			for (int i = 0; i < m_gts->size(); i++) {
				displayLabel((*m_gts)[i]);
			}
		}

		if (ImGui::Begin("gt info", NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove)) {

			ImGui::SetWindowPos(ImVec2(20, 110), ImGuiCond_Always);
			ImGui::SetWindowSize(ImVec2(180, static_cast<float>(m_parent->m_wcam.dsize[1] - 135)), ImGuiCond_Always);

			ImGui::Text("labels");

			for (int i = 0; i < m_labelinfo.size(); i++) {
				char str[SP_STRMAX];
				sprintf(str, "label%d", i);
				ImGui::BeginChild(str, ImVec2(0, 24));

				ImGui::PushItemWidth(100);
				ImGui::InputText("", m_labelinfo[i].name, SP_STRMAX);
				ImGui::SameLine();

				if (ImGui::Button("del")) {
					delLabel(i--);
				}
				ImGui::EndChild();
			}

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
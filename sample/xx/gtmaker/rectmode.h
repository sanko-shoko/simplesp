#ifndef __RECTMODE__
#define __RECTMODE__

#include "gtutil.h"

using namespace sp;


class RectMode : public BaseMode {
private:
	struct RectGT {
		Rect rect;
		int label;
	};

	Mem1<MemP<RectGT>> m_gtdata;

	Mem1<LabelInfo> m_labelinfo;

	RectGT *m_make;
	RectGT *m_edit;
	RectGT *m_focus;
	Vec2 *m_act;

private:

	void displayRect(const RectGT &gt, const Col3 &col, bool focus = false) {

		for (int i = 0; i < 2; i++) {
			glLineWidth(i == 0 ? 3.0f : 1.0f);
			glBegin(GL_LINES);
			glColor(i == 0 ? col / 4.0 : col);
			glRect(gt.rect);
			glEnd();
		}
		if (focus == false) return;

		for (int i = 0; i < 2; i++) {
			glPointSize(i == 0 ? 5.0f : 3.0f);
			glBegin(GL_POINTS);
			glColor(i == 0 ? col / 4.0 : col);
			glRect(gt.rect);
			glEnd();
		}
	}

	void displayLabel(RectGT &gt) {
		char *combolist[1000];
		for (int i = 0; i < m_labelinfo.size(); i++) {
			combolist[i] = m_labelinfo[i].name;
		}

		char str[SP_STRMAX];
		sprintf(str, "RectGT %p", &gt);

		if (ImGui::Begin(str, NULL, ImGuiWindowFlags_Block | ImGuiWindowFlags_NoFocusOnAppearing)) {
			{
				const Vec2 pos = m_vmat * getVec(gt.rect.dbase[0], gt.rect.dbase[1]) + getVec(0.0, -40.0);
				ImGui::SetWindowPos(ImVec2(static_cast<float>(pos.x), static_cast<float>(pos.y)), ImGuiCond_Always);
			}

			if (&gt != m_focus && ImGui::IsMouseClicked(0) == true && ImGui::IsWindowHovered() == true) {
				m_focus = &gt;
			}

			ImGui::AlignTextToFramePadding();

			if(&gt != m_focus){
				ImGui::Text(">");

				const int wsize = (gt.label >= 0) ? sp::strlen(m_labelinfo[gt.label].name) : 0;
			
				ImGui::SetWindowSize(ImVec2(wsize * 8.0f + 30.0f, 35.0f), ImGuiCond_Always);

				if (gt.label >= 0) {
					ImGui::SameLine();
					ImGui::Text(m_labelinfo[gt.label].name);
				}
			}
			else{
				ImGui::Text("-");

				ImGui::SetWindowSize(ImVec2(180.0f, 35.0f), ImGuiCond_Always);

				ImGui::SameLine();

				ImGui::PushItemWidth(100);
				ImGui::Combo("", &gt.label, combolist, m_labelinfo.size());
				ImGui::PopItemWidth();

				ImGui::SameLine();

				if (ImGui::Button("del")) {
					m_gtdata[m_selectid].free(&gt);
					m_focus = NULL;
				}
			}
			ImGui::End();
		}
	}

	void updateLabel(const int id, const int val) {
		for (int i = 0; i < m_gtdata.size(); i++) {
			MemP<RectGT> &gts = m_gtdata[i];

			for (int j = 0; j < gts.size(); j++) {
				RectGT &gt = gts[j];
				if (gt.label == id && val < 0) {
					gt.label = -1;
					continue;
				}
				if (gt.label >= id) {
					gt.label += val;
				}
			}
		}
	}

public:

	RectMode(){
		reset();

		m_labelinfo.push(LabelInfo("dog"));
		m_labelinfo.push(LabelInfo("cat"));
	}

	virtual void reset() {
		m_make = NULL;
		m_edit = NULL;
		m_focus = NULL;
		m_act = NULL;
	}

	virtual bool select(const int id) {
		m_selectid = maxVal(0, minVal(m_names.size() - 1, id));

		reset();

		const string path = m_imdir + "\\" + m_names[m_selectid];
		SP_ASSERT(cvLoadImg(m_img, path.c_str()));

		m_gtdata.resize(m_names.size());

		return true;
	}

	virtual void save() {
		if (m_selectid < 0) return;

		const string dir = string("rect_") + getTimeStamp();
		mkdir(dir.c_str());

		{
			File file((dir + "\\" + "_labels.csv").c_str(), "w");

			file.printf("num, %d, \n", m_labelinfo.size());

			for (int i = 0; i < m_labelinfo.size(); i++) {
				file.printf("%s, \n", m_labelinfo[i].name);
			}
		}
		for(int i = 0; i < m_gtdata.size(); i++){
			MemP<RectGT> &gts = m_gtdata[i];
			if (gts.size() == 0) continue;

			File file((dir + "\\" + m_names[i] + ".csv").c_str(), "w");

			file.printf("num, %d, \n", gts.size());
			file.printf("index, label, x, y, width, height\n");

			for (int j = 0; j < gts.size(); j++) {
				const RectGT &gt = gts[j];
				file.printf("%d, %d, ", j, gt.label);
				file.printf("%d, %d, %d, %d, ", gt.rect.dbase[0], gt.rect.dbase[1], gt.rect.dsize[0], gt.rect.dsize[1]);
				file.printf("\n");
			}
		}
	}

	virtual void load() {
		const char *path = tinyfd_selectFolderDialog("select gt directory", getCrntDir().c_str());
		if (path == NULL) return;

		reset();

		const string dir = path;
		if (findFile((dir + "\\" + "_labels.csv").c_str()) == false) return;

		const Mem1<string> names = getFileList(dir.c_str(), "csv");

		{
			File file((dir + "\\" + "_labels.csv").c_str(), "r");

			int num = 0;
			file.scanf("num, %d, \n", &num);
			file.scanf("index, label, x, y, width, height\n");

			m_labelinfo.resize(num);
			for (int i = 0; i < m_labelinfo.size(); i++) {
				char str[SP_STRMAX];
				file.gets(str);

				const Mem1<string> split = strSplit(str);
				::strcpy(m_labelinfo[i].name, (split.size() > 0) ? split[0].c_str() : "");
			}
		}

		for (int i = 0; i < m_names.size(); i++) {
			MemP<RectGT> &gts = m_gtdata[i];
			gts.clear();

			string name;
			for (int j = 0; j < names.size(); j++) {
				if (names[j] == "_labels.csv" || names[j] != (m_names[i] + ".csv")) continue;

				name = names[j];
				break;
			}
			if (name.size() == 0) continue;

			File file((dir + "\\" + m_names[i] + ".csv").c_str(), "r");

			int num = 0;
			file.scanf("num, %d, \n", &num);
			file.scanf("index, label, x, y, width, height\n");

			for (int j = 0; j < num; j++) {
				int buf;
				RectGT &gt = *gts.malloc();
				gt.rect.dim = 2;
				file.scanf("%d, %d, ", &buf, &gt.label);
				file.scanf("%d, %d, %d, %d, ", &gt.rect.dbase[0], &gt.rect.dbase[1], &gt.rect.dsize[0], &gt.rect.dsize[1]);
				file.scanf("\n");
			}

		}

	}

	virtual void menu(const char *name) {
		if (m_selectid < 0) return;

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
		if (m_img.size() == 0 || m_selectid < 0) return;

		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);
			glRenderImage(m_img);
		}

		MemP<RectGT> &gts = m_gtdata[m_selectid];
		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_vmat);

			if (m_make != NULL && minVal(m_make->rect.dsize[0], m_make->rect.dsize[1]) > 5) {
				displayRect(*m_make, getCol(80, 180, 180));
			}

			for (int i = 0; i < gts.size(); i++) {
				if (&gts[i] == m_focus) continue;
				displayRect(gts[i], getCol(80, 180, 180));
			}

			if (m_focus != NULL) {
				displayRect(*m_focus, getCol(220, 240, 220), true);
			}
		}
		{
			for (int i = 0; i < gts.size(); i++) {
				displayLabel(gts[i]);
			}
		}

		if (ImGui::Begin("gt info", NULL, ImGuiWindowFlags_Block)) {

			ImGui::SetWindowPos(ImVec2(15, 115), ImGuiCond_Always);
			ImGui::SetWindowSize(ImVec2(190, static_cast<float>(m_parent->m_wcam.dsize[1] - 135)), ImGuiCond_Always);

			{
				ImGui::BeginChild("label", ImVec2(0, 24));

				ImGui::AlignTextToFramePadding();
				ImGui::Text("labels");

				ImGui::SameLine(0.0f, 67.0f);

				if (ImGui::Button("add")) {
					m_labelinfo.add(0, LabelInfo());
					updateLabel(0, +1);
				}
				ImGui::EndChild();
			}

			for (int i = 0; i < m_labelinfo.size(); i++) {
				char str[SP_STRMAX];
				sprintf(str, "label%d", i);
				ImGui::BeginChild(str, ImVec2(0, 24));

				ImGui::PushItemWidth(100);
				ImGui::InputText("", m_labelinfo[i].name, SP_STRMAX);
				ImGui::PopItemWidth();

				ImGui::SameLine();

				if (ImGui::Button("add")) {
					m_labelinfo.add(i + 1, LabelInfo());
					updateLabel(i + 1, +1);
				}

				ImGui::SameLine();

				if (ImGui::Button("del")) {
					m_labelinfo.del(i);
					updateLabel(i, -1);
					i--;
				}

				ImGui::EndChild();
			}

			ImGui::End();
		}

	}

	virtual void mouseButton(int button, int action, int mods) {
		if (m_selectid < 0) return;
		MemP<RectGT> &gts = m_gtdata[m_selectid];

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
						return;
					}
				}
			}
			if (m_edit != NULL && m_parent->m_mouse.bDownL == 0) {
				m_edit->rect = andRect(m_edit->rect, getRect2(m_img.dsize));
				m_edit = NULL;
				m_make = NULL;
				return;
			}

		}
		if (m_edit == NULL) {
			if (m_make == NULL && m_parent->m_mouse.bDownL == 1) {
				static RectGT gt;

				m_make = &gt;
				m_make->rect = getRect2(pix);
				m_make->label = -1;
				return;
			}

			if (m_make != NULL && m_parent->m_mouse.bDownL == 0) {
				m_make->rect = andRect(m_make->rect, getRect2(m_img.dsize));
				if (minVal(m_make->rect.dsize[0], m_make->rect.dsize[1]) > 5) {
					RectGT *gt = gts.malloc();
					*gt = *m_make;
					m_focus = gt;
				}
				else {
					m_focus = NULL;
				}
				m_make = NULL;
				return;
			}
		}
	}

	virtual void mousePos(double x, double y) {
		if (m_selectid < 0) return;

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
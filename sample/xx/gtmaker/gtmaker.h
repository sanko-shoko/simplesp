#define SP_USE_IMGUI 1

#include "gtutil.h"
#include "rectmode.h"

using namespace sp;

class GTMakerGUI : public BaseWindow {

private:
	DataBase m_database;

	Mem2<Col3> m_img;

	int m_selectid;

	RectGT *m_focus;
	Vec2 *m_base;

private:

	void displayRect(const RectGT &gt, const Col3 &col, bool focus) {
		{
			for (int i = 0; i < 2; i++) {
				glLineWidth(i == 0 ? 3.0f : 1.0f);
				glBegin(GL_LINES);
				glColor(i == 0 ? col / 4.0 : col);
				glRect(gt.rect);
				glEnd();
			}
		}
		if (focus == true) {
			for (int i = 0; i < 2; i++) {
				glPointSize(i == 0 ? 5.0f : 3.0f);
				glBegin(GL_POINTS);
				glColor(i == 0 ? col / 4.0 : col);
				glRect(gt.rect);
				glEnd();
			}
		}
	}

	void displayLabel(RectGT &gt) {
		Mem1<const char *> combolist;
		for (int i = 0; i < m_database.gtNames.size(); i++) {
			combolist.push(m_database.gtNames[i].c_str());
		}

		const Mat vmat = getViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

		if (ImGui::Begin(strFormat("RectGT %p", &gt).c_str(), NULL, ImGuiWindowFlags_Block)) {
			{
				const Vec2 pos = vmat * getVec(gt.rect.dbase[0], gt.rect.dbase[1]) + getVec(0.0, -40.0);
				ImGui::SetWindowPos(ImVec2(static_cast<float>(pos.x), static_cast<float>(pos.y)), ImGuiCond_Always);
			}

			if (&gt != m_focus && ImGui::IsMouseClicked(0) == true && ImGui::IsWindowHovered() == true) {
				m_focus = &gt;
			}

			ImGui::AlignTextToFramePadding();

			if (&gt != m_focus) {
				ImGui::Text(">");

				const int wsize = (gt.label >= 0) ? sp::strlen(m_database.gtNames[gt.label].c_str()) : 0;

				ImGui::SetWindowSize(ImVec2(wsize * 8.0f + 30.0f, 35.0f), ImGuiCond_Always);

				if (gt.label >= 0) {
					ImGui::SameLine();
					ImGui::Text(m_database.gtNames[gt.label].c_str());
				}
			}
			else {
				ImGui::Text("-");

				ImGui::SetWindowSize(ImVec2(180.0f, 35.0f), ImGuiCond_Always);

				ImGui::SameLine();

				ImGui::PushItemWidth(100);
				ImGui::Combo("", &gt.label, combolist.ptr, combolist.size());
				ImGui::PopItemWidth();

				ImGui::SameLine();

				if (ImGui::Button("del")) {
					m_database.gtsList[m_selectid].free(&gt);
					m_focus = NULL;
				}
			}
			ImGui::End();
		}
	}

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


	bool isValid() {
		return (m_database.imNames.size() > 0 && m_img.size() > 0 && m_selectid >= 0) ? true : false;
	}

	void select(const int id) {
		m_selectid = maxVal(0, minVal(m_database.imNames.size() - 1, id));

		const string path = m_database.imDir + "\\" + m_database.imNames[m_selectid];
		SP_ASSERT(cvLoadImg(m_img, path.c_str()));
		reset();
		m_database.gtsList.resize(m_database.imNames.size());
	}


private:

	virtual void init() {

		ImGui::GetIO().IniFilename = NULL;

	}

	void adjustImg() {
		if (m_img.size() == 0) return;

		m_viewPos = getVec(100.0, 10.0);
		m_viewScale = 0.92 * minVal(static_cast<double>(m_wcam.dsize[0] - 180) / m_img.dsize[0], static_cast<double>(m_wcam.dsize[1]) / m_img.dsize[1]);
	}

	virtual void display() {

		if (ImGui::BeginMainMenuBar()) {

			if (ImGui::BeginMenu("file")) {

				if (ImGui::MenuItem("open image dir") && m_database.open_imDir() == true) {
					select(0);
					adjustImg();
				}

				ImGui::EndMenu();
			}

			ImGui::EndMainMenuBar();
		}

		if (isValid() == false) return;

		if (ImGui::Begin("dataset", NULL, ImGuiWindowFlags_Block)) {

			ImGui::SetWindowRect(getRect2(15, 35, 190, 90), ImGuiCond_Always);

			{
				ImGui::Text("\n");
				ImGui::BulletText(m_database.imNames[m_selectid].c_str());
			}
			{
				ImGui::PushItemWidth(120);

				if (ImGui::InputInt("", &m_selectid, 1, 100)) {
					select(m_selectid);
					adjustImg();
				}
				ImGui::PopItemWidth();
			}
			{
				ImGui::SameLine();
				ImGui::Text("/%d", m_database.imNames.size());
			}
			ImGui::End();
		}

		if (isValid() == false) return;

		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
			glRenderImage(m_img);
		}

		MemP<RectGT> &gts = m_database.gtsList[m_selectid];

		{
			glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

			for (int i = 0; i < gts.size(); i++) {
				displayRect(gts[i], getCol(80, 180, 180), false);
			}

			if (m_focus != NULL) {
				displayRect(*m_focus, getCol(220, 240, 220), true);
			}

			for (int i = 0; i < gts.size(); i++) {
				displayLabel(gts[i]);
			}
		}


		if (ImGui::Begin("label info", NULL, ImGuiWindowFlags_Block)) {

			ImGui::SetWindowRect(getRect2(15, 135, 190, m_wcam.dsize[1] - 155), ImGuiCond_Always);
			{
				ImGui::Text("\n");
				ImGui::BulletText("work dir");

				{
					ImGui::Text("");

					ImGui::SameLine(0, 10);
					ImGui::BeginChild("work dir", ImVec2(110, 24));
					ImGui::AlignTextToFramePadding();

					const Mem1<string> split = strSplit(m_database.wkDir.c_str(), "\\");
					if (ImGui::MenuItem(split.last()->c_str())) {
						m_database.open_wkDir();
						reset();
					}
					ImGui::EndChild();
				}
				{
					ImGui::SameLine();

					if (ImGui::Button("save")) {
						m_database.save();
					}
				}
				ImGui::Text("\n");
			}
			ImGui::Separator();

			{
				ImGui::Text("\n");
				ImGui::BeginChild("label", ImVec2(0, 24));

				ImGui::AlignTextToFramePadding();
				ImGui::BulletText("label names");

				ImGui::SameLine(0, 11);

				if (ImGui::Button("add")) {
					m_database.gtNames.add(0, "");
					m_database.updateLabel(0, +1);
				}
				ImGui::EndChild();
			}

			for (int i = 0; i < m_database.gtNames.size(); i++) {
				ImGui::BeginChild(strFormat("name%d", i).c_str(), ImVec2(0, 24));

				{
					char tmp[SP_STRMAX];
					sp::strcpy(tmp, m_database.gtNames[i].c_str());

					ImGui::PushItemWidth(100);
					if (ImGui::InputText("", tmp, SP_STRMAX)) {
						m_database.gtNames[i] = tmp;
					}
					ImGui::PopItemWidth();
				}
				{
					ImGui::SameLine();

					if (ImGui::Button("add")) {
						m_database.gtNames.add(i + 1, "");
						m_database.updateLabel(i + 1, +1);
					}
				}
				{
					ImGui::SameLine();

					if (ImGui::Button("del")) {
						m_database.gtNames.del(i);
						m_database.updateLabel(i, -1);
						i--;
					}
				}
				ImGui::EndChild();
			}

			ImGui::End();
		}
	}

	virtual void windowSize(int width, int height) {
		adjustImg();
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {

		if (m_keyAction[GLFW_KEY_A] > 0) {
			select(m_selectid + 1);
			adjustImg();
		}
		if (m_keyAction[GLFW_KEY_S] > 0) {
			select(m_selectid - 1);
			adjustImg();
		}
	}
	virtual void mouseButton(int button, int action, int mods) {
		if (isValid() == false) return;

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
		if (isValid() == false) return;

		const Mat vmat = getViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
		const Vec2 pix = invMat(vmat) * m_mouse.pos;

		if (m_focus != NULL && m_base != NULL) {
			m_focus->rect = orRect(getRect2(pix), getRect2(*m_base));
		}
	}

	virtual void mouseScroll(double x, double y) {
	}

};


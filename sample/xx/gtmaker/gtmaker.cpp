#include "gtmaker.h"

using namespace sp;

void GTMakerGUI::dispRectGT() {
	if (m_database.isValid() == false) return;

	glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

	auto dispRectLine = [](const Rect &rect, const Col3 &col, float size)-> void {
		for (int i = 0; i < 2; i++) {
			glLineWidth(i == 0 ? size : size - 2.0f);
			glBegin(GL_LINES);
			glColor(i == 0 ? col / 3.0 : col);
			glRect(rect);
			glEnd();
		}
	};
	auto dispRectPoint = [](const Rect &rect, const Col3 &col, float size)-> void {
		for (int i = 0; i < 2; i++) {
			glPointSize(i == 0 ? size : size - 2.0f);
			glBegin(GL_POINTS);
			glColor(i == 0 ? col / 3.0 : col);
			glRect(rect);
			glEnd();
		}
	};

	MemP<RectGT> &gts = m_database.gtsList[m_selectid];

	for (int i = 0; i < gts.size(); i++) {
		RectGT &gt = gts[i];
		if (&gt != m_focus && m_mode >= 0) continue;

		dispRectLine(gt.rect, getCol(80, 180, 180), 3.0f);
	}

	if (m_focus != NULL) {
		if (m_mode == Mode::Base) {
			dispRectLine(m_focus->rect, getCol(220, 240, 220), 3.0f);
			dispRectPoint(m_focus->rect, getCol(220, 240, 220), 7.0f);
		}
		if (m_mode == Mode::Paint) {
			dispRectLine(m_focus->rect, getCol(180, 240, 180), 3.0f);
		}
	}

	Mem1<const char *> combolist;
	for (int i = 0; i < m_database.gtNames.size(); i++) {
		combolist.push(m_database.gtNames[i].c_str());
	}

	const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

	for (int i = 0; i < gts.size(); i++) {
		RectGT &gt = gts[i];

		if (&gt != m_focus && m_mode >= 0) continue;

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

				const int wsize = (gt.label >= 0) ? static_cast<int>(m_database.gtNames[gt.label].size()) : 0;

				ImGui::SetWindowSize(ImVec2(wsize * 7.0f + 30.0f, 35.0f), ImGuiCond_Always);

				if (gt.label >= 0) {
					ImGui::SameLine();
					ImGui::Text(m_database.gtNames[gt.label].c_str());
				}
			}
			else {
				ImGui::Text("-");

				int size = 0;
				if (m_usePaint == true) size += 50;

				ImGui::SetWindowSize(ImVec2(168.0f + size, 35.0f), ImGuiCond_Always);

				ImGui::SameLine();

				ImGui::PushItemWidth(100);
				ImGui::Combo("", &gt.label, combolist.ptr, combolist.size());
				ImGui::PopItemWidth();

				ImGui::SameLine();

				if (m_usePaint == true && ImGui::Button("paint")) {
					m_mode = (m_mode == Mode::Base) ? Mode::Paint : Mode::Base;
				}

				ImGui::SameLine();

				if (ImGui::Button("del")) {
					gts.free(&gt);
					m_focus = NULL;
				}
			}
			ImGui::End();
		}
	}
}

void GTMakerGUI::dispDataBase() {
	if (m_database.isValid() == false) return;


	if (ImGui::Begin("database", NULL, ImGuiWindowFlags_Block)) {

		ImGui::SetWindowRect(getRect2(15, 35, 190, m_wcam.dsize[1] - 52), ImGuiCond_Always);

		{
			ImGui::Text("\n");

			ImGui::AlignTextToFramePadding();
			ImGui::BulletText(m_database.imNames[m_selectid].c_str());

			ImGui::BeginChild("image", ImVec2(0, 26));
			ImGui::AlignTextToFramePadding();
			if (ImGui::InputInt(strFormat("/%d", m_database.imNames.size()).c_str(), &m_selectid, 1, 100)) {
				select(m_selectid);
			}

			ImGui::EndChild();
		}

		ImGui::Separator();

		{
			ImGui::Text("\n");

			ImGui::AlignTextToFramePadding();
			ImGui::BulletText("work dir");

			ImGui::BeginChild("work dir", ImVec2(118, 24));

			ImGui::AlignTextToFramePadding();
			const Mem1<string> split = strSplit(m_database.wkDir.c_str(), "\\");
			if (ImGui::MenuItem(split.last()->c_str())) {
				m_database.open_wkDir();
				reset();
			}
			ImGui::EndChild();

			ImGui::SameLine();

			if (ImGui::Button("save")) {
				m_database.save();
			}
		}

		ImGui::Separator();

		{
			ImGui::Text("\n");

			ImGui::AlignTextToFramePadding();
			ImGui::BeginChild("label", ImVec2(0, 24));

			ImGui::AlignTextToFramePadding();
			ImGui::BulletText("labels");

			ImGui::SameLine(0, 31);

			if (ImGui::Button("add")) {
				m_database.gtNames.add(0, "");
				m_database.updateLabel(0, +1);
			}
			ImGui::EndChild();


			ImGui::BeginChild("names", ImVec2(180, 140));

			for (int i = 0; i < m_database.gtNames.size(); i++) {
				ImGui::BeginChild(strFormat("name%d", i).c_str(), ImVec2(0, 24));

				{
					char tmp[SP_STRMAX];
					sp::strcpy(tmp, m_database.gtNames[i].c_str());

					ImGui::PushItemWidth(85);
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

					ImGui::SameLine();

					if (ImGui::Button("del")) {
						m_database.gtNames.del(i);
						m_database.updateLabel(i, -1);
						i--;
					}
				}
				ImGui::EndChild();
			}
			ImGui::EndChild();
		}

		ImGui::Separator();

		ImGui::End();
	}
}

void GTMakerGUI::mousebuttonRect(int button, int action, int mods) {

	const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
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

void GTMakerGUI::mousePosRect(double x, double y) {
	const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
	const Vec2 pix = invMat(vmat) * m_mouse.pos;

	if (m_focus != NULL && m_base != NULL) {
		m_focus->rect = orRect(getRect2(pix), getRect2(*m_base));
	}
}


void GTMakerGUI::mousebuttonPaint(int button, int action, int mods) {

}

void GTMakerGUI::mousePosPaint(double x, double y) {

}

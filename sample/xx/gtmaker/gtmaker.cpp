#include "gtmaker.h"

using namespace sp;

void GTMakerGUI::dispRectGT() {
	if (m_database.isValid() == false) return;

	glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

	auto dispRect = [](const Rect &rect, const Col3 &col, bool focus)-> void {
		{
			for (int i = 0; i < 2; i++) {
				glLineWidth(i == 0 ? 3.0f : 1.0f);
				glBegin(GL_LINES);
				glColor(i == 0 ? col / 4.0 : col);
				glRect(rect);
				glEnd();
			}
		}
		if (focus == true) {
			for (int i = 0; i < 2; i++) {
				glPointSize(i == 0 ? 5.0f : 3.0f);
				glBegin(GL_POINTS);
				glColor(i == 0 ? col / 4.0 : col);
				glRect(rect);
				glEnd();
			}
		}
	};


	MemP<RectGT> &gts = m_database.gtsList[m_selectid];

	for (int i = 0; i < gts.size(); i++) {
		dispRect(gts[i].rect, getCol(80, 180, 180), false);
	}

	if (m_focus != NULL) {
		dispRect(m_focus->rect, getCol(220, 240, 220), true);
	}

	Mem1<const char *> combolist;
	for (int i = 0; i < m_database.gtNames.size(); i++) {
		combolist.push(m_database.gtNames[i].c_str());
	}

	const Mat vmat = getViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

	for (int i = 0; i < gts.size(); i++) {
		RectGT &gt = gts[i];


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
		}

		{
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




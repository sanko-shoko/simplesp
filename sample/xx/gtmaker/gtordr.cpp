#include "gtmaker.h"

using namespace sp;

//--------------------------------------------------------------------------------
// global
//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------
// member
//--------------------------------------------------------------------------------

void GTMakerGUI::initOrdr() {

}

void GTMakerGUI::menuOrdr() {
    const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

    MemP<GT> &gts = m_database.gtsList[m_selectid];

    Mem1<const char *> combolist;
    for (int i = 0; i < m_database.gtNames.size(); i++) {
        combolist.push(m_database.gtNames[i].c_str());
    }

    for (int i = 0; i < gts.size(); i++) {
        GT &gt = gts[i];

        if (&gt == m_focus && m_state == S_Init) continue;

        if (ImGui::Begin(strFormat("GT %p", &gt).c_str(), NULL, ImGuiWindowFlags_Block)) {
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
                ImGui::SetWindowSize(ImVec2(210.0f, 35.0f), ImGuiCond_Always);
                
                ImGui::Text("-");
               
                ImGui::SameLine();

                ImGui::PushItemWidth(100);
                ImGui::Combo("", &gt.label, combolist.ptr, combolist.size());
                ImGui::PopItemWidth();

                ImGui::SameLine();

                if (ImGui::ButtonPopup("del", "delete?")) {
                    gts.free(&gt);
                    m_focus = NULL;
                }

                ImGui::SameLine();

                if (ImGui::Button("edit")) {
                    setMode(M_Cont);
                }
            }
            ImGui::End();
        }
    }
}

void GTMakerGUI::dispOrdr() {

    MemP<GT> &gts = m_database.gtsList[m_selectid];

    glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

    for (int i = 0; i < gts.size(); i++) {
        GT &gt = gts[i];

        {
            Render::line(gt.contour, RENDER_GRAY, 3.0f, true);
        }
        
        if (&gt != m_focus) {
            Render::line(vertex2(gt.rect), RENDER_BASE, 3.0f, true);
        }
    }

    if (m_focus != NULL) {
        if (m_state == S_Edit) {
            Render::line(vertex2(m_focus->rect), RENDER_HIGH, 3.0f, true);
        }
        else {
            Render::line(vertex2(m_focus->rect), RENDER_HIGH, 3.0f, true);
            Render::point(vertex2(m_focus->rect), RENDER_HIGH, 7.0f);
        }
    }

}

void GTMakerGUI::mouseButtonOrdr(int button, int action, int mods) {
}

void GTMakerGUI::mousePosOrdr(double x, double y) {
}


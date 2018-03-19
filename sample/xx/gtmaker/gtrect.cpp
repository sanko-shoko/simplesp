#include "gtmaker.h"

using namespace sp;

//--------------------------------------------------------------------------------
// global
//--------------------------------------------------------------------------------

// rectangle base pos
Vec2 g_basePos;


//--------------------------------------------------------------------------------
// member
//--------------------------------------------------------------------------------

void GTMakerGUI::initRect() {
}

void GTMakerGUI::menuRect() {
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
                const int wsize = (gt.label >= 0) ? static_cast<int>(m_database.gtNames[gt.label].size()) : 0;

                ImGui::SetWindowSize(ImVec2(wsize * 7.0f + 30.0f, 35.0f), ImGuiCond_Always);

                ImGui::Text(">");

                if (gt.label >= 0) {
                    ImGui::SameLine();
                    ImGui::Text(m_database.gtNames[gt.label].c_str());
                }
            }
            else {
                ImGui::SetWindowSize(ImVec2(168.0f, 35.0f), ImGuiCond_Always);
                
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

            }
            ImGui::End();
        }
    }

}

void GTMakerGUI::dispRect() {

    MemP<GT> &gts = m_database.gtsList[m_selectid];

    glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

    for (int i = 0; i < gts.size(); i++) {
        GT &gt = gts[i];

        Render::line(getVtx2(gt.rect), RENDER_BASE, 3.0f, true);
    }

    if (m_focus != NULL) {
        if (m_state == S_Edit) {
            Render::line(getVtx2(m_focus->rect), RENDER_HIGH, 3.0f, true);
        }
        else {
            Render::line(getVtx2(m_focus->rect), RENDER_HIGH, 3.0f, true);
            Render::point(getVtx2(m_focus->rect), RENDER_HIGH, 7.0f);
        }
    }

}

void GTMakerGUI::mouseButtonRect(int button, int action, int mods) {

    const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
    const Vec2 pix = invMat(vmat) * m_mouse.pos;

    switch (m_mouse.bDownL) {
    case 1:
    {
        int find = -1;
        g_basePos = pix;

        if (m_focus != NULL) {
            const Mem1<Vec2> pixs = getVtx2(m_focus->rect);
            find = findNearPos(pixs, pix);
            if(find >= 0){
                g_basePos = pixs[(find + 2) % 4];
            }
        }

        if (find < 0) {
            m_state = S_Init;
            m_focus = m_database.gtsList[m_selectid].malloc();
            m_focus->init(getRect2(pix));
        }
        else {
            m_state = S_Edit;
        }
        break;
    }
    case 2:
    {
        break;
    }
    case 0:
    {
        if (m_focus == NULL) break;

        if (m_state == S_Init && minVal(m_focus->rect.dsize[0], m_focus->rect.dsize[1]) < 10) {
            m_database.gtsList[m_selectid].free(m_focus);
            m_focus = NULL;
        }

        if (m_focus != NULL) {
            m_focus->rect = andRect(m_focus->rect, getRect2(m_img.dsize));
        }

        m_state = S_Base;

        break;
    }
    }
}

void GTMakerGUI::mousePosRect(double x, double y) {
    if (m_focus == NULL || m_state == S_Base) return;

    const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
    const Vec2 pix = invMat(vmat) * m_mouse.pos;

    m_focus->rect = orRect(getRect2(pix), getRect2(g_basePos));
}


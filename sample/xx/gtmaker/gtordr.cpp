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

    for (int i = 0; i < gts.size(); i++) {
        GT &gt = gts[i];

        if (gt.contour.size() == 0) continue;
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
                ImGui::SetWindowSize(ImVec2(50.0f, 35.0f), ImGuiCond_Always);

                ImGui::Text("> %02d", i);
            }
            else {
                ImGui::SetWindowSize(ImVec2(168.0f, 35.0f), ImGuiCond_Always);
                
                ImGui::Text("- %02d", i);
               
                ImGui::SameLine();

                ImGui::Text("order");

                ImGui::SameLine(0, 15.0f);

                if (ImGui::Button("++")) {
                    if (i < gts.size() - 1) {
                        sp::swap(gts[i], gts[i + 1]);
                        m_focus = &gts[i + 1];
                    }
                }
                
                ImGui::SameLine(0, 15.0f);

                if (ImGui::Button("--")) {
                    if (i > 0) {
                        sp::swap(gts[i], gts[i - 1]);
                        m_focus = &gts[i - 1];
                    }
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

        if (gt.contour.size() == 0) continue;

        Render::line(getVtx2(gt.rect), RENDER_GRAY, 3.0f, true);

        const Mem1<Mesh2> meshes = divMesh(gt.contour);
        Render::fill(meshes, getCol(gt.label), 3.0f);
    }

    if (m_focus != NULL) {
        Render::line(m_focus->contour, RENDER_HIGH, 3.0f, true);
        Render::point(m_focus->contour, RENDER_HIGH, 7.0f);
    }

}

void GTMakerGUI::mouseButtonOrdr(int button, int action, int mods) {
}

void GTMakerGUI::mousePosOrdr(double x, double y) {
}


#include "gtmaker.h"

using namespace sp;

//--------------------------------------------------------------------------------
// ui
//--------------------------------------------------------------------------------

void GTMakerGUI::dispData() {

    if (ImGui::Begin("database", NULL, ImGuiWindowFlags_Block)) {

        ImGui::SetWindowRect(getRect2(15, 35, 190, m_wcam.dsize[1] - 52), ImGuiCond_Always);

        // image
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

        // directory
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

        // label
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

//--------------------------------------------------------------------------------
// others
//--------------------------------------------------------------------------------

int GTMakerGUI::findNearPos(const Mem1<Vec2> &pnts, const Vec2 &pix) {
    const double thresh = 8.0 / m_viewScale;

    int pos = -1;

    double minv = SP_INFINITY;
    for (int i = 0; i < pnts.size(); i++) {
        const double norm = normVec(pnts[i] - pix);
        if (norm < minVal(minv, thresh)) {
            minv = norm;
            pos = i;
        }
    }

    return pos;
}

int GTMakerGUI::findNearLine(const Mem1<Vec2> &pnts, const Vec2 &pix) {
    if (pnts.size() <= 1) return -1;

    const double thresh = 8.0 / m_viewScale;

    int pos = -1;

    double minv = SP_INFINITY;
    for (int i = 0; i < pnts.size(); i++) {
        const Vec2 a = pnts[(i + 0) % pnts.size()];
        const Vec2 b = pnts[(i + 1) % pnts.size()];
        const Vec2 v = unitVec(a - b);

        const Vec2 nrm = getVec(-v.y, v.x);
        const double norm = ::fabs(dotVec(nrm, a - pix));
        const double in = dotVec(v, a - pix) * dotVec(v, b - pix);

        if (norm < minVal(minv, thresh) && in <= 0) {
            minv = norm;
            pos = i;
        }
    }

    return pos;
}
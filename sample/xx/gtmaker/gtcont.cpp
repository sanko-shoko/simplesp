#include "gtmaker.h"

using namespace sp;

//--------------------------------------------------------------------------------
// global
//--------------------------------------------------------------------------------

Mem1<Vec2> *g_crnt;

Vec2 *g_select;


//--------------------------------------------------------------------------------
// member
//--------------------------------------------------------------------------------

void GTMakerGUI::initCont(){
    g_crnt = &m_focus->contour;
    g_select = NULL;
}

void GTMakerGUI::menuCont() {
    if (m_focus == NULL) return;

    const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

    const GT &gt = *m_focus;

    if (ImGui::Begin("editor", NULL, ImGuiWindowFlags_Block)) {
        {
            const Vec2 pos = vmat * getVec(gt.rect.dbase[0], gt.rect.dbase[1]) + getVec(0.0, -40.0);
            ImGui::SetWindowPos(ImVec2(static_cast<float>(pos.x), static_cast<float>(pos.y)), ImGuiCond_Always);
        }

        ImGui::AlignTextToFramePadding();

        {
            ImGui::SetWindowSize(ImVec2(168.0f, 35.0f), ImGuiCond_Always);

            ImGui::Text("-");

            ImGui::SameLine();

            ImGui::Text("contour");

            ImGui::SameLine();

            if (ImGui::ButtonPopup("reset", "reset?")) {
                g_crnt->clear();
            }

            ImGui::SameLine();

            if (ImGui::Button("ok")) {
                setMode(M_Rect);
            }
        }

        ImGui::End();
    }
}

void GTMakerGUI::dispCont() {
    if (m_focus == NULL) return;

    glLoadView2D(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);

    const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
    const Vec2 pix = invMat(vmat) * m_mouse.pos;

    const int findPos = findNearPos(m_focus->contour, pix);
    const int findLine = findNearLine(*g_crnt, pix);

    MemP<GT> &gts = m_database.gtsList[m_selectid];

    {
        {
            Render::line(vertex2(m_focus->rect), RENDER_BASE, 3.0f, true);
        }

        Mem1<Vec2> contour = *g_crnt;
        bool loop = true;

        if (m_state == S_Init) {
            contour.push(*g_select);
            loop = false;
        }
        
        Render::line(contour, RENDER_HIGH, 3.0f, loop);
        Render::point(contour, RENDER_HIGH, 7.0f);
    }

    if (findPos >= 0) {
        Render::point(m_focus->contour[findPos], RENDER_NEAR, 7.0f);
    }
    else if (m_focus->contour.size() > 0 && findLine >= 0) {
        const Vec2 a = m_focus->contour[(findLine + 0) % m_focus->contour.size()];
        const Vec2 b = m_focus->contour[(findLine + 1) % m_focus->contour.size()];
        const Vec2 v = unitVec(a - b);

        const Vec2 nrm = getVec(-v.y, v.x);
        const double norm = ::fabs(dotVec(nrm, a - pix));
        const double in = dotVec(v, a - pix) * dotVec(v, b - pix);
        const double thresh = 8.0 / m_viewScale;

        if (norm < thresh && in <= 0) {
            const Vec2 p = pix + nrm * dotVec(nrm, a - pix);
            Render::point(p, RENDER_NEAR, 7.0f);
        }
    }

}

void GTMakerGUI::mouseButtonCont(int button, int action, int mods) {
    if (m_focus == NULL) return;

    const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
    const Vec2 pix = invMat(vmat) * m_mouse.pos;

    const int findPos = findNearPos(*g_crnt, pix);
    const int findLine = findNearLine(*g_crnt, pix);
 
    static Mem1<Vec2> edit;
    static Vec2 select;

    select = pix;

    switch (m_mouse.bDownL) {
    case 1:
    {
        if (m_state == S_Base && m_focus->contour.size() == 0){
            edit.clear();

            g_crnt = &edit;
            g_crnt->push(pix);
            
            g_select = &select;

            m_state = S_Init;
        }
        else if (m_state == S_Init) {
            if (findPos == 0) {
                m_focus->contour = edit;
                g_crnt = &m_focus->contour;

                g_select = NULL;

                m_state = S_Base;
            }
            else {
                g_crnt->push(pix);
                g_select = &select;

                m_state = S_Init;
            }
        }
        else if (findPos >= 0) {
            edit = m_focus->contour;
            g_crnt = &edit;

            g_select = &edit[findPos];
            m_state = S_Edit;
        }
        else if (findLine >= 0) {
            m_focus->contour.add(findLine + 1, pix);

            edit = m_focus->contour;
            g_crnt = &edit;

            g_select = &edit[findLine + 1];
            m_state = S_Edit;
        }
        break;
    }
    case 0:
    {
        if (m_state == S_Edit) {
            m_focus->contour = edit;
            g_crnt = &m_focus->contour;
            m_state = S_Base;
        }
        break;
    }
    }

    switch (m_mouse.bDownR) {
    case 1:
    {  
        if (findPos >= 0) {
            if (m_focus->contour.size() > 1) {
                m_focus->contour.del(findPos);
            }
            else {
                m_focus->contour.clear();
            }
        }

        break;
    }
    case 0:
    {
        break;
    }
    }

    {
        const Rect rect = getRect2(m_img.dsize);
        const Vec2 a = getVec(rect.dbase[0], rect.dbase[1]);
        const Vec2 b = a + getVec(rect.dsize[0] - 1, rect.dsize[1] - 1);
        for (int i = 0; i < g_crnt->size(); i++) {
            (*g_crnt)[i].x = sp::round((*g_crnt)[i].x);
            (*g_crnt)[i].y = sp::round((*g_crnt)[i].y);

            (*g_crnt)[i].x = maxVal((*g_crnt)[i].x, a.x);
            (*g_crnt)[i].y = maxVal((*g_crnt)[i].y, a.y);
            (*g_crnt)[i].x = minVal((*g_crnt)[i].x, b.x);
            (*g_crnt)[i].y = minVal((*g_crnt)[i].y, b.y);
        }
    }
}

void GTMakerGUI::mousePosCont(double x, double y) {
    if (m_focus == NULL || m_state == S_Base) return;

    const Mat vmat = glGetViewMat(m_img.dsize[0], m_img.dsize[1], m_viewPos, m_viewScale);
    const Vec2 pix = invMat(vmat) * m_mouse.pos;
   
    const int findPos = findNearPos(m_focus->contour, pix);
    
    if (g_select != NULL) {
        *g_select = pix;
    }
    if (m_state == S_Edit) {
        m_focus->contour = *g_crnt;
    }
}

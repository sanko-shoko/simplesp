//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_IMGUI_H__
#define __SP_IMGUI_H__

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "imgui_internal.h"

#include "GLFW/glfw3.h"

#include "spcore/spcore.h"

#include <string>

namespace ImGui {

    SP_CPUFUNC void SetWindowRect(const sp::Rect &rect, const ImGuiCond cond) {
        SP_ASSERT(rect.dim == 2);

        ImGui::SetWindowPos(ImVec2(static_cast<float>(rect.dbase[0]), static_cast<float>(rect.dbase[1])), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(static_cast<float>(rect.dsize[0]), static_cast<float>(rect.dsize[1])), ImGuiCond_Always);
    }

    SP_CPUFUNC bool ShowText(const std::string text, const ImVec2 &pos, const ImVec4 &col = ImVec4(1.f, 1.f, 1.f, 1.f), const float scale = 1.f) {
        
        char name[32] = { 0 };
        const int maxv = 100;
        for (int i = 0; i < maxv; i++) {
            sprintf(name, "showtext%04d", i);
            const ImGuiWindow* window = ImGui::FindWindowByName(name);
            if (window == NULL || window->Active == false) {
                break;
            }
        }

        ImGui::PushStyleColor(ImGuiCol_Text, col);
        {
            ImGui::Begin(name, NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground);
            const float backup = ImGui::GetFontSize();
            ImGui::SetWindowFontScale(scale);
            ImGui::SetWindowPos(pos, ImGuiCond_Always);
 
            ImGui::Text(text.c_str());

            ImGui::SetWindowFontScale(backup);

            ImGui::End();
        }
        ImGui::PopStyleColor(1);
        return true;
    }
}

#endif
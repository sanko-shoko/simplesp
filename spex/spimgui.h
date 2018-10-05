//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_IMGUI_H__
#define __SP_IMGUI_H__

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_internal.h"

#include "spcore/spcore.h"
#include "GLFW/glfw3.h"

namespace ImGui {

#define ImGuiWindowFlags_Block (ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing)

    SP_CPUFUNC void SetWindowRect(const sp::Rect &rect, const ImGuiCond cond) {
        SP_ASSERT(rect.dim == 2);

        ImGui::SetWindowPos(ImVec2(static_cast<float>(rect.dbase[0]), static_cast<float>(rect.dbase[1])), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(static_cast<float>(rect.dsize[0]), static_cast<float>(rect.dsize[1])), ImGuiCond_Always);
    }

    SP_CPUFUNC bool ButtonPopup(const char *name, const char *popup) {
        if (ImGui::Button(name)) {
            ImGui::OpenPopup(popup);
        }

        bool ret = false;
        if (ImGui::BeginPopupModal(popup, NULL, ImGuiWindowFlags_AlwaysAutoResize)){
            if (ImGui::Button("ok", ImVec2(80, 0))) {
                ImGui::CloseCurrentPopup();
                ret = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("cancel", ImVec2(80, 0))) {
                ImGui::CloseCurrentPopup();
                ret = false;
            }
            ImGui::EndPopup();
        }
        return ret;
    }

    SP_CPUFUNC bool showText(const char *text, const ImVec2 &pos, const ImVec4 &col = ImVec4(1.f, 1.f, 1.f, 1.f), const float scale = 1.f) {

        char name[32] = { 0 };
        const int maxv = 1000;
        for (int i = 0; i < maxv; i++) {
            sprintf(name, "showtext%04d", i);
            const ImGuiWindow* window = ImGui::FindWindowByName(name);
            if (window == NULL || window->Active == false) {
                break;
            }
        }

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.f, 0.f, 0.f, 0.f));
        ImGui::PushStyleColor(ImGuiCol_Text, col);

        if (ImGui::Begin(name, NULL, ImGuiWindowFlags_Block | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoInputs)) {
            const float backup = ImGui::GetWindowFontSize();
            ImGui::SetWindowFontScale(scale);
            ImGui::SetWindowSize(ImVec2(400.f, 30.f));
            ImGui::SetWindowPos(pos, ImGuiCond_Always);
 
            ImGui::Text(text);

            ImGui::SetWindowFontScale(backup);

            ImGui::End();
        }

        ImGui::PopStyleColor(2);
        return true;
    }
}

#endif
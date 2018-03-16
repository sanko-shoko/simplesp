//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_IMGUI_H__
#define __SP_IMGUI_H__

#include "imgui.h"
#include "imgui_impl_glfw.h"

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
}

#endif
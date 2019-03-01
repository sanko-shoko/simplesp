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

namespace sp {
    using namespace ImGui;

    SP_CPUFUNC Col4 getCol(const ImVec4 &imv) {
        const Col4 col = getCol(static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.w * SP_BYTEMAX + 0.5));
        return col;
    }

    SP_CPUFUNC void glColor(const ImVec4 &vec) {
        glColor4d(vec.x, vec.y, vec.z, vec.w);
    }
}

namespace ImGui {

#define ImGuiWindowFlags_Block (ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing)
    
    using namespace sp;

    SP_CPUFUNC ImVec4 getImVec4(const Col3 &col, const Byte a = SP_BYTEMAX) {
        const ImVec4 vec(static_cast<float>(col.r) / SP_BYTEMAX, static_cast<float>(col.g) / SP_BYTEMAX, static_cast<float>(col.b) / SP_BYTEMAX, static_cast<float>(a) / SP_BYTEMAX);
        return vec;
    }

    SP_CPUFUNC ImVec4 getImVec4(const Col4 &col) {
        const ImVec4 vec(static_cast<float>(col.r) / SP_BYTEMAX, static_cast<float>(col.g) / SP_BYTEMAX, static_cast<float>(col.b) / SP_BYTEMAX, static_cast<float>(col.a) / SP_BYTEMAX);
        return vec;
    }

    SP_CPUFUNC bool operator == (ImVec4 &vec0, ImVec4 &vec1) {
        bool ret = true;
        ret &= (vec0.x == vec1.x);
        ret &= (vec0.y == vec1.y);
        ret &= (vec0.z == vec1.z);
        ret &= (vec0.w == vec1.w);
        return ret;
    }

    SP_CPUFUNC bool operator != (ImVec4 &vec0, ImVec4 &vec1) {
        return !(vec0 == vec1);
    }

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
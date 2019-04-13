﻿//--------------------------------------------------------------------------------
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

    SP_CPUFUNC Col4 getCol(const ImVec4 &imv) {
        const Col4 col = getCol(static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.w * SP_BYTEMAX + 0.5));
        return col;
    }

    SP_CPUFUNC void glColor(const ImVec4 &imv) {
        glColor4d(imv.x, imv.y, imv.z, imv.w);
    }

    SP_CPUFUNC Vec2 getVec2(const ImVec2 &imv) {
        return getVec2(imv.x, imv.y);
    }

}

SP_CPUFUNC bool operator == (const ImVec4 &vec0, const ImVec4 &vec1) {
    bool ret = true;
    ret &= (vec0.x == vec1.x);
    ret &= (vec0.y == vec1.y);
    ret &= (vec0.z == vec1.z);
    ret &= (vec0.w == vec1.w);
    return ret;
}

SP_CPUFUNC bool operator != (const ImVec4 &vec0, const ImVec4 &vec1) {
    return !(vec0 == vec1);
}

SP_CPUFUNC ImVec2 getImVec2(const sp::Vec2 &spvec) {
    const ImVec2 vec(static_cast<float>(spvec.x), static_cast<float>(spvec.y));
    return vec;
}

SP_CPUFUNC ImVec4 getImVec4(const sp::Col3 &spcol, const sp::Byte a = SP_BYTEMAX) {
    const ImVec4 vec(static_cast<float>(spcol.r) / SP_BYTEMAX, static_cast<float>(spcol.g) / SP_BYTEMAX, static_cast<float>(spcol.b) / SP_BYTEMAX, static_cast<float>(a) / SP_BYTEMAX);
    return vec;
}

SP_CPUFUNC ImVec4 getImVec4(const sp::Col4 &spcol) {
    const ImVec4 vec(static_cast<float>(spcol.r) / SP_BYTEMAX, static_cast<float>(spcol.g) / SP_BYTEMAX, static_cast<float>(spcol.b) / SP_BYTEMAX, static_cast<float>(spcol.a) / SP_BYTEMAX);
    return vec;
}

namespace ImGui {

#define ImGuiWindowFlags_Block (ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing)
    
    using namespace sp;


    SP_CPUFUNC void SetNextWindowRect(const sp::Rect &rect, const ImGuiCond cond) {
        SP_ASSERT(rect.dim == 2);

        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(rect.dbase[0]), static_cast<float>(rect.dbase[1])), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(rect.dsize[0]), static_cast<float>(rect.dsize[1])), ImGuiCond_Always);
    }
    SP_CPUFUNC void SetWindowRect(const sp::Rect &rect, const ImGuiCond cond) {
        SP_ASSERT(rect.dim == 2);

        ImGui::SetWindowPos(ImVec2(static_cast<float>(rect.dbase[0]), static_cast<float>(rect.dbase[1])), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(static_cast<float>(rect.dsize[0]), static_cast<float>(rect.dsize[1])), ImGuiCond_Always);
    }

    SP_CPUFUNC void Spacing(const float space) {
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(1.0f, space));
        ImGui::Spacing();
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::PopStyleVar();
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
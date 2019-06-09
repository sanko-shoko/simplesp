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

    SP_CPUFUNC Col4 getCol4(const ImVec4 &imv) {
        const Col4 col = getCol4(static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.w * SP_BYTEMAX + 0.5));
        return col;
    }
    SP_CPUFUNC Col3 getCol3(const ImVec4 &imv) {
        const Col3 col = getCol3(static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5));
        return col;
    }

    SP_CPUFUNC Vec2 getVec2(const ImVec2 &imv) {
        return getVec2(imv.x, imv.y);
    }
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

    static void SetNextWindowRect(const Rect2 &rect, const ImGuiCond cond, const Rect2 *limit = NULL) {
        Rect2 _rect = rect;
        if (limit != NULL) {
            for (int i = 0; i < 2; i++) {
                if (_rect.dbase[i] < limit->dbase[i]) _rect.dbase[i] = limit->dbase[i];
                if (_rect.dbase[i] + _rect.dsize[i] > limit->dbase[i] + limit->dsize[i]) _rect.dbase[i] = limit->dbase[i] + limit->dsize[i] - _rect.dsize[i];
            }
        }
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(_rect.dbase[0]), static_cast<float>(_rect.dbase[1])), cond);
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(_rect.dsize[0]), static_cast<float>(_rect.dsize[1])), cond);
    }

    static void SetWindowRect(const Rect2 &rect, const ImGuiCond cond, const Rect2 *limit = NULL) {
        Rect2 _rect = rect;
        if (limit != NULL) {
            for (int i = 0; i < 2; i++) {
                if (_rect.dbase[i] < limit->dbase[i]) _rect.dbase[i] = limit->dbase[i];
                if (_rect.dbase[i] > limit->dbase[i] + limit->dsize[i] - _rect.dsize[i]) _rect.dbase[i] = limit->dbase[i] + limit->dsize[i] - _rect.dsize[i];
            }
        }
        ImGui::SetWindowPos(ImVec2(static_cast<float>(_rect.dbase[0]), static_cast<float>(_rect.dbase[1])), cond);
        ImGui::SetWindowSize(ImVec2(static_cast<float>(_rect.dsize[0]), static_cast<float>(_rect.dsize[1])), cond);
    }

    static Rect2 GetWindowRect() {
        ImGuiWindow* window = ImGui::GetCurrentWindow();

        return getRect2(sp::round(window->Pos.x), sp::round(window->Pos.y), sp::round(window->Size.x), sp::round(window->Size.y));
    }

    static void Spacing(const float space) {
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(1.0f, 0.0f));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::Dummy(ImVec2(0.0f, space));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::PopStyleVar();
    }

    static bool ShowText(const std::string text, const ImVec2 &pos, const ImVec4 &col = ImVec4(1.f, 1.f, 1.f, 1.f), const float scale = 1.f) {

        char name[32] = { 0 };
        const int maxv = 100;
        for (int i = 0; i < maxv; i++) {
            sprintf(name, "##showtext%04d", i);
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
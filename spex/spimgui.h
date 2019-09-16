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

namespace sp {

    SP_CPUFUNC Col3 getCol3(const ImVec4 &imv) {
        return getCol3(static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5));
    }
    SP_CPUFUNC Col4 getCol4(const ImVec4 &imv) {
        return getCol4(static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5), static_cast<Byte>(imv.w * SP_BYTEMAX + 0.5));
    }
    SP_CPUFUNC Col3f getCol3f(const ImVec4 &imv) {
        return getCol3f(imv.x, imv.y, imv.z);
    }
    SP_CPUFUNC Col4f getCol4f(const ImVec4 &imv) {
        return getCol4f(imv.x, imv.y, imv.z, imv.w);
    }
}

static ImVec4 getImVec4(const sp::Col3 &spcol, const sp::Byte a = SP_BYTEMAX) {
    const ImVec4 vec(static_cast<float>(spcol.r) / SP_BYTEMAX, static_cast<float>(spcol.g) / SP_BYTEMAX, static_cast<float>(spcol.b) / SP_BYTEMAX, static_cast<float>(a) / SP_BYTEMAX);
    return vec;
}

static ImVec4 getImVec4(const sp::Col4 &spcol) {
    const ImVec4 vec(static_cast<float>(spcol.r) / SP_BYTEMAX, static_cast<float>(spcol.g) / SP_BYTEMAX, static_cast<float>(spcol.b) / SP_BYTEMAX, static_cast<float>(spcol.a) / SP_BYTEMAX);
    return vec;
}

static ImVec4 getImVec4(const sp::Col3f &spcol, const float a = 1.0f) {
    const ImVec4 vec(spcol.r, spcol.g, spcol.b, a);
    return vec;
}

static ImVec4 getImVec4(const sp::Col4f &spcol) {
    const ImVec4 vec(spcol.r, spcol.g, spcol.b, spcol.a);
    return vec;
}

namespace sp {

    class StyleStack {
    private:
        int pcount[2] = { 0 };
    public:
        ~StyleStack() {
            popAll();
        }
        void popAll() {
            if (pcount[0] > 0) {
                ImGui::PopStyleVar(pcount[0]);
                pcount[0] = 0;
            }
            if (pcount[1] > 0) {
                ImGui::PopStyleColor(pcount[1]);
                pcount[1] = 0;
            }
        }

        void pushVar(ImGuiStyleVar idx, float val) {
            ImGui::PushStyleVar(idx, val);
            pcount[0]++;
        }
        void pushVar(ImGuiStyleVar idx, const ImVec2& val) {
            ImGui::PushStyleVar(idx, val);
            pcount[0]++;
        }
        void pushColor(ImGuiCol idx, const ImVec4& col) {
            ImGui::PushStyleColor(idx, col);
            pcount[1]++;
        }

        void pushButton(const ImVec4 &btn, const ImVec4 &btnHovered, const ImVec4 &btnActive) {
            pushColor(ImGuiCol_Button, btn);
            pushColor(ImGuiCol_ButtonHovered, btnHovered);
            pushColor(ImGuiCol_ButtonActive, btnActive);
        }

    };

    class Popup {
    public:
        bool start;

        const char *mess;
        std::function<void()> func;
        std::function<void()> init;
        std::function<void()> fini;
        std::function<bool()> begin;

    public:

        Popup() {
            reset();
        }

        void reset() {
            func = NULL;
            fini = NULL;
            begin = NULL;
            start = false;
        }

        void open(const char *mess, std::function<void()> init, std::function<void()> func, std::function<void()> fini, const bool modal) {
            reset();
            this->mess = mess;
            this->init = init;
            this->func = func;
            this->fini = fini;
            start = true;

            if (modal) {
                begin = [&]()->bool {
                    return ImGui::BeginPopupModal(this->mess, NULL, ImGuiWindowFlags_AlwaysAutoResize);
                };
            }
            else {
                begin = [&]()->bool {
                    return ImGui::BeginPopup(this->mess, ImGuiWindowFlags_AlwaysAutoResize);
                };
            }
        }

        void run() {
            if (func == NULL) return;

            if (start) {

                ImGui::OpenPopup(mess);
                if (init != NULL) {
                    init();
                }
            }

            if (begin()) {
                if (func != NULL) {
                    func();
                }
                ImGui::EndPopup();
            }
            if (ImGui::IsPopupOpen(mess) == false) {
                if (fini != NULL) fini();
                reset();
            }
            start = false;
        }
    };

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

    static bool ShowText(const char *text, const ImVec2 &pos, const ImVec4 &col = ImVec4(1.f, 1.f, 1.f, 1.f), const float scale = 1.f) {

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

            ImGui::Text(text);

            ImGui::SetWindowFontScale(backup);

            ImGui::End();
        }
        ImGui::PopStyleColor(1);
        return true;
    }

    static int ColorPicker(ImVec4 &imcol, const bool alpha = false) {
        const int noedit = ImGuiColorEditFlags_AlphaPreviewHalf | ImGuiColorEditFlags_RGB | ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop | ImGuiColorEditFlags_NoTooltip;

        {
            ImGui::PushItemWidth(254.0f);
            if (alpha == true) {
                ImGui::ColorPicker4("##picker", (float*)&imcol, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview);
            }
            else {
                ImGui::ColorPicker3("##picker", (float*)&imcol, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview);
            }
            ImGui::PopItemWidth();
        }

        ImGui::SameLine();

        int ret = -1;
        {
            ImGui::BeginGroup();

            ImGui::ColorButton("##current", imcol, noedit, ImVec2(56.0f, 40.0f));
            ImGui::Spacing(2.0f);

            for (int r = 0; r < 13; r++) {
                for (int c = 0; c < 4; c++) {
                    StyleStack stack;
                    stack.pushColor(ImGuiCol_Border, ImVec4(0.95f, 0.95f, 0.95f, 1.00f));
                    stack.pushVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

                    if (c != 0) ImGui::SameLine();

                    ImGui::PushID(r * 4 + c);
                    if (ImGui::ColorButton("##std color", getImVec4(sp::stdcol(r, c)), noedit, ImVec2(14.0f, 14.0f))) {
                        imcol = getImVec4(sp::stdcol(r, c), static_cast<sp::Byte>(imcol.w * SP_BYTEMAX));
                    }
                    ImGui::PopID();
                }
            }
            ImGui::Spacing();

            if (ImGui::Button("ok", ImVec2(-1.0f, 0.0f)) == true) {
                ret = 0;
                ImGui::CloseCurrentPopup();
            }
            if (ImGui::Button("cancel", ImVec2(-1.0f, 0.0f)) == true) {
                ret = 1;
                ImGui::CloseCurrentPopup();
            }
            if (ImGui::Button("clear", ImVec2(-1.0f, 0.0f)) == true) {
                ret = 2;
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndGroup();
        }
        return ret;
    }

}


namespace sp {

}
#endif
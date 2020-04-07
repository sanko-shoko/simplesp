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


#define ImGuiColorEditFlags_NoEdit (ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop | ImGuiColorEditFlags_NoTooltip)

#define ImGuiWindowFlags_Block (ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoFocusOnAppearing)

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


namespace ImGui {

    static void SetNextWindowRect(const sp::Rect2 &rect, const ImGuiCond cond, const sp::Rect2 *limit = NULL) {
        sp::Rect2 _rect = rect;
        if (limit != NULL) {
            for (int i = 0; i < 2; i++) {
                if (_rect.dbase[i] < limit->dbase[i]) _rect.dbase[i] = limit->dbase[i];
                if (_rect.dbase[i] + _rect.dsize[i] > limit->dbase[i] + limit->dsize[i]) _rect.dbase[i] = limit->dbase[i] + limit->dsize[i] - _rect.dsize[i];
            }
        }
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(_rect.dbase[0]), static_cast<float>(_rect.dbase[1])), cond);
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(_rect.dsize[0]), static_cast<float>(_rect.dsize[1])), cond);
    }

    static void SetWindowRect(const sp::Rect2 &rect, const ImGuiCond cond, const sp::Rect2 *limit = NULL) {
        sp::Rect2 _rect = rect;
        if (limit != NULL) {
            for (int i = 0; i < 2; i++) {
                if (_rect.dbase[i] < limit->dbase[i]) _rect.dbase[i] = limit->dbase[i];
                if (_rect.dbase[i] > limit->dbase[i] + limit->dsize[i] - _rect.dsize[i]) _rect.dbase[i] = limit->dbase[i] + limit->dsize[i] - _rect.dsize[i];
            }
        }
        ImGui::SetWindowPos(ImVec2(static_cast<float>(_rect.dbase[0]), static_cast<float>(_rect.dbase[1])), cond);
        ImGui::SetWindowSize(ImVec2(static_cast<float>(_rect.dsize[0]), static_cast<float>(_rect.dsize[1])), cond);
    }

    static sp::Rect2 GetWindowRect() {
        ImGuiWindow* window = ImGui::GetCurrentWindow();

        return sp::getRect2(sp::round(window->Pos.x), sp::round(window->Pos.y), sp::round(window->Size.x), sp::round(window->Size.y));
    }

    static void Spacing(const float space) {
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(1.0f, 0.0f));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::Dummy(ImVec2(0.0f, space));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::PopStyleVar();
    }

    static void align(const ImVec2 size, const int list, const int id = 0) {
        const float p = ImGui::GetStyle().WindowPadding.x * 0.5f;
        const float d = (ImGui::GetWindowWidth() - size.x * list) * 0.5f;

        const float s = (d * 0.2f > 6.0f) ? d * 0.2f : 6.0f;
        const float m = (ImGui::GetWindowWidth() - size.x * list - s * (list - 1)) * 0.5f;
        if (id == 0) ImGui::Dummy(ImVec2(1.0f, 1.0f));
        ImGui::SameLine(-1.0f, m + (size.x + s) * id + 1.0f);
    }
    static bool Button(const char *label, const ImVec2 size, const int list, const int id = 0) {
        align(size, list, id);

        return ImGui::Button(label, size);
    }

}
namespace sp {

    SP_CPUFUNC void _cast(Col3 &dst, const ImVec4 &imv) {
        dst.r = static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5);
        dst.g = static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5);
        dst.b = static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5);
    }
    SP_CPUFUNC void _cast(Col4 &dst, const ImVec4 &imv) {
        dst.r = static_cast<Byte>(imv.x * SP_BYTEMAX + 0.5);
        dst.g = static_cast<Byte>(imv.y * SP_BYTEMAX + 0.5);
        dst.b = static_cast<Byte>(imv.z * SP_BYTEMAX + 0.5);
        dst.a = static_cast<Byte>(imv.w * SP_BYTEMAX + 0.5);
    }
    SP_CPUFUNC void _cast(Col3f &dst, const ImVec4 &imv) {
        dst.r = imv.x;
        dst.g = imv.y;
        dst.b = imv.z;
    }
    SP_CPUFUNC void _cast(Col4f &dst, const ImVec4 &imv) {
        dst.r = imv.x;
        dst.g = imv.y;
        dst.b = imv.z;
        dst.a = imv.w;
    }

    class StyleStack {
    private:
        int pcount[3] = { 0 };
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
            if (pcount[2] > 0) {
                for (int i = 0; i < pcount[2]; i++) {
                    ImGui::PopItemWidth();
                }
                pcount[2] = 0;
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
        void pushItemWidth(const float width) {
            ImGui::PushItemWidth(width);
        }

    };

    class Popup {
    public:
        bool start;

        const char *name;
        std::function<void()> func;
        std::function<void()> init;
        std::function<void()> fini;
        std::function<bool()> begin;

        char mess[SP_STRMAX];
        std::function<void()> func_ok;
        std::function<void()> func_cn;

        ImGuiWindowFlags flag;
    public:

        Popup() {
            reset();
        }

        void reset() {
            func = NULL;
            fini = NULL;
            begin = NULL;
            start = false;

            func_ok = NULL;
            func_cn = NULL;
        }

        void open(const char *name, std::function<void()> init, std::function<void()> func, const bool modal, const bool title = true) {

            this->name = name;
            this->init = init;
            this->func = func;
            start = true;

            this->flag = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize;
            if (title == false) {
                this->flag = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar;
            }

            if (modal) {
                begin = [&]()->bool {
                    return ImGui::BeginPopupModal(this->name, NULL, this->flag);
                };
            }
            else {
                begin = [&]()->bool {
                    return ImGui::BeginPopup(this->name, this->flag);
                };
            }
        }

        void dialog(const char *name, const char *mess, std::function<void()> init, std::function<void()> ok, std::function<void()> cn) {

            strcpy(this->mess, mess);
            this->func_ok = ok;
            this->func_cn = cn;

            std::function<void()> func = [&]() {
                {
                    const ImVec4 col = ImGui::GetStyle().Colors[ImGuiCol_PopupBg];
                    sp::StyleStack stack;
                    stack.pushButton(col, col, col);
                    ImGui::Button((std::string(this->mess) + "##message").c_str(), ImVec2(280.0f, 0.0f), 1);
                }

                {
                    const int num = ((func_ok != NULL) ? 1 : 0) + ((func_cn != NULL) ? 1 : 0);
                    int cnt = 0;
                    if (func_ok != NULL && ImGui::Button("ok##message", ImVec2(100.0f, 0.0f), num, cnt++)) {
                        func_ok();
                        ImGui::CloseCurrentPopup();
                    }
                    if (func_cn != NULL && ImGui::Button("cancel##message", ImVec2(100.0f, 0.0f), num, cnt++)) {
                        func_cn();
                        ImGui::CloseCurrentPopup();
                    }
                }
            };
            open(name, init, func, true);
        }

        void run() {
            if (func == NULL) return;

            if (start) {

                ImGui::OpenPopup(name);
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
            if (start == false && ImGui::IsPopupOpen(name) == false) {
                //if (fini != NULL) fini();
                reset();
            }
            start = false;
        }

    };

    class PopupMgr {
    public:
        static Popup& inst(const int i) {
            SP_ASSERT(i >= 0 && i < 10);
            static Popup popup[10];
            return popup[i];
        }

        static Popup& get() {
            Popup *p = NULL;
            for (int i = 0; i < 10; i++) {
                if (inst(i).func == NULL) {
                    p = &inst(i);
                    break;
                }
            }
            SP_ASSERT(p != NULL);
            return *p;
        }

        static void run() {
            for (int i = 0; i < 10; i++) {
                inst(i).run();
            }
        }

        static std::function<void()> mes(const char *mess, std::function<void()> func_ok, std::function<void()> func_cn) {
            static char _mess[SP_STRMAX];
            static std::function<void()> _func_ok;
            static std::function<void()> _func_cn;

            strcpy(_mess, mess);
            _func_ok = func_ok;
            _func_cn = func_cn;

            std::function<void()> func = [&]() {
                {
                    const ImVec4 col = ImGui::GetStyle().Colors[ImGuiCol_PopupBg];
                    sp::StyleStack stack;
                    stack.pushButton(col, col, col);
                    ImGui::Button((std::string(_mess) + "##message").c_str(), ImVec2(280.0f, 0.0f), 1);
                }

                {
                    const int num = ((_func_ok != NULL) ? 1 : 0) + ((_func_cn != NULL) ? 1 : 0);
                    int cnt = 0;

                    if (_func_ok != NULL && ImGui::Button("ok##message", ImVec2(100.0f, 0.0f), num, cnt++)) {
                        _func_ok();
                        ImGui::CloseCurrentPopup();
                    }
                    if (_func_cn != NULL && ImGui::Button("cancel##message", ImVec2(100.0f, 0.0f), num, cnt++)) {
                        _func_cn();
                        ImGui::CloseCurrentPopup();
                    }
                }

            };
            return func;
        }

    };


    // -1: closed, 0: init, 1: opened, 2: ok, 3: cancel, 4: clear
    SP_CPUFUNC int ColorPicker(const char *popup, Col4f &col, const bool alpha = false) {
        StyleStack stack;
        stack.pushVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 4.0f));

        static Col4f backup;
        static ImVec4 imcol;

        int ret = 1;
        if (ImGui::IsMouseReleased(1) && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByPopup)) {
            ImGui::OpenPopup(popup);
            backup = col;
            imcol = getImVec4(col);
            ret = 0;
        }
        if (ImGui::BeginPopup(popup) == false) return -1;

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

        {
            ImGui::BeginGroup();
            const int type = (alpha == false) ? ImGuiColorEditFlags_None : ImGuiColorEditFlags_AlphaPreviewHalf;
            ImGui::ColorButton("##current", imcol, ImGuiColorEditFlags_NoEdit | type, ImVec2(56.0f, 40.0f));
            ImGui::Spacing(2.0f);

            for (int r = 0; r < 13; r++) {
                for (int c = 0; c < 4; c++) {
                    StyleStack stack;
                    stack.pushColor(ImGuiCol_Border, ImVec4(0.82f, 0.82f, 0.82f, 1.00f));
                    stack.pushVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
                    stack.pushVar(ImGuiStyleVar_FrameRounding, 4.0f);

                    if (c != 0) ImGui::SameLine();

                    ImGui::PushID(r * 4 + c);
                    int A[] = { 2, 3, 4, 5 };
                    int B[] = { 3, 4, 5, 6 };
                    int *list = (r == 0) ? A : B;

                    ImGui::ColorButton("##std color", getImVec4(sp::stdcol(r, list[c])), ImGuiColorEditFlags_NoEdit, ImVec2(14.0f, 14.0f));
                    if (ImGui::IsItemClicked(0)) {
                        imcol = getImVec4(sp::stdcol(r, list[c]), static_cast<sp::Byte>(imcol.w * SP_BYTEMAX));
                    }
                    ImGui::PopID();
                }
            }
            ImGui::Spacing();

            if (ImGui::Button("ok", ImVec2(-1.0f, 0.0f)) == true) {
                ret = 2;
                ImGui::CloseCurrentPopup();
            }
            if (ImGui::Button("cancel", ImVec2(-1.0f, 0.0f)) == true) {
                ret = 3;
                col = backup;
                ImGui::CloseCurrentPopup();
            }
            if (ImGui::Button("clear", ImVec2(-1.0f, 0.0f)) == true) {
                ret = 4;
                col = sp::getCol4f(1.0f, 1.0f, 1.0f, 1.0f);
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndGroup();
        }
        if (ret == 1) {
            col = sp::cast<sp::Col4f>(imcol);
        }

        ImGui::EndPopup();

        return ret;
    }
}
#endif
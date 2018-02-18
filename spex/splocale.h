//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_LOCALE_H__
#define __SP_LOCALE_H__

#include "simplesp.h"

#include <vector>
#include <string>
#include <locale>
#include <codecvt>

namespace sp {

    using namespace std;

    bool cnvUTF16ToUTF8(string &dst, const void *ptr, const int size) {

        wstring_convert<codecvt_utf8<uint16_t>, uint16_t> cnv;

        dst = cnv.to_bytes((uint16_t*)ptr, (uint16_t*)ptr + size / 2);
        return true;
    }

    bool cnvUTF8ToMB(string &dst, const string &src) {
        wstring_convert<codecvt_utf8_utf16<wchar_t>> converter;
        wstring wstr = converter.from_bytes(src);

        size_t converted{};
        vector<char> dest(wstr.size() * sizeof(wchar_t) + 1, '\0');

        if (::_wcstombs_s_l(&converted, dest.data(), dest.size(), wstr.data(), _TRUNCATE, ::_create_locale(LC_ALL, "jpn")) != 0) {
            throw system_error{ errno, system_category() };
        }
        dst = string(dest.begin(), dest.end());
        return true;
    }

    bool cnvUTF16ToMB(string &dst, const string &str) {
        string utf8;
        cnvUTF16ToUTF8(utf8, str.data(), static_cast<int>(str.size()));
        cnvUTF8ToMB(dst, utf8);
        return true;
    }


}
#endif
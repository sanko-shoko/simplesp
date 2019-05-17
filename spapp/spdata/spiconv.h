//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_ICONV_H__
#define __SP_ICONV_H__

#include "spcore/spcore.h"

#if defined(_WIN32)
#include <windows.h>
#else
#endif

#include <string>

namespace sp{

    //--------------------------------------------------------------------------------
    // character code convert
    //--------------------------------------------------------------------------------

    SP_CPUFUNC char* _cnvSJIStoUTF8(char *dst, const char *src) {

#if defined(_WIN32)
        char* utf8 = NULL;
        char* utf16 = NULL;

        { // SJIS -> UTF16
            const int size = ::MultiByteToWideChar(CP_ACP, 0, (LPCSTR)src, -1, NULL, 0);

            utf16 = new char[size * 2 + 2];
            ::memset(utf16, 0, size * 2 + 2);
            ::MultiByteToWideChar(CP_ACP, 0, (LPCSTR)src, -1, (LPWSTR)utf16, size);
        }
        { // UTF16 -> UTF8
            const int size = ::WideCharToMultiByte(CP_UTF8, 0, (LPCWSTR)utf16, -1, NULL, 0, NULL, NULL);

            utf8 = new char[size + 1];
            ::memset(utf8, 0, size + 1);
            ::WideCharToMultiByte(CP_UTF8, 0, (LPCWSTR)utf16, -1, (LPSTR)utf8, size, NULL, NULL);
        }
        {
            const int size = ::lstrlen((char*)utf8);
            memcpy(dst, utf8, size + 1);
        }

        delete utf16;
        delete utf8;
#endif

        return dst;
    }

    SP_CPUFUNC char* _cnvUTF8toSJIS(char *dst, const char *src) {

#if defined(_WIN32)
        char* sjis = NULL;
        char* utf16 = NULL;

        { // UTF8 -> UTF16
            const int size = ::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)src, -1, NULL, 0);

            utf16 = new char[size * 2 + 2];
            ::memset(utf16, 0, size * 2 + 2);
            ::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)src, -1, (LPWSTR)utf16, size);
        }
        { // UTF16 -> SJIS
            const int size = ::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)utf16, -1, NULL, 0, NULL, NULL);

            sjis = new char[size + 1];
            ::memset(sjis, 0, size + 1);
            ::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)utf16, -1, (LPSTR)sjis, size, NULL, NULL);
        }
        {
            const int size = strlen((char*)sjis);
            memcpy(dst, sjis, size + 1);
        }

        delete utf16;
        delete sjis;
#endif

        return dst;
    }

#define SP_SJIS_TO_UTF8 0
#define SP_UTF8_TO_SJIS 1

    SP_CPUFUNC char* strcode(char *dst, const char *src, const int mode) {
        switch (mode) {
        case SP_SJIS_TO_UTF8: _cnvSJIStoUTF8(dst, src); break;
        case SP_UTF8_TO_SJIS: _cnvUTF8toSJIS(dst, src); break;
        }
        return dst;
    }

    SP_CPUFUNC std::string strcode(const char *src, const int mode) {

        char dst[SP_STRMAX] = { 0 };
        strcode(dst, src, mode);
        return std::string(dst);
    }

}
#endif
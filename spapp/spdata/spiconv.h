//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
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

    //--------------------------------------------------------------------------------
    // SJIS - UTF16
    //--------------------------------------------------------------------------------
    SP_CPUFUNC int _cnvSJIStoUTF16(char *dst, const char *src) {
        int msize = 0;
#if defined(_WIN32)
        char* utf16 = NULL;

        { // SJIS -> UTF16
            const int size = ::MultiByteToWideChar(CP_ACP, 0, (LPCSTR)src, -1, NULL, 0);

            utf16 = new char[2 * (size + 1)];
            ::memset(utf16, 0, size * 2 + 2);
            ::MultiByteToWideChar(CP_ACP, 0, (LPCSTR)src, -1, (LPWSTR)utf16, size);
        }
        {
            const int size = static_cast<int>(::wcslen((wchar_t*)utf16));
            msize = 2 * (size + 1);
            memcpy(dst, utf16, msize);
        }

        delete utf16;
#endif

        return msize;
    }

    SP_CPUFUNC int _cnvUTF16toSJIS(char *dst, const char *src) {
        int msize = 0;

#if defined(_WIN32)
        char* sjis = NULL;

        { // UTF16 -> SJIS
            const int size = ::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)src, -1, NULL, 0, NULL, NULL);

            sjis = new char[size + 1];
            ::memset(sjis, 0, size + 1);
            ::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)src, -1, (LPSTR)sjis, size, NULL, NULL);
        }
        {
            const int size = strlen((char*)sjis);
            msize = size + 1;
            memcpy(dst, sjis, msize);
        }

        delete sjis;
#endif

        return msize;
    }

    //--------------------------------------------------------------------------------
    // SJIS - UTF8
    //--------------------------------------------------------------------------------
    SP_CPUFUNC int _cnvSJIStoUTF8(char *dst, const char *src) {
        int msize = 0;

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
            msize = size + 1;
            memcpy(dst, utf8, msize);
        }

        delete utf16;
        delete utf8;
#endif

        return msize;
    }

    SP_CPUFUNC int _cnvUTF8toSJIS(char *dst, const char *src) {
        int msize = 0;

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
            msize = size + 1;
            memcpy(dst, sjis, msize);
        }

        delete utf16;
        delete sjis;
#endif

        return msize;
    }

#define SP_SJIS_TO_UTF8 0
#define SP_UTF8_TO_SJIS 1
#define SP_SJIS_TO_UTF16 2
#define SP_UTF16_TO_SJIS 3

    SP_CPUFUNC int strcode(char *dst, const char *src, const int mode) {
        int msize = 0;
        switch (mode) {
        case SP_SJIS_TO_UTF8: msize = _cnvSJIStoUTF8(dst, src); break;
        case SP_UTF8_TO_SJIS: msize = _cnvUTF8toSJIS(dst, src); break;
        case SP_SJIS_TO_UTF16: msize = _cnvSJIStoUTF16(dst, src); break;
        case SP_UTF16_TO_SJIS: msize = _cnvUTF16toSJIS(dst, src); break;
        }
        return msize;
    }


}
#endif
//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STR_H__
#define __SP_STR_H__

#include "spcore/spcom.h"
#include "spcore/spwrap.h"

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else

#endif

#include <string>

namespace sp{

    SP_CPUFUNC bool split(char *dst, const char *src, const int x, const char *tok = " ,\t\n\r"){
        char *buf = new char[static_cast<int>(::strlen(src) + 1)];

        int cnt = 0;
        const char *ret = NULL;

        if (x < 0) {
            ::strcpy(buf, src);

            const char *tmp = NULL;
            for (int i = 0; ; i++, cnt++) {
                tmp = (i == 0) ? ::strtok(buf, tok) : ::strtok(NULL, tok);
                if (tmp == NULL) break;
            }
        }
        {
            ::strcpy(buf, src);

            for (int i = 0; i <= cnt + x; i++) {
                ret = (i == 0) ? ::strtok(buf, tok) : ::strtok(NULL, tok);
                if (ret == NULL) break;
            }
        }

        if (ret != NULL) {
            ::strcpy(dst, ret);
        }

        delete[]buf;
        return (ret != NULL) ? true : false;
    }
    
    SP_CPUFUNC std::string split(const char *src, const int x, const char *tok = " ,\t\n\r") {
        std::string ret;

        char *buf = new char[static_cast<int>(::strlen(src) + 1)];
        if (split(buf, src, x, tok) == true) {
            ret = buf;
        }
        delete[]buf;

        return ret;
    }

    //--------------------------------------------------------------------------------
    // code
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool _convSJIStoUTF8(char *dst, char *src) {
        bool ret = false;

        int size = 0;
        int *pSize = &size;

#if defined(_WIN32) || defined(_WIN64)
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

            utf8 = new char[size * 2 + 2];
            ::memset(utf8, 0, size * 2 + 2);
            ::WideCharToMultiByte(CP_UTF8, 0, (LPCWSTR)utf16, -1, (LPSTR)utf8, size, NULL, NULL);
        }
        {
            const int size = ::lstrlen((char*)utf8);
            memcpy(dst, utf8, size + 1);
        }

        delete utf16;
        delete utf8;

        ret = true;
#endif

        return ret;
    }

#define SP_SJIS_TO_UTF8 0

    SP_CPUFUNC bool strcode(char* dst, char* src, const int type) {
        bool ret = false;
        switch (type) {
        case SP_SJIS_TO_UTF8: ret = _convSJIStoUTF8(dst, src); break;
        }
        return ret;
    }


    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC char* progress(const double crnt, const double size) {
        const double rate = (size > 1) ? crnt / (size - 1.0) : 1.0;

        static char bar[11] = { 0 };
        for (int i = 0; i < 10; i++) {
            bar[i] = (i <= 10 * rate) ? '>' : '-';
        }
        return bar;
    }

    SP_CPUFUNC std::string tstamp(const char *format = "%Y%m%d_%H%M%S") {
        char str[SP_STRMAX];
        time_t t = time(NULL);
        strftime(str, sizeof(str), format, localtime(&t));
        return std::string(str);
    }

}
#endif
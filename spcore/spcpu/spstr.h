//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STR_H__
#define __SP_STR_H__

#include "spcore/spcom.h"
#include <string.h>

namespace sp{

    //--------------------------------------------------------------------------------
    // string
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void memset(void *dst, const int val, const int size) {
        ::memset(dst, val, size);
    }

    SP_CPUFUNC void memcpy(void *dst, const void *src, const int size) {
        ::memcpy(dst, src, size);
    }

    SP_CPUFUNC int strlen(const char *str) {
        return static_cast<int>(::strlen(str));
    }

    SP_CPUFUNC char* strcpy(char *dst, const char *str) {
        return ::strcpy(dst, str);
    }

    SP_CPUFUNC char* strcat(char *dst, const char *str) {
        return ::strcat(dst, str);
    }

    SP_CPUFUNC int strcmp(const char *str0, const char *str1) {
        return ::strcmp(str0, str1);
    }

    SP_CPUFUNC const char* strstr(const char *str, const char *substr) {
        return ::strstr(str, substr);
    }

    SP_CPUFUNC char* strtok(char *str, const char * tok) {
        return ::strtok(str, tok);
    }


    SP_CPUFUNC char* strget(char *dst, const char *src, const int x, const char *tok = " ,\t\n\r"){
        char *buf = new char[strlen(src) + 1];

        int c = 0;
        if (x < 0) {
            strcpy(buf, src);

            char *tmp = NULL;
            for (int i = 0; ; i++, c++) {
                tmp = strtok((i == 0) ? buf : NULL, tok);
                if (tmp == NULL) break;
            }
        }

        char *str = NULL;
        {
            strcpy(buf, src);

            for (int i = 0; i <= c + x; i++) {
                str = strtok((i == 0) ? buf : NULL, tok);
                if (str == NULL) break;
            }
        }
        if (str != NULL) {
            strcpy(dst, str);
        }

        delete[]buf;

        return (str != NULL) ? dst : NULL;
    }
    
}
#endif
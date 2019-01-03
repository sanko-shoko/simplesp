//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_WRAP_H__
#define __SP_WRAP_H__


#ifndef SP_USE_CONSOLE
#define SP_USE_CONSOLE 1
#endif

#ifndef SP_USE_WRAPPER
#define SP_USE_WRAPPER 1
#endif


//--------------------------------------------------------------------------------
// base include
//--------------------------------------------------------------------------------

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>


namespace sp{

    //--------------------------------------------------------------------------------
    // stdio
    //--------------------------------------------------------------------------------

#if SP_USE_CONSOLE
#define SP_PRINTF(...) ::printf(__VA_ARGS__);
#else
#define SP_PRINTF(...) if(0){ ::printf(__VA_ARGS__); }
#endif

#if SP_USE_ASSERT
#define SP_ASSERT(EXP) if(!(EXP)){ ::printf(" assert (%s)\n file: %s\n line: %d\n", #EXP, __FILE__, __LINE__); ::exit(0); }
#else
#define SP_ASSERT(EXP) (EXP);
#endif


#if SP_USE_WRAPPER

    //--------------------------------------------------------------------------------
    // stdlib
    //--------------------------------------------------------------------------------

    SP_GENFUNC int abs(const int x){
        return ::abs(x);
    }

    SP_GENFUNC double fabs(const double x){
        return ::fabs(x);
    }

    static unsigned int SP_RANDSEED = 0;
    SP_GENFUNC void srand(const int seed) {
        SP_RANDSEED = static_cast<unsigned int>(seed);
    }

    SP_GENFUNC int rand() {
        unsigned int x = SP_RANDSEED + 1;
        x ^= (x << 13);
        x ^= (x >> 17);
        x ^= (x << 15);
        SP_RANDSEED = x;
        return static_cast<int>(x >> 1);
    }

    SP_GENFUNC void qsort(void *base, const int nsize, const int esize, int compare(const void *a, const void *b)) {
        ::qsort(base, nsize, esize, compare);
    }

    //--------------------------------------------------------------------------------
    // string
    //--------------------------------------------------------------------------------

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

    SP_CPUFUNC char* strtok(char *str, const char * tok){
        return ::strtok(str, tok);
    }

    SP_CPUFUNC void memset(void *dst, const int val, const int size){
        ::memset(dst, val, size);
    }

    SP_CPUFUNC void memcpy(void *dst, const void *src, const int size){
        ::memcpy(dst, src, size);
    }


    //--------------------------------------------------------------------------------
    // math
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC double pow(const double x, const double y){
        return static_cast<double>(::pow(x, y));
    }

    SP_GENFUNC double sin(const double x){
        return static_cast<double>(::sin(x));
    }

    SP_GENFUNC double asin(const double x){
        const double t = (x > +1.0) ? +1.0 : (x < -1.0) ? -1.0 : x;
        return static_cast<double>(::asin(t));
    }

    SP_GENFUNC double cos(const double x){
        return static_cast<double>(::cos(x));
    }

    SP_GENFUNC double acos(const double x){
        const double t = (x > +1.0) ? +1.0 : (x < -1.0) ? -1.0 : x;
        return static_cast<double>(::acos(t));
    }

    SP_GENFUNC double tan(const double x){
        return static_cast<double>(::tan(x));
    }

    SP_GENFUNC double atan(const double x){
        return static_cast<double>(::atan(x));
    }

    SP_GENFUNC double atan2(const double y, const double x){
        return static_cast<double>(::atan2(y, x));
    }

    SP_GENFUNC double sqrt(const double x){
        return static_cast<double>(::sqrt(x));
    }

    SP_GENFUNC double exp(const double x){
        return static_cast<double>(::exp(x));
    }

    SP_GENFUNC double log(const double x){
        return static_cast<double>(::log(x));
    }

    SP_GENFUNC double log2(const double x){
        return static_cast<double>(::log(x) / ::log(2.0));
    }

#endif

}

#endif
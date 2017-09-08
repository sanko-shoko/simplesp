//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_WRAP_H__
#define __SP_WRAP_H__


//--------------------------------------------------------------------------------
// base include
//--------------------------------------------------------------------------------

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


namespace sp{


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


	//--------------------------------------------------------------------------------
	// string
	//--------------------------------------------------------------------------------

	SP_CPUFUNC int strlen(const char *str) {
		return static_cast<int>(::strlen(str));
	}

	SP_CPUFUNC char* strcpy(char *dst, const char *str) {
		return ::strcpy(dst, str);
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
		return static_cast<double>(::asin(x));
	}

	SP_GENFUNC double cos(const double x){
		return static_cast<double>(::cos(x));
	}

	SP_GENFUNC double acos(const double x){
		return static_cast<double>(::acos(x));
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
		return static_cast<double>(::log2(x));
	}

#endif

}

#endif
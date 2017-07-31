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

	SP_GENCALL int abs(const int x){
		return ::abs(x);
	}

	SP_GENCALL double fabs(const double x){
		return ::fabs(x);
	}

	static unsigned int SP_RANDSEED = 0;
	SP_GENCALL void srand(const int seed) {
		SP_RANDSEED = static_cast<unsigned int>(seed);
	}

	SP_GENCALL int rand() {
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

	SP_CPUCALL int strlen(const char *str) {
		return static_cast<int>(::strlen(str));
	}

	SP_CPUCALL char* strcpy(char *dst, const char *str) {
		return ::strcpy(dst, str);
	}

	SP_CPUCALL int strcmp(const char *str0, const char *str1) {
		return ::strcmp(str0, str1);
	}

	SP_CPUCALL const char* strstr(const char *str, const char *substr) {
		return ::strstr(str, substr);
	}

	SP_CPUCALL char* strtok(char *str, const char * tok){
		return ::strtok(str, tok);
	}

	SP_CPUCALL void memset(void *dst, const int val, const int size){
		::memset(dst, val, size);
	}

	SP_CPUCALL void memcpy(void *dst, const void *src, const int size){
		::memcpy(dst, src, size);
	}


	//--------------------------------------------------------------------------------
	// math
	//--------------------------------------------------------------------------------
	
	SP_GENCALL double pow(const double x, const double y){
		return static_cast<double>(::pow(x, y));
	}

	SP_GENCALL double sin(const double x){
		return static_cast<double>(::sin(x));
	}

	SP_GENCALL double asin(const double x){
		return static_cast<double>(::asin(x));
	}

	SP_GENCALL double cos(const double x){
		return static_cast<double>(::cos(x));
	}

	SP_GENCALL double acos(const double x){
		return static_cast<double>(::acos(x));
	}

	SP_GENCALL double tan(const double x){
		return static_cast<double>(::tan(x));
	}

	SP_GENCALL double atan(const double x){
		return static_cast<double>(::atan(x));
	}

	SP_GENCALL double atan2(const double y, const double x){
		return static_cast<double>(::atan2(y, x));
	}

	SP_GENCALL double sqrt(const double x){
		return static_cast<double>(::sqrt(x));
	}

	SP_GENCALL double exp(const double x){
		return static_cast<double>(::exp(x));
	}

	SP_GENCALL double log(const double x){
		return static_cast<double>(::log(x));
	}

	SP_GENCALL double log2(const double x){
		return static_cast<double>(::log2(x));
	}

#endif

}

#endif
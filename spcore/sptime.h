//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_TIME_H__
#define __SP_TIME_H__

#include <time.h>

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#else
#include <chrono>
#endif

//--------------------------------------------------------------------------------
// time util
//--------------------------------------------------------------------------------

namespace sp {
    static char* timestamp(char *dst, const char *format = "%Y%m%d_%H%M%S") {
        time_t t = time(NULL);
        strftime(dst, SP_STRMAX - 1, format, localtime(&t));
        return dst;
    }
}


//--------------------------------------------------------------------------------
// timer
//--------------------------------------------------------------------------------

namespace sp {

    class Timer {
    public:

#if defined(_WIN32)
        typedef LARGE_INTEGER Point;
#else
        typedef std::chrono::system_clock::time_point Point;
#endif

        static Point now() {
            Point n;
#if defined(_WIN32)
            QueryPerformanceCounter(&n);
#else
            n = std::chrono::system_clock::now();
#endif
            return n;
        }

        static double dif(const Point &tp0, const Point &tp1) {
            double ms = 0.0;
#if defined(_WIN32)
            Point freq;
            QueryPerformanceFrequency(&freq);
            ms = static_cast<double>((tp1.QuadPart - tp0.QuadPart) * 1000.0 / freq.QuadPart);
#else
            ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(tp1 - tp0).count() / 1000.0);
#endif
            return (ms > 0) ? +ms : -ms;
        }

    private:

        Point tp[2];

    public:

        Timer() {
            tp[0] = now();
            tp[1] = tp[0];
        }

        Timer(const Timer &timer) {
            *this = timer;
        }

        Timer & operator = (const Timer &timer) {
            tp[0] = timer.tp[0];
            tp[1] = timer.tp[1];
            return *this;
        }

        void start() {
            tp[0] = now();
        }

        void stop() {
            tp[1] = now();
        }

        double getms() {
            return dif(tp[0], tp[1]);
        }
    };
}


#endif


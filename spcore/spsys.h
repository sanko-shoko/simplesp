//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SYS_H__
#define __SP_SYS_H__

#ifndef SP_USE_SYS
#define SP_USE_SYS 1
#endif
//--------------------------------------------------------------------------------
// timer
//--------------------------------------------------------------------------------


#if SP_USE_SYS
#if WIN32
#include <windows.h>
#else
#include <chrono>
#endif
#endif

namespace sp {

    class Timer {

    public:

#if SP_USE_SYS
#if WIN32
        typedef LARGE_INTEGER tpoint;
#else
        typedef std::chrono::system_clock::time_point tpoint;
#endif
#else
        typedef double tpoint;
#endif

        static tpoint now() {
            tpoint n;
#if SP_USE_SYS
#if WIN32
            QueryPerformanceCounter(&n);
#else
            n = std::chrono::system_clock::now();
#endif
#else
            n = 0.0;
#endif
            return n;
        }

        static double dif(const tpoint &tp0, const tpoint &tp1) {
            double ms = 0.0;
#if SP_USE_SYS
#if WIN32
            tpoint freq;
            QueryPerformanceFrequency(&freq);

            ms = static_cast<double>((tp1.QuadPart - tp0.QuadPart) * 1000.0 / freq.QuadPart);
#else
            ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(tp1 - tp0).count() / 1000.0);
#endif
#endif
            return (ms > 0) ? +ms : -ms;
        }
    private:
        tpoint tp[2];

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

//--------------------------------------------------------------------------------
// resource
//--------------------------------------------------------------------------------

#if SP_USE_SYS
#if WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <sys/time.h>
#include <sys/resource.h>
#endif
#endif

// get memory usage [KB]
static int getMemoryUsage() {
    int m = -1;
#if SP_USE_SYS
#ifdef WIN32
    HANDLE h = GetCurrentProcess();
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(h, (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        m = static_cast<int>(pmc.PrivateUsage / (1000.0));
    }
    CloseHandle(h);
#else
    rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
        m = static_cast<int>(usage.ru_marss);
    }
#endif
#endif
    return m;
}


//--------------------------------------------------------------------------------
// file util
//--------------------------------------------------------------------------------

#if SP_USE_SYS
#if WIN32
#include <windows.h>
#include <direct.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#endif

static int makeDir(const char *dir) {
    int ret = 0;
#if SP_USE_SYS
#if WIN32
    ret = _mkdir(dir);
#else
    ret = mkdir(dir, 0755);
#endif
#endif
    return ret;
}

static char* getCrntDir() {
    static char dir[SP_STRMAX] = { 0 };
#if SP_USE_SYS
#if WIN32
    GetCurrentDirectory(SP_STRMAX, dir);
#else
    getcwd(dir, SP_STRMAX);
#endif
#endif
    return dir;
}

#endif


//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SYS_H__
#define __SP_SYS_H__

#ifndef SP_USE_SYS
#define SP_USE_SYS 1
#endif

#if SP_USE_SYS

//--------------------------------------------------------------------------------
// timer
//--------------------------------------------------------------------------------

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else
#include <chrono>
#endif

namespace sp {

    class Timer {
    public:

#if defined(_WIN32) || defined(_WIN64)
        typedef LARGE_INTEGER tpoint;
#else
        typedef std::chrono::system_clock::time_point tpoint;
#endif

        static tpoint now() {
            tpoint n;
#if defined(_WIN32) || defined(_WIN64)
            QueryPerformanceCounter(&n);
#else
            n = std::chrono::system_clock::now();
#endif
            return n;
        }

        static double dif(const tpoint &tp0, const tpoint &tp1) {
            double ms = 0.0;
#if defined(_WIN32) || defined(_WIN64)
            tpoint freq;
            QueryPerformanceFrequency(&freq);
            ms = static_cast<double>((tp1.QuadPart - tp0.QuadPart) * 1000.0 / freq.QuadPart);
#else
            ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(tp1 - tp0).count() / 1000.0);
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

#if WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <sys/time.h>
#include <sys/resource.h>
#endif

namespace sp {

    // get memory usage [KB]
    static int getMemoryUsage() {
        int m = -1;
#if defined(_WIN32) || defined(_WIN64)
        HANDLE h = GetCurrentProcess();
        PROCESS_MEMORY_COUNTERS_EX pmc;
        if (GetProcessMemoryInfo(h, (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
            m = static_cast<int>(pmc.PrivateUsage / (1000.0));
        }
        CloseHandle(h);
#else
        rusage usage;
        if (getrusage(RUSAGE_SELF, &usage) == 0) {
            m = static_cast<int>(usage.ru_maxrss);
        }
#endif
        return m;
    }
}


//--------------------------------------------------------------------------------
// file util
//--------------------------------------------------------------------------------

#include <stdlib.h>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#include <direct.h>

#elif defined(__APPLE__)
#include <sys/stat.h>
#include <dlfcn.h>
#include <unistd.h>
#include <libgen.h>

#else
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#endif


namespace sp {

    static int makeDir(const char *dir) {
        int ret = 0;
#if defined(_WIN32) || defined(_WIN64)
        ret = _mkdir(dir);
#else
        ret = mkdir(dir, 0755);
#endif
        return ret;
    }

    static void splitPath(char *drive, char *dir, char *name, char *ext, const char *path) {
        char buff[4][512] = { 0 };
#if defined(_WIN32) || defined(_WIN64)
        _splitpath(path, drive ? drive : buff[0], dir ? dir : buff[1], name ? name : buff[2], ext ? ext : buff[3]);
#else
        
#endif
    }

    static char* getCrntDir() {
        static char dir[512] = { 0 };
#if defined(_WIN32) || defined(_WIN64)
        GetCurrentDirectory(512, dir);
#else
        getcwd(dir, SP_STRMAX);
#endif
        return dir[0] != 0 ? dir : NULL;
    }

    static char* getModulePath() {
        static char path[512] = { 0 };
#if defined(_WIN32) || defined(_WIN64)
        GetModuleFileName(NULL, path, MAX_PATH);
        
#elif defined(__APPLE__)
        Dl_info module_info;
        if (dladdr(reinterpret_cast<void*>(getModulePath), &module_info) != 0) {
            strcpy(path, module_info.dli_fname);
        }

#else
        //readlink("/proc/self/exe", path, sizeof(path) - 1);
#endif
        return path[0] != 0 ? path : NULL;
    }

    static char* getModuleDir() {
        static char dir[512] = { 0 };
        const char *path = getModulePath();
        splitPath(NULL, dir, NULL, NULL, path);
        return dir[0] != 0 ? dir : NULL;;
    }

    static char* getModuleName() {
        static char name[512] = { 0 };
        const char *path = getModulePath();
        splitPath(NULL, NULL, name, NULL, path);
        return name[0] != 0 ? name : NULL;;
    }

}


//--------------------------------------------------------------------------------
// thread
//--------------------------------------------------------------------------------

#include <thread>
#include <mutex>

namespace sp {

    SP_CPUFUNC void sleep(const long long ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    class Thread {
    private:
        bool m_used;
        std::mutex m_mtx;

    public:
        Thread() {
            m_used = false;
        }

        bool used() {
            return m_used;
        }

        template<class Class, void (Class::*Func)()>
        bool run(Class *ptr, const bool wait = true) {
            if (wait == false && m_used == true) return false;

            std::thread th([this, ptr, wait] {
                m_mtx.lock();
                m_used = true;
                (ptr->*Func)();
                m_used = false;
                m_mtx.unlock();
            });
            th.detach();
            return true;
        }

        bool run(std::function<void()> func, const bool wait = true) {
            if (wait == false && m_used == true) return false;

            std::thread th([this, func, wait] {
                m_mtx.lock();
                m_used = true;
                func();
                m_used = false;
                m_mtx.unlock();
            });
            th.detach();
            return true;
        }

        bool run(void (*func)(), const bool wait = true) {
            if (wait == false && m_used == true) return false;

            std::thread th([this, func, wait] {
                m_mtx.lock();
                m_used = true;
                func();
                m_used = false;
                m_mtx.unlock();
            });
            th.detach();
            return true;
        }
    };
}

#endif //SP_USE_SYS

#endif

